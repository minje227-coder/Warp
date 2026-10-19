#include <jni.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include "WarpUvcBulk.h"
static _Atomic int stop_requested=0;
/* Separate stop flag and bounded snapshot cache: idle preview never writes raw data. */
static _Atomic int source_fps_active=0;
static _Atomic double source_fps=-1;
static _Atomic int64_t source_last_frame_ns=0;
JNIEXPORT jdouble JNICALL Java_dev_warp_capture_WarpUsbCamera_measuredFps(JNIEnv *e,jclass c);
static _Atomic int preview_stop=0, preview_snapshot_requested=1;
static pthread_mutex_t preview_lock=PTHREAD_MUTEX_INITIALIZER;
static unsigned char *preview_jpeg=NULL;
static int preview_bytes=0;
static int64_t preview_last_ns=0;
static int64_t now_ns(void);
JNIEXPORT jdouble JNICALL Java_dev_warp_capture_WarpUsbCamera_measuredFps(JNIEnv *e,jclass c){
    (void)e;(void)c;
    if(!atomic_load(&source_fps_active))return -1;
    int64_t last=atomic_load(&source_last_frame_ns);
    if(last>0&&now_ns()-last>1000000000LL)return 0;
    return atomic_load(&source_fps);
}

static void clear_preview(void) {
    pthread_mutex_lock(&preview_lock);
    free(preview_jpeg);preview_jpeg=NULL;preview_bytes=0;preview_last_ns=0;
    pthread_mutex_unlock(&preview_lock);
}
static void publish_preview(const unsigned char *data,int length) {
    if(!atomic_exchange(&preview_snapshot_requested,0))return;
    unsigned char *copy=malloc((size_t)length);if(!copy)return;
    memcpy(copy,data,(size_t)length);
    pthread_mutex_lock(&preview_lock);
    free(preview_jpeg);preview_jpeg=copy;preview_bytes=length;preview_last_ns=now_ns();
    pthread_mutex_unlock(&preview_lock);
}
JNIEXPORT void JNICALL Java_dev_warp_capture_WarpUsbCamera_resetPreviewStop(JNIEnv *e,jclass c){
    (void)e;(void)c;clear_preview();atomic_store(&preview_snapshot_requested,1);atomic_store(&preview_stop,0);
}
JNIEXPORT void JNICALL Java_dev_warp_capture_WarpUsbCamera_requestPreviewStop(JNIEnv *e,jclass c){(void)e;(void)c;atomic_store(&preview_stop,1);}
JNIEXPORT jbyteArray JNICALL Java_dev_warp_capture_WarpUsbCamera_latestPreviewJpeg(JNIEnv *e,jclass c){
    (void)c;atomic_store(&preview_snapshot_requested,1);
    pthread_mutex_lock(&preview_lock);
    jbyteArray out=preview_bytes>0&&now_ns()-preview_last_ns<1000000000LL?(*e)->NewByteArray(e,preview_bytes):NULL;
    if(out)(*e)->SetByteArrayRegion(e,out,0,preview_bytes,(const jbyte*)preview_jpeg);
    pthread_mutex_unlock(&preview_lock);
    return out;
}

JNIEXPORT void JNICALL Java_dev_warp_capture_WarpUsbCamera_resetStop(JNIEnv *e,jclass c){(void)e;(void)c;atomic_store(&stop_requested,0);}
JNIEXPORT void JNICALL Java_dev_warp_capture_WarpUsbCamera_requestStop(JNIEnv *e,jclass c){(void)e;(void)c;atomic_store(&stop_requested,1);}

#define SLOTS 8
#define PACKETS 32
#define MAX_FRAME (8*1024*1024)
typedef struct {struct usbdevfs_urb *urb; unsigned char *buffer; int submitted;} Slot;
static int64_t now_ns(void) {struct timespec t;clock_gettime(CLOCK_BOOTTIME,&t);return (int64_t)t.tv_sec*1000000000+t.tv_nsec;}
static uint32_t le32(const unsigned char *p) {return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}

/* Only owns URBs, never the Java-owned USB fd. Raw timestamp provenance is retained. */
JNIEXPORT jstring JNICALL Java_dev_warp_capture_WarpUsbCamera_capture(JNIEnv *env,jclass cls,jint fd,jint endpoint,jint packet,jstring path,jint duration_ms) {
    (void)cls;
    int preview_mode=path==NULL;
    int64_t fps_window_start=0;int fps_window_count=0;
    atomic_store(&source_fps,-1);atomic_store(&source_last_frame_ns,0);atomic_store(&source_fps_active,1);
    FILE *video=NULL,*log=NULL;
    if(!preview_mode) {
        const char *name=(*env)->GetStringUTFChars(env,path,0);
        if(!name){atomic_store(&source_fps_active,0);return NULL;}
        char logname[1024];snprintf(logname,sizeof(logname),"%s.jsonl",name);
        video=fopen(name,"wb");log=fopen(logname,"w");
        (*env)->ReleaseStringUTFChars(env,path,name);
    }
    Slot slots[SLOTS]={0}; unsigned char *frame=malloc(MAX_FRAME);
    int used=0,fid=-1,broken=0,frames=0,errors=0,result=0,cleanup_pending=0;
    int64_t first=0,start=now_ns(),offset=0;uint32_t pts=0;int have_pts=0;
    char error[128]="";
    if((!preview_mode&&(!video||!log))||!frame||packet<1||packet>49152){snprintf(error,sizeof(error),"allocation_or_file_error");goto cleanup;}
    for(int i=0;i<SLOTS;i++) {
        slots[i].urb=calloc(1,sizeof(struct usbdevfs_urb)+PACKETS*sizeof(struct usbdevfs_iso_packet_desc));
        slots[i].buffer=malloc(PACKETS*packet);
        if(!slots[i].urb||!slots[i].buffer){snprintf(error,sizeof(error),"urb_allocation_failed");goto cleanup;}
        struct usbdevfs_urb *u=slots[i].urb;u->type=USBDEVFS_URB_TYPE_ISO;u->endpoint=endpoint;u->flags=USBDEVFS_URB_ISO_ASAP;
        u->buffer=slots[i].buffer;u->buffer_length=PACKETS*packet;u->number_of_packets=PACKETS;
        for(int j=0;j<PACKETS;j++)u->iso_frame_desc[j].length=packet;
        if(ioctl(fd,USBDEVFS_SUBMITURB,u)<0){snprintf(error,sizeof(error),"submit_errno_%d",errno);goto cleanup;}
        slots[i].submitted=1;
    }
    while(!atomic_load(preview_mode?&preview_stop:&stop_requested)&&now_ns()-start<(int64_t)duration_ms*1000000) {
        struct usbdevfs_urb *u=NULL;
        if(ioctl(fd,USBDEVFS_REAPURBNDELAY,&u)<0){if(errno==EAGAIN){usleep(1000);continue;}snprintf(error,sizeof(error),"reap_errno_%d",errno);break;}
        Slot *slot=NULL;for(int i=0;i<SLOTS;i++)if(slots[i].urb==u){slot=&slots[i];slot->submitted=0;break;}
        if(!slot){snprintf(error,sizeof(error),"unknown_urb");break;}
        for(int j=0;j<PACKETS;j++) {
            struct usbdevfs_iso_packet_desc *d=&u->iso_frame_desc[j];
            if(d->status){errors++;broken=1;continue;}
            int n=d->actual_length;if(n<2)continue;
            unsigned char *p=slot->buffer+j*packet;int h=p[0],flags=p[1],next_fid=flags&1;
            if(h<2||h>n){errors++;broken=1;continue;}
            if(next_fid!=fid){used=0;broken=0;first=now_ns();fid=next_fid;have_pts=0;}
            if(flags&0x40){broken=1;errors++;}
            if((flags&4)&&h>=6){pts=le32(p+2);have_pts=1;}
            int payload=n-h;
            if(payload>MAX_FRAME-used){broken=1;errors++;}
            else if(!broken&&payload){memcpy(frame+used,p+h,payload);used+=payload;}
            if(flags&2) {
                if(!broken&&used>=4&&frame[0]==0xff&&frame[1]==0xd8&&frame[used-2]==0xff&&frame[used-1]==0xd9) {
                    int64_t last=now_ns();
                    atomic_store(&source_last_frame_ns,last);
                    if(!fps_window_start){fps_window_start=last;fps_window_count=0;}
                    else {
                        fps_window_count++;
                        if(last-fps_window_start>=500000000LL){
                            atomic_store(&source_fps,fps_window_count*1e9/(last-fps_window_start));
                            fps_window_start=last;fps_window_count=0;
                        }
                    }
                    if(preview_mode){publish_preview(frame,used);frames++;}
                    else {
                    publish_preview(frame,used);
                    if(fwrite(frame,1,used,video)!=(size_t)used){snprintf(error,sizeof(error),"frame_write_failed");goto cleanup;}
                    fprintf(log,"{\"index\":%d,\"offset\":%lld,\"bytes\":%d,\"host_first_arrival_elapsed_ns\":%lld,\"host_last_arrival_elapsed_ns\":%lld,\"uvc_pts_present\":%s,\"uvc_pts_ticks\":%u,\"time_source\":\"usb_payload_arrival_not_exposure\"}\n",frames,(long long)offset,used,(long long)first,(long long)last,have_pts?"true":"false",pts);
                    offset+=used;frames++;
                    }
                } else if(used)errors++;
                used=0;broken=0;first=now_ns();have_pts=0;
            }
        }
        for(int j=0;j<PACKETS;j++){u->iso_frame_desc[j].actual_length=0;u->iso_frame_desc[j].status=0;}
        if(ioctl(fd,USBDEVFS_SUBMITURB,u)<0){snprintf(error,sizeof(error),"resubmit_errno_%d",errno);break;}
        slot->submitted=1;
    }
cleanup:
    for(int i=0;i<SLOTS;i++)if(slots[i].submitted)ioctl(fd,USBDEVFS_DISCARDURB,slots[i].urb);
    int64_t cleanup_until=now_ns()+1000000000;
    do {
        cleanup_pending=0;for(int i=0;i<SLOTS;i++)cleanup_pending+=slots[i].submitted;
        if(!cleanup_pending)break;
        struct usbdevfs_urb *u=NULL;
        if(ioctl(fd,USBDEVFS_REAPURBNDELAY,&u)==0){for(int i=0;i<SLOTS;i++)if(slots[i].urb==u)slots[i].submitted=0;}
        else usleep(1000);
    } while(now_ns()<cleanup_until);
    cleanup_pending=0;for(int i=0;i<SLOTS;i++)cleanup_pending+=slots[i].submitted;
    /* If a kernel URB cannot be reaped, retain its storage until process exit. */
    for(int i=0;i<SLOTS;i++)if(!slots[i].submitted){free(slots[i].urb);free(slots[i].buffer);}
    free(frame);if(video){fflush(video);fsync(fileno(video));fclose(video);}if(log){fflush(log);fsync(fileno(log));fclose(log);}
    atomic_store(&source_fps_active,0);
    if(preview_mode)clear_preview();
    char report[512];result=frames>0&&!error[0]&&!cleanup_pending;
    snprintf(report,sizeof(report),"{\"ok\":%s,\"frames\":%d,\"payload_errors\":%d,\"duration_s\":%.6f,\"cleanup_pending\":%d,\"error\":\"%s\"}",result?"true":"false",frames,errors,(now_ns()-start)/1e9,cleanup_pending,error);
    return (*env)->NewStringUTF(env,report);
}

/* D435 RGB uses bulk UVC and uncompressed YUY2. Keep original pixels and the
 * same host-arrival provenance as MJPEG; JPEG conversion is preview-only. */
JNIEXPORT jstring JNICALL Java_dev_warp_capture_WarpUsbCamera_captureBulk(JNIEnv *env,jclass cls,jint fd,jint endpoint,jint payload,jint frame_bytes,jstring path,jint duration_ms) {
    (void)cls;
    int preview_mode=path==NULL,frames=0,cleanup_pending=0;
    int64_t start=now_ns(),offset=0,fps_start=0;int fps_count=0;
    FILE *video=NULL,*log=NULL;Slot slots[SLOTS]={0};
    WarpBulkFrame frame={.expected=frame_bytes,.fid=-1};
    char error[128]="";
    atomic_store(&source_fps,-1);atomic_store(&source_last_frame_ns,0);atomic_store(&source_fps_active,1);
    if(frame_bytes<4||frame_bytes>MAX_FRAME||payload<12||payload>MAX_FRAME+65536){snprintf(error,sizeof(error),"invalid_bulk_dimensions");goto bulk_cleanup;}
    frame.data=malloc((size_t)frame_bytes);
    if(!frame.data){snprintf(error,sizeof(error),"bulk_allocation_failed");goto bulk_cleanup;}
    if(!preview_mode){
        const char *name=(*env)->GetStringUTFChars(env,path,0);
        if(!name){snprintf(error,sizeof(error),"bulk_path_failed");goto bulk_cleanup;}
        char logname[1024];snprintf(logname,sizeof(logname),"%s.jsonl",name);
        video=fopen(name,"wb");log=fopen(logname,"w");(*env)->ReleaseStringUTFChars(env,path,name);
        if(!video||!log){snprintf(error,sizeof(error),"bulk_file_failed");goto bulk_cleanup;}
    }
    int bulk_slots=(4*1024*1024)/payload;
    if(bulk_slots<2)bulk_slots=2;if(bulk_slots>SLOTS)bulk_slots=SLOTS;
    for(int i=0;i<bulk_slots;i++){
        slots[i].urb=calloc(1,sizeof(struct usbdevfs_urb));slots[i].buffer=malloc((size_t)payload);
        if(!slots[i].urb||!slots[i].buffer){snprintf(error,sizeof(error),"bulk_urb_allocation_failed");goto bulk_cleanup;}
        struct usbdevfs_urb *u=slots[i].urb;u->type=USBDEVFS_URB_TYPE_BULK;u->endpoint=endpoint;u->buffer=slots[i].buffer;u->buffer_length=payload;
        if(ioctl(fd,USBDEVFS_SUBMITURB,u)<0){snprintf(error,sizeof(error),"bulk_submit_errno_%d",errno);goto bulk_cleanup;}
        slots[i].submitted=1;
    }
    while(!atomic_load(preview_mode?&preview_stop:&stop_requested)&&now_ns()-start<(int64_t)duration_ms*1000000){
        struct usbdevfs_urb *u=NULL;
        if(ioctl(fd,USBDEVFS_REAPURBNDELAY,&u)<0){
            if(errno==EAGAIN){
                int64_t last=atomic_load(&source_last_frame_ns);
                if(now_ns()-(last?last:start)>5000000000LL){snprintf(error,sizeof(error),"bulk_no_complete_frames_5s");break;}
                usleep(1000);continue;
            }
            snprintf(error,sizeof(error),"bulk_reap_errno_%d",errno);break;
        }
        Slot *slot=NULL;for(int i=0;i<SLOTS;i++)if(slots[i].urb==u){slot=&slots[i];slot->submitted=0;break;}
        if(!slot){snprintf(error,sizeof(error),"bulk_unknown_urb");break;}
        if(u->status){snprintf(error,sizeof(error),"bulk_status_%d",u->status);break;}
        int64_t arrival=now_ns();
        if(u->actual_length>0&&warp_bulk_payload(&frame,slot->buffer,u->actual_length,arrival)){
            atomic_store(&source_last_frame_ns,arrival);
            if(!fps_start){fps_start=arrival;fps_count=0;}
            else if(++fps_count&&arrival-fps_start>=500000000LL){atomic_store(&source_fps,fps_count*1e9/(arrival-fps_start));fps_start=arrival;fps_count=0;}
            if(preview_mode)publish_preview(frame.data,frame_bytes);
            else {
                publish_preview(frame.data,frame_bytes);
                if(fwrite(frame.data,1,(size_t)frame_bytes,video)!=(size_t)frame_bytes){snprintf(error,sizeof(error),"bulk_frame_write_failed");break;}
                if(fprintf(log,"{\"index\":%d,\"offset\":%lld,\"bytes\":%d,\"host_first_arrival_elapsed_ns\":%lld,\"host_last_arrival_elapsed_ns\":%lld,\"uvc_pts_present\":%s,\"uvc_pts_ticks\":%u,\"time_source\":\"usb_payload_arrival_not_exposure\"}\n",frames,(long long)offset,frame_bytes,(long long)frame.first,(long long)arrival,frame.have_pts?"true":"false",frame.pts)<0){snprintf(error,sizeof(error),"bulk_index_write_failed");break;}
                offset+=frame_bytes;
            }
            frames++;
        }
        u->actual_length=0;u->status=0;
        if(ioctl(fd,USBDEVFS_SUBMITURB,u)<0){snprintf(error,sizeof(error),"bulk_resubmit_errno_%d",errno);break;}
        slot->submitted=1;
    }
bulk_cleanup:
    for(int i=0;i<SLOTS;i++)if(slots[i].submitted)ioctl(fd,USBDEVFS_DISCARDURB,slots[i].urb);
    int64_t until=now_ns()+1000000000LL;
    do{
        cleanup_pending=0;for(int i=0;i<SLOTS;i++)cleanup_pending+=slots[i].submitted;if(!cleanup_pending)break;
        struct usbdevfs_urb *u=NULL;
        if(ioctl(fd,USBDEVFS_REAPURBNDELAY,&u)==0){for(int i=0;i<SLOTS;i++)if(slots[i].urb==u)slots[i].submitted=0;}else usleep(1000);
    }while(now_ns()<until);
    cleanup_pending=0;for(int i=0;i<SLOTS;i++){cleanup_pending+=slots[i].submitted;if(!slots[i].submitted){free(slots[i].urb);free(slots[i].buffer);}}
    free(frame.data);
    if(video){if(fflush(video)||fsync(fileno(video)))snprintf(error,sizeof(error),"bulk_video_flush_failed");fclose(video);}
    if(log){if(fflush(log)||fsync(fileno(log)))snprintf(error,sizeof(error),"bulk_index_flush_failed");fclose(log);}
    atomic_store(&source_fps_active,0);if(preview_mode)clear_preview();
    char report[512];
    snprintf(report,sizeof(report),"{\"ok\":%s,\"frames\":%d,\"payload_errors\":%d,\"duration_s\":%.6f,\"cleanup_pending\":%d,\"error\":\"%s\"}",frames>0&&!error[0]&&!cleanup_pending?"true":"false",frames,frame.errors,(now_ns()-start)/1e9,cleanup_pending,error);
    return (*env)->NewStringUTF(env,report);
}
