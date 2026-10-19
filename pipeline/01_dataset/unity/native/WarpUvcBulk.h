#ifndef WARP_UVC_BULK_H
#define WARP_UVC_BULK_H
#include <stdint.h>
#include <string.h>

/* One complete bulk UVC payload per call. Never infer a frame from a byte count
 * alone: require EOF, exact YUY2 dimensions and an intact FID/header sequence. */
typedef struct {
    unsigned char *data;
    int expected,used,fid,broken,ended,errors,have_pts;
    int64_t first;
    uint32_t pts;
} WarpBulkFrame;
static int warp_bulk_payload(WarpBulkFrame *f,const unsigned char *p,int n,int64_t arrival) {
    if(n<2||p[0]<2||p[0]>n||((p[1]&4)&&p[0]<6)||((p[1]&8)&&p[0]<(p[1]&4?12:8))){f->broken=1;f->errors++;return 0;}
    int h=p[0],flags=p[1],fid=flags&1;
    if(fid!=f->fid){
        if(f->used&&!f->ended)f->errors++;
        f->used=0;f->broken=0;f->ended=0;f->have_pts=0;f->first=arrival;f->fid=fid;
    }
    if(f->ended)return 0;
    if(flags&0x40){f->broken=1;f->errors++;}
    if(flags&4){
        uint32_t pts=(uint32_t)p[2]|((uint32_t)p[3]<<8)|((uint32_t)p[4]<<16)|((uint32_t)p[5]<<24);
        if(f->have_pts&&f->pts!=pts){f->broken=1;f->errors++;}
        f->pts=pts;f->have_pts=1;
    }
    int bytes=n-h;
    if(bytes>f->expected-f->used){f->broken=1;f->errors++;}
    else if(!f->broken&&bytes){memcpy(f->data+f->used,p+h,(size_t)bytes);f->used+=bytes;}
    if(!(flags&2))return 0;
    f->ended=1;
    if(!f->broken&&f->used==f->expected)return 1;
    f->errors++;return 0;
}
#endif
