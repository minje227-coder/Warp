#include <assert.h>
#include <stdio.h>
#include "../unity/native/WarpUvcBulk.h"
int main(void){
 unsigned char out[8]={0};WarpBulkFrame f={.data=out,.expected=8,.fid=-1};
 unsigned char a[]={2,0,1,2,3,4},b[]={2,2,5,6,7,8};
 assert(!warp_bulk_payload(&f,a,sizeof(a),100));assert(warp_bulk_payload(&f,b,sizeof(b),200));assert(f.first==100);assert(!memcmp(out,"\1\2\3\4\5\6\7\10",8));
 /* Duplicate EOF and truncated startup frames must not become observations. */
 assert(!warp_bulk_payload(&f,b,sizeof(b),201));
 unsigned char partial[]={2,3,1,2};assert(!warp_bulk_payload(&f,partial,sizeof(partial),300));
 a[1]=0;b[1]=2;assert(!warp_bulk_payload(&f,a,sizeof(a),400));assert(warp_bulk_payload(&f,b,sizeof(b),500));
 /* Device error, overflow and incomplete PTS each invalidate the whole frame. */
 unsigned char bad[]={2,0x41,1,2,3,4};b[1]=3;
 assert(!warp_bulk_payload(&f,bad,sizeof(bad),600));assert(!warp_bulk_payload(&f,b,sizeof(b),700));
 unsigned char overflow[]={2,2,1,2,3,4,5,6,7,8,9};assert(!warp_bulk_payload(&f,overflow,sizeof(overflow),800));
 unsigned char malformed[]={2,5,1,2};assert(!warp_bulk_payload(&f,malformed,sizeof(malformed),900));
 unsigned char pts1[]={6,5,11,0,0,0,1,2,3,4},pts2[]={6,7,11,0,0,0,5,6,7,8};
 assert(!warp_bulk_payload(&f,pts1,sizeof(pts1),1000));assert(warp_bulk_payload(&f,pts2,sizeof(pts2),1100));assert(f.pts==11&&f.have_pts);
 pts1[1]=4;pts2[1]=6;pts2[2]=12;
 assert(!warp_bulk_payload(&f,pts1,sizeof(pts1),1200));assert(!warp_bulk_payload(&f,pts2,sizeof(pts2),1300));
 puts("bulk payload integrity cases passed");return 0;
}
