package dev.warp.capture;
import java.nio.file.*;
import java.io.IOException;
public class WarpD435ModeTest {
 static void check(boolean ok){if(!ok)throw new AssertionError();}
 public static void main(String[] args)throws Exception{
  byte[] raw=Files.readAllBytes(Paths.get(args[0]));
  WarpUsbCamera.RgbMode mode=WarpUsbCamera.d435RgbMode(raw);
  check(mode.iface==4&&mode.width==640&&mode.height==480&&mode.interval==333333);
  check(mode.format==1&&mode.frame==2);
  // A USB3 descriptor exposing 1080p at 30 must override the USB2 VGA mode.
  byte[] high=raw.clone();int highIface=-1;
  for(int o=0;o+2<=high.length;){int n=high[o]&255;if((high[o+1]&255)==4)highIface=high[o+2]&255;
   if(highIface==4&&(high[o+1]&255)==0x24&&(high[o+2]&255)==17&&(high[o+3]&255)==4){
    for(int at:new int[]{17,26})for(int j=0;j<4;j++)high[o+at+j]=(byte)(333333>>(8*j));
   }o+=n;
  }
  mode=WarpUsbCamera.d435RgbMode(high);check(mode.iface==4&&mode.width==1920&&mode.height==1080&&mode.interval==333333);
  // Exclude the largest RGB mode; the smaller advertised RGB mode must win,
  // never a same-sized depth or IR mode in another interface.
  byte[] small=raw.clone();int iface=-1;
  for(int o=0;o+2<=small.length;){int n=small[o]&255;if((small[o+1]&255)==4)iface=small[o+2]&255;
   if(iface==4&&(small[o+1]&255)==0x24&&(small[o+2]&255)==17&&(small[o+3]&255)==2)small[o+5]|=1;
   o+=n;
  }
  mode=WarpUsbCamera.d435RgbMode(small);check(mode.iface==4&&mode.width==424&&mode.height==240&&mode.interval==333333);
  byte[] missing=raw.clone();
  for(int o=0;o+2<=missing.length;){int n=missing[o]&255;if((missing[o+1]&255)==0x24&&(missing[o+2]&255)==16&&missing[o+5]=='Y'&&missing[o+6]=='U'&&missing[o+7]=='Y'&&missing[o+8]=='2')missing[o+5]='Z';o+=n;}
  try{WarpUsbCamera.d435RgbMode(missing);throw new AssertionError("Selected non-RGB mode");}catch(IOException expected){}
  byte[] broken=raw.clone();broken[18]=0;
  try{WarpUsbCamera.d435RgbMode(broken);throw new AssertionError("Accepted malformed descriptor");}catch(IOException expected){}
  System.out.println("D435 captured descriptor selection cases passed");
 }
}
