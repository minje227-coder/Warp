"""Only tag backend: remote apt apriltag_ros with the live RWH settings."""
import base64,json,os,select,shlex,subprocess
import cv2
class Detector:
 def __init__(self):
  host=os.environ.get('RWH_APRILTAG_HOST','leju_kuavo@192.168.0.21')
  self.proc=subprocess.Popen(['ssh','-S',os.environ.get('RWH_APRILTAG_CONTROL','/tmp/kuavo-ssh-leju_kuavo-192.168.0.21'),'-o','BatchMode=yes','-o','ConnectTimeout=5',host,
   'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://192.168.26.1:11311 ROS_IP=192.168.26.12; python3 /home/leju_kuavo/kuavo_ros_rwhlab/test/oxeunn/tools/apt_offline_ros_worker.py'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
 def detect(self,gray,*,camera_params):
  fx,fy,cx,cy=camera_params;ok,png=cv2.imencode('.png',gray)
  if not ok:raise ValueError('image encode failed')
  self.proc.stdin.write(json.dumps(dict(png=base64.b64encode(png).decode(),K=[fx,0,cx,0,fy,cy,0,0,1]))+'\n');self.proc.stdin.flush()
  if not select.select([self.proc.stdout],[],[],30)[0]:raise RuntimeError('apt apriltag_ros backend timeout')
  r=json.loads(self.proc.stdout.readline())
  if not r['ok']:raise RuntimeError(r['error'])
  return r['tags']
 def close(self):
  if self.proc.poll() is None:
   self.proc.stdin.close()
   try:self.proc.wait(timeout=6)
   except subprocess.TimeoutExpired:self.proc.terminate()
 def __del__(self):
  if hasattr(self,'proc'):self.close()
