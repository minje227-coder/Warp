"""Rack feedback and actual main-loop regressions; ROS IO is simulated."""
import math
import unittest

from visual_robot_fixture import Simulation, module


class VisualAlignmentTests(unittest.TestCase):
    def test_actual_main_uses_live_velocity_and_records_reached_pose(self):
        sim=Simulation(module())
        self.assertEqual(sim.run(),0,sim.outputs[-5:])
        self.assertFalse(any(t=='/cmd_pose' for _,t,_ in sim.published))
        moving=[m for _,t,m in sim.published if t=='/cmd_vel' and
                abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z)>0]
        self.assertTrue(moving)
        self.assertTrue(all(math.hypot(m.linear.x,m.linear.y)<=.030001 and
                            abs(m.angular.z)<=math.radians(2)+1e-9 for m in moving))
        error=sim.w.relative(sim.pose,sim.target['base_pose'])
        self.assertLessEqual(math.hypot(*error[:2]),.01)
        self.assertLessEqual(abs(error[2]),math.radians(1))
        self.assertEqual(sim.outputs[-1]['base_goal_odom'],sim.pose)
        self.assertFalse(any(x.get('vision_locked') for x in sim.outputs))
        self.assertEqual(sim.base_want,[0.,0.,0.])

    def test_bias_in_odom_displacement_does_not_define_the_visual_goal(self):
        w=module()
        class BiasedOdom(w.VisualTracker):
            def odometry(self,pose,velocity,stamp,frame,now):
                pose=[5.+1.3*pose[0],-3.+.8*pose[1],pose[2]+.2]
                super().odometry(pose,velocity,stamp,frame,now)
        w.VisualTracker=BiasedOdom
        sim=Simulation(w)
        self.assertEqual(sim.run(),0,sim.outputs[-5:])
        error=sim.w.relative(sim.pose,sim.target['base_pose'])
        self.assertLessEqual(math.hypot(*error[:2]),.01)
        self.assertGreater(sim.outputs[-1]['base_goal_odom'][0],4.)

    def test_new_camera_error_after_start_changes_motion_instead_of_freezing(self):
        class NewGoal(Simulation):
            changed=False
            def tick(self,dt):
                if not self.changed and self.now>4.:
                    self.target['base_pose']=[-.02,.02,-.02];self.changed=True
                super().tick(dt)
        sim=NewGoal(module())
        self.assertEqual(sim.run(),0,sim.outputs[-5:])
        self.assertTrue(sim.changed)
        self.assertLessEqual(math.hypot(*sim.w.relative(sim.pose,sim.target['base_pose'])[:2]),.01)

    def test_camera_loss_jump_odom_loss_and_other_controller_stop_actual_loop(self):
        class CameraFault(Simulation):
            def __init__(self,kind):
                super().__init__(module());self.kind=kind;self.changed=False
            def tick(self,dt):
                if self.tracker is not None and self.now>4. and not self.changed:
                    original=self.tracker.observation
                    def observation(data,now):
                        if self.kind=='loss':self.tracker.heartbeat(now);return
                        if self.kind=='repeat':data=dict(data,frame_id=self.tracker.frame)
                        if self.kind=='jump':data=dict(data,error=[.5,.5,1.])
                        original(data,now)
                    self.tracker.observation=observation;self.changed=True
                super().tick(dt)
        for sim in [CameraFault(x) for x in ('loss','repeat','jump')]+[
                Simulation(module(),fault=x) for x in ('odom_loss','foreign','stop','feedback_loss')]:
            self.assertEqual(sim.run(),1,sim.outputs[-5:])
            self.assertFalse(sim.outputs[-1]['success'])
            self.assertEqual(sim.base_want,[0.,0.,0.])

    def test_delay_and_wheel_lag_converge_then_wait_for_new_stopped_frames(self):
        for delay,lag in ((.25,.15),(.75,.3),(1.,.3)):
            with self.subTest(delay=delay,lag=lag):
                w=module();tr=w.VisualTracker();pose=[0.,0.,0.];goal=[.3,-.2,.25]
                actual=[0.,0.,0.];history=[];packet=None;dt=.05
                for i in range(2200):
                    now=i*dt;history.append(list(pose))
                    if i%10==0:
                        packet=dict(frame_id=str(i),error=w.relative(history[max(0,i-round(delay/dt))],goal),age_s=delay)
                    tr.odometry(pose,actual,int((now+1)*1e9),'odom',now)
                    tr.observation(packet,now)
                    velocity,done,_=tr.step(now,dt)
                    actual=[a+(b-a)*dt/lag for a,b in zip(actual,velocity)]
                    c,s=math.cos(pose[2]),math.sin(pose[2])
                    pose=[pose[0]+(c*actual[0]-s*actual[1])*dt,
                          pose[1]+(s*actual[0]+c*actual[1])*dt,w.wrap(pose[2]+actual[2]*dt)]
                    if done:break
                self.assertTrue(done)
                self.assertLessEqual(math.hypot(*w.relative(pose,goal)[:2]),w.POSITION_TOL)
                self.assertLessEqual(abs(w.relative(pose,goal)[2]),w.YAW_TOL)
                self.assertLess(math.hypot(*actual[:2]),.001)
                self.assertGreaterEqual(now-tr.stopped_at,1.)

    def test_motion_and_reused_frames_cannot_satisfy_stopped_arrival(self):
        w=module();tr=w.VisualTracker()
        for i in range(50):
            now=i*.1
            tr.odometry([0.,0.,0.],[.01,0.,0.],int((now+1)*1e9),'odom',now)
            tr.observation(dict(frame_id=str(i//5),error=[0.,0.,0.],age_s=0),now)
            _,done,_=tr.step(now,.1)
            self.assertFalse(done)
        self.assertIsNone(tr.stopped_at)
        for i in range(10):
            now=5+i*.1
            tr.odometry([0.,0.,0.],[0.,0.,0.],int((now+1)*1e9),'odom',now)
            tr.observation(dict(frame_id='one',error=[0.,0.,0.],age_s=0),now)
            self.assertFalse(tr.step(now,.1)[1])


if __name__=='__main__':unittest.main()
