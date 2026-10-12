# Choices to make before a runnable collector

1. **Quest hardware:** exact model (Quest 3 vs 3S vs Pro vs 2), firmware, hand-tracking/body-tracking availability.
2. **Recorder host:** workstation OS and networking (same Wi-Fi / secure LAN), Quest APK vs Unity development environment.
3. **Observation dataset:** body pose only for first pilot, or body+hand+egocentric RGB from start? If RGB, left/right stream, codec, format, frame timing, camera calibration.
4. **Object ground truth:** fixed known box pose (pilot), AprilTag before grasp, external RGB-D, or tracked whole episode. Define hand–object contact phase; occlusion fallback is only a hypothesis until tested.
5. **Robot URDF/MJCF and kinematic axes:** arms 7+7, waist X/Y/pitch/yaw, wheeled base DOFs and motion constraints, gripper TCP and closing axis, joint limits, collision meshes.
6. **Task:** which box sizes/weight, which rack slots (x/y/z/depth), allowable arm choice(s), gripper grasp face or flap, target reach→close→lift criteria.
7. **Policy interface:** fixed frequency, observations, q vs delta-q, gripper open/close, whether base moves while gripper is in contact.
8. **Simulator:** MuJoCo/Isaac Sim, robot model fidelity, collision and contact setup.

Only the choices above that block the **capture milestone** need answers now: (1), (2), and (3); robot model/gripper can be deferred until retargeting.
