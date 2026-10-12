# Pinned upstream sources — inspected 2026-10-10

| Stage | Repo | Git commit SHA | Caveat |
|---|---|---|---|
| Quest Unity | [XRT-Client](https://github.com/yunho-c/XRT-Client) | `a82247aaf1100cbdd045034fe34c7d56c106412c` | Body-only streaming reference, not synchronized RGB capture; license not confirmed |
| Quest receiver | [XRT_devices](https://github.com/Euler-Rodrigues-Lab/XRT_devices) | `f364006ea81d6ae4de7c75c5be089266f9773efc` | Device receiver, CSV/BVH; MIT license |
| Quest RGB | [Meta Camera Samples](https://github.com/oculus-samples/Unity-PassthroughCameraApiSamples) | `3baac1a1a53aa00af3f639425914f14238a97f84` | Separate Unity sample, needs integration; verify sample-specific license |
| Retargeting | [rby1_teleop](https://github.com/Euler-Rodrigues-Lab/rby1_teleop) | `6cc5a629cf0b9e907c9edda09cc4c9a730923c6c` | RB-Y1 reference; analytic solver licensed; nested submodules |
| SEW math | [stereo-sew](https://github.com/rpiRobotics/stereo-sew) | `d6917478037b924e1292e65a8f52398da3948851` | Mostly MATLAB reference |
| IK math | [ik-geo](https://github.com/rpiRobotics/ik-geo) | `a3a1675e1f01ad6f8f15f2cc787fa01472082a11` | IK math implementation/reference; sizable |
| Gripper paper toolkit | [Robowheel-Toolkits](https://github.com/zhangyuhong01/Robowheel-Toolkits) | `a8fdd9610944f4f9d0e9fb907c776b9c61b65c74` | Dataset/training/visualization only as confirmed; not full gripper retargeting source; sizable |

SHA values pin the reference state; update deliberately, after reading upstream changes. This list is about what we found, **not** a statement that these packages run together.
