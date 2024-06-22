# ロボコン2024Bチーム ロボット1「鶴明」ROSプログラム

## ROS<->STM通信フォーマット
### ROS -> STM　　

|バイト|用途|
|:----:|:----:|
|1byte|0xA0|
|2byte|x方向速度|
|3byte|^|
|4byte|^|
|5byte|^|
|6byte|y方向速度|
|7byte|^|
|8byte|^|
|9byte|^|
|10byte|角速度|
|11byte|^|
|12byte|^|
|13byte|^|
|14byte|ロボット状態|
|15byte|^|

