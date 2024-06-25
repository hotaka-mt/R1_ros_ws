# ロボコン2024Bチーム ロボット1「鶴明」ReadMe
## ロボット構成
### アクチュエータ
* メカナム4輪
* 電磁弁×1(鶴パルト発射用)
* サーボ×1(遠鶴探索機発射用)
* MD×1(遠鶴探索機再装填用)
### センサ
* RealeSense(自己位置)
* OTOS(自己位置)
* リミット×4(自己位置)
* リミット×2(遠隔探索機再装填用)
## ROS<->STM通信フォーマット
### ROS -> STM　　
ROSからSTMに送る通信フォーマット
|バイト|動作|M1設定|M2設定|M3設定|M4設定|ロボット設定|バイト|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|1byte|0xA0|0xA1|0xA2|0xA3|0xA4|0xA5|1byte|
|2byte|x方向速度|M1 Pゲイン|M2 Pゲイン|M3 Pゲイン|M4 Pゲイン|タイヤ直径[mm]|2byte|
|3byte|^|^|^|^|^|^|3byte|
|4byte|^|^|^|^|^|^|4byte|
|5byte|^|^|^|^|^|^|5byte|
|6byte|y方向速度|M1 Iゲイン|M2 Iゲイン|M3 Iゲイン|M4 Iゲイン|旋回直径[mm]|6byte|
|7byte|^|^|^|^|^|^|7byte|
|8byte|^|^|^|^|^|^|8byte|
|9byte|^|^|^|^|^|^|9byte|
|10byte|角速度|M1 Dゲイン|M2 Dゲイン|M3 Dゲイン|M4 Dゲイン|ロリコン分解能|10byte|
|11byte|^|^|^|^|^|^|11byte|
|12byte|^|^|^|^|^|^|12byte|
|13byte|^|^|^|^|^|^|13byte|
|14byte|機構操作||||||14byte|
|15byte|^||||||15byte|

### STM -> ROS
STMからROSに送る通信フォーマット
|バイト|内容|バイト|内容|バイト|内容|
|:--:|:--:|:--:|:--:|:--:|:--:|
|1byte|0xA5|14byte|x加速度[m/s^2]|27byte|^|
|2byte|x位置[mm]|15byte|^|28byte|y速度標準偏差|
|3byte|^|16byte|y加速度[m/s^2]|29byte|^|
|4byte|y位置[mm]|17byte|^|30byte|角速度標準偏差|
|5byte|^|18byte|角加速度[deg/s^2]|31byte|^|
|6byte|角度[deg]|19byte|^|32byte|x加速度標準偏差|
|7byte|^|20byte|x位置標準偏差|33byte|^|
|8byte|x速度[m/s]|21byte|^|34byte|y加速度標準偏差|
|9byte|^|22byte|y位置標準偏差|35byte|^|
|10byte|y速度[m/s]|23byte|^|36byte|角加速度標準偏差|
|11byte|^|24byte|角度標準偏差|37byte|^|
|12byte|角速度[deg/s]|25byte|^|38byte|機構状態|
|13byte|^|26byte|x速度標準偏差|39byte|^|
