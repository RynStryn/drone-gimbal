### 安全第一

~~你知道吗~~:发疯的电机可以干碎机械限位

### Tips

6020,2006init只需要init pid,并入在对应的pid init

如果想给大喵(PV/V)加前馈:
- 把目标值订的虚高：例. 鼠标移动的时候多加一部分(临时,不进入累积)

大喵enable态即使不下指令也会转,**必须**disable

1 rad/s = 9.549296585513721 rpm

//_在这里添加功能 : void Control_Mode_Choise(void)

上debug脚手架前,先备份完好代码

调试结束时,烧录回稳定版本,供操作手使用

刚上电的一段时间,cnt尚未积累到报错,
Peripheral_State.Remote_Control.Link_State==NORMAL,!=DISCONNECT

空中自稳:需要跟随机身(陀螺仪):pitch不太够,前馈或换模式

shoot<->can1
gimbal<->can2
yaw电机安装最小可转动:120 or 90 -> 30




0 uint8_t 1-数据包标签:0x91
1 uint8_t 1-保留
2 uint8_t 1-保留
3 int8_t 1 °C模块陀螺仪平均温度
4 float 4 Pa气压(部分型号支持)
8 uint32_t 4 ms节点本地时间戳信息，从系统开机开始累加，每毫秒增加1
12 float 12 G(1G = 1重力 加速度)经过出厂校准后的加速度,顺序为：XYZ
24 float 12 deg/s经过出厂校准后的角速度,顺序为： XYZ
36 float 12 uT磁强度,顺序为： XYZ
48 float 12 deg
节点欧拉角 顺序为：横滚角(Roll, -180°~180°)，俯仰角(Pitch，-90°~90°)，航向角(Yaw, -180°~180°)
60 float 16-节点四元数集合,顺序为WXYZ

其实一开始Effort不叫Effort, 命名是Forcer ,但后来一查, occupied!
