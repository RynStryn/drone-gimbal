### TODO

灯条亮导致视角模糊,把枪管位置往下调?

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






其实一开始Effort不叫Effort, 命名是Forcer ,但后来一查, occupied!
