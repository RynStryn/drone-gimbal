
### TODO



### 安全第一

待机拨到relax,下电,人走拔电池

发疯的电机可以干碎机械限位

### Tips

如果想给达妙(PV/V)加前馈:
- 把目标值订的虚高：例. 鼠标移动的时候多加一部分(临时,不进入累积)

达妙enable态即使不下指令也会转,**必须**disable

//_在这里添加功能 : void Control_Mode_Choise(void)

上debug脚手架前,先备份完好代码

调试结束时,烧录回稳定版本,供操作手使用

刚上电的一段时间,cnt尚未积累到报错,
Peripheral_State.Remote_Control.Link_State==NORMAL,!=DISCONNECT

frc2:默认反着的

yaw电机安装最小可转动:120 or 90 -> 30

usart1 接收机
usart3 陀螺仪
uart5  裁判系统
# done
pitch不太够,前馈/换模式
MIT->纯力矩
