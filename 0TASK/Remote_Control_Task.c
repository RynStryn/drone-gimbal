#include "main.h"
/**************************提示***************************/

/*******************************************************/

/**********************结构体定义**********************/
Remote         Remote_data;       //遥控数据结构体（包含键鼠）
Control_Mode_e Control_Mode;      //本次控制方式
Control_Mode_e Control_Mode_Last; //上次控制方式
/*****************************************************/


/********************************
*@Brief： 遥控器数据处理函数
*@Cal：   内部或外部
*@param:  遥控的结构体
*@Note:   放在对应串口空闲中断中确保信息的实时性
*@RetVal: 无
********************************/


void Get_Remote_Data(uint8_t *RemoteData)
{
    if (RemoteData == NULL)
    {
        return;
    }
    Remote_data.Remote_Control.ch0 =
            ((int16_t) RemoteData[0] | ((int16_t) RemoteData[1] << 8)) & 0x07FF;
    Remote_data.Remote_Control.ch1 =
            (((int16_t) RemoteData[1] >> 3) | ((int16_t) RemoteData[2] << 5)) &
            0x07FF;
    Remote_data.Remote_Control.ch2 =
    (((int16_t) RemoteData[2] >> 6) | ((int16_t) RemoteData[3] << 2) | (
         (int16_t) RemoteData[4] << 10)) & 0x07FF;
    Remote_data.Remote_Control.ch3 =
            (((int16_t) RemoteData[4] >> 1) | ((int16_t) RemoteData[5] << 7)) &
            0x07FF;
    Remote_data.Remote_Control.ch4 =
            ((int16_t) RemoteData[16] >> 0 | ((int16_t) RemoteData[17] << 8)) &
            0x07FF;
    Remote_data.Remote_Control.s1 = ((RemoteData[5] >> 4) & 0x000C) >> 2;
    Remote_data.Remote_Control.s2 = ((RemoteData[5] >> 4) & 0x0003); //模式切换

    Remote_data.Remote_Mouse.x =
            ((int16_t) RemoteData[6]) | ((int16_t) RemoteData[7] << 8);
    Remote_data.Remote_Mouse.y =
            ((int16_t) RemoteData[8]) | ((int16_t) RemoteData[9] << 8);
    Remote_data.Remote_Mouse.z =
            ((int16_t) RemoteData[10]) | ((int16_t) RemoteData[11] << 8);
    Remote_data.Remote_Mouse.press_l = RemoteData[12];
    Remote_data.Remote_Mouse.press_r = RemoteData[13];
    Remote_data.Remote_Key.v         = ((int16_t) RemoteData[14]) | (
                                           (int16_t) RemoteData[15] << 8);
}

/********************************
*@Brief： 遥控拨杆与拨轮动作判断函数
*@Cal：   内部或外部
*@param: 	遥控的结构体
*@Note:   记录遥控拨杆和拨轮的动作，用于各模式的切换
*@RetVal: 无
********************************/
void Remote_Switch_Action_Get(Remote_t *Remote)
{
    int static Diff_s1 = 0; //读s1的瞬时变化值，用于判断s1的动作。


    Diff_s1 = Remote->s1 - Remote->s1_last;

    if (Diff_s1 == 0)
    {
        Remote->s1_Action = Keep;
    } else if (Diff_s1 == 1)
    {
        Remote->s1_Action = L_2_M;
    } else if (Diff_s1 == -2)
    {
        Remote->s1_Action = M_2_H;
    } else if (Diff_s1 == 2)
    {
        Remote->s1_Action = H_2_M;
    } else if (Diff_s1 == -1)
    {
        Remote->s1_Action = M_2_L;
    }
    if (Remote->ch4 <= 364 + CH4_Dead_Zone)
    {
        Remote->Trigger_H = 1;
        Remote->Trigger_L = 0;
        Remote->Trigger_H_Cnt++;
        Remote->Trigger_L_Cnt = 0;
        if (Remote->Trigger_H_Cnt > KEY_SHORT_LONG_BOUNDARY)
        {
            Remote->Trigger_H_Hold = 1;
        }
    } else if (Remote->ch4 >= 1684 - CH4_Dead_Zone)
    {
        Remote->Trigger_H = 0;
        Remote->Trigger_L = 1;
        Remote->Trigger_L_Cnt++;
        Remote->Trigger_H_Cnt = 0;
        if (Remote->Trigger_L_Cnt > KEY_SHORT_LONG_BOUNDARY)
        {
            Remote->Trigger_L_Hold = 1;
        }
    } else
    {
        Remote->Trigger_H      = 0;
        Remote->Trigger_L      = 0;
        Remote->Trigger_H_Cnt  = 0;
        Remote->Trigger_L_Cnt  = 0;
        Remote->Trigger_H_Hold = 0;
        Remote->Trigger_L_Hold = 0;
    }

    Remote->s1_last = Remote->s1;
    Remote->s2_last = Remote->s2;
}


/********************************
*@Brief：  	按键检测 按下等待一会，怕连续识别
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void Key_Single_Click_Cooling(Key_Mouse_Action_t *_Key_Mouse_Action)
//判断是否按下，防止毛刺信号
{
    if (!_Key_Mouse_Action->Flag)
    {
        if (_Key_Mouse_Action->Cnt < KEY_LOW_LIMIT)
            _Key_Mouse_Action->Cnt = _Key_Mouse_Action->Cnt + 1;
        if (_Key_Mouse_Action->Cnt >= KEY_LOW_LIMIT)
            _Key_Mouse_Action->Flag = 1;
    }
}


/********************************
*@Brief：  	按键检测flag清除函数
*@Cal：     内部和外部
*@param:    无
*@Note:     无
*@RetVal:   无
*******************************/
void Key_Single_Click_CD(Key_Mouse_Action_t *_Key_Mouse_Action) //
{
    _Key_Mouse_Action->Flag = 0;
    _Key_Mouse_Action->Cnt  = 0;
}


/**********************
*@Brief:单个按键状态检测函数
*@Call:内部或外部
*@Param:State：按键的状态
*@Note:无
*@RetVal:无
**********************/
void Single_Key_Mouse_Action_Detection(Key_Mouse_Action_t *_Key_Mouse_Action,
                                       int                 _State)
{
    if (_State)
    {
        //按键按下超过设定阈值拉高标志位
        Key_Single_Click_Cooling(_Key_Mouse_Action);
    } else
    {
        //未按下时清除计数器和标志位
        Key_Single_Click_CD(_Key_Mouse_Action);
    }

    if (_Key_Mouse_Action->Flag)
    {
        //相应标志位被拉高时，按键状态为TAP_AND_HOLD，并且保持计数器累加
        _Key_Mouse_Action->Key_Mouse_Action = TAP_AND_HOLD;
        _Key_Mouse_Action->Hold_Cnt++;
        //当计数器累加到一定值时，直接判定为长按
        if (_Key_Mouse_Action->Hold_Cnt > KEY_SHORT_LONG_BOUNDARY) //长按
            _Key_Mouse_Action->Key_Mouse_Action = CLICK_LONG_TIME;
    } else
    {
        if (_Key_Mouse_Action->Flag_Last)
        {
            //当相应标志位出现下降沿时，若不处在长按状态下，则判定为短按
            if (_Key_Mouse_Action->Key_Mouse_Action != CLICK_LONG_TIME)
                _Key_Mouse_Action->Key_Mouse_Action = CLICK_FAST;
            //计数器清零
            _Key_Mouse_Action->Hold_Cnt = 0;
        } else
        {
            //相应标志位该时刻状态与上一时刻状态都为0时判断按键为无输入状态，、
            _Key_Mouse_Action->Key_Mouse_Action = ZERO_INPUT;
        }
    }

    //记录上一时刻的值用于判断
    _Key_Mouse_Action->Flag_Last
            = _Key_Mouse_Action->Flag;
}


void Key_Mouse_Action_Get(Mouse_t *_Mouse, Key_t *_Key)
{
    //鼠标左键状态检测
    Single_Key_Mouse_Action_Detection(&_Mouse->Press_L_Action, _Mouse->press_l);
    //鼠标右键状态检测
    Single_Key_Mouse_Action_Detection(&_Mouse->Press_R_Action, _Mouse->press_r);
    //W
    Single_Key_Mouse_Action_Detection(&_Key->Key_W_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_W);
    //S
    Single_Key_Mouse_Action_Detection(&_Key->Key_S_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_S);
    //A
    Single_Key_Mouse_Action_Detection(&_Key->Key_A_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_A);
    //D
    Single_Key_Mouse_Action_Detection(&_Key->Key_D_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_D);
    //Q
    Single_Key_Mouse_Action_Detection(&_Key->Key_Q_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_Q);
    //E
    Single_Key_Mouse_Action_Detection(&_Key->Key_E_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_E);
    //R
    Single_Key_Mouse_Action_Detection(&_Key->Key_R_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_R);
    //F
    Single_Key_Mouse_Action_Detection(&_Key->Key_F_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_F);
    //G
    Single_Key_Mouse_Action_Detection(&_Key->Key_G_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_G);
    //Z
    Single_Key_Mouse_Action_Detection(&_Key->Key_Z_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_Z);
    //X
    Single_Key_Mouse_Action_Detection(&_Key->Key_X_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_X);
    //C
    Single_Key_Mouse_Action_Detection(&_Key->Key_C_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_C);
    //V
    Single_Key_Mouse_Action_Detection(&_Key->Key_V_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_V);
    //B
    Single_Key_Mouse_Action_Detection(&_Key->Key_B_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_B);
    //SHIFT
    Single_Key_Mouse_Action_Detection(&_Key->Key_SHIFT_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_SHIFT);
    //CTRL
    Single_Key_Mouse_Action_Detection(&_Key->Key_CTRL_Action,
                                      _Key->v & KEY_PRESSED_OFFSET_CTRL);
}

void Remote_Control(void)
{
    Key_Mouse_Action_Get(&Remote_data.Remote_Mouse, &Remote_data.Remote_Key);
    //获取键鼠按键状态
    Remote_Switch_Action_Get(&Remote_data.Remote_Control); //获取遥控器拨杆、按键等状态
    Control_Mode_Choise();                                 //控制模式选择以及对应按键的使用
}

void Control_Mode_Choise(void)
{
    //通过右上拨杆来选择控制模式
    if (1)
    {
        if (Peripheral_State.Remote_Control.Link_State)
        {
            Control_Mode_Last = Control_Mode;
            switch (Remote_data.Remote_Control.s2)
            {
                case 1:
                    Control_Mode = REMOTE;
                    break;
                case 3:
                    Control_Mode = KEY_MOUSE;
                    break;
                case 2:
                    Control_Mode = Null;
                    break;
                default:
                    Control_Mode = Null;
                    break;
            }
        } else
        {
            Control_Mode = Null;
        }
    }
    //对应控制模式下的任务
    if (1)
    {
        switch (Control_Mode)
        {
            case Null:
                if (Gimbal.Mode != GIMBAL_INIT)
                    Gimbal.Mode = GIMBAL_RELAX;

                Shooter.Poke_State     = POKE_OFF;
                Shooter.State          = STOP;
                Shooter.Friction_State = Friction_OFF;
                break;

            /*****遥控器模式*****/
            case REMOTE:

                /*云台只需切换模式*/
                if (Gimbal.Mode != GIMBAL_INIT)
                    Gimbal.Mode = GIMBAL_RC;

                if (1)
                {
                    //发射机构判断拨轮状态，选择开启/关闭摩擦轮
                    static volatile uint8_t Hold_Flag = 0;
                    if (Remote_data.Remote_Control.Trigger_H == 0)
                    {
                        Hold_Flag = 0;
                    }
                    if (Remote_data.Remote_Control.Trigger_H_Hold == 1 &&
                        Shooter.Friction_State == Friction_OFF && Hold_Flag ==
                        0) //向上保持开摩擦轮
                    {
                        Shooter.Friction_State = Friction_ON;
                        Hold_Flag              = 1;
                    } else if (Remote_data.Remote_Control.Trigger_H == 1 &&
                               Shooter.Friction_State == Friction_ON &&
                               Hold_Flag == 0) //向上短按关摩擦轮
                    {
                        Shooter.Friction_State = Friction_OFF;
                        Hold_Flag              = 1;
                    }
                    //通过左上拨杆选择开火模式，通过拨轮判断开关摩擦轮/是否开火
                    switch (Remote_data.Remote_Control.s1)
                    {
                        case 1:
                            Shooter.Mode = FullAuto_FIRE; //拨杆在上，连发开火
                            if (Shooter.Mode != Shooter.Mode_Last)
                            {
                                Shooter.State = START;
                            }
                            if (Remote_data.Remote_Control.Trigger_L == 1)
                            //向下拨动拨轮以开火
                            {
                                Shooter.Poke_State = POKE_ON;
                            } else
                            {
                                Shooter.Poke_State = POKE_OFF;
                            }
                            break;
                        case 2:
                            Shooter.State = STOP;
                            Shooter.Friction_State = Friction_OFF; //拨杆在下关闭发射机构
                            break;
                        case 3:
                            Shooter.Mode = SINGLE_SHOT; //拨杆在中，单发开火
                            if (Shooter.Mode != Shooter.Mode_Last)
                            {
                                Shooter.State = START;
                            }
                            if (Remote_data.Remote_Control.Trigger_L == 1)
                            //拨轮控制开火
                            {
                                Shooter.Poke_State = POKE_ON;
                            } else
                            {
                                Shooter.Poke_State = POKE_OFF;
                            }
                            break;
                        default:
                            Shooter.State = STOP;
                            Shooter.Friction_State = Friction_OFF;
                            break;
                    }
                }
                break;

            /*****键鼠模式*****/
            case KEY_MOUSE:
                if (Gimbal.Mode != GIMBAL_INIT)
                    Gimbal.Mode = GIMBAL_KeyMouse_NORMAL;
                //短按C开摩擦轮
                if (Remote_data.Remote_Key.Key_C_Action.Key_Mouse_Action ==
                    CLICK_FAST && Shooter.Friction_State == Friction_OFF)
                {
                    Shooter.Friction_State = Friction_ON;
                }
                //长按C关摩擦轮
                else if (Remote_data.Remote_Key.Key_C_Action.Key_Mouse_Action ==
                         CLICK_LONG_TIME && Shooter.Friction_State ==
                         Friction_ON)
                {
                    Shooter.Friction_State = Friction_OFF;
                }
                //短按B切换开火模式（单发、连发）
                if (Remote_data.Remote_Key.Key_B_Action.Key_Mouse_Action ==
                    CLICK_FAST && Shooter.Mode != FullAuto_FIRE)
                {
                    Shooter.Mode = FullAuto_FIRE;
                } else if (Remote_data.Remote_Key.Key_B_Action.Key_Mouse_Action
                           == CLICK_FAST && Shooter.Mode != SINGLE_SHOT)
                {
                    Shooter.Mode = SINGLE_SHOT;
                }
                //短按G切换连发模式下射频（5，10，15，20）
                if (Remote_data.Remote_Key.Key_G_Action.Key_Mouse_Action ==
                    CLICK_FAST)
                {
                    Shooter.Freq += 5;
                    if (Shooter.Freq > 20)
                    {
                        Shooter.Freq = 5;
                    }
                }
                //按住鼠标左键开火
                if (Remote_data.Remote_Mouse.Press_L_Action.Key_Mouse_Action ==
                    TAP_AND_HOLD || Remote_data.Remote_Mouse.Press_L_Action.
                    Key_Mouse_Action == CLICK_LONG_TIME)
                {
                    Shooter.Poke_State = POKE_ON;
                } else
                {
                    Shooter.Poke_State = POKE_OFF;
                }
                //长按X进行软件复位
                if (Remote_data.Remote_Key.Key_X_Action.Key_Mouse_Action ==
                    CLICK_LONG_TIME)
                {
                    SoftReset();
                }
                //长按R进行拨盘复位
                if (Remote_data.Remote_Key.Key_R_Action.Key_Mouse_Action ==
                    CLICK_LONG_TIME && Shooter.Init_Flag != 1)
                {
                    Shooter.Mode      = SHOOTER_INIT;
                    Shooter.Init_Flag = 1;
                } else if (Remote_data.Remote_Key.Key_R_Action.Key_Mouse_Action
                           == ZERO_INPUT)
                {
                    Shooter.Init_Flag = 0;
                }
                break;
            default:
                Gimbal.Mode = GIMBAL_RELAX;
                Shooter.Poke_State     = POKE_OFF;
                Shooter.State          = STOP;
                Shooter.Friction_State = Friction_OFF;
                break;
        }
    }
}
