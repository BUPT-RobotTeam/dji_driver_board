#include "motor_class.h"
#include "message.h"
#include "../user_motor/user_motor.h"
#include "motor_math.h"
# include "config.h"

void* Motor::operator new(size_t)
{
    void* ret = NULL;
    for (int i = 0; i < COUNT; i++)		// 找一个空位放对象
    {
        if ( !c_map[i] )
        {
            c_map[i] = 1;
            ret = c_buffer + i * Max_Size;
            break;
        }
    }
    return ret;
}

void Motor::operator delete(void* p)
{
    if (p != NULL)
    {
        char* mem = reinterpret_cast<char*>(p);
        int index = (mem - c_buffer) / Max_Size;	// 第几个位置
        int flag = (mem - c_buffer) % Max_Size;		// 查看给出的位置是否正确
        if (flag == 0 && 0 <= index && index < COUNT)
        {
            c_map[index] = 0;							// 标志置0，表示空间可用
        }
    }
}

Motor* P_Motor[Motor::COUNT];
//基类的函数
Motor::Motor(){
    On = 0;
    MotorType = NONE;
}

void Motor::Motor_SetTarget(float target) {
    switch (MotorCtrlMode) {
        case VEL_Mode:
            MotorPID.Vel_PID.target = target;
        case POS_Mode:
            MotorPID.Pos_PID.target = LimitPos_f(target,1);
        case Multi_POS_Mode:
            MotorPID.Pos_PID.target = target;
            PosArrive_Flag = 0; // 设置了新目标，故清除位置环到达标志位
        case CUR_Mode:
            MotorPID.Cur_PID.target = target;
    }
}

float Motor::Get_State()
{
    switch (MotorCtrlMode) {
        case VEL_Mode:
            return MotorState.Vel_Now;
        case POS_Mode:
            return LimitPos_f(MotorState.Pos_Now,1);
        case Multi_POS_Mode:
            return MotorState.Pos_Now;
        case CUR_Mode:
            return MotorState.Cur_Now;
        default:
            return 0;
    }
}

float Motor::Get_State_Choose(MotorCtrlMode_Def Mode)
{
    switch (Mode) {
        case VEL_Mode:
            return MotorState.Vel_Now;
        case POS_Mode:
            return LimitPos_f(MotorState.Pos_Now,1);
        case Multi_POS_Mode:
            return MotorState.Pos_Now;
        case CUR_Mode:
            return MotorState.Cur_Now;
        default:
            return 0;
    }
}

Motor::~Motor(){
    MotorType = NONE;
}

MotorType_Def Motor::Get_Type(){
    return MotorType;
}

void Motor::Write_Type(MotorType_Def Type){
    MotorType = Type;
}

CanId Motor::Get_CanId(){
    return can_ID;
}

MotorCtrlMode_Def Motor::Get_CtrlMode(){
    return MotorCtrlMode;
}

void Motor::Turn_On() {
    On = 1;
    Ctrl_Reset();
}

void Motor::Turn_Off() {
    On = 0;
    delete_Id_address(can_ID);
}

uint8_t Motor::If_On() {
    return On;
}

void Motor::Write_CtrlMode(MotorCtrlMode_Def CtrlMode){
    MotorCtrlMode = CtrlMode;
    Ctrl_Reset();
    if(CtrlMode == Multi_POS_Mode ||CtrlMode == POS_Mode)
        PosUsed_Flag = 1;
}

void Motor::Write_MaxPosVel(int num, int vel) {
    MotorPID.Pos_PID.ctrl_max = vel;
}

int Motor::Get_MotorMaxPosVel(){
    return MotorPID.Pos_PID.ctrl_max;
}

__weak void Motor::PosArrive_Callback(){
    uprintf("motorId is %d\r\n", index + 1);
    can_msg msg;
    msg.i16[0] = BOARDID;
    msg.i16[1] = index + 1;

    OSLIB_CAN_SendMessage(&hcan2, CAN_ID_STD, 0x286, &msg);
}