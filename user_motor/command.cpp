#include "command.h"
#include "../motor_cxx/motor_class.h"
#include "motorflash.h"
#include "user_motor.h"
#include "config.h"

void Motor_Init( int num, MotorType_Def Type){
    if(num < 0 || num > 3)
    {
        return;
    }
    switch(Type){
        case RM_3508:
            P_Motor[num] = new Motor_RM_3508;
            P_Motor[num]->Init(num);
            break;
        case M_2006:
            P_Motor[num] = new Motor_M_2006;
            P_Motor[num]->Init(num);
            break;
        case GM_6020:
            P_Motor[num] = new Motor_GM_6020;
            P_Motor[num]->Init(num);
            break;
        case VESC:
            P_Motor[num] = new Motor_VESC;
            P_Motor[num]->Init(num);
            break;
        default:
            break;
    }
}

void DriverInit(){
    motor_ReadParam();
    PrintfMotorsInfo();
}

void PrintfMotorsInfo(){
    for (int i = 0; i < 4; i++)
    {
        uprintf("motorType[%d] = %d\r\n", i, P_Motor[i]->Get_Type());
    }
    for (int i = 0; i < 4; i++)
    {
        uprintf("motorCtrlMode[%d] = %d\r\n", i, P_Motor[i]->Get_CtrlMode());
    }
    for (int i = 0; i < 4; i++)
    {
        uprintf("motorPosMaxVel[%d] = %d\r\n", i, P_Motor[i]->Get_MotorMaxPosVel());
    }
    uprintf("BoardId = %d\r\n",BOARDID);
}

MotorType_Def get_MotorType(int num){
    return P_Motor[num]->Get_Type();
}

MotorCtrlMode_Def get_MotorCtrlMode(int num){
    return P_Motor[num]->Get_CtrlMode();
}

float get_MotorState(int num){
    return P_Motor[num]->Get_State();
}

float get_MotorState_Choose(int num, MotorCtrlMode_Def Mode){
    return P_Motor[num]->Get_State_Choose(Mode);
}


int get_MaxPosVel(int num){
    return P_Motor[num]->Get_MotorMaxPosVel();
}

uint8_t If_used(int num){
    return P_Motor[num]->If_On();
}

void write_MotorType(int num,MotorType_Def Type){
    delete P_Motor[num];
    Motor_Init(num,Type);
}

void write_MotorCtrlMode(int num, MotorCtrlMode_Def CtrlMode){
    P_Motor[num]->Write_CtrlMode(CtrlMode);
}

void write_MotorTarget(int num,float target){
    P_Motor[num]->Motor_SetTarget(target);
}

void motor_On(int num){
    P_Motor[num]->Turn_On();
}

void motor_Off(int num){
    if(If_used(num)) {
        P_Motor[num]->Turn_Off();
    }
}

void VelCtrlAll(CAN_Message *msg)
{
    for (int i = 0; i < 4; i++)
    {
        if(If_used(i) && get_MotorCtrlMode(i) == VEL_Mode)
            write_MotorTarget(i, msg->i16[i]);
    }
}

void PosCtrlAll(CAN_Message *msg)
{
    for (int i = 0; i < 4; i++)
    {
        if(If_used(i) && get_MotorCtrlMode(i) == POS_Mode)
            write_MotorTarget(i, msg->i16[i]);
    }
}

void write_MotorMaxPosVel(int num, int vel){
    P_Motor[num]->Write_MaxPosVel(num,vel);
}

void write_MotorMaxCur(int num, uint16_t cur){
    P_Motor[num]->Write_MaxCur(cur);
}

static void decode(uint32_t packed, int32_t *pos, uint16_t *cur) {
    *cur = packed & 0xFFF;
    uint32_t pos_bits = (packed >> 12) & 0xFFFFF;
    if (pos_bits & 0x80000) {
        *pos = (int32_t)(pos_bits | 0xFFF00000);
    } else {
        *pos = (int32_t)pos_bits;
    }
}

void can2Handle(CAN_Message *msg){
    uint16_t motorId = msg->ui16[0];
    uint16_t option = msg->ui16[1];
    int32_t num = msg->in[1];
    if (motorId < 1 || motorId > 4)
        return;
    motorId -= 1;
    switch (option)
    {
        case MOTORON:
            motor_On(motorId);
            break;
        case MOTOROFF:
            motor_Off(motorId);
            break;
        case VELCFG:
            write_MotorCtrlMode(motorId,VEL_Mode);
            break;
        case POSCFG:
            write_MotorCtrlMode(motorId,POS_Mode);
            break;
        case MULTIPOSCFG:
            write_MotorCtrlMode(motorId,Multi_POS_Mode);
            break;
        case CURCFG:
            write_MotorCtrlMode(motorId,CUR_Mode);
            break;
        case VELCTRL:
            if(If_used(motorId) && get_MotorCtrlMode(motorId) == VEL_Mode)
                write_MotorTarget(motorId, num);
            break;
        case POSCTRL:
            if(If_used(motorId) && get_MotorCtrlMode(motorId) == POS_Mode)
                write_MotorTarget(motorId, num);
            break;
        case MULTIPOSCTRL:
            if(If_used(motorId) && get_MotorCtrlMode(motorId) == Multi_POS_Mode)
                write_MotorTarget(motorId, num);
            break;
        case CURCTRL:
            if(If_used(motorId) && get_MotorCtrlMode(motorId) == CUR_Mode)
                write_MotorTarget(motorId, num);
            break;
        case POSCURCFG:
            write_MotorCtrlMode(motorId, POS_CUR_Mode);
            break;
        case POSCURCTRL:
            if(If_used(motorId) && get_MotorCtrlMode(motorId) == POS_CUR_Mode){
                int32_t pos;
                uint16_t cur;
                decode(num, &pos, &cur);
                write_MotorTarget(motorId, pos);
                write_MotorMaxCur(motorId, cur);
            }
            break;
        default:
            break;
    }
}