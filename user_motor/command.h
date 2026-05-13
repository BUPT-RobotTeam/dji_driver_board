#ifndef DJI_DRIVER_BOARD_COMMAND_H
#define DJI_DRIVER_BOARD_COMMAND_H


#ifdef __cplusplus
extern "C"
{
#endif
#include "stdint.h"
#include "SimpleLib/utils/utils.h"
#include "oslib_can.h"

typedef enum
{
    RM_3508 = 1,
    M_2006,
    GM_6020,
    CyberGear,
    VESC,
    NONE
} MotorType_Def;

typedef enum
{
    VEL_Mode = 1,   // 速度环
    POS_Mode,       // 单圈位置环
    Multi_POS_Mode, // 多圈位置环
    CUR_Mode,       // 电流环
    POS_CUR_Mode,   // 位置电流环
    BRAKE_Mode,
    N_Mode
} MotorCtrlMode_Def;

typedef struct CanId{
    int send_Id;
    int get_Id;
    uint8_t get_ide;
    uint8_t send_ide;
    uint32_t send_mask;
    uint32_t get_mask;
    int32_t length;
};

typedef struct MotorState_Def{
    float Pos_Now;
    float Vel_Now;
    float Cur_Now;
    float Pos_Begin;
    float Pos_Last;
};

typedef struct MotorPID_Def{
    PID_s Vel_PID;
    PID_s Pos_PID;
    PID_s Cur_PID;
};


typedef enum
{
    MOTORON = 1,
    MOTOROFF,
    VELCFG,
    POSCFG,
    MULTIPOSCFG,
    CURCFG,
    VELCTRL,
    POSCTRL,
    MULTIPOSCTRL,
    CURCTRL,
    POSCURCFG,
    POSCURCTRL,
} CANOPTION;

uint8_t If_used(int num);//判断是否使用

void Motor_Init(int num, MotorType_Def Type);//初始化
void PrintfMotorsInfo();
MotorType_Def get_MotorType(int num);//返回电机类型
MotorCtrlMode_Def get_MotorCtrlMode(int num);//返回控制模式
float get_MotorState(int num);
int get_MaxPosVel(int num);

void write_MotorType(int num, MotorType_Def Type);//改写电机类型
void write_MotorCtrlMode(int num, MotorCtrlMode_Def CtrlMode);//改写控制模式
void write_MotorTarget(int num, float target);//改写目标值
void write_MotorMaxPosVel(int num, int vel);
void write_MotorMaxCur(int num, uint16_t cur);
void motor_On(int num);
void motor_Off(int num);
float get_MotorState_Choose(int num, MotorCtrlMode_Def Mode);

void DriverInit();
void MotorCtrl();
void VelCtrlAll(CAN_Message *msg);
void PosCtrlAll(CAN_Message *msg);
void can2Handle(CAN_Message *msg);
#ifdef __cplusplus
}
#endif

#endif
