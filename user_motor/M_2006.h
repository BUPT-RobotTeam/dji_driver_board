#ifndef DJI_DRIVER_BOARD_M_2006_H
#define DJI_DRIVER_BOARD_M_2006_H
#include "../motor_cxx/motor_cxx.h"

const float CURRENT_MAX_2006 = 10000.0f;
const float VEL_KP_2006 = 300.0f;
const float VEL_KI_2006 = 4.0f;
const float VEL_KD_2006 = 0.0f;
const float POS_KP_2006 = 0.5f;
const float POS_KI_2006 = 0.0f;
const float POS_KD_2006 = 0.0f;
const float CUR_KP_2006 = 0.5f;
const float CUR_KI_2006 = 0.0f;
const float CUR_KD_2006 = 0.0f;
const float Reduction_Ratio_2006 = 36.0f / 1.0f;

class Motor_M_2006 : public Motor{
public:
    void Init(int num);
    void Ctrl_Reset();
    void Data_Receive(CAN_ConnMessage msg);
    void Write_CtrlMode(MotorCtrlMode_Def CtrlMode);
    void Motor_MessageCreate(int num);
};

#endif
