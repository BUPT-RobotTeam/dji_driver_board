#ifndef DJI_DRIVER_BOARD_RM_3508_H
#define DJI_DRIVER_BOARD_RM_3508_H
#include "../motor_cxx/motor_cxx.h"

// # define NORMAL
# define RC2026_ARM
// # define RC2026_LEG

//RM_3608常量
const float CURRENT_MAX_3508 = 16384.0f;
# ifdef NORMAL
const float VEL_KP_3508 = 250.0f;
const float VEL_KI_3508 = 4.0f;
# endif
# ifdef RC2026_ARM
const float VEL_KP_3508 = 350.0f;
const float VEL_KI_3508 = 1.0f;
# endif
# ifdef RC2026_LEG
const float VEL_KP_3508 = 350.0f;
const float VEL_KI_3508 = 1.0f;
# endif
const float VEL_KD_3508 = 0.0f;
const float POS_KP_3508 = 8.0f;
const float POS_KI_3508 = 0.0f;
const float POS_KD_3508 = 10.0f;
const float CUR_KP_3508 = 1.0f;
const float CUR_KI_3508 = 0.0f;
const float CUR_KD_3508 = 0.0f;
const float Reduction_Ratio_3508 = 3509.0f / 187.0f;

class Motor_RM_3508 : public Motor {
public:
    void Init(int num);
    void Ctrl_Reset();
    void Write_CtrlMode(MotorCtrlMode_Def CtrlMode);
    void Data_Receive(CAN_ConnMessage msg);
    void Motor_MessageCreate(int num);
};


#endif //DJI_DRIVER_BOARD_RM_3508_H
