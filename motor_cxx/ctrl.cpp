#include "motor_class.h"
#include "message.h"
#include "motor_math.h"

# define POS_ARRIVE_THRESHOLD 0.2f// 位置环到达阈值

extern osMutexId_t motorsMutexHandle;

void MotorCtrl(){
//    uprintf("%f\r\n",P_Motor[0]->Get_State());
    osMutexAcquire(motorsMutexHandle, osWaitForever);
    for(int i=0;i<4;i++)
        if(If_used(i)) {
            P_Motor[i]->Motor_CtrlMode_Choose();
            P_Motor[i]->Motor_MessageCreate(i);
        }
    Send_Message();
    osMutexRelease(motorsMutexHandle);
}

//Motor基类的控制函数
float Motor::PID_GetOutPut(PID_s *PID, float err){
    float delta_err;
    float result;
    delta_err = err - PID->last_err;

    delta_err *= 0.384f;
    delta_err += PID->last_delta_err * 0.615f; //低通滤波


    PID->last_err = err;

    PID->int_sum += err * PID->int_duty; // 积分量

    __LIMIT(PID->int_sum, PID->int_max); // 限制积分量大小
    PID->last_delta_err = delta_err;
    result = err * PID->Kp + delta_err * PID->Kd + PID->int_sum * PID->Ki;
    __LIMIT(result, PID->ctrl_max);
    return result;
}

void Motor::Vel_Ctrl()
{
    float temp_err = MotorPID.Vel_PID.target - Get_State();
    Final_OutPut.i16[0] = PID_GetOutPut(&MotorPID.Vel_PID,temp_err);
}

void Motor::Pos_Ctrl()
{
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - Get_State();
    Temp_OutPut = PID_GetOutPut(&MotorPID.Pos_PID, LimitPos_f(temp_err,1));

    MotorPID.Vel_PID.target = Temp_OutPut;
    temp_err = MotorPID.Vel_PID.target - MotorState.Vel_Now;
    Final_OutPut.i16[0] =  PID_GetOutPut(&MotorPID.Vel_PID,temp_err);
}

void Motor::Cur_Ctrl()
{
    float temp_err = MotorPID.Cur_PID.target - Get_State();
    Final_OutPut.i16[0] =  PID_GetOutPut(&MotorPID.Cur_PID,temp_err);
}

void Motor::Multi_Pos_Ctrl() {
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - Get_State();
    if(fabs(temp_err) < POS_ARRIVE_THRESHOLD && !PosArrive_Flag) {
        PosArrive_Flag = 1;
        PosArrive_Callback();
    }

    Temp_OutPut = PID_GetOutPut(&MotorPID.Pos_PID,temp_err);

    MotorPID.Vel_PID.target = Temp_OutPut;
    temp_err = MotorPID.Vel_PID.target - MotorState.Vel_Now;
    Final_OutPut.i16[0] =  PID_GetOutPut(&MotorPID.Vel_PID,temp_err);
}

void Motor:: Motor_CtrlMode_Choose(){
    switch (MotorCtrlMode) {
        case VEL_Mode:
            Vel_Ctrl();
            break;
        case POS_Mode:
            Pos_Ctrl();
            break;
        case Multi_POS_Mode:
            Multi_Pos_Ctrl();
            break;
        case CUR_Mode:
            Cur_Ctrl();
            break;
        default:
            break;
    }
}