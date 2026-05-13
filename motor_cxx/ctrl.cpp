#include "motor_class.h"
#include "message.h"
#include "motor_math.h"
#include "FreeRTOS.h"
#include "task.h"

# define POS_ARRIVE_THRESHOLD 0.5f  // 位置环到达阈值
# define STUCK_THRESHOLD 0.3f       // 堵转速度判定阈值

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
    Final_OutPut.i16[0] = MotorPID.Cur_PID.target + PID_GetOutPut(&MotorPID.Cur_PID,temp_err);
    // Final_OutPut.i16[0] = MotorPID.Cur_PID.target;
}

void Motor::Multi_Pos_Ctrl() {
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - Get_State();
    if(fabs(temp_err) < POS_ARRIVE_THRESHOLD && !PosArrive_Flag) {
        PosArrive_Flag = 1;
        PosArrive_Callback(index);
    }

    Temp_OutPut = PID_GetOutPut(&MotorPID.Pos_PID,temp_err);

    MotorPID.Vel_PID.target = Temp_OutPut;
    temp_err = MotorPID.Vel_PID.target - MotorState.Vel_Now;
    Final_OutPut.i16[0] =  PID_GetOutPut(&MotorPID.Vel_PID,temp_err);
}

/**
 * @brief   位置电流环控制
 * @note    电机将转到目标位置，到位后告知；若中途发生堵转也会告知
 */
void Motor::PosCur_Ctrl(){
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - MotorState.Pos_Now;

    /* 堵转判定 */
    for(;fabs(MotorState.Vel_Now) < STUCK_THRESHOLD;){
        if(PosCurStuck_Flag)break; // 若已堵转则不再触发
        if(PosArrive_Flag)break;   // 若因到位而停止，不触发
        if(xTaskGetTickCount() - TaskStartTick < 500)break;    // 任务开始后0.5秒内速度慢是正常的，不触发
        if(fabs(temp_err) < POS_ARRIVE_THRESHOLD * 20)break;    // 快到位时速度慢是正常的，所以不触发

        /* 堵转判定通过，触发堵转回调函数 */
        PosCurStuck_Flag = 1;
        PosCurStuck_Callback(index);
    }
    if(PosCurStuck_Flag){
        if(MotorState.Vel_Now > STUCK_THRESHOLD * 5 && temp_err > 0 || MotorState.Vel_Now < -STUCK_THRESHOLD * 5 && temp_err < 0 ){
            PosCurContinue_Callback(index);
            PosCurStuck_Flag = 0;
        }
    }

    /* 位置环到位 */
    if(fabs(temp_err) < POS_ARRIVE_THRESHOLD && !PosArrive_Flag) {
        PosArrive_Flag = 1;
        PosArrive_Callback(index);
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
        case POS_CUR_Mode:
            PosCur_Ctrl();
            break;
        default:
            break;
    }
}