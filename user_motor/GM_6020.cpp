#include "GM_6020.h"
#include "../motor_cxx/motor_math.h"

#define re_ratio Reduction_Ratio_6020

const int RM_RECV_BASE = 0x205; //收报基准ID
const int RM_SEND_BASE = 0x1FF; //发报基准ID

void Motor_GM_6020::Init(int num)
{
    index = num;
    MotorType = GM_6020;
    On = false;
    Ctrl_Reset();
    can_ID.send_ide= CAN_ID_STD;
    can_ID.get_ide = CAN_ID_STD;
    can_ID.send_Id = RM_SEND_BASE;
    can_ID.get_Id = RM_RECV_BASE + num;
    can_ID.length = 8;
    can_ID.send_mask = FULLMASK;
    can_ID.get_mask = FULLMASK;
}

void Motor_GM_6020::Ctrl_Reset()
{
    MotorPID.Vel_PID = (PID_s){VEL_KP_6020,VEL_KI_6020,VEL_KD_6020,1,2000,CURRENT_MAX_6020,0,0,0,0,0};
    MotorPID.Pos_PID = (PID_s){POS_KP_6020,POS_KI_6020,POS_KD_6020,0.0005,10,MotorPID.Pos_PID.ctrl_max,0,0,0,0,0};
    MotorPID.Cur_PID = (PID_s){CUR_KP_6020,CUR_KI_6020,CUR_KD_6020,1,0,CURRENT_MAX_6020,0,0,0,0,0};
    MotorState = MotorState_Def{0,0,0,0,0};
    Final_OutPut.i16[0] = 0;
    PosUsed_Flag = 0;
}

void Motor_GM_6020::Motor_MessageCreate(int num) {
    int msg_num;
    msg_num = get_Id_address(can_ID);
    if(msg_num== 9)
        msg_num = add_Id_address();
    send_Msg[msg_num].msg.ui8[num * 2] = Final_OutPut.ui8[1] & 0xff;
    send_Msg[msg_num].msg.ui8[num * 2 + 1] = Final_OutPut.ui8[0] & 0xff;
    send_Msg[msg_num].can_Id = can_ID.send_Id;
    send_Msg[msg_num].length = can_ID.length;
    send_Msg[msg_num].ide = can_ID.send_ide;
}

void Motor_GM_6020::Write_CtrlMode(MotorCtrlMode_Def CtrlMode) {
    MotorCtrlMode = CtrlMode;
    Ctrl_Reset();
    if(CtrlMode == Multi_POS_Mode)
        PosUsed_Flag = 1;
    if(CtrlMode == POS_Mode)
        MotorState.Pos_Begin = 0;
}

void Motor_GM_6020::Data_Receive(CAN_ConnMessage msg) {
    int32_t temp_content;
    float temp_pos;
    float del_pos;

    temp_content = (int16_t)((msg.payload.ui8[0] << 8) + msg.payload.ui8[1]);
    temp_pos = (float)temp_content * 360.0f / 8196.0f;

    temp_content = (int16_t)((msg.payload.ui8[2] << 8) + msg.payload.ui8[3]);
    MotorState.Vel_Now = (float)temp_content;

    temp_content = (int16_t)((msg.payload.ui8[4] << 8) + msg.payload.ui8[5]);
    MotorState.Cur_Now = (float)temp_content * 3.0f / CURRENT_MAX_6020;


    if(PosUsed_Flag)
    {
        PosUsed_Flag = 0;
        MotorState.Pos_Begin = temp_pos;
        MotorState.Pos_Last = temp_pos;
    }

    del_pos = temp_pos - MotorState.Pos_Last;
    MotorState.Pos_Last = temp_pos;
    MotorState.Pos_Now += LimitPos_f(del_pos,re_ratio);
}