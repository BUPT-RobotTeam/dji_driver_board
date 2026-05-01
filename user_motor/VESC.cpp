#include "VESC.h"
#include "../motor_cxx/motor_math.h"

#define re_ratio Reduction_Ratio_VESC

const int RM_RECV_BASE = 0x4E; //收报基准ID
const int RM_SEND_BASE = 0x4E; //发报基准ID

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

uint32_t encode_ctrl_mode(MotorCtrlMode_Def CtrlType){
    switch (CtrlType) {
        case CUR_Mode:
            return CAN_PACKET_SET_CURRENT;
        case BRAKE_Mode:
            return CAN_PACKET_SET_CURRENT_BRAKE;
        case POS_Mode:
            return CAN_PACKET_SET_POS;
        case Multi_POS_Mode:
            return CAN_PACKET_SET_POS;
        case VEL_Mode:
            return CAN_PACKET_SET_RPM;
        default:
            return CAN_PACKET_MAKE_ENUM_32_BITS;
    }
}

void comm_can_set_pos(float pos, uint8_t* buffer, int32_t* send_index);
void comm_can_set_current(float current, uint8_t* buffer, int32_t* send_index);
void comm_can_set_current_brake(float current, uint8_t* buffer, int32_t* send_index);
void comm_can_set_rpm(float rpm, uint8_t* buffer, int32_t* send_index);
//VESC的函数
void Motor_VESC::Init(int num)
{
    index = num;
    MotorType = VESC;
    On = false;
    Ctrl_Reset();
    can_ID.send_ide = CAN_ID_EXT;
    can_ID.get_ide = CAN_ID_EXT;
    can_ID.send_Id = RM_SEND_BASE + num;
    can_ID.get_mask = 0xFF;
    can_ID.send_mask = 0xFF;
    can_ID.get_Id = RM_RECV_BASE + num;
    can_ID.length = 0;
}

void Motor_VESC::Write_CtrlMode(MotorCtrlMode_Def CtrlMode) {
    MotorCtrlMode = CtrlMode;
    Ctrl_Reset();
    if(CtrlMode == Multi_POS_Mode)
        PosUsed_Flag = 1;
    if(CtrlMode == POS_Mode)
        MotorState.Pos_Begin = 0;
}

void Motor_VESC:: Motor_CtrlMode_Choose(){
    switch (MotorCtrlMode) {
        case VEL_Mode:
//            if(MotorPID.Vel_PID.target <= 1)
//                comm_can_set_current_brake(1,Final_OutPut.ui8,&can_ID.length);
//            else
                comm_can_set_rpm(MotorPID.Vel_PID.target,Final_OutPut.ui8, &can_ID.length);
            break;
        case POS_Mode:
            comm_can_set_pos(MotorPID.Pos_PID.target,Final_OutPut.ui8, &can_ID.length);
            break;
        case Multi_POS_Mode:
            Multi_Pos_Ctrl();
            break;
        case CUR_Mode:
            comm_can_set_current(MotorPID.Cur_PID.target,Final_OutPut.ui8, &can_ID.length);
            break;
        default:
            break;
    }
}

void Motor_VESC::Pos_Ctrl(){
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - Get_State();
    Temp_OutPut = PID_GetOutPut(&MotorPID.Pos_PID, LimitPos_f(temp_err,1));

    MotorPID.Vel_PID.target = Temp_OutPut;
    if(MotorPID.Vel_PID.target <= 1)
        comm_can_set_current_brake(1,Final_OutPut.ui8,&can_ID.length);
    else
        comm_can_set_rpm(MotorPID.Vel_PID.target,Final_OutPut.ui8, &can_ID.length);
}

void Motor_VESC::Multi_Pos_Ctrl() {
    float Temp_OutPut;
    float temp_err;

    temp_err = MotorPID.Pos_PID.target - Get_State();
    Temp_OutPut = PID_GetOutPut(&MotorPID.Pos_PID,temp_err);

    MotorPID.Vel_PID.target = Temp_OutPut;
    if(MotorPID.Vel_PID.target <= 1)
        comm_can_set_current_brake(1,Final_OutPut.ui8,&can_ID.length);
    else
        comm_can_set_rpm(MotorPID.Vel_PID.target,Final_OutPut.ui8, &can_ID.length);
}

void Motor_VESC::Ctrl_Reset() {
    MotorPID.Vel_PID = (PID_s){VEL_KP_VESC,VEL_KI_VESC,VEL_KD_VESC,0,2000,CURRENT_MAX_VESC,0,0,0,0,0};
    MotorPID.Pos_PID = (PID_s){POS_KP_VESC,POS_KI_VESC,POS_KD_VESC,0,0,MotorPID.Pos_PID.ctrl_max,0,0,0,0,0};
    MotorPID.Cur_PID = (PID_s){CUR_KP_VESC,CUR_KI_VESC,CUR_KD_VESC,0,0,CURRENT_MAX_VESC,0,0,0,0,0};
    MotorState = MotorState_Def{0,0,0,0,0};
    Final_OutPut.in[0] = 0;
    Final_OutPut.in[1] = 0;
    PosUsed_Flag = 0;
}

void Motor_VESC::Motor_MessageCreate(int num) {
    int msg_num;
    uint32_t send_Id = can_ID.send_Id |(encode_ctrl_mode(MotorCtrlMode) << 8);
    msg_num = get_Id_address(can_ID);
    if(msg_num== 9)
        msg_num = add_Id_address();
    send_Msg[msg_num].can_Id = send_Id;
    send_Msg[msg_num].length = can_ID.length;
    send_Msg[msg_num].ide = can_ID.send_ide;
    send_Msg[msg_num].msg = Final_OutPut;
}

void Motor_VESC::Data_Receive(CAN_ConnMessage msg) {
    int32_t temp_content;
    float temp_pos;
    float del_pos;
    if(msg.id == can_ID.get_Id | (9 << 8)) {

        temp_content = msg.payload.in[0];
        MotorState.Vel_Now = (float) temp_content / re_ratio;

        temp_content = msg.payload.i16[2];
        MotorState.Cur_Now = (float) temp_content / 10.0f;
    }

    if(msg.id == can_ID.get_Id | (9 << 8)) {
        temp_content = msg.payload.ui16[3];
        temp_pos = (float) temp_content / 50.0f / re_ratio;

        if (PosUsed_Flag) {
            PosUsed_Flag = 0;
            MotorState.Pos_Begin = temp_pos;
            MotorState.Pos_Last = temp_pos;
        }

        del_pos = temp_pos - MotorState.Pos_Last;
        MotorState.Pos_Last = temp_pos;
        MotorState.Pos_Now += LimitPos_f(del_pos, re_ratio);
    }
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void comm_can_set_duty(float duty, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), send_index);

}

void comm_can_set_current(float current, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), send_index);
}

void comm_can_set_current_off_delay(float current, float off_delay, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), send_index);
    buffer_append_float16(buffer, off_delay, 1e3, send_index);
}

void comm_can_set_current_brake(float current, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), send_index);
}

void comm_can_set_rpm(float rpm, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)rpm, send_index);
}

void comm_can_set_pos(float pos, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), send_index);
}

void comm_can_set_current_rel(float current_rel, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, send_index);
}

/**
 * Same as above, but also sets the off delay. Note that this command uses 6 bytes now. The off delay is useful to set to keep the current controller running for a while even after setting currents below the minimum current.
 */
void comm_can_set_current_rel_off_delay(float current_rel, float off_delay, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, send_index);
    buffer_append_float16(buffer, off_delay, 1e3, send_index);
}

void comm_can_set_current_brake_rel(float current_rel, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, send_index);
}

void comm_can_set_handbrake(float current, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_float32(buffer, current, 1e3, send_index);
}

void comm_can_set_handbrake_rel(float current_rel, uint8_t* buffer, int32_t* send_index) {
    *send_index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, send_index);
}