# DJI-Driver-Board0-Cxx_feature 大疆驱动板

看飞书——电控——电机控制——新大疆驱动板使用指南

## 开发指南
CAN消息解析：见[message.cpp](motor_cxx\message.cpp)中的`can2ReceiveFunc`函数及[command.cpp](user_motor\command.cpp)中的`can2Handle`函数

## 2026年4月30日：新增多圈角度环下电机到位回调逻辑  
电机到位后调用`PosArrive_Callback`函数，默认操作为通过串口与CAN告知电机ID：  
```cpp
__weak void Motor::PosArrive_Callback(int idx){
    uprintf("%d: arrive\r\n", idx + 1);
    can_msg msg;
    msg.i16[0] = boardState.boardId;
    msg.i16[1] = idx + 1;

    OSLIB_CAN_SendMessage(&hcan2, CAN_ID_STD, 0x286, &msg);
}
```
通过重写该函数，可以在电机到位后执行自定义操作  
可以通过修改[ctrl.cpp](motor_cxx/ctrl.cpp)中的`POS_ARRIVE_THRESHOLD`来调整位置环到达判定阈值    

## 2026年5月11日：新增“位置电流环”
新增位置电流环`POS_CUR_Mode`：
- 本质：在**多圈位置环**的基础上增加电流限制，并增加堵转通知机制
- 控制指令：`poscurctrl <motorid> <pos> <cur>`，其中`cur`表示最大电流
- 不发生堵转时，位置电流环的行为与多圈位置环**完全相同**
- 发生堵转时，电流会保持在最大电流，这将导致电机输出一个恒定大小的力矩。同时，调用`PosCurStuck_Callback`函数，默认操作为通过串口与CAN告知电机ID：
```cpp
__weak void Motor::PosCurStuck_Callback(int idx){
    uprintf("%d: stuck\r\n", idx + 1);
    can_msg msg;
    msg.i16[0] = BOARDID;
    msg.i16[1] = idx + 1;
    OSLIB_CAN_SendMessage(&hcan2, CAN_ID_STD, 0x287, &msg);
}
```
- 堵转结束时调用`PosCurContinue_Callback`函数，默认操作为通过串口与CAN告知电机ID：
```cpp
__weak void Motor::PosCurContinue_Callback(int idx){
    uprintf("%d: continue\r\n", idx + 1);
    can_msg msg;
    msg.i16[0] = BOARDID;
    msg.i16[1] = idx + 1;
    OSLIB_CAN_SendMessage(&hcan2, CAN_ID_STD, 0x288, &msg);
}
```
- 上述两回调接口是弱定义，支持用户自定义实现

这样的功能对于夹爪等机构来说十分适用。  
位置电流环控制方法：
- CAN消息`u16[0]`为电机ID(1~4)，`u16[1]`为12，`u32[1]`前20位为位置，后12位为最大电流(0~4095)
- 串口：`poscurctrl <motorid> <pos> <cur>`

## TODO: 将原先的“电流环”改为“速度电流环”