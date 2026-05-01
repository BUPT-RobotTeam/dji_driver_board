# DJI-Driver-Board0-Cxx_feature 大疆驱动板

看飞书——电控——电机控制——新大疆驱动板使用指南

2026年4月30日：新增多圈角度环下电机到位回调逻辑  
电机到位后调用`PosArrive_Callback`函数，默认操作为通过串口与CAN告知电机ID：  
```cpp
__weak void Motor::PosArrive_Callback(){
    uprintf("motorId is %d\r\n", index + 1);
    can_msg msg;
    msg.i16[0] = boardState.boardId;
    msg.i16[1] = index + 1;

    OSLIB_CAN_SendMessage(&hcan2, CAN_ID_STD, 0x286, &msg);
}
```
通过重写该函数，可以在电机到位后执行自定义操作  
可以通过修改[ctrl.cpp](motor_cxx/ctrl.cpp)中的`POS_ARRIVE_THRESHOLD`来调整位置环到达判定阈值  
