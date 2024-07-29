/**
 * @file    Communication.h
 * @brief   通讯功能模块
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-15
 * @version v1.0
 */

#ifndef __FML_COMMUNICATION_H
#define __FML_COMMUNICATION_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "User_Uart.h"

#include "Crc.h"
#include "Chassis.h"

/* 宏定义 ----------------------------------------------------------------------------------------------------------------*/

#define __packed __attribute__((packed))

/* 结构体定义 ----------------------------------------------------------------------------------------------------------*/
/**
 * @brief   鲁班猫上位机Rx数据结构体
 */
struct __packed Struct_RxData_LuBanCat
{
    Enum_ChassisState Chassis_State;    /*!< 底盘设定状态 */
    float Chassis_Vel_X;                /*!< 底盘X轴速度 (m/s) */
    float Chassis_Vel_Y;                /*!< 底盘Y轴速度 (m/s) */
    float Chassis_Omega;                /*!< 底盘旋转角速度 (rad/s) */
};

/**
 * @brief   鲁班猫上位机Tx数据结构体
 */
struct __packed Struct_TxData_LuBanCat
{
    float Chassis_Motor_Omega[4];       /*!< 底盘电机实际转速 */
};

/* 枚举类型定义 --------------------------------------------------------------------------------------------------------*/

/* 类定义 --------------------------------------------------------------------------------------------------------------*/
/**
 * @brief   自定义串口功能模块类（定长数据包）
 */
class Class_CustomCOM
{
public:
    /* 变量 */
    Struct_UART_Manage_Object * UART;           /*!< 串口处理结构体指针 */

    /* 函数 */
    Class_CustomCOM(void (* __COM_TxCallback)(uint8_t Pack_Type_Tx, void * Data_Parameter, void * Data_Tx) = nullptr,
                    void (* __COM_RxCallback)(void * Data_Rx, uint8_t Pack_Type_Rx) = nullptr,
                    void (* __COM_OffCallback)() = nullptr,
                    Struct_UART_Manage_Object * __UART = nullptr);
    void Init(uint8_t __Packet_Length_Tx = MAX_Len_Tx, uint8_t __Packet_Length_Rx = MAX_Len_Rx, uint32_t __Pack_Head = 0x20250301);
    void AliveCheck(uint16_t Period);
    void DataSend(uint8_t Pack_Type_Tx, void * Data_Parameter = nullptr);
    void DataProcess(uint8_t Pack_Size);
protected:
    /* 常量 */
    uint32_t Pack_Head;                         /*!< 包头 (4byte) */
    uint8_t Packet_Length_Tx;                   /*!< Tx数据包长度 */
    uint8_t Packet_Length_Rx;                   /*!< Rx数据包长度 */
    void (* COM_TxCallback)                     /*!< Tx回调函数指针 */
         (uint8_t Pack_Type_Tx, void * Data_Parameter, void * Data_Tx);
    void (* COM_RxCallback)                     /*!< Rx回调函数指针 */
         (void * Data_Rx, uint8_t Pack_Type_Rx);
    void (* COM_OffCallback)                    /*!< 离线回调函数指针 */
         ();
    constexpr static uint8_t MAX_Len_Tx         /*!< Tx缓冲区最大长度 */
                             = 128U;
    constexpr static uint8_t MAX_Len_Rx         /*!< Rx缓冲区最大长度 */
                             = 128U;

    /* 读写变量 */
    uint8_t Buffer_Tx[MAX_Len_Tx];              /*!< Tx缓冲区 */
    uint8_t Buffer_Rx[MAX_Len_Rx];              /*!< Rx缓冲区 */
    void * Data_Tx;                             /*!< 发送的数据指针 */
    void * Data_Rx;                             /*!< 解析到的数据指针 */
    uint32_t Error_Number = 0U;                 /*!< 接收数据校验错误次数 */

    /* 内部变量 */
    volatile uint8_t Flag = 0;                  /*!< 存活检测标志 */
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_CustomCOM COM_LuBanCat;

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
void COM_TxCallback_LuBanCat(uint8_t Pack_Type_Tx, void * Data_Parameter, void * Data_Tx);
void COM_RxCallback_LuBanCat(void * Data_Rx, uint8_t Pack_Type_Rx);
void COM_OffCallback_LuBanCat();

/* 接口函数定义 --------------------------------------------------------------------------------------------------------*/

#endif  /* FML_Communication.h */
