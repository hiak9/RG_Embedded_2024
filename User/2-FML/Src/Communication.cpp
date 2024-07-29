/**
 * @file    Communication.cpp
 * @brief   通讯功能模块
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-15
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Communication.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/
Class_CustomCOM COM_LuBanCat(COM_TxCallback_LuBanCat, COM_RxCallback_LuBanCat, COM_OffCallback_LuBanCat, &UART3_Manage_Object);

/* 函数定义 ------------------------------------------------------------------------------------------------------------*/
/************************************************************************************************************************
 * @brief       串口Tx回调函数
 * 
 * @param[in]   Pack_Type_Tx    发送包类型
 * @param[in]   Data_Parameter  发送数据可能需要的参数指针
 * @param[out]  Data_Tx         发送包数据
 ***********************************************************************************************************************/
void COM_TxCallback_LuBanCat(uint8_t Pack_Type_Tx, void * Data_Parameter, void * Data_Tx)
{
    if (Pack_Type_Tx == 0x00)
    {
        /* 当前包为上行包0 */
        auto Data = (Struct_TxData_LuBanCat *)Data_Tx;

        for (uint8_t i = 0; i < 4; i++)
        {
            Data->Chassis_Motor_Omega[i] = Committee_Chariot.Motor_Wheel[i].Get_TargetOmega();
        }
    }
}

/************************************************************************************************************************
 * @brief   串口Rx回调函数
 *
 * @param   Data_Rx         解析到的数据结构体指针
 * @param   Pack_Type_Rx    解析到的包类型
 ***********************************************************************************************************************/
void COM_RxCallback_LuBanCat(void * Data_Rx, uint8_t Pack_Type_Rx)
{
    if (Pack_Type_Rx == 0xF0)
    {
        /* 当前包为下行包0 */
        auto Data = (Struct_RxData_LuBanCat *)Data_Rx;

        if (Data->Chassis_State == Chassis_Run)
        {
            /* 底盘运动设置 */
            Committee_Chariot.Set_Motion(Data->Chassis_Vel_X, Data->Chassis_Vel_Y, Data->Chassis_Omega);
        }
        else if (Data->Chassis_State == Chassis_Suspend || Data->Chassis_State == Chassis_Brake)
        {
            /* 底盘停止设置 */
            Committee_Chariot.Set_Stop(Data->Chassis_State);
        }
    }
}

/************************************************************************************************************************
 * @brief       串口离线回调函数
 ***********************************************************************************************************************/
void COM_OffCallback_LuBanCat()
{
    /* 底盘悬空 */
    Committee_Chariot.Set_Stop(Chassis_Suspend);
}

/************************************************************************************************************************
 * @brief   自定义串口类构造函数
 *
 * @param   __COM_TxCallback    Tx回调函数指针
 * @param   __COM_RxCallback    Rx回调函数指针
 * @param   __UART              串口外设句柄
 ***********************************************************************************************************************/
Class_CustomCOM::Class_CustomCOM(void (* __COM_TxCallback)(uint8_t Pack_Type_Tx, void * Data_Parameter, void * Data_Tx),
                                 void (* __COM_RxCallback)(void * Data_Rx, uint8_t Pack_Type_Rx),
                                 void (* __COM_OffCallback)(),
                                 Struct_UART_Manage_Object * __UART)
{
    this->COM_TxCallback = __COM_TxCallback;
    this->COM_RxCallback = __COM_RxCallback;
    this->COM_OffCallback = __COM_OffCallback;
    this->UART = __UART;
}

/************************************************************************************************************************
 * @brief   自定义串口类初始化函数
 * 
 * @param   __Packet_Length_Tx  Tx数据包长度（最大不超过Tx缓冲区长度）
 * @param   __Packet_Length_Rx  Rx数据包长度（最大不超过Rx缓冲区长度）
 * @param   __Pack_Head         包头 (4byte)
 ***********************************************************************************************************************/
void Class_CustomCOM::Init(uint8_t __Packet_Length_Tx, uint8_t __Packet_Length_Rx, uint32_t __Pack_Head)
{
    /* 参数赋值 */
    if (__Packet_Length_Tx > this->MAX_Len_Tx)
    {
        __Packet_Length_Tx = this->MAX_Len_Tx;
    }
    if (__Packet_Length_Rx > this->MAX_Len_Rx)
    {
        __Packet_Length_Rx = this->MAX_Len_Rx;
    }
    this->Packet_Length_Tx = __Packet_Length_Tx;
    this->Packet_Length_Rx = __Packet_Length_Rx;
    this->Pack_Head = __Pack_Head;

    /* Tx包头填充 */
    memcpy(this->Buffer_Tx, &this->Pack_Head, 4);
    
    /* UART用户层初始化 */
    UART_Init(this->UART, this->Packet_Length_Rx);

    /* 开启一次 DMA-IDLE 串口数据接收 */
    UART_ReceiveToIdle_DMA(this->UART);
}

/************************************************************************************************************************
 * @brief   自定义串口存活检测函数
 * 
 * @param   Period  存活检测周期
 ***********************************************************************************************************************/
void Class_CustomCOM::AliveCheck(uint16_t Period)
{
    static uint16_t count;

    if (count < Period - 1)
    {
        count += 1;
    }
    else
    {
		count = 0;
		
        /* 判断是否存活 */
        if (this->Flag == 0)
        {
            /* 串口离线，执行离线回调函数 */
            this->COM_OffCallback();
        }

        /* 存活检测标志复位 */
        this->Flag = 0U;
    }
}

/************************************************************************************************************************
 * @brief   自定义串口数据发送函数
 * 
 * @param   Pack_Type_Tx    发送包类型
 * @param   Data_Parameter  发送数据可能需要的参数指针
 ***********************************************************************************************************************/
void Class_CustomCOM::DataSend(uint8_t Pack_Type_Tx, void * Data_Parameter)
{
    /* 包类型填充 */
    this->Buffer_Tx[4] = Pack_Type_Tx;

    /* Tx包数据指针 */
    this->Data_Tx = &this->Buffer_Tx[5];

    /* Tx回调函数调用（根据包类型填充包数据） */
    this->COM_TxCallback(Pack_Type_Tx, Data_Parameter, this->Data_Tx);

    /* CRC校验位填充 */
    this->Buffer_Tx[this->Packet_Length_Tx - 1] = Calculate_CRC8(this->Buffer_Tx, this->Packet_Length_Tx - 1);

    /* 数据发送 */
    UART_Send(this->UART, this->Buffer_Tx, this->Packet_Length_Tx);
}

/************************************************************************************************************************
 * @brief   自定义串口数据处理函数
 * 
 * @param   Pack_Size   接收到的包大小
 ***********************************************************************************************************************/
void Class_CustomCOM::DataProcess(uint8_t Pack_Size)
{
    /* 存活检测标志置位 */
    this->Flag = 1U;

    /* 长度校验 */
    if (Pack_Size != this->Packet_Length_Rx)
    {
        this->Error_Number += 1U;
        return;
    }

    /* 数据拷贝 */
    memcpy(this->Buffer_Rx, this->UART->Rx_Buffer, this->Packet_Length_Rx);

    /* 包头校验 */
    for (uint8_t i = 0; i < 4; i++)
    {
        if (this->Buffer_Rx[i] != ((uint8_t *)&this->Pack_Head)[i])
        {
            this->Error_Number += 1U;
            return;
        }
    }

    /* CRC校验 */
    if (this->Buffer_Rx[this->Packet_Length_Rx - 1] != Calculate_CRC8(this->Buffer_Rx, this->Packet_Length_Rx - 1))
    {
        this->Error_Number += 1U;
        return;
    }

    /* 数据解析 */
    this->Data_Rx = (void *)&this->Buffer_Rx[5];

    /* Rx回调函数调用 */
    this->COM_RxCallback(this->Data_Rx, this->Buffer_Rx[4]);
}
