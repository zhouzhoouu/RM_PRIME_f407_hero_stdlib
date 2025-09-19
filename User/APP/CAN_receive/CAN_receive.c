#include "CAN_Receive.h"
#include <stdio.h>
#include "stm32f4xx.h"
// #include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

// #include "Detect_Task.h"

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
static void get_power_measure(power_measure_t *ptr,CanRxMsg *rx_message)                                                        
{                                                                                                                                                       \
        (ptr)->input_power = 0.01*((uint16_t)((rx_message)->Data[1] << 8 | (rx_message)->Data[0]));     
        (ptr)->output_power = 0.01*((uint16_t)((rx_message)->Data[3] << 8 | (rx_message)->Data[2]));     
        (ptr)->electric_quantity = ((rx_message)->Data[4]);
	      (ptr)->err = ((rx_message)->Data[5]);
}
	
//统一处理can接收函数
static void CAN_hook(CAN_TypeDef* CANx, CanRxMsg *rx_message);
//声明电机变量
static motor_measure_t motor_trigger, motor_fric[4] ,motor_chassis[4];
static DMMotor_measure_t motor_yaw,motor_pitch;
static motor_measure_t chassis_power_from_supercap;
		
static power_measure_t chassis_super_power;
static CanTxMsg GIMBAL_TxMessage;
//传入UI交互的全局变量
uint8_t cap_electric_quantity=0;
uint8_t shoot_vel_state=0;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(CAN1, &rx1_message);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(CAN2, &rx2_message);
    }
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_solve(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif

//发送摩擦轮控制命令，其中rev为保留字节
void CAN_CMD_Fric(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit( CAN2,  &GIMBAL_TxMessage );//GIMBAL_CAN
#endif

}
//发送超级电容通讯信息，其中rev为保留字节
void CAN_CMD_CHASSIS_Power(uint16_t limit_power,uint16_t rev1,uint16_t rev2,uint16_t rev3)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x124;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = limit_power >> 8;
    TxMessage.Data[1] = limit_power;
	  TxMessage.Data[2] = rev1 >> 8;
    TxMessage.Data[3] = rev1;
    TxMessage.Data[4] = rev2 >> 8;
    TxMessage.Data[5] = rev2;
    TxMessage.Data[6] = rev3 >> 8;
    TxMessage.Data[7] = rev3;
    CAN_Transmit(CAN1, &TxMessage);
}
//发送拨弹轮控制命令，其中rev为保留字节
void CAN_CMD_Stir(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit(CAN1,  &GIMBAL_TxMessage );//GIMBAL_CAN
#endif

}
void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//发送底盘电机控制命令
/**
  * @brief          发送转矩电流到底盘电机
  * @author         pxx
  * @param[in]      motor1：0x201(M3507)   [-16384:16384]
  * @param[in]      motor2：0x202(M3507)   [-16384:16384]
  * @param[in]      motor3：0x203(M3507)   [-16384:16384]
  * @param[in]      motor4：0x204(M3507)   [-16384:16384]
  * @retval         返回空
  */
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    //CAN 1
    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const DMMotor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const DMMotor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pitch;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回fric电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Fric_Motor_Measure_Point(uint8_t i)
{
    return &motor_fric[(i & 0x03)];
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//获取底盘功率和能量
void get_chassis_power(fp32 *power,fp32 *out)
{
    *power = chassis_super_power.input_power;
		*out = chassis_super_power.output_power;
}

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CAN_TypeDef* CANx, CanRxMsg *rx_message)
{
    //printf("ID : %d\r\n",rx_message->StdId);

    if(CANx==CAN1) {
        //底盘can数据处理
        switch (rx_message->StdId) {

            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                //处理电机ID号
                i = rx_message->StdId - CAN_3508_M1_ID;
                //处理电机数据宏函数
                get_motor_measure(&motor_chassis[i], rx_message);
                //记录时间
                //DetectHook(ChassisMotor1TOE + i);
                break;
            }
            case CAN_TRIGGER_MOTOR_ID:
            {
                //处理电机数据宏函数
                get_motor_measure(&motor_trigger, rx_message);
                //记录时间
                //DetectHook(TriggerMotorTOE);
                break;
            }
            case CAN_YAW_MOTOR_ID:
            {
                //处理电机数据宏函数
                get_DMMotor_measure(&motor_yaw, rx_message);
                motor_yaw.pos = uint_to_float(motor_yaw.pos_int, -12.56637f, 12.56637f, 16);
                motor_yaw.vel = uint_to_float(motor_yaw.vel_int, V_MIN, V_MAX, 12);
                motor_yaw.tor = uint_to_float(motor_yaw.tor_int, T_MIN, T_MAX, 12);
                //printf("YAW\r\n");
                //记录时间
                //DetectHook(YawGimbalMotorTOE);
                break;
            }
            case CAN_CHASSIS_POWER_ID:
            {
                get_power_measure(&chassis_super_power,rx_message);
//				cap_electric_quantity=(int16_t)chassis_super_power.electric_quantity<<8;
                //printf("test: %d",chassis_super_power.electric_quantity);
                //printf("ERR: %d\n",chassis_super_power.err);
                //printf("super_power %f %f %d \n",chassis_super_power.input_power,chassis_super_power.output_power,chassis_super_power.electric_quantity);
                break;
            }
            default:
                break;
        }
    }
    else if(CANx==CAN2) {
        switch (rx_message->StdId)
        {
            case CAN_Fric1_MOTOR_ID:
            case CAN_Fric2_MOTOR_ID:
            case CAN_Fric3_MOTOR_ID:
            case CAN_Fric4_MOTOR_ID:
            {
                static uint8_t i = 0;
                //处理电机ID号
                i = rx_message->StdId - CAN_Fric1_MOTOR_ID;
                get_motor_measure(&motor_fric[i], rx_message);

                if(motor_fric[0].speed_rpm>10){
                    shoot_vel_state=1;
                }
                else
                    shoot_vel_state=0;

                break;
            }
            case CAN_PIT_MOTOR_ID:
            {
                //处理电机数据宏函数
                //printf("PIT\r\n");
                get_DMMotor_measure(&motor_pitch, rx_message);
                motor_pitch.pos = uint_to_float(motor_pitch.pos_int, P_MIN, P_MAX, 16);
                motor_pitch.vel = uint_to_float(motor_pitch.vel_int, V_MIN, V_MAX, 12);
                motor_pitch.tor = uint_to_float(motor_pitch.tor_int, T_MIN, T_MAX, 12);
                //DetectHook(PitchGimbalMotorTOE);
                break;
            }
            default:
                break;
        }
    }



}

// const DMMotor_measure_t *get_DM_Motor_Measure_Point(void)
// {
//     //return &DMMotor;
// }
