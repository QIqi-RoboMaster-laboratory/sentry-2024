/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "referee.h"

extern frame_header_struct_t referee_receive_header;
extern frame_header_struct_t referee_send_header;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP_t;

extern ext_event_data_t field_event;
extern ext_supply_projectile_action_t supply_projectile_action_t;
extern ext_supply_projectile_booking_t supply_projectile_booking_t;
extern ext_referee_warning_t referee_warning_t;


extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_pos_t game_robot_pos_t;
extern ext_buff_musk_t buff_musk_t;
extern aerial_robot_energy_t robot_energy_t;
extern ext_robot_hurt_t robot_hurt_t;
extern ext_shoot_data_t shoot_data_t;
extern ext_bullet_remaining_t bullet_remaining_t;
extern ext_student_interactive_data_t student_interactive_data_t;

extern ext_rfid_status_t  rfid_status_t;
extern ext_robot_command_t robot_command_t;

extern Shoot_Motor_t trigger_motor; 

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                 \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
		
		
		void receive_game_robot_HP_data(CAN_HandleTypeDef *hcan, ext_game_robot_HP_t *game_robot_HP_t)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_data[sizeof(ext_game_robot_HP_t)]; 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data);  

    // 将接收到的数据复制到game_robot_HP_t结构体中
    memcpy(game_robot_HP_t, rx_data, sizeof(ext_game_robot_HP_t));
}
/*
		
		

电机数据, CAN1 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 
		CAN2 0:摩擦轮电机1 3508电机,   1:摩擦轮电机2 3508电机， 3:拨弹轮电机 2006电机*/
static motor_measure_t motor_chassis[7];
static motor_measure_t motor_shoot[2];
static motor_measure_t motor_tri;
		
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  gimbal_callx_message;
static uint8_t              gimbal_can_call_data[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case CAN_YAW_MOTOR_ID:
        {

            get_motor_measure(&motor_chassis[4], rx_data);
            detect_hook(YAW_GIMBAL_MOTOR_TOE);
            break;
        }
        case CAN_PIT_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[5], rx_data);
            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            break;
        }
				case CAN_REF_ID:
        {
         trigger_motor.heat=((uint16_t)(rx_data[0] << 8 | rx_data[1]));  
				  trigger_motor.bulletspeed=(uint16_t)(rx_data[2] << 8 | rx_data[3]);       
//					trigger_motor.vy_set_CANsend=(int16_t)(rx_data[4] << 8 | rx_data[5]);	
//          trigger_motor.chassis_mode_CANsend=(int16_t)(rx_data[6] << 8 | rx_data[7]);   //底盘模式
            break;
				}
	case 	CAN_blue_ROBOT_HP_ID:
{
	game_robot_HP_t.blue_1_robot_HP=((uint16_t)(rx_data[0] << 8 | rx_data[1])); 
game_robot_HP_t.blue_3_robot_HP=((uint16_t)(rx_data[2] << 8 | rx_data[3])); 
	game_robot_HP_t.blue_4_robot_HP=((uint16_t)(rx_data[4] << 8 | rx_data[5])); 
	game_robot_HP_t.blue_base_HP=((uint16_t)(rx_data[6] << 8 | rx_data[7])); 
    break;
}
case 	CAN_red_ROBOT_HP_ID:
{
		game_robot_HP_t.red_1_robot_HP=((uint16_t)(rx_data[0] << 8 | rx_data[1])); 
		game_robot_HP_t.red_3_robot_HP=((uint16_t)(rx_data[2] << 8 | rx_data[3])); 
		game_robot_HP_t.red_4_robot_HP=((uint16_t)(rx_data[4] << 8 | rx_data[5])); 
		game_robot_HP_t.red_base_HP=((uint16_t)(rx_data[6] << 8 | rx_data[7])); 
    break;
}
case CAN_game_state	:
{
		game_state.game_progress=((uint8_t)(rx_data[0])); 
		 
		game_state.stage_remain_time=((uint16_t)(rx_data[4] << 8 | rx_data[5])); 
    break;
}
case 	CAN_sentry_outpot:
{
		game_robot_HP_t.red_7_robot_HP=((uint16_t)(rx_data[0] << 8 | rx_data[1])); 
		game_robot_HP_t.red_outpost_HP=((uint16_t)(rx_data[2] << 8 | rx_data[3])); 
		game_robot_HP_t.blue_7_robot_HP=((uint16_t)(rx_data[4] << 8 | rx_data[5])); 
		game_robot_HP_t.blue_outpost_HP=((uint16_t)(rx_data[6] << 8 | rx_data[7])); 
    break;
}

        default:
        {
            break;
        }
        }
    }
    else if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case CAN_TRIGGER_MOTOR_ID:
        {
            get_motor_measure(&motor_tri, rx_data);
            detect_hook(TRIGGER_MOTOR_TOE);
            break;
        }
        case CAN_3508_S1_ID:
        {
            get_motor_measure(&motor_shoot[rx_header.StdId - CAN_3508_S1_ID], rx_data);
            detect_hook(FRIC_LEFT_MOTOR_TOE);
            break;
        }
        case CAN_3508_S2_ID:
        {
            get_motor_measure(&motor_shoot[rx_header.StdId - CAN_3508_S1_ID], rx_data);
            detect_hook(FRIC_RIGHT_MOTOR_TOE);
            break;
        }
        default:
        {
            break;
        }
        }
    }
}

void CAN_cmd_shoot(int16_t fric1,int16_t fric2 ,int16_t shoot)
{
   uint32_t send_mail_box;
   shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
   shoot_tx_message.IDE = CAN_ID_STD;
   shoot_tx_message.RTR = CAN_RTR_DATA;
   shoot_tx_message.DLC = 0x08;
   shoot_can_send_data[0] = (fric1 >> 8);
   shoot_can_send_data[1] = fric1;
   shoot_can_send_data[2] = (fric2 >> 8);
   shoot_can_send_data[3] = fric2;
   shoot_can_send_data[4] = (shoot >> 8);
   shoot_can_send_data[5] = shoot;
   shoot_can_send_data[6] = (0 >> 8);
   shoot_can_send_data[7] = 0;
   HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (0 >> 8);
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = (0 >> 8);
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief 发送底盘控制命令
 *
 * @param[in]      relative_angle: 云台相对角
 * @param[in]      chassis_vx: 底盘x轴方向速度分量
 * @param[in]      chassis_vy: 底盘y轴方向速度分量
 * @param[in]      chassis_behaviour: 底盘运动模式
 * @retval         none
 */
void CAN_cmd_chassis(int16_t relative_angle, int32_t chassis_vx, int32_t chassis_vy, int16_t chassis_behaviour)
{
    uint32_t send_mail_box;
    gimbal_callx_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
    gimbal_callx_message.IDE = CAN_ID_STD;
    gimbal_callx_message.RTR = CAN_RTR_DATA;
    gimbal_callx_message.DLC = 0x08;
    gimbal_can_call_data[0] = (relative_angle >> 8);
    gimbal_can_call_data[1] = relative_angle;
    gimbal_can_call_data[2] = (chassis_vx >> 8);
    gimbal_can_call_data[3] = chassis_vx;
    gimbal_can_call_data[4] = (chassis_vy >> 8);
    gimbal_can_call_data[5] = chassis_vy;
    gimbal_can_call_data[6] = (chassis_behaviour >> 8);
    gimbal_can_call_data[7] = chassis_behaviour;
    HAL_CAN_AddTxMessage( &hcan1,  &gimbal_callx_message, gimbal_can_call_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

const motor_measure_t *get_shoot_motor_measure_point(uint8_t j)
{
    return &motor_shoot[(j & 0x01)];
}
/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_tri;
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
