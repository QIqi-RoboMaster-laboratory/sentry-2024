/**
 * @file radar_usart.c
 * @author liuxinyang 
 * @brief �ڱ��״����񣬷����������ݸ��״���λ��
 *				������λ�������ߣ��轫�������ݴ��䣬������������������Ӱ����������
 *				�ڴ˵���һ���̡߳�
 * @version 0.1
 * @date 2024-03-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "bsp_usart.h"
#include "referee.h"
#include "radar_usart.h"
#include "string.h"
extern UART_HandleTypeDef huart6;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_shoot_data_t shoot_data_t;



extern frame_header_struct_t referee_receive_header;
extern frame_header_struct_t referee_send_header;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP_t;

extern ext_event_data_t field_event;
extern ext_supply_projectile_action_t supply_projectile_action_t;
//extern ext_supply_projectile_booking_t supply_projectile_booking_t;
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
extern CAN_HandleTypeDef hcan1;
int error_usarttime=0;

vision_rxfifo_t *vision_rx;
// ��RTOS�߳��з����������ݸ��״���λ��
void radar_usart_task(void const *pvParameters)
{
			
   
	while (1)
    {
			error_usarttime++;
			if(error_usarttime>3)
			{
		memset(&vision_rxfifo, 0, sizeof(vision_rxfifo));
		
			}
			vision_rx=get_vision_fifo();
		
robot_state.robot_id=vision_rx->robot_id;	
robot_state.shooter_id1_17mm_cooling_limit=vision_rx ->shooter1_barrel_heat_limit;
robot_state.chassis_power_limit=vision_rx->chassis_power_limit;
power_heat_data_t.chassis_power=vision_rx->chassis_power;
power_heat_data_t.shooter_id1_17mm_cooling_heat=vision_rx->shooter1_17mm_1_barrel_heat;
power_heat_data_t.chassis_power_buffer=vision_rx->buffer_energry;
shoot_data_t.bullet_speed=vision_rx->initial_speed;
        // ����һ��radar_txfifo_t�ṹ�����
     extern   radar_txfifo_t radar_txfifo;
        
        // �����״����ݵĸ����ֶ�ֵ
        radar_txfifo.header = 0x5A;
		
//			radar_txfifo.stage_remain_time+=1;

			radar_txfifo.game_progress = (game_state.game_progress) & 0x0F;
			radar_txfifo.cur_hp=(uint8_t)((robot_state.remain_HP/100)+1);
			radar_txfifo .remain_time=(uint8_t)(game_state.stage_remain_time/2);
//			radar_txfifo.stage_remain_time=135;
//	radar_txfifo.stage_remain_time = game_state.stage_remain_time;
//        radar_txfifo.blue_1_robot_HP = game_robot_HP_t.blue_1_robot_HP;
//        radar_txfifo.blue_3_robot_HP = game_robot_HP_t.blue_3_robot_HP;
//        radar_txfifo.blue_4_robot_HP = game_robot_HP_t.blue_4_robot_HP;
//			 radar_txfifo.blue_7_robot_HP = 0;
//			
//        radar_txfifo.red_1_robot_HP = game_robot_HP_t.red_1_robot_HP;
//        radar_txfifo.red_3_robot_HP = game_robot_HP_t.red_3_robot_HP;
//        radar_txfifo.red_4_robot_HP = game_robot_HP_t.red_4_robot_HP;
//        radar_txfifo.red_7_robot_HP = 600;// game_robot_HP_t.red_7_robot_HP;
//      if(robot_state.remain_HP /)
//			{
//					radar_txfifo.pose=2;
//			
//			}
//			if(game_state.stage_remain_time<=270)
//			 {
//						radar_txfifo.pose=2;
//			 }

        
        // ����һ�����ͻ�����
        uint8_t send_buffer[200];
        
        // ���״����ݸ��Ƶ����ͻ�������
        memset(send_buffer, 0, 200);
        memcpy(send_buffer, &radar_txfifo, sizeof(radar_txfifo_t));
        
        // ͨ��UART�������ݸ���λ��
        HAL_UART_Transmit(&huart6, send_buffer, 200, HAL_MAX_DELAY);
        
        // �߳���ʱ��ȴ���һ������
        osDelay(1); // ������ʱ100ms
    }
}