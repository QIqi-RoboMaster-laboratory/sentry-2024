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
extern UART_HandleTypeDef huart6;
extern ext_game_robot_HP_t game_robot_HP_t;
extern ext_game_state_t game_state;


vision_rxfifo_t *vision_rx;
// ��RTOS�߳��з����������ݸ��״���λ��
void radar_usart_task(void const *pvParameters)
{
			vision_init();
   
	while (1)
    {
			vision_rx=get_vision_fifo();
        // ����һ��radar_txfifo_t�ṹ�����
     extern   radar_txfifo_t radar_txfifo;
        
        // �����״����ݵĸ����ֶ�ֵ
        radar_txfifo.header = 0x5A;
		
			radar_txfifo.stage_remain_time+=1;
			
			radar_txfifo.game_progress = 4;//(game_state.game_progress) & 0x0F;
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
//      if(radar_txfifo.game_progress ==4)
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
        osDelay(100); // ������ʱ100ms
    }
}