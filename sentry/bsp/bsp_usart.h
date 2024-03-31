#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "referee.h"
#define VISION_RX_LEN_2 34u
#define VISION_RX_LEN 17u
typedef struct {
	uint8_t header;
//	uint8_t scan;
//	uint8_t spin;
	float  vx;
	float  vy;
	float  ang_z;
//	uint8_t robot_id;
//	uint16_t shooter1_limit;
//	uint16_t chassis_power_limit;
//	uint16_t shooter1_heatlimit;
//	float chassis_power;
//	uint16_t buffer_energry;
//	uint8_t shooter_speed;
	float rubbish;
	
} vision_rxfifo_t;

typedef struct __attribute__((packed))
{
			uint8_t header;
			uint8_t pose;

uint8_t game_progress;
	uint8_t cur_hp;
	uint8_t remain_time;
//	double distance;
//uint16_t stage_remain_time;
//uint16_t red_1_robot_HP;
//uint16_t red_3_robot_HP;
//uint16_t red_4_robot_HP;
//uint16_t red_7_robot_HP;
//uint16_t blue_1_robot_HP;
//uint16_t blue_3_robot_HP;
//uint16_t blue_4_robot_HP;
////uint32_t event_data;
//uint16_t blue_7_robot_HP;

} radar_txfifo_t;


extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);



extern vision_rxfifo_t vision_rxfifo;
extern uint8_t vision_rx_buf[2][VISION_RX_LEN_2];
extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
extern vision_rxfifo_t *get_vision_fifo(void);

//radar
extern radar_txfifo_t  radar_txfifo;
extern void send_data_to_upper_computer(uint8_t *send_buffer, radar_txfifo_t *radar_txfifo); 


#endif