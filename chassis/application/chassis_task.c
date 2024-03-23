/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "gimbal_task.h"
#include "referee.h"
fp32 chassis_yaw;
fp32 sin_yaw;
fp32 gaibian = 0.0f;
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
 extern  float firstyaw;
      extern  float yaw;
			float Xy=0.0f; 
			float Xx=0.0f; 
			float distance=0.0f;
        fp32 relative_angle1 = 0.25f;

#define abs(x) ((x) > 0 ? (x) : (-x))
#define Motor_Ecd_to_rad 0.00076708402f
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }




/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
/**
  * @brief          //�ͽ���λ�Ƕȴ���ȡ�ӻ�
  */


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

/*----------------------------------�ڲ�����---------------------------*/
int x_flag = 0; // ������־λ�����ڽǶȴ���
int X_FLAG = 0;
fp32 K;
fp32 speed_set_x,speed_set_y=0; //�����˶�ʱ�ٶ�����
chassis_move_t chassis_move;       // �����˶�����

int	linkState_2=0;
int	linkState_1=0;  //�ж�˫��ͨ���Ƿ�����

fp32 kx = 1.f, ky = 1.f, kw = 1.f; // �ٶ�ת���ļ���ϵ��
/*----------------------------------�ⲿ����---------------------------*/


/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    speed_set_x = 2.1f;
    speed_set_y = 1.82f;
    K = 105.f;
    chassis_move.power_control.POWER_MAX = 140;
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //���̳�ʼ��
    chassis_init(&chassis_move);
    // make sure all chassis motor is online,
    // �жϵ��̵���Ƿ�����
//    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//    {
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }
    while (1)
    {
			
			
				CAN_shoot_data(power_heat_data_t.shooter_id1_17mm_cooling_heat, shoot_data_t.bullet_speed*1000.,game_state.game_progress, robot_state.robot_id*10,robot_state.current_HP);
				CAN_game_state(game_state.stage_remain_time);
					
					
				
        //set chassis control mode
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
//CAN_blue_robot_hp(game_robot_HP_t.blue_1_robot_HP, game_robot_HP_t.blue_3_robot_HP, game_robot_HP_t.blue_4_robot_HP, game_robot_HP_t.blue_base_HP);
//					CAN_red_robot_hp(game_robot_HP_t.red_1_robot_HP, game_robot_HP_t.red_3_robot_HP, game_robot_HP_t.red_4_robot_HP, game_robot_HP_t.red_base_HP);
        //whenmode changes, some data save
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
//
//					CAN_sentry_outpot_state(game_robot_HP_t.red_7_robot_HP,game_robot_HP_t.red_outpost_HP,game_robot_HP_t.blue_7_robot_HP,game_robot_HP_t.blue_outpost_HP);
//CAN_shoot_data(power_heat_data_t.shooter_id1_17mm_cooling_heat, shoot_data_t.bullet_speed*1000.,game_state.game_progress, 0,robot_state.current_HP);
//CAN_shoot_data(power_heat_data_t.shooter_id1_17mm_cooling_heat, shoot_data_t.bullet_speed*1000.,game_state.game_progress, 0,robot_state.current_HP);

        //chassis data update
        //�������ݸ���
  			chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //���̿���PID����
        chassis_control_loop(&chassis_move);
		
        
       if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
      {
            // ȷ������һ���������
            linkState_1++;
						linkState_2++;
		if(linkState_1>50||linkState_2>50)  
		{
			
//			
				CAN_cmd_chassis(0,0,0,0);
			
		
		}		
      
                
   
	//���Ϳ��Ƶ���
       CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                       chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);




     }
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    //chassis motor speed PID
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD};
    //chassis angle PID
    //���̽Ƕ�pidֵ
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning�� chassis mode is raw 
    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_point = get_INS_point();
    
    //��ȡ����״ָ̬��
    chassis_move_init->chassis_auto.ext_game_robot_state_point = get_game_robot_status_point();
    //��ȡ�˺�����ָ��
    chassis_move_init->chassis_auto.ext_robot_hurt_point = get_robot_hurt_point();
   
//    //��ȡ�Զ��ƶ�����ָ��
//    chassis_move_init->chassis_auto.chassis_auto_move = get_auto_move_point();
    //��ȡ����״ָ̬��
    chassis_move_init->chassis_auto.field_event_point = get_field_event_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 4; i++)
    {
       chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
       PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
			
    }
    //initialize angle PID
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
		PID_init(&chassis_move_init->buffer_pid,        PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT,		  M3505_MOTOR_POWER_PID_MAX_IOUT);   
		//first order low-pass filter  replace ramp function
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //��ʼ��Ѫ��
    chassis_move_init->chassis_auto.auto_HP.max_HP = chassis_move_init->chassis_auto.ext_game_robot_state_point->maximum_HP;
    chassis_move_init->chassis_auto.auto_HP.cur_HP = chassis_move_init->chassis_auto.ext_game_robot_state_point->maximum_HP;
    chassis_move_init->chassis_auto.auto_HP.last_HP = chassis_move_init->chassis_auto.ext_game_robot_state_point->maximum_HP;

    //��ʼ�������Զ��ƶ�������
    chassis_auto_move_controller_init(&chassis_move_init->chassis_auto.chassis_auto_move_controller, AUTO_MOVE_K_DISTANCE_ERROR, AUTO_MOVE_MAX_OUTPUT_SPEED, AUTO_MOVE_MIN_OUTPUT_SPEED);

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}


/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_behaviour_mode_set(chassis_move_mode);
}


/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    // ����С����ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_SPIN) && 
			   chassis_move_transit->chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
		 if ((chassis_move_transit->last_chassis_mode != CHASSIS_WORLD_SPIN) && 
			   chassis_move_transit->chassis_mode == CHASSIS_WORLD_SPIN)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow gimbal angle mode
    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //���������̽Ƕ�ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // change to no follow angle
    // ���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    if ((chassis_move_transit->last_chassis_mode == CHASSIS_VECTOR_SPIN) &&
        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        //С����ģʽ�ͽ���λ
        chassis_move_transit->mode_flag = 1;
    }
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
		chassis_move.gimbal_data.relative_angle=((fp32)(chassis_move.gimbal_data.relative_angle_receive))*Motor_Ecd_to_Rad;
    chassis_move_update->chassis_yaw = rad_format(chassis_move_update->chassis_INS_point->Yaw - chassis_move_update->chassis_INS_point->firstyaw);
    chassis_move_update->chassis_roll = chassis_move_update->chassis_INS_point->Roll;
}

/**
  * @brief          ��������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
//    fp32 vx_set_channel_RC, vy_set_channel_RC;
//    int16_t vx_channel_RC, vy_channel_RC;
//    // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
//    // �������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel_RC, CHASSIS_RC_DEADLINE);
//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel_RC, CHASSIS_RC_DEADLINE);

//    vx_set_channel_RC = vx_channel_RC * CHASSIS_VX_RC_SEN;
//    vy_set_channel_RC = vy_channel_RC * CHASSIS_VY_RC_SEN;

//    // first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
//    // һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel_RC);
//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel_RC);

   *vy_set += -chassis_move_rc_to_vector->vy_set_CANsend/1000;
    *vx_set += +chassis_move_rc_to_vector->vx_set_CANsend/1000;
   
}

/**
  * @brief          ��������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_vision_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_vision_to_vector)
{
    // �趨ֵ
    fp32 vx = 0;
    fp32 vy = 0;

    //�����˶�ģʽ�ж��Ƿ�Ҫ�����˶�
    if (chassis_move_vision_to_vector->chassis_auto.chassis_vision_control_point->vision_control_chassis_mode == FOLLOW_TARGET)
    {
        // ���������
        chassis_auto_move_controller_calc(&chassis_move_vision_to_vector->chassis_auto.chassis_auto_move_controller, AUOT_MOVE_SET_DISTANCE, chassis_move_vision_to_vector->chassis_auto.chassis_vision_control_point->distance);
        // ���������
        vx = chassis_move_vision_to_vector->chassis_auto.chassis_auto_move_controller.output;
        vy = 0;

        // һ�׵�ͨ�˲���������
        first_order_filter_cali(&chassis_move_vision_to_vector->chassis_cmd_slow_set_vx, vx);

        if (vx == 0)
        {
            chassis_move_vision_to_vector->chassis_cmd_slow_set_vx.out = 0;
        }

        // ��ֵ
        *vx_set = chassis_move_vision_to_vector->chassis_cmd_slow_set_vx.out;
        *vy_set = vy;
    }
    else if (chassis_move_vision_to_vector->chassis_auto.chassis_vision_control_point->vision_control_chassis_mode == UNFOLLOW_TARGET)
    {
        //���ƶ�
        *vx_set = 0;
        *vy_set = 0;
    }
    else
    {
        //���ƶ�
        *vx_set = 0;
        *vy_set = 0;
    }

    if (X_FLAG % 2 == 1)
    {
        *vx_set = -1.f * (*vx_set);
        *vy_set = -1.f * (*vy_set);
    }
}

/**
  * @brief          ��������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_auto_move_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_auto_move_control_vector)
{
    fp32 vx = chassis_auto_move_control_vector->chassis_auto.chassis_auto_move->command_chassis_vx;
    // һ�׵�ͨ�˲���������
    first_order_filter_cali(&chassis_auto_move_control_vector->chassis_cmd_slow_set_vx, vx);
    if (vx == 0)
    {
        chassis_auto_move_control_vector->chassis_cmd_slow_set_vx.out = 0;
    }

    *vx_set = chassis_auto_move_control_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = 0;
}



/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
  
				
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f,angle_set = 0.0f;
    volatile fp32 relative_angle = 0.0f;
    // get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, &angle_set, chassis_move_control,chassis_move_control);
    // ������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_WORLD_FOLLOW_CHASSIS_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

        // ���ÿ��������̨�Ƕ�
      chassis_move_control->chassis_relative_angle_set =firstyaw;
   
        relative_angle1=rad_format(yaw-firstyaw);
        // ��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        if (relative_angle1 > PI)
            relative_angle1 = -2 * PI + relative_angle1;

        sin_yaw = arm_sin_f32(relative_angle1);
        cos_yaw = arm_cos_f32(relative_angle1);

        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
				
				Xx+=vx_set;
				Xy+=vy_set;
       
				chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		

    // С����ģʽ
    else if (chassis_move_control->chassis_mode == CHASSIS_WORLD_SPIN)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        fp32 ab_angle = 0.0f;
    
        // ��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        ab_angle =rad_format(yaw-firstyaw);
       
    sin_yaw = arm_sin_f32(ab_angle);
        cos_yaw = arm_cos_f32(ab_angle);
        // ���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(firstyaw);  
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->wz_set = SPIN_SPEED;
        // �ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_SPIN)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        fp32 relative_angle = 0.0f;
    
        // ��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        relative_angle=chassis_move_control->gimbal_data.relative_angle;
        if (relative_angle > PI)
            relative_angle = -2 * PI + relative_angle;
    sin_yaw = arm_sin_f32(relative_angle);
        cos_yaw = arm_cos_f32(relative_angle);
        // ���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set =  rad_format(angle_set);  
        chassis_move_control->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
        chassis_move_control->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->wz_set = SPIN_SPEED;
        // �ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_move_control->gimbal_data.relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->gimbal_data.relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //������תPID���ٶ�
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->gimbal_data.relative_angle, chassis_move_control->chassis_relative_angle_set);
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        // ��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        // in raw mode, set-point is sent to CAN bus
        // ��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		  else if (chassis_move_control->chassis_mode == CHASSIS_GOBACK)
    {
        // in raw mode, set-point is sent to CAN bus
        // ��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
        chassis_move_control->vx_set = 1.0f;
        chassis_move_control->vy_set = 1.0f;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 1.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 1.0f;
    }
}


/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	//CHASSIS_WZ_SET_SCALE
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.180f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.180f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
		extern chassis_move_t chassis_move;
							
    //mecanum wheel speed calculation
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw����ֱ�ӷ���
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
//    //����pid
//    for (i = 0; i < 4; i++)
//    {
//        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
//        chassis_move_control_loop->power_control.speed[i] = chassis_move_control_loop->motor_chassis[i].speed;
//        if (abs(chassis_move_control_loop->power_control.speed[i]) < chassis_move_control_loop->power_control.SPEED_MIN)
//        {
//            chassis_move_control_loop->power_control.speed[i] = chassis_move_control_loop->power_control.SPEED_MIN;
//        }
//				CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
//    }
//		
	for (i = 0; i < 4; i++)
	{
		chassis_move.motor_chassis[i].give_current = PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, 
																															chassis_move_control_loop->motor_chassis[i].speed_set);
		if (abs(chassis_move.power_control.speed[i]) < chassis_move.power_control.SPEED_MIN)
		{
			chassis_move.power_control.speed[i] = chassis_move.power_control.SPEED_MIN;
		}
		CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
	}

//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->power_control.current[i] = chassis_move_control_loop->motor_chassis[i].give_current = (fp32)(chassis_move_control_loop->motor_speed_pid[i].out);
//        chassis_move_control_loop->power_control.totalCurrentTemp += abs(chassis_move_control_loop->power_control.current[i]);
//    }

//    // ���ʿ���
//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->power_control.MAX_current[i] = (K * chassis_move_control_loop->power_control.current[i] / chassis_move_control_loop->power_control.totalCurrentTemp) * (chassis_move_control_loop->power_control.POWER_MAX) / abs(chassis_move_control_loop->motor_chassis[i].speed);
//    }
//    chassis_move_control_loop->power_control.totalCurrentTemp = 0;

//    // ��ֵ����ֵ
//    for (i = 0; i < 4; i++)
//    {
//        if (abs(chassis_move_control_loop->motor_chassis[i].give_current) >= abs(chassis_move_control_loop->power_control.MAX_current[i]))
//        {
//            chassis_move_control_loop->motor_chassis[i].give_current = chassis_move_control_loop->power_control.MAX_current[i];
//        }
//        else
//        {
//            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
//        }
//    }
//    chassis_move_control_loop->mode_flag = 0;
}


extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_state_t robot_state;
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_move)
{
	uint16_t max_power_limit = 40;
	fp32 input_power = 0;		 // ���빦��(����������)
	fp32 scaled_motor_power[4];
	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  �˲������������ת��ΪŤ��
	fp32 k2 = 1.23e-07;						 // �Ŵ�ϵ��
	fp32 k1 = 1.453e-07;					 // �Ŵ�ϵ��
	fp32 constant = 4.081f;  //3508����Ļ�е���
    
	chassis_move->power_control.POWER_MAX = 0; //���յ��̵������
	chassis_move->power_control.forecast_total_power = 0; // Ԥ���ܹ���
	PID_calc(&chassis_move->buffer_pid,power_heat_data_t.chassis_power_buffer,80); //ʹ��������ά����һ���ȶ��ķ�Χ,�����PIDû��Ҫ��ֲ�ҵģ�������һ������
  max_power_limit = robot_state.chassis_power_limit;  //��ò���ϵͳ�Ĺ���������ֵ
	
	input_power = max_power_limit - chassis_move->buffer_pid.out; //ͨ������ϵͳ�������
	chassis_move->power_control.POWER_MAX = input_power;
	

	for (uint8_t i = 0; i < 4; i++) // �������3508����Ĺ��ʺ��ܹ���
	{
	chassis_move->power_control.forecast_motor_power[i] =
    	chassis_move->motor_chassis[i].give_current * toque_coefficient * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm +
		k1 * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm +
		k2* chassis_move->motor_chassis[i].give_current *chassis_move->motor_chassis[i].give_current + constant;

		if (chassis_move->power_control.forecast_motor_power < 0)  	continue; // ���Ը���
		
		chassis_move->power_control.forecast_total_power += chassis_move->power_control.forecast_motor_power[i];
	}
	
	if (chassis_move->power_control.forecast_total_power > chassis_move->power_control.POWER_MAX) // ������ģ��˥��
	{
		fp32 power_scale = chassis_move->power_control.POWER_MAX / chassis_move->power_control.forecast_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_motor_power[i] = chassis_move->power_control.forecast_motor_power[i] * power_scale; // ���˥����Ĺ���
			
			if (scaled_motor_power[i] < 0)		continue;

			fp32 b = toque_coefficient * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k1 * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

			if (chassis_move->motor_chassis[i].give_current> 0)  //���ⳬ��������
			{					
				chassis_move->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);  
				if (chassis_move->power_control.MAX_current[i] > 16000)
				{
					chassis_move->motor_chassis[i].give_current = 16000;
				}
				else
					chassis_move->motor_chassis[i].give_current = chassis_move->power_control.MAX_current[i];
			}
			else
			{
				chassis_move->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
				if (chassis_move->power_control.MAX_current[i] < -16000)
				{
					chassis_move->motor_chassis[i].give_current = -16000;
				}
				else
					chassis_move->motor_chassis[i].give_current = chassis_move->power_control.MAX_current[i];
			}
			
			chassis_move->motor_chassis[3].give_current *= 0.5f;
			
		}
	}
}




void chassis_auto_move_controller_init(chassis_follow_auto_move_controller_t* controller, fp32 k_distance_error, fp32 max_out, fp32 min_out)
{
    controller->k_distance_error = k_distance_error;
    controller->max_output = max_out;
    controller->min_output = min_out;
}

void chassis_auto_move_controller_calc(chassis_follow_auto_move_controller_t* controller, fp32 set_distance, fp32 current_distance)
{
    //��ֵ��ֵ
    controller->set_distance = set_distance;
    controller->current_distance = current_distance;

    //���ֵ
    fp32 output = 0;

    //�жϵ�ǰ�����Ƿ�С���趨ֵ
    if (controller->current_distance < controller->set_distance)
    {
        //���ƶ�
        output = 0;
    }
    else
    {
        //�������
        output = controller->k_distance_error * (current_distance - set_distance); 
    }

    //�޷�
    if (output >= controller->max_output)
    {
        output = controller->max_output;
    }
    else if (output <= controller->min_output)
    {
        output = controller->min_output;
    }

    //��ֵ���ֵ
    controller->output = output;
}



