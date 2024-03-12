    /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "chassis_task.h"

#define abs(x) ((x) > 0 ? (x) : (-x))
