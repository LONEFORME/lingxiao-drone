#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "my_fun.h"
#include "SysConfig.h"
#include "stm32f4xx.h"
#include "my_protocol.h"
void UserTask_OneKeyCmd(void);
extern volatile struct sdata received_data;
extern u8 height_set_sign;
extern u8 mission_stage;
extern u8 mission_done_flag;
extern u8 land_timeout_gaveup_f;
#endif
