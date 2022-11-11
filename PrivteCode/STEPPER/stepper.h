#ifndef __STEPPER_H__
#define __STEPPER_H__
 
#include "main.h"

void Step_Motor_GPIO_Init(void);
/*
	功能：转1/64圈
	步距角5.625 360/5.625=64 减速比1/64
	故64*64个脉冲转一圈
	n 圈数
	direction 方向 1正转 非1反转
	delay delay时长 >= 2
*/
void SetMotor(unsigned char InputData);
void motor_circle(int angle, int direction, int delay);
void Motor_contin(int direction, int delay);

#define cw 1
#define ccw 0
typedef struct
{
	uint8_t  home_state;   //是否回家的标志 1 OK / 0 NOT
	uint16_t scan_count;   //已经扫描完的次数
	uint8_t  jog_dir;      //1 cw , 0 ccw
	uint8_t  incap_state;  //输入捕获状态
	uint8_t  in_halfposition;  //半轮扫描完成
	uint8_t  in_position;  //一轮扫描完成
}MachineState;

typedef struct
{
	uint8_t mode;      //自动模式1/手动模式0，home_state=1下开启
}set_workmode;


#endif

