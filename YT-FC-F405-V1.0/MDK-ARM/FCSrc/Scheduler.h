/**
 * @file       Scheduler.h
 * @brief      Task Scheduler (Based on Anonymous LingXiao Flight Controller)
 * @version    V1.0.0
 * @date       2026-02-21
 * @Encoding   UTF-8 
 * @note       Derived from Anonymous Tech LingXiao FC scheduler implementation
 */

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "main.h"

#include "propulsion.h"
#include "Interrupt.h"
#include "VOFA.h"
#include "WT901C.h"
#include "T1Plus.h"
#include "RemoteControl.h"
#include "pid_control.h"
#include "Buzzer.h"
#include "flight_state.h"
#include "OpticalFlow.h"
#include "imu.h"

/* 系统时钟频率: 1000Hz（1ms时基） */
#define TICK_PER_SECOND	1000

/* 任务调度结构（函数指针、频率、间隔、上次运行时间戳） */
typedef struct
{
	void(*task_func)(void);   /* 任务函数指针 */
	uint16_t rate_hz;         /* 执行频率(Hz) */
	uint16_t interval_ticks;  /* 执行间隔(ms) */
	uint32_t last_run;        /* 上次运行时间戳(ms) */
}sched_task_t;

/** 飞控系统初始化（硬件外设、传感器、电机解锁等） */
void FC_init(void);

/** 任务调度器初始化（配置任务表与执行周期） */
void Scheduler_Setup(void);

/** 任务调度器主循环（按频率分发任务，需1ms周期调用） */
void Scheduler_Run(void);

#endif
