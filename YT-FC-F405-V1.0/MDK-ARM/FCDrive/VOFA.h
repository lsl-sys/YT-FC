/**
 * @file       VOFA.h
 * @author	   lsl-sys
 * @brief      Using VOFA+ software, parse "name:value" format commands to automatically match registered variables and assign values.
 * @version    V1.1.0
 * @date       2026-1-30 2026-2-10
 * @Encoding   UTF-8
 */

#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"   

#define VOFA_TYPE_INT    0
#define VOFA_TYPE_FLOAT  1
#define VOFA_TYPE_DOUBLE 2
#define VOFA_TYPE_BOOL   3

#define VOFA_MAX_VARS 20

/* 变量类型 */
typedef enum {
    TYPE_INT,      // 整型
    TYPE_FLOAT,    // 单精度浮点
    TYPE_DOUBLE,   // 双精度浮点
    TYPE_BOOL      // 布尔型
} vofa_type_t;

/* 变量字典表项：绑定名称、内存地址和数据类型 */
typedef struct {
    const char* name;    // 变量名
    void*       addr;    // 变量地址
    vofa_type_t type;    // 变量类型
} vofa_var_t;

// 临时数值存储
static union {
    int    i;
    float  f;
    double d;
} vofa_temp_value;

/* 初始化：启动DMA空闲中断，关闭半传输中断 */
void vofa_init(void);

/* 串口中断内调用：保存接收数据到缓冲区，重启DMA接收 */
void vofa_receive_data(void);

/* 定时器中断调用：解析"name:value"指令，自动匹配注册变量并赋值 */
void vofa_analysis_data(void);

/* 注册变量：将全局变量加入字典，供上位机通过名称访问 */
void vofa_login_name(const char* name, void* addr, vofa_type_t type);

/* 查询接收标志：返回1表示有新数据(读取后自动清0) */
bool vofa_get_flag_of_receive(void);

/* 按变量名获取当前值：用于上传数据到上位机，未找到返回0 */
double vofa_get_data(const char* name);

extern vofa_var_t var_table[VOFA_MAX_VARS];
extern uint8_t    var_count;

/* PID调参 */
typedef struct {
  float kp;
	float ki;
	float kd;
	float iSepThresh;   // 积分分离阈值，0表示禁用积分分离
} vofa_pid_value;

#endif 
