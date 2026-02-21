#include "Scheduler.h"

/**
 * @brief  原子设置解锁标志（带临界区保护）
 */
static void Set_Arm_Flag(uint8_t flag)
{
    __disable_irq();
    g_pid.arm_flag = flag;
    __enable_irq();
}

/**
 * @brief  检查是否允许控制（状态+传感器双重确认）
 * @return 1-可以执行PID和输出，0-禁止
 */
static uint8_t Is_Control_Allowed(void)
{
    // 双重检查：状态机状态 + PID标志 + 传感器在线
    if (FState_GetState() != STATE_ARMED) return 0;
    if (!g_pid.arm_flag) return 0;
    if (!imu.online || !imu.valid) return 0;
    if (!elrs_is_connected()) return 0;  // RC必须在线
    
    return 1;
}

float target_pitch;  // 右摇杆Y控制俯仰
float target_roll;   // 右摇杆X控制横滚
float target_yaw;    // 左摇杆X控制偏航速率

bool buzzer_armed_done = 0;

char vofa_ble_buf[64];
vofa_pid_value vofa_pid = {15,0,0,20};

pidParam_t param;

Buzzer_HandleTypeDef buzzer = {&htim3,TIM_CHANNEL_4};

const MotorHandle_t g_motors[MOTOR_COUNT] = {
    {&htim4, TIM_CHANNEL_1},  // M1 前左 (FL)
    {&htim2, TIM_CHANNEL_4},  // M2 前右 (FR)  
    {&htim2, TIM_CHANNEL_3},  // M3 后右 (BR)
    {&htim4, TIM_CHANNEL_2},  // M4 后左 (BL)
};

void FC_init(void)
{

	Buzzer_init(&buzzer);
	
	Propulsion_Init(g_motors);
	
	HAL_Delay(2000);//等待硬件与电机初始化，接收机连接遥控器
	
	vofa_init();
	vofa_login_name("KP",&vofa_pid.kp,TYPE_FLOAT);
	vofa_login_name("KI",&vofa_pid.ki,TYPE_FLOAT);
	vofa_login_name("KD",&vofa_pid.kd,TYPE_FLOAT);
	vofa_login_name("ST",&vofa_pid.iSepThresh,TYPE_FLOAT);
	
	imu_init();
	
	rc_init();
	
	optical_flow_init();
	
	PID_InitAll(15.0f);
	PID_SetMode(MODE_ANGLE);

	Scheduler_Setup();
	
	Buzzer_DoubleBeep(&buzzer);
	
	// 启动TIM1定时器中断
  //	HAL_TIM_Base_Start_IT(&htim1);
	
}

static void Loop_1000Hz(void)
{

}

static void Loop_500Hz(void)
{
	
}

static void Loop_200Hz(void)
{
	  
}

static void Loop_100Hz(void)
{
	  vofa_analysis_data();
	  
    wt901c_analysis_data();
    imu_update();
    
    elrs_analysis_data();
    rc_filter_process();
    
    T1Plus_analysis_data();
    optical_flow_update();
    
    FState_Update();
    
    static ArmState_t last_state = STATE_DISARMED;
    ArmState_t curr_state = FState_GetState();
    
    if (curr_state != last_state) {
        if (curr_state == STATE_ARMED) {
            Set_Arm_Flag(1);
            PID_SystemReset();// 状态变化时重置PID（防止积分累积）
        } else {
            Set_Arm_Flag(0);
            __disable_irq();// 退出ARMED时强制清零输出（紧急制动）
            g_pid.out.throttle = 0;
            g_pid.out.pitch = 0;
            g_pid.out.roll = 0;
            g_pid.out.yaw = 0;
            __enable_irq();
        }
        last_state = curr_state;
    }
    
    if (curr_state == STATE_ARMED) {//200Hz 紧急保护
        if (!imu.online || !elrs_is_connected()) {// 快速检查：如果IMU或RC异常，立即强制停止（不等待100Hz）
            FState_ForceEmergency();
            Set_Arm_Flag(0);
            Propulsion_Stop();
        }
    } 
		
    if (!Is_Control_Allowed()) {
        Propulsion_Stop();
        return;  // 快速返回，不执行后续计算
    }
    
    int8_t rc_ry = filtered_rc.RY;
    int8_t rc_rx = filtered_rc.RX;
    int8_t rc_lx = filtered_rc.LX;
    int8_t rc_ly = filtered_rc.LY;
    int8_t rc_sa = filtered_rc.SA;
    int8_t rc_sd = filtered_rc.SD;
    
    if (abs(rc_ry) > 100 || abs(rc_rx) > 100) {// 检查摇杆有效性（防止滤波器输出异常值）
        FState_ForceEmergency();
        return;
    }
    
    target_pitch = PID_StickToAngle(rc_ry);
    target_roll  = PID_StickToAngle(rc_rx);
    target_yaw   = PID_StickToRate(rc_lx);
    
    
    uint8_t fault = PID_UpdateAttitude(
        target_pitch, target_roll, target_yaw,
        imu.pitch, imu.roll, imu.yaw,
        imu.gx, imu.gy, imu.gz
    );
    
    if (fault) {
        FState_ForceEmergency();
        Propulsion_Stop();
        return;
    }
    
    float throttle = (rc_ly + 100.0f) / 2.0f;
    if (throttle < 0) throttle = 0;
    if (throttle > 100) throttle = 100;
    
    __disable_irq();
    g_pid.out.throttle = throttle;
    __enable_irq();
    
    // SA和SD拨到高位才允许输出
    if (rc_sa > 80 && rc_sd > 80) {
        Propulsion_MixOutput(
            g_pid.out.throttle,
            g_pid.out.pitch,
            g_pid.out.roll,
            0  // yaw输出
        );
    } else {
        Propulsion_Stop();// 安全开关未打开，保持怠速或停止
    }
}

static void Loop_50Hz(void)
{ 
}

static void Loop_10Hz(void)
{
//	printf("[RC  PRY]:%.2f,%.2f,%.2f\r\n",target_pitch,target_roll,target_yaw);	
//	printf("[PRY]:%d,%.2f,%.2f,%.2f\r\n",wt901c_data.online,wt901c_data.pitch,wt901c_data.roll,wt901c_data.yaw);
//	printf("[PRY]:%.2f,%.2f,%.2f\r\n",wt901c_data.pitch,wt901c_data.roll,wt901c_data.yaw);
//	printf("ax/y/z:%.2f,%.2f,%.2f\r\n",wt901c_data.wx,wt901c_data.wy,wt901c_data.wz);
//  printf("pry:%.2f,%.2f,%.2f\r\n",pitch_out,roll_out,yaw_out);
//	printf("[CRSF]:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",rc_raw_channels.ch1,rc_raw_channels.ch2,rc_raw_channels.ch3,rc_raw_channels.ch4,rc_raw_channels.ch5,rc_raw_channels.ch6,rc_raw_channels.ch7,rc_raw_channels.ch8,rc_raw_channels.ch9,rc_raw_channels.ch10);
//	printf("[RC]:%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f%.0f,%.0f,%.0f,%.0f\r\n",elrs_status.is_connected,filtered_rc.LX,filtered_rc.LY,filtered_rc.RX,filtered_rc.RY,filtered_rc.SA,filtered_rc.SB,filtered_rc.SC,filtered_rc.SD,filtered_rc.SE,filtered_rc.SL);
//	printf("state:%d,arm:%d,falult:%d\r\n",FState_GetState(),g_pid.arm_flag,g_pid.fault);
//	printf("[PID/TH]:%.2f,%.2f,%.2f,%.2f\r\n",vofa_pid.kp/100,vofa_pid.ki/100,vofa_pid.kd/100,vofa_pid.iSepThresh);
//	printf("[T1Plus]:%d,%d\r\n",t1plus_data.laser_distance,t1plus_data.valid);
//	printf("[OpticalFlow]:%d,%d,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%d\r\n",optflow.online,optflow.valid,optflow.height,optflow.quality,optflow.vel_x,optflow.vel_y,optflow.pos_x,optflow.pos_y,optflow.is_moving);
//	printf("[IMU]:%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",imu.online,imu.roll, imu.pitch, imu.yaw,imu.gx, imu.gy, imu.gz);
//  printf("[G_PID]:%.2f,%.2f,%.2f\r\n",g_pid.out.throttle,g_pid.out.pitch,g_pid.out.roll);	
//	printf("speed:%f,%f,%f,%f\r\n",m[MOTOR_FL],m[MOTOR_FR],m[MOTOR_BR],m[MOTOR_BL]);
	
	
//    param = (pidParam_t){vofa_pid.kp/100,vofa_pid.ki/1000,vofa_pid.kd/100,vofa_pid.iSepThresh};
// 		PID_SetRateParam(AXIS_PITCH,&param);
//		PID_SetRateParam(AXIS_ROLL,&param);
//		sprintf(vofa_ble_buf, "%.2f,%.2f,%.2f\r\n",vofa_pid.kp/100.0f,vofa_pid.ki/1000,vofa_pid.kd/100);
//		HAL_UART_Transmit(&huart1, (uint8_t*)vofa_ble_buf, strlen(vofa_ble_buf), 80);

}

static void Loop_2Hz(void)
{
//  	if (FState_GetState() == STATE_ARMED && !buzzer_armed_done) {
//			Buzzer_SetVolume(&buzzer,10);
//			buzzer_armed_done = 1; 
//	  }
}

/**
 * @brief 系统任务配置表
 * @note 创建不同执行频率的任务列表
 */
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},  /*!< 1000Hz任务 */
		{Loop_500Hz, 500, 0, 0},    /*!< 500Hz任务 */
		{Loop_200Hz, 200, 0, 0},    /*!< 200Hz任务 */
		{Loop_100Hz, 100, 0, 0},    /*!< 100Hz任务 */
		{Loop_50Hz, 50, 0, 0},      /*!< 50Hz任务 */
		{Loop_10Hz, 10, 0, 0},      /*!< 20Hz任务 */
		{Loop_2Hz, 2, 0, 0},        /*!< 2Hz任务 */
	};
	
/**
 * @brief 计算任务数量
 * @note 根据任务配置表的大小自动计算任务数量
 */
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))
	
/**
 * @brief 任务调度器初始化函数
 * @param 无
 * @retval 无
 * @note 计算每个任务的执行周期，并初始化任务表
 */
void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for (index = 0; index < TASK_NUM; index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}

/**
 * @brief 任务调度器运行函数
 * @param 无
 * @retval 无
 * @note 该函数需放在main函数的while(1)中，不断检查并执行到期的任务
 */
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有任务，是否应该执行

	for (index = 0; index < TASK_NUM; index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = HAL_GetTick();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该任务的执行周期，则执行任务
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//更新任务的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行任务函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}
