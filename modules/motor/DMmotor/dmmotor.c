#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx = 0;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
// static osThreadId dm_task_handle[DM_MOTOR_CNT];
// static CANInstance sender_assignment = {
//     .can_handle = &hcan1, 
//     .txconf.StdId = 0x101, 
//     .txconf.IDE = CAN_ID_STD, 
//     .txconf.RTR = CAN_RTR_DATA, 
//     .txconf.DLC = 0x08, 
//     };
void Math_Constrain_float(float *x, float Min, float Max)
{
    if(*x < Min)
    {
        *x = Min;
    }
    else if(*x > Max)
    {
        *x = Max;
    }
}
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instance, 1);
}
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
        motor->motor_settings.angle_feedback_source = type;
    }
    else if (loop == SPEED_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
        motor->motor_settings.angle_feedback_source = type;
    }
    else if (loop == OPEN_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
        motor->motor_settings.angle_feedback_source = type;
    }
    else
        LOGERROR("[jz_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);
    measure->state=rxbuff[0] >> 4;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    // if (measure->position - measure->position > xx)
    //     measure->total_round--;
    // else if (measure->position - measure->position < -xx)
    //     measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->position * DM_ECD_TO_ANGLE;
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instance = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);
    // DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

float set_to; 
// uint8_t * PBuf, * VBuf; 
// DMMotor_Send_s motor_send_mailbox;
//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask()
{  
    float  pid_ref, set, pid_measure;
    DMMotorInstance *motor;
    DM_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    CANInstance *motor_can;
    uint16_t tmp;
    DMMotor_Send_s *motor_send_mailbox;
    for (size_t i = 0; i < idx; ++i)
    {   
        uint8_t * PBuf, * VBuf;
        motor=dm_motor_instance[i];
        measure=&motor->measure;
        setting=&motor->motor_settings;
        pid_ref = motor->pid_ref;
        motor_send_mailbox= &motor->motor_send_mailbox;

        if(motor->motor_settings.angle_feedback_source == OTHER_FEED)
        {
            pid_measure = *motor->other_angle_feedback_ptr;
            set = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            // set = pid_ref - pid_measure;
        }
        else if(motor->motor_settings.angle_feedback_source == MOTOR_FEED)
        {
            set = pid_ref;
        }

        if(motor->stop_flag == MOTOR_STOP)
        {
            motor_send_mailbox->torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox->velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox->position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 12);
            DMMotorSetMode(DM_CMD_CLEAR_ERROR,motor);
            DWT_Delay(0.01);
        }
        else if(motor->stop_flag == MOTOR_ENALBED)
        {
            DMMotorSetMode(DM_CMD_MOTOR_MODE,motor);
        }
        motor_send_mailbox->position_des=set/DM_ECD_TO_ANGLE;
        motor_send_mailbox->velocity_des=2.0f;
        Math_Constrain_float(&motor_send_mailbox->position_des, DM_MIN_POSITION, DM_MAX_POSITION);
        Math_Constrain_float(&motor_send_mailbox->velocity_des, DM_V_MIN, DM_V_MAX);
        PBuf = (uint8_t *)(&motor_send_mailbox->position_des);
        VBuf = (uint8_t *)(&motor_send_mailbox->velocity_des);

        motor->motor_can_instance->tx_buff[0] = PBuf[0];
        motor->motor_can_instance->tx_buff[1] = PBuf[1];
        motor->motor_can_instance->tx_buff[2] = PBuf[2];
        motor->motor_can_instance->tx_buff[3]= PBuf[3];
        motor->motor_can_instance->tx_buff[4] = VBuf[0];
        motor->motor_can_instance->tx_buff[5] = VBuf[1];
        motor->motor_can_instance->tx_buff[6] = VBuf[2];
        motor->motor_can_instance->tx_buff[7] =  VBuf[3];

        // motor->motor_can_instance->tx_buff[0] = *PBuf;
        // motor->motor_can_instance->tx_buff[1] = *(PBuf+1);
        // motor->motor_can_instance->tx_buff[2] = *(PBuf+2);
        // motor->motor_can_instance->tx_buff[3]= *(PBuf+3);
        // motor->motor_can_instance->tx_buff[4] = *VBuf;
        // motor->motor_can_instance->tx_buff[5] =*(VBuf+1);
        // motor->motor_can_instance->tx_buff[6] =*(VBuf+2);
        // motor->motor_can_instance->tx_buff[7] = *(VBuf+3);

        CANTransmit(motor->motor_can_instance, 1);
        osDelay(2);
    }
}

