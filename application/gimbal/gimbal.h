#ifndef GIMBAL_H
#define GIMBAL_H

#define DEG2RAD 0.01745329252f // π/180
/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H