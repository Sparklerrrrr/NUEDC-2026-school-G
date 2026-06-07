#ifndef _filter_h
#define _filter_h
#include "math.h"

// 卡尔曼滤波器结构体（一维，含陀螺仪零偏估计）
typedef struct
{
	float Q_angle; // 过程噪声-角度方差（越大越信任陀螺仪积分）
	float Q_bias;  // 过程噪声-陀螺仪零偏方差（越大零偏估计越灵敏）
	float R;       // 观测噪声方差（越大越不信任加速度计/磁力计）
	float P[2][2]; // 误差协方差矩阵
	float dt;      // 采样时间间隔（秒）
	float K1, K2;  // 卡尔曼增益：K1=角度修正系数, K2=零偏修正系数

	float Angle;     // 角度最优估计值
	float Gyro_bias; // 陀螺仪零偏估计值

} KF_t;

float Mahony_Filter(float gyro, float acc);
float Kalman_Filter(KF_t *kf, float obsValue, float ut);

extern KF_t KF_Yaw, KF_Roll, KF_Pitch;

#endif
