#include "filter.h"

// 互补滤波权重系数（gyro 占 ~95%，acc 占 ~5%）
#define alpha  0.95238

// --- 预定义 3 个卡尔曼滤波器实例 ---

// 航向角（Yaw）：陀螺仪 + 磁力计融合
KF_t KF_Yaw = {
	0.001,            // Q_angle: 过程噪声-角度方差
	0.003,            // Q_bias:  过程噪声-陀螺仪零偏方差
	0.5,              // R:       观测噪声（磁力计）
	{{1, 0}, {0, 1}}, // P:       初始误差协方差矩阵
	0.05              // dt:      采样周期 50ms
};

// 横滚角（Roll）
KF_t KF_Roll = {
	0.001,
	0.003,
	0.5,
	{{1, 0}, {0, 1}},
	0.05
};

// 俯仰角（Pitch）
KF_t KF_Pitch = {
	0.001,
	0.003,
	0.5,
	{{1, 0}, {0, 1}},
	0.05
};


// 一阶互补滤波：陀螺仪角度 + 加速度计角度按权重混合
// gyro * alpha + acc * (1-alpha)，其中 alpha≈0.95 表示信任陀螺仪更多
float Mahony_Filter(float gyro, float acc)
{
	return (alpha * gyro + (1 - alpha) * acc);
}


// 标准一维卡尔曼滤波（角度 + 陀螺仪零偏估计）
// kf:     滤波器实例指针
// obsValue: 观测量（加速度计/磁力计角度）
// ut:       控制量（陀螺仪角速度，度/秒）
// 返回值:   融合后的最优角度估计
float Kalman_Filter(KF_t *kf, float obsValue, float ut)
{
	// --- 预测步骤 ---
	// 状态预测：角度 += (角速度 - 零偏) * dt
	kf->Angle = kf->Angle + (ut - kf->Gyro_bias) * kf->dt;
	kf->Gyro_bias = kf->Gyro_bias;

	// 误差协方差 P 的预测更新
	kf->P[0][0] = kf->P[0][0] - (kf->P[0][1] + kf->P[1][0]) * kf->dt
	              + kf->P[1][1] * kf->dt * kf->dt + kf->Q_angle;
	kf->P[0][1] = kf->P[0][1] - kf->P[1][1] * kf->dt;
	kf->P[1][0] = kf->P[1][0] - kf->P[1][1] * kf->dt;
	kf->P[1][1] = kf->P[1][1] + kf->Q_bias;

	// --- 更新步骤 ---
	// 计算卡尔曼增益 K1（角度修正增益）、K2（零偏修正增益）
	kf->K1 = kf->P[0][0] / (kf->P[0][0] + kf->R);
	kf->K2 = kf->P[1][0] / (kf->P[0][0] + kf->R);

	// 用观测量修正状态估计
	kf->Angle = kf->Angle + kf->K1 * (obsValue - kf->Angle);
	kf->Gyro_bias = kf->Gyro_bias + kf->K2 * (obsValue - kf->Angle);

	// 更新误差协方差矩阵
	kf->P[0][0] = (1 - kf->K1) * kf->P[0][0];
	kf->P[0][1] = (1 - kf->K1) * kf->P[0][1];
	kf->P[1][0] = kf->P[1][0] - kf->P[0][0] * kf->K2;
	kf->P[1][1] = kf->P[1][1] - kf->P[0][1] * kf->K2;

	return kf->Angle;
}
