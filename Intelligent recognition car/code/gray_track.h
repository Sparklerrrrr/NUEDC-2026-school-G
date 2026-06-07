#ifndef __gray_track_h_
#define __gray_track_h_
#include "headfile.h"

// 8 路灰度传感器快捷宏（D1=最左, D8=最右）
#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)

// --- 循迹 PID 参数（可被 LLM-PID-Tuner 覆盖） ---
#define TRACK_BASE_SPEED  360  // 基准速度（编码器单位/10ms）
#define TRACK_K           90   // 循迹比例系数
#define TRACK_I           6    // 循迹积分系数
#define TRACK_D           62   // 循迹微分系数

// 掉线判定阈值：sensor_count=0 持续超过此值视为完全脱线
#define TRACK_LOST_VALUE  500

// --- 赛道参数 ---
#define LAP_ENCODER_TARGET  5400  // 单圈编码器脉冲目标（决定圈长）
#define TOTAL_LAPS          2     // 默认圈数

void gray_init(void);
void track(void);
void track_with_speed(int speed);
int get_sensor_position(void);
unsigned char digtal(unsigned char channel);

#endif
