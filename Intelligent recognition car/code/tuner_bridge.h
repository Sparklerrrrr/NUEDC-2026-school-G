#ifndef __tuner_bridge_h_
#define __tuner_bridge_h_
#include "headfile.h"

// CSV 数据发送间隔（毫秒），50ms = 20Hz
#define TUNER_SEND_INTERVAL_MS  50

// 串口命令缓冲区大小
#define TUNER_CMD_BUF_SIZE      80

extern uint8_t tuner_cmd_ready;
extern char    tuner_cmd_buf[TUNER_CMD_BUF_SIZE];

void tuner_bridge_send_csv(void);
void tuner_bridge_process_cmd(void);

#endif
