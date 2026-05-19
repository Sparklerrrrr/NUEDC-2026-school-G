#ifndef __gray_track_h_
#define __gray_track_h_
#include "headfile.h"

#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)

#define LINE_LOST    999

#define POKER_LOST_THRESHOLD  5
#define REAL_LOST_THRESHOLD   150

#define SENSOR_COUNT  8

void gray_init(void);
void track(void);
int get_track_error(void);
void track_pid(void);
unsigned char digtal(unsigned char channel);
int get_active_sensor_count(void);

extern volatile int last_track_error;
extern volatile int lost_line_count;
extern volatile uint8_t poker_crossing;

#endif
