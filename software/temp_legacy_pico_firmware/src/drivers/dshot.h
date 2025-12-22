#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>
#include "pico/stdlib.h"

// DShot types
#define DSHOT_150  150000
#define DSHOT_300  300000
#define DSHOT_600  600000

// Configuration
#define DSHOT_SPEED DSHOT_600
#define NUM_MOTORS 6

// Motor pins (Update these based on actual hardware wiring plan)
// Assuming consecutive pins for now or defined array
extern const uint dshot_pins[NUM_MOTORS];

void dshot_init(void);
void dshot_write_throttle(uint8_t motor_index, float throttle); // 0.0 to 1.0
void dshot_send_frame(void); // Trigger PIO push

#endif // DSHOT_H
