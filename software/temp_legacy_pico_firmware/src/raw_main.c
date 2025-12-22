#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "drivers/mpu6050.h"

// Configuration (same as pico_node.c)
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define LED_PIN 25

// Mode Selection (compile-time)
// Set to 1 for 100Hz output (verbose), 0 for 10Hz (less flooding)
#define HIGH_FREQUENCY_MODE 1

int main() {
    stdio_init_all();

    // LED Init
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initial Boot Pattern: 5 fast blinks
    printf("\n=== ALTAIR Raw Serial Test ===\n");
    printf("MPU6050 @ GPIO%d(SDA), GPIO%d(SCL)\n", SDA_PIN, SCL_PIN);
    
    for(int i = 0; i < 5; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
    
    // I2C Init
    printf("Initializing I2C at 400kHz...\n");
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // MPU6050 Init
    printf("Initializing MPU6050...\n");
    mpu6050_init(I2C_PORT);
    printf("MPU6050 Ready.\n\n");
    
    // Test LED pattern: Solid ON if MPU6050 init succeeded
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);

    mpu6050_data_t imu_data;
    
    // Stats
    uint32_t success_count = 0;
    uint32_t error_count = 0;
    uint64_t start_time = to_ms_since_boot(get_absolute_time());
    uint64_t last_print = start_time;
    
#if HIGH_FREQUENCY_MODE
    const uint32_t loop_delay_ms = 10;  // 100Hz
    const uint32_t print_interval = 10; // Print every 10th sample
#else
    const uint32_t loop_delay_ms = 100; // 10Hz
    const uint32_t print_interval = 1;  // Print every sample
#endif

    printf("Starting data loop at %d Hz...\n", 1000 / loop_delay_ms);
    printf("Format: RAW,ax,ay,az,gx,gy,gz\n\n");
    
    while (1) {
        if (mpu6050_read_burst(I2C_PORT, &imu_data)) {
            success_count++;
            
            // Print CSV format for easy parsing
            if (success_count % print_interval == 0) {
                printf("RAW,%d,%d,%d,%d,%d,%d\n", 
                    imu_data.ax, imu_data.ay, imu_data.az,
                    imu_data.gx, imu_data.gy, imu_data.gz
                );
            }
            
            // LED: Quick blink on success
            gpio_put(LED_PIN, 1);
        } else {
            error_count++;
            printf("ERR,Read Failed\n");
            
            // LED: OFF on error
            gpio_put(LED_PIN, 0);
        }
        
        // Rate stats every 5 seconds
        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print >= 5000) {
            float elapsed = (now - start_time) / 1000.0f;
            float rate = success_count / elapsed;
            printf("STAT,elapsed=%.1fs,success=%lu,errors=%lu,rate=%.1fHz\n", 
                   elapsed, success_count, error_count, rate);
            last_print = now;
        }
        
        sleep_ms(loop_delay_ms);
        gpio_put(LED_PIN, 0);
    }
    
    return 0;
}
