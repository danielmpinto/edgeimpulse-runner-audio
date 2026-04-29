/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "FreeRTOS.h"
#include "ei_accelerometer.h"
#include "ei_analogsensor.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_device_raspberry_rp2xxx.h"
#include "ei_dht11sensor.h"
#include "ei_inertialsensor.h"
#include "ei_rp2xxx_internal_temperature.h"
#include "ei_run_impulse.h"
#include "ei_ultrasonicsensor.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <time.h>



// edited

#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/stdio.h>
#include <hardware/i2c.h>


// // freertos
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdlib.h>

// // específico
#include "mpu6050.h"
// edited
// -- Adicione estas 3 linhas em main.cpp --
#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

using namespace ei;

// Forward-declaration sem incluir ei_run_classifier.h
// (a definição já está compilada em ei_run_fusion_impulse.cpp)
extern "C" EI_IMPULSE_ERROR run_classifier(
    ei::signal_t *signal,
    ei_impulse_result_t *result,
    bool debug);
// definindo necessário pra MPU - I2C
static bool debug_nn = false;
const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}


#if defined(RASPBERRYPI_PICO2_W) || defined(RASPBERRYPI_PICO_W)
#include "pico/cyw43_arch.h"
#pragma message("Including WiFi support for Raspberry Pi Pico 2 W")
#endif

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceRP2xxx::get_device());
static ATServer *at;

void ei_init(void)
{
    EiDeviceRP2xxx *dev = static_cast<EiDeviceRP2xxx *>(EiDeviceRP2xxx::get_device());

    ei_sleep(2000); // Wait for the serial port to be ready

    ei_printf(
        "Hello from Edge Impulse\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);

    // Setup internal temperature sensor on RP2350/RP2040
    ei_rp2xxxtemp_sensor_init();

    // Setup ADXL345 Accelerometer
    if (ei_accelerometer_init() == false) {
        ei_printf("ADXL345 initialization failed");
    }

    // Setup the inertial sensor
    if (ei_inertial_sensor_init() == false) {
        ei_printf("Inertial sensor communication error occurred\r\n");
    }

    // Setup the temp&humidity sensor
    if (ei_dht11_sensor_init() == false) {
        ei_printf("DHT11 initialization failed\r\n");
    }
    else {
        ei_printf("DHT11 initialization successful");
    }

    // Setup the ultrasonic sensor
    if (ei_ultrasonic_sensor_init() == false) {
        ei_printf("Ultrasonic ranger initialization failed\r\n");
    }

    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor initialization failed\r\n");
    }

    // cannot init device id before main() started on RP2XXX
    dev->init_device_id();
    dev->load_config();
    dev->set_state(eiStateFinished);

    // init AT command parser
    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
}

void ei_main(void *pvParameters)
{
    /* Initialize Edge Impulse sensors and commands */
    ei_init();

    while (true) {
        /* handle command comming from uart */
        ei_sleep(5);
        char data = ei_get_serial_byte();

        while (data != 0xFF) {
            at->handle(data);
            data = ei_get_serial_byte();
        }
    }
}

/* To verify FreeRTOS is working across both Pico targets */
void test_task(void *pvParameters)
{
    while (1) {
#if defined(RASPBERRYPI_PICO2_W) || defined(RASPBERRYPI_PICO_W)

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Turn LED on
        sleep_ms(2050); // Wait for 2.05 seconds
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Turn LED off
        sleep_ms(1250); // Wait for 1.25 seconds

#elif defined(RASPBERRYPI_PICO2) || defined(RASPBERRYPI_PICO)

        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(2050);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1250);

#endif
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second before repeating
    }
}

void vLaunch(void)
{
    TaskHandle_t task;

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

static void gesture_recognize_task(void *p)
{
    

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);
    gpio_put(16,0);

    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17,0);

    gpio_init(18);
    gpio_set_dir(18, GPIO_OUT);
    gpio_put(18,0);
    // uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
    // Initialize I2C port 0 and configuring Pins 0 and 1 for MPU6050
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();



    int16_t accelerometer[3],gyro[3],temp;



    while (true) 
    
    {
        ei_printf("\nStarting inferencing in 2 seconds...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        //uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
        mpu6050_read_raw(accelerometer,gyro,&temp);
        buffer[ix + 0]= accelerometer[0];
        buffer[ix + 1]= accelerometer[1];
        buffer[ix + 2]= accelerometer[2];

        
        //IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);
    }

    // signal_t signal;
    ei::signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        break;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    // err = run_classifier(&signal, &result, debug_nn);
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        break;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    
    if (result.classification[0].value > result.classification[1].value && result.classification[0].value > result.classification[2].value) {
        gpio_put(16, 1);
        gpio_put(17, 0);
        gpio_put(18, 0);
    } else if (result.classification[1].value > result.classification[0].value && result.classification[1].value > result.classification[2].value) {
        gpio_put(16, 0);
        gpio_put(17, 1);
        gpio_put(18, 0);
    } else if (result.classification[2].value > result.classification[0].value && result.classification[2].value > result.classification[1].value) {
        gpio_put(17, 0);
        gpio_put(16, 0);
        gpio_put(18, 1);
    }
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("teste    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    
    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    }
    vTaskDelay(pdMS_TO_TICKS(10));
}


int main(void)
{

    //stdio_usb_init();
    stdio_init_all();

    

    xTaskCreate(gesture_recognize_task, "gesture_task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}