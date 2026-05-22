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

// imu
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>
#include <pico/stdio.h>

// freertos
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdlib.h>
#include <task.h>

// // específico
#include "mpu6050.h"
// edited
// -- Adicione estas 3 linhas em main.cpp --
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "model-parameters/model_metadata.h"

using namespace ei;

extern "C" EI_IMPULSE_ERROR
run_classifier(ei::signal_t *signal, ei_impulse_result_t *result, bool debug);

static bool debug_nn = false;


#include "hardware/adc.h"


#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))


volatile int g_timer_0 = 0;

bool timer_0_callback(repeating_timer_t *rt) {
    g_timer_0 = 1;
    return true; // keep repeating
}

static void wakework_recognition(void *p)
{
    uint adc_raw = 0;

    while (true) {
        //        ei_printf("\nStarting inferencing in 2 seconds...\n");
        //        vTaskDelay(pdMS_TO_TICKS(2000));
        //        ei_printf("Sampling...\n");

        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        if(g_timer_0){
            for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            adc_raw = adc_read(); // raw voltage from ADC
            // printf("%.2f\n", adc_raw * ADC_CONVERT);
            buffer[ix + 0] = adc_raw;


            g_timer_0 = 0;
             }
        }

     
        
        // Prepara sinal
        ei::signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            break;
        }

        // Run the classifier
        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            break;
        }

        // print the predictions
        ei_printf("Predictions ");
        ei_printf(
            "(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf(
                "teste    %s: %.5f\n",
                result.classification[ix].label,
                result.classification[ix].value);
            // if(result.classification[ix].value > 0.8 && result.classification[ix].label == "alexo"){
            //     ei_printf("Acende LED para %s\n", result.classification[ix].label);
            // }
            if(result.classification[ix].value > 0.8 && result.classification[ix].label == "alexa"){
                gpio_put(16, 1); // Acende o LED
                printf("Acende LED para %s\n", result.classification[ix].label);
            } else {
                gpio_put(16, 0); // Apaga o LED
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(500));

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
    }
}

int main(void)
{
    stdio_init_all();

    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( ADC_NUM);
    gpio_init(16); // Inicializa o GPIO para o LED
    gpio_set_dir(16, GPIO_OUT); // Configura o GPIO como saída      

    repeating_timer_t timer_0;

    if (!add_repeating_timer_us(62.5, 
                                timer_0_callback,
                                NULL, 
                                &timer_0)) {
        printf("Failed to add timer\n");
    }
    xTaskCreate(wakework_recognition, "gesture_task 1", 8192, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}
