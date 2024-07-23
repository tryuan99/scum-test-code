#include "sensor_potentiometric.h"

#include <stdint.h>
#include <stdlib.h>

#include "adc.h"

// Number of samples to average over.
#define NUM_SAMPLES_TO_AVERAGE 64

uint16_t sensor_potentiometric_measure(void) {
    // Average over multiple ADC outputs.
    uint16_t adc_output_sum = 0;
    for (size_t i = 0; i < NUM_SAMPLES_TO_AVERAGE; ++i) {
        adc_output_sum += adc_read_output();
    }
    return adc_output_sum / 64;
}
