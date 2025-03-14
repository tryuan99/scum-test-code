#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc.h"
#include "gpio.h"
#include "memory_map.h"
#include "optical.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "sensor.h"
#include "sensor_resistive.h"
#include "sensors.h"

// RF timer ID.
#define RFTIMER_ID 7

// Number of for loop cycles between ADC reads.
// 700000 for loop cycles roughly correspond to 1 second.
#define NUM_CYCLES_BETWEEN_ADC_READS 400000

// ADC configuration.
static const adc_config_t g_adc_config = {
    .reset_source = ADC_RESET_SOURCE_FSM,
    .convert_source = ADC_CONVERT_SOURCE_FSM,
    .pga_amplify_source = ADC_PGA_AMPLIFY_SOURCE_FSM,
    .pga_gain = 0,
    .settling_time = 0,
    .bandgap_reference_tuning_code = 1,
    .const_gm_tuning_code = 0xFF,
    .vbat_div_4_enabled = false,
    .ldo_enabled = true,
    .input_mux_select = ADC_INPUT_MUX_SELECT_EXTERNAL_SIGNAL,
    .pga_bypass = true,
};

// Sensors configuration.
static const sensors_config_t g_sensors_config = {
    .gpio_strobe = GPIO_0,
    .gpios_select =
        {
            GPIO_1,
            GPIO_2,
            GPIO_3,
        },
    .num_sensors = 5,
    .sensors =
        {
            SENSOR_TYPE_POTENTIOMETRIC,
            SENSOR_TYPE_POTENTIOMETRIC,
            SENSOR_TYPE_POTENTIOMETRIC,
            SENSOR_TYPE_POTENTIOMETRIC,
            SENSOR_TYPE_POTENTIOMETRIC,
        },
    .sensor_configs =
        {
            {0},
            {0},
            {0},
            {0},
            {0},
        },
};

// Callback for the RF timer.
static void rftimer_callback(void) { sensor_resistive_rftimer_callback(); }

int main(void) {
    initialize_mote();

    // Configure the RF timer.
    rftimer_set_callback_by_id(rftimer_callback, RFTIMER_ID);
    rftimer_enable_interrupts();
    rftimer_enable_interrupts_by_id(RFTIMER_ID);

    // Configure the ADC.
    printf("Configuring the ADC.\n");
    adc_config(&g_adc_config);
    adc_enable_interrupt();

    analog_scan_chain_write();
    analog_scan_chain_load();

    crc_check();
    perform_calibration();

    GPO_control(6, 6, 6, 6);
    analog_scan_chain_write();
    analog_scan_chain_load();

    sensors_init(&g_sensors_config);
    while (true) {
        // Measure the sensors.
        printf("Measuring the sensors.\n");
        sensors_measurements_t sensor_measurements;
        sensors_measure(&sensor_measurements);

        // Print the measurement results.
        for (size_t i = 0; i < g_sensors_config.num_sensors; ++i) {
            switch (g_sensors_config.sensors[i]) {
                case SENSOR_TYPE_POTENTIOMETRIC: {
                    printf("Sensor %u: ADC output: %u\n", i,
                           sensor_measurements.measurements[i].adc_output);
                    break;
                }
                case SENSOR_TYPE_RESISTIVE: {
                    printf("Sensor %u: estimated time constant: %lld / %lld\n",
                           i,
                           sensor_measurements.measurements[i]
                               .time_constant.time_constant,
                           sensor_measurements.measurements[i]
                               .time_constant.scaling_factor);
                    break;
                }
                case SENSOR_TYPE_PH:
                case SENSOR_TYPE_INVALID:
                default: {
                    break;
                }
            }
        }

        // Wait for the next ADC read.
        for (size_t i = 0; i < NUM_CYCLES_BETWEEN_ADC_READS; ++i) {}
    }
}
