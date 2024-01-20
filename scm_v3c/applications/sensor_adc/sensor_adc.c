#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc.h"
#include "gpio.h"
#include "memory_map.h"
#include "optical.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "sensor_resistive.h"

// Number of for loop cycles between ADC reads.
// 700000 for loop cycles roughly correspond to 1 second.
#define NUM_CYCLES_BETWEEN_ADC_READS 700000

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

// Resistive sensor configuration.
static const sensor_resistive_config_t g_sensor_resistive_config = {
    .rftimer_id = 7,
    .gpio_excitation = GPIO_0,
    .sampling_period_ms = 10,
    .sensor_capacitor_config =
        {
            .num_capacitors = 1,
            .gpios = {GPIO_1},
            .num_capacitor_masks = 1,
            .capacitor_masks = {CAPACITOR_MASK_1},
        },
};

// Callback for the RF timer.
static void rftimer_callback(void) { sensor_resistive_rftimer_callback(); }

int main(void) {
    initialize_mote();

    // Configure the RF timer.
    rftimer_set_callback_by_id(rftimer_callback,
                               g_sensor_resistive_config.rftimer_id);
    rftimer_enable_interrupts();
    rftimer_enable_interrupts_by_id(g_sensor_resistive_config.rftimer_id);

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

    // Initialize measuring a resistive sensor.
    sensor_resistive_init(&g_sensor_resistive_config);

    while (true) {
        printf("Measuring the resistive sensor.\n");
        sensor_resistive_time_constant_t time_constant;
        sensor_resistive_measure(&time_constant);
        printf("Estimated time constant: %lld / %lld\n",
               time_constant.time_constant, time_constant.scaling_factor);

        // Wait for the next ADC read.
        for (size_t i = 0; i < NUM_CYCLES_BETWEEN_ADC_READS; ++i) {}
    }
}
