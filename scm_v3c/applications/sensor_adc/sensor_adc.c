#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../channel_cal/channel_cal.h"
#include "adc.h"
#include "gpio.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"
#include "sensor.h"
#include "sensor_resistive.h"
#include "sensors.h"
#include "tuning.h"

// Maximum number of sensors in the packet.
#define MAX_NUM_SMARTSTAKE_SENSORS 8

// RF timer ID.
#define RFTIMER_ID 7

// Number of for loop cycles between ADC reads.
// 700000 for loop cycles roughly correspond to 1 second.
#define NUM_CYCLES_BETWEEN_ADC_READS 300000

// Start coarse code for the sweep to find 802.15.4 channels.
#define START_COARSE_CODE 26

// End coarse code for the sweep to find 802.15.4 channels.
#define END_COARSE_CODE 26

// 802.15.4 channel on which to transmit the ADC data.
#define IEEE_802_15_4_TX_CHANNEL 26

// TX packet containing the ADC data.
typedef struct __attribute__((packed)) {
    // Sequence number.
    uint8_t sequence_number;

    // Channel.
    uint8_t channel;

    // Reserved.
    uint8_t reserved1;

    // Reserved.
    uint8_t reserved2;

    // Measurement output.
    uint32_t output[MAX_NUM_SMARTSTAKE_SENSORS];

    // Tuning code.
    tuning_code_t tuning_code;

    // Reserved.
    uint8_t reserved3;

    // CRC.
    uint16_t crc;
} smart_stake_tx_packet_t;

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

// TX tuning code for the ADC data.
static tuning_code_t g_smart_stake_tx_tuning_code;

// TX sequence number for the ADC data.
static uint8_t g_smart_stake_tx_sequence_number = 0;

// TX packet containing the ADC data.
static smart_stake_tx_packet_t g_smart_stake_tx_packet;

// Callback for the RF timer.
static void rftimer_callback(void) { sensor_resistive_rftimer_callback(); }

int main(void) {
    initialize_mote();

    // Initialize the channel calibration.
    printf("Initializing channel calibration.\n");
    if (!channel_cal_init(START_COARSE_CODE, END_COARSE_CODE)) {
        return EXIT_FAILURE;
    }

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

    printf("Running channel calibration.\n");
    if (!channel_cal_run()) {
        return EXIT_FAILURE;
    }

    if (!channel_cal_get_tx_tuning_code(IEEE_802_15_4_TX_CHANNEL,
                                        &g_smart_stake_tx_tuning_code)) {
        printf("No TX tuning code found for channel %u.\n",
               IEEE_802_15_4_TX_CHANNEL);
        return EXIT_FAILURE;
    }
    printf("Transmitting on channel %u: (%u, %u, %u).\n",
           IEEE_802_15_4_TX_CHANNEL, g_smart_stake_tx_tuning_code.coarse,
           g_smart_stake_tx_tuning_code.mid, g_smart_stake_tx_tuning_code.fine);

    // Configure the RF timer.
    rftimer_set_callback_by_id(rftimer_callback, RFTIMER_ID);
    rftimer_enable_interrupts();
    rftimer_enable_interrupts_by_id(RFTIMER_ID);

    sensors_init(&g_sensors_config);
    while (true) {
        // Measure the sensors.
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

        // Transmit the measurement results.
        memset(&g_smart_stake_tx_packet, 0, sizeof(smart_stake_tx_packet_t));
        g_smart_stake_tx_packet.sequence_number =
            g_smart_stake_tx_sequence_number;
        g_smart_stake_tx_packet.channel = IEEE_802_15_4_TX_CHANNEL;
        g_smart_stake_tx_packet.tuning_code = g_smart_stake_tx_tuning_code;
        for (size_t i = 0; i < g_sensors_config.num_sensors; ++i) {
            switch (g_sensors_config.sensors[i]) {
                case SENSOR_TYPE_POTENTIOMETRIC: {
                    g_smart_stake_tx_packet.output[i] =
                        sensor_measurements.measurements[i].adc_output;
                    break;
                }
                case SENSOR_TYPE_RESISTIVE: {
                    g_smart_stake_tx_packet.output[i] =
                        sensor_measurements.measurements[i]
                            .time_constant.time_constant;
                    break;
                }
                case SENSOR_TYPE_PH:
                case SENSOR_TYPE_INVALID:
                default: {
                    break;
                }
            }
        }

        radio_rfOn();
        tuning_tune_radio(&g_smart_stake_tx_tuning_code);
        send_packet(&g_smart_stake_tx_packet, sizeof(smart_stake_tx_packet_t));
        radio_rfOff();

        ++g_smart_stake_tx_sequence_number;

        // Wait for the next ADC read.
        for (size_t i = 0; i < NUM_CYCLES_BETWEEN_ADC_READS; ++i) {}
    }
}
