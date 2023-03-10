// This application finds the SCuM's tuning codes for some 802.15.4 channels
// with the help of an OpenMote and then transmits ADC data over an 802.15.4
// channel to the OpenMote.

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../channel_cal/channel_cal.h"
#include "adc.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "scm3c_hw_interface.h"
#include "tuning.h"

// Start coarse code for the sweep to find 802.15.4 channels.
#define START_COARSE_CODE 23

// End coarse code for the sweep to find 802.15.4 channels.
#define END_COARSE_CODE 24

// 802.15.4 channel on which to transmit the ADC data.
#define IEEE_802_15_4_TX_CHANNEL 17

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

    // ADC output.
    uint16_t adc_output;

    // Tuning code.
    tuning_code_t tuning_code;

    // Reserved.
    uint8_t reserved3;

    // CRC.
    uint16_t crc;
} smart_stake_tx_packet_t;

// ADC configuration.
static const adc_config_t g_smart_stake_adc_config = {
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

// TX tuning code for the ADC data.
static tuning_code_t g_smart_stake_tx_tuning_code = {
    .coarse = 23,
    .mid = 28,
    .fine = 18,
};

// TX sequence number for the ADC data.
static uint8_t g_smart_stake_tx_sequence_number = 0;

// TX packet containing the ADC data.
static smart_stake_tx_packet_t g_smart_stake_tx_packet;

// Trigger an ADC read.
static inline bool smart_stake_trigger_adc_read(void) {
    // Trigger an ADC read.
    printf("Triggering ADC.\n");
    adc_trigger();
    while (!g_adc_output.valid) {
    }
    if (!g_adc_output.valid) {
        printf("ADC output should be valid.\n");
        return false;
    }
    return true;
}

int main(void) {
    initialize_mote();

    // Initialize the channel calibration.
    printf("Initializing channel calibration.\n");
    if (!channel_cal_init(START_COARSE_CODE, END_COARSE_CODE)) {
        return EXIT_FAILURE;
    }

    // Configure the ADC.
    printf("Configuring the ADC.\n");
    adc_config(&g_smart_stake_adc_config);
    adc_enable_interrupt();

    analog_scan_chain_write();
    analog_scan_chain_load();

    crc_check();
    perform_calibration();

    printf("Transmitting on channel %u: (%u, %u, %u).\n",
           IEEE_802_15_4_TX_CHANNEL, g_smart_stake_tx_tuning_code.coarse,
           g_smart_stake_tx_tuning_code.mid, g_smart_stake_tx_tuning_code.fine);
    rftimer_enable_interrupts();

    printf("Measuring the ADC.\n");
    while (true) {
        // Wait a bit.
        for (uint32_t i = 0; i < 700000; ++i) {
        }

        if (!smart_stake_trigger_adc_read()) {
            continue;
        }
        printf("ADC output: %u\n", g_adc_output.data);

        memset(&g_smart_stake_tx_packet, 0, sizeof(smart_stake_tx_packet_t));
        g_smart_stake_tx_packet.sequence_number =
            g_smart_stake_tx_sequence_number;
        g_smart_stake_tx_packet.channel = IEEE_802_15_4_TX_CHANNEL;
        g_smart_stake_tx_packet.adc_output = g_adc_output.data;
        g_smart_stake_tx_packet.tuning_code = g_smart_stake_tx_tuning_code;

        radio_rfOn();
        tuning_tune_radio(&g_smart_stake_tx_tuning_code);
        send_packet(&g_smart_stake_tx_packet, sizeof(smart_stake_tx_packet_t));
        radio_rfOff();

        ++g_smart_stake_tx_sequence_number;
    }

    return EXIT_SUCCESS;
}
