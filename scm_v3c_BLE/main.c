// ------------------------------------------------------------------------------------------
// This flag determines whether the demo code will continuously transmit or continuously receive
// 0 = Transmit
// 1 = Receive
// Both will occur on BLE channel 37 (2.402 GHz)
unsigned int tx_rx_flag = 0;
// ------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------
// This flag determines whether SCuM is sweeping all counters
// 0 = Not sweeping
// 1 = Sweeping
unsigned int sweeping_counters = 0;
// ------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------
// This flag determines whether SCuM should calibrate LC coarse and mid codes for channel 37
// 0 = Not calibrating
// 1 = Calibrating
unsigned int calibrate_LC = 1;
// ------------------------------------------------------------------------------------------

#include <stdio.h>
#include <time.h>
#include <rt_misc.h>
#include <stdlib.h>
#include "Memory_map.h"
#include "Int_Handlers.h"
#include "rf_global_vars.h"
#include "scm3C_hardware_interface.h"
#include "scm3_hardware_interface.h"
#include "bucket_o_functions.h"
#include <math.h>
#include "scum_radio_bsp.h"
#include "scm_ble_functions.h"

extern unsigned int current_lfsr;

extern char send_packet[127];
extern unsigned int ASC[38];

// Bootloader will insert length and pre-calculated CRC at these memory addresses	
#define crc_value         (*((unsigned int *) 0x0000FFFC))
#define code_length       (*((unsigned int *) 0x0000FFF8))

unsigned int LC_target;
unsigned int LC_code;

// HF_CLOCK tuning settings
unsigned int HF_CLOCK_fine = 17;
unsigned int HF_CLOCK_coarse = 3;

// RC 2MHz tuning settings
unsigned int RC2M_coarse = 21;
unsigned int RC2M_fine = 15;
unsigned int RC2M_superfine = 15;

// Receiver clock settings
unsigned int IF_clk_target = 1900000;
unsigned int IF_coarse = 11;
unsigned int IF_fine = 18;

unsigned int cal_iteration = 0;
unsigned int run_test_flag = 0;
unsigned int num_packets_to_test = 1;

unsigned short optical_cal_iteration = 0, optical_cal_finished = 0;

unsigned short doing_initial_packet_search;
unsigned short current_RF_channel;
unsigned short do_debug_print = 0;

unsigned int LC_sweep_code;
unsigned int LC_min_diff;

unsigned int coarse_code = 23;
unsigned int mid_code = 12;
	
//////////////////////////////////////////////////////////////////
// Temperature Function
//////////////////////////////////////////////////////////////////

double temp = 20;

void measure_temperature() {
	// RFCONTROLLER_REG__INT_CONFIG = 0x3FF;   // Enable all interrupts and pulses to radio timer
	// RFCONTROLLER_REG__ERROR_CONFIG = 0x1F;  // Enable all errors
	
	// Enable RF timer interrupt
	ISER = 0x0080;

	RFTIMER_REG__CONTROL = 0x7;
	RFTIMER_REG__MAX_COUNT = 0x0003D090;
	RFTIMER_REG__COMPARE6 = 0x0003D090;
	RFTIMER_REG__COMPARE6_CONTROL = 0x03;

	// Reset all counters
	ANALOG_CFG_REG__0 = 0x0000;

	// Enable all counters
	ANALOG_CFG_REG__0 = 0x3FFF;
}

void measure_counters() {
	// Enable static divider
	divProgram(480, 1, 1);
	
	// Turn on LO, DIV, PA, IF
	ANALOG_CFG_REG__10 = 0x78;
	
	// Turn off polyphase and disable mixer
	ANALOG_CFG_REG__16 = 0x6;
	
	// Enable RF timer interrupt
	ISER = 0x0080;

	RFTIMER_REG__CONTROL = 0x7;
	RFTIMER_REG__MAX_COUNT = 0x0000C350;
	RFTIMER_REG__COMPARE7 = 0x0000C350;
	RFTIMER_REG__COMPARE7_CONTROL = 0x03;

	// Reset all counters
	ANALOG_CFG_REG__0 = 0x0000;

	// Enable all counters
	ANALOG_CFG_REG__0 = 0x3FFF;
}

//////////////////////////////////////////////////////////////////
// Main Function
//////////////////////////////////////////////////////////////////

int main(void) {
	int t,t2;
	int c, m, f;
	unsigned int calc_crc;
	int fine;

	unsigned int rdata_lsb, rdata_msb, count_LC, count_32k, count_2M;
	
	unsigned char AdvA[6];
	
	printf("Initializing...");
	
	// Set up mote configuration
	initialize_mote_ble();
		
	// Check CRC
	printf("\n-------------------\n");
	printf("Validating program integrity..."); 
	
	calc_crc = crc32c(0x0000, code_length);

	if(calc_crc == crc_value){
		printf("CRC OK\n");
	}
	else{
		printf("\nProgramming Error - CRC DOES NOT MATCH - Halting Execution\n");
		while(1);
	}
	// Debug output
	//printf("\nCode length is %u bytes",code_length); 
	//printf("\nCRC calculated by SCM is: 0x%X",calc_crc);	
	
	printf("Calibrating frequencies...\n");
	
	// For initial calibration, turn on AUX, DIV, LO and IF(RX) or PA(TX)
	// Aux is inverted (0 = on)
	// Memory-mapped LDO control
	// ANALOG_CFG_REG__10 = AUX_EN | DIV_EN | PA_EN | IF_EN | LO_EN | PA_MUX | IF_MUX | LO_MUX
	// For MUX signals, '1' = FSM control, '0' = memory mapped control
	// For EN signals, '1' = turn on LDO
	

	// TX
	if(tx_rx_flag == 0){

		// Calibration counts for 100ms
		// Divide ratio is currently 480
		LC_code = 690; // Starting LC code

		// For TX, target LC freq = 2.402G - 0.25M = 2.40175 GHz
		LC_target = 250020;
		
		// Turn on LO, DIV, PA
		ANALOG_CFG_REG__10 = 0x68;
		
		// Turn off polyphase and disable mixer
		ANALOG_CFG_REG__16 = 0x6;
		
	}
	// RX
	else {
		// For RX, target LC freq = 2.402G - 2.5M = 2.3995 GHz
		LC_target = 250000; 
		
		// Turn on LO, DIV, IF
		ANALOG_CFG_REG__10 = 0x58;
		
		// Enable polyphase and mixers via memory-mapped I/O
		ANALOG_CFG_REG__16 = 0x1;
	}
	
	if (calibrate_LC) {
		LC_sweep_code = (21U << 10) | (0U << 5) | (15U); // start at coarse=21, mid=0, fine=15
		LC_min_diff = 1000000U;
		
		LC_FREQCHANGE((LC_sweep_code >> 10) & 0x1F,
		              (LC_sweep_code >> 5) & 0x1F,
		              LC_sweep_code & 0x1F);
	}
	
	// Enable optical SFD interrupt for optical calibration
	ISER = 0x0800;
		
	// Wait for optical cal to finish
	while(optical_cal_finished == 0);
	optical_cal_finished = 0;
	
	printf("Cal complete\n");
	
	// Disable static divider to save power
	divProgram(480,0,0);

	// Hard code LO value for TX - works on board #5; 2.402G - 250kHz
	if (tx_rx_flag == 0) {
		// LC_monotonic(680);
	}

	// Some more RX setup
	if (tx_rx_flag == 1) {
		// BLE RX only works using the 32-bit raw chips shift register receiver
		// So you must know 32-bits at the beginning of the packet to trigger on
		
		//	The contents of the packet this demo code is searching for
		//	blepkt[0] =	0x1D55ADF6;
		//	blepkt[1] =	0x45C7C50E;
		//	blepkt[2] =	0x2613C2AC;
		//	blepkt[3] =	0x9837B830;
		//	blepkt[4] =	0xA1C9E493;
		//	blepkt[5] =	0x75B7416C;
		//	blepkt[6] =	0xD12DB800;
		
		// The packet above includes [preamble Access_Address PDU] (I think...)
		// We want to search for the Access Address (AA) (this assumes we can know this value ahead of time)
		// The AA used in this packet is 0x8E89_BED6
		// But the endianness is flipped for sending so we are searching for 0x6B7D_9171
		
		// Set up the 32-bit value we are searching for
		ANALOG_CFG_REG__1 = 0x9171;	//lsbs
		ANALOG_CFG_REG__2 = 0x6B7D;	//msbs
		
		// The correlation threshold determines whether an interrupt gets thrown
		// For BLE we need 100% correct bits so we set the Hamming distance to 0 here
		acfg3_val = 0x60;
		ANALOG_CFG_REG__3 = acfg3_val;
		
		// Turn on the radio
		radio_rxEnable();
		
		// Hard code the frequency for now
		//LC_FREQCHANGE(21,10,16);
		// LC_FREQCHANGE(23,2,14);
		
		// Enable the raw chips startval interrupt
		// This interrupt will go off when 32-bit value is in the shift register
		// The ISR will then retrieve the packet data and determine if it was valid
		ISER = 0x0100;
	}
	
	// Enable UART interrupt
	ISER = 0x0001;
	
	AdvA[0] = 0x00;
	AdvA[1] = 0x02;
	AdvA[2] = 0x72;
	AdvA[3] = 0x32;
	AdvA[4] = 0x80;
	AdvA[5] = 0xC6;
	
	if (sweeping_counters) {
		while (1) {
			measure_counters();
			for (t = 0; t < 200000; ++t);
		}
	}
		
	while(1) {
			
		unsigned char packetBLE[64];
		
		measure_temperature();
		
		if (tx_rx_flag == 0) {
			
			// Create some BLE packet
			// gen_test_ble_packet(packetBLE);
			// gen_ble_packet(packetBLE, AdvA, 37, 32767U);
			
			for (fine = 0; fine < 31; ++fine) {
				
				// Load the packet into the arbitrary TX FIFO
				load_tx_arb_fifo(packetBLE);
				
				// Turn on LO and PA
				radio_txEnable();
				
				// Tune frequency
				LC_FREQCHANGE(coarse_code, mid_code, fine);
				
				// Generate new packet with LC tuning
				gen_ble_packet(packetBLE, AdvA, 37, ((coarse_code & 0x1F) << 10) | ((mid_code & 0x1F) << 5) | (fine & 0x1F));
				
				// Wait for frequency to settle
				for(t = 0; t < 5000; ++t);
				
				// Send the packet
				transmit_tx_arb_fifo();
							
				// Wait for transmission to finish
				// Don't know if there is some way to know when this has finished or just busy wait (?)
				for(t = 0; t < 20000; ++t);
				
				// Turn radio back off
				//radio_rfOff();
				
				printf("Transmitted on fine %d\n", fine);
			}
		}
		
		// Wait  - this sets packet transmission rate
		for(t = 0; t < 2000; ++t);

	}
}
