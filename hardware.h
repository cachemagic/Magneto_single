////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//						HARDWARE.H
//
//		This is the hardware description header file.  This file describes all 
//  processor dependant functions and the pinout descriptions of the processor 
//  used in this application
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifndef HARDWARE_H
#define HARDWARE_H

#include "msp430fr5739.h"


#define SERIAL_BUFFER_SIZE			16		// Serial buffer size and MINIMUM size of get data buffer

////////////////////////////////////////////////////////////////////////////////
//  PIN_DEFINITIONS

// PORT 1
#define PIN_ADC_CURRENT		BIT0		// P1.0 input 
#define PIN_ADC_VOLTAGE		BIT1		// P1.1 input
#define PIN_SWITCH_STATE	BIT2		// P1.2 input 
#define PIN_SOLONOID1		BIT3
#define PIN_SOLONOID2		BIT4
#define PIN_SOLONOID3		BIT5
#define PIN_I2C_SDA			BIT6		// P1.6 output alternate use 01
#define PIN_I2C_SCL			BIT7		// P1.7 output alternate use 01

#define INIT_P1SEL0			(PIN_ADC_CURRENT | PIN_ADC_VOLTAGE)
#define INIT_P1SEL1			(PIN_ADC_CURRENT | PIN_ADC_VOLTAGE | PIN_I2C_SDA | PIN_I2C_SCL)
#define INIT_P1DIR			(PIN_I2C_SDA | PIN_I2C_SCL | PIN_SOLONOID1 | PIN_SOLONOID2 | PIN_SOLONOID3)		// 		


//  PORT2
#define PIN_TX					BIT0		// P2.0
#define PIN_RX					BIT1		// P2.1
#define PIN_SPEAKER			BIT2		// P2.3
#define PIN_TX_USB			BIT5		// P2.5
#define PIN_RX_USB			BIT6		// P2.6

#define INIT_P2SEL0 			(0x0000)					// Set port 2 pins to enable RX & TX
#define INIT_P2SEL1 			(PIN_RX | PIN_TX | PIN_RX_USB | PIN_TX_USB)		// Set port 2 pins to enable RX & TX
#define INIT_P2DIR			(PIN_SPEAKER)	
	

// PORT 3
#define PIN_DEVICE_POWER	BIT0 		// BIT0
#define PIN_C2_CABLE			BIT1 		// BIT1
#define PIN_C3_CABLE			BIT2 		// BIT2
//#define PIN_DEVICE_3VPOWER	BIT1 		// BIT1
//#define PIN_DEVICE_5VPOWER	BIT2 		// BIT2
#define PIN_STATUS_LED		BIT4		// P3.2 output
#define PIN_CABLE_CONNECT	BIT3
#define INIT_P3DIR			(PIN_DEVICE_POWER)// | PIN_STATUS_LED) // | PIN_DEVICE_3VPOWER | PIN_DEVICE_5VPOWER 
#define INIT_P3REN			(PIN_C2_CABLE | PIN_C3_CABLE) 		// enable pulldown
#define INIT_P3OUT			(PIN_C2_CABLE | PIN_C3_CABLE) 		// pulldown 

#define ROTARY_SWITCH_2C	(P3IN & PIN_C2_CABLE)
#define ROTARY_SWITCH_3C	(P3IN & PIN_C3_CABLE)


// PORT 4
#define PIN_GREEN_LED		BIT0		// P4.0 output
#define PIN_RED_LED			BIT1		// P4.1 output
#define INIT_P4DIR			(PIN_GREEN_LED | PIN_RED_LED)

// PORTJ	JTAG 
#define PIN_TEST1				BIT0		// PJ.0
#define PIN_TEST2				BIT1		// PJ.1
#define PIN_TEST3				BIT2		// PJ.2
#define PIN_TEST4				BIT3		// PJ.3

#define INIT_PJDIR			(PIN_TEST1 | PIN_TEST2 | PIN_TEST3 | PIN_TEST4)


// SET, CLEAR, and TOGGLE
#define ON 						1
#define OFF 					0
#define SET_GREEN_LED(X)	((X)?(P4OUT |= PIN_GREEN_LED):(P4OUT &= ~PIN_GREEN_LED))
#define SET_RED_LED(X)		((X)?(P4OUT |= PIN_RED_LED):(P4OUT &= ~PIN_RED_LED))
#define SET_STATUS_LED(X)	((X)?(P3OUT |= PIN_STATUS_LED):(P3OUT &= ~PIN_STATUS_LED))
#define TOGGLE_STATUS_LED	(P3OUT ^= PIN_STATUS_LED)
#define TOGGLE_RED_LED		(P4OUT ^= PIN_RED_LED)
#define TOGGLE_GREEN_LED	(P4OUT ^= PIN_GREEN_LED)
#define SET_DEVICE_POWER(X)	((X)?(P3OUT |= PIN_DEVICE_POWER):(P3OUT &= ~PIN_DEVICE_POWER))
//#define SET_3V_POWER(X)			((X)?(P3OUT |= PIN_DEVICE_3VPOWER):(P3OUT &= ~PIN_DEVICE_3VPOWER))
//#define SET_5V_POWER(X)			((X)?(P3OUT |= PIN_DEVICE_5VPOWER):(P3OUT &= ~PIN_DEVICE_5VPOWER))

//#define SET_DEVICE_POWER(A,X)	( ((X) && (A <= 8)) ? (P3OUT |= 0x01 << (A-1)) : (P3OUT = 0x00) )
//#define ACTIVATE_SOLONOID(X)	( (X < 3) ? (P1OUT |= (PIN_SOLONOID1 << X)) : ( P1OUT &= ~(PIN_SOLONOID1 | PIN_SOLONOID1 | PIN_SOLONOID3) ) )
//#define DEACTIVATE_SOLONOIDS	( P1OUT &= ~(PIN_SOLONOID1 | PIN_SOLONOID1 | PIN_SOLONOID3) )

#define START_BUTTON_PRESSED		(P1IN & PIN_SWITCH_STATE)			// Button signal pulled hi


////////////////////////////////////////////////////////////////////////////////
// TIMERS AND COUNTERS
#define MS_COUNT								0x00A0	// number of while wait cycles to equal 1ms
#define TIMEOUT_COUNT_100MS				0x4000	// Standard 100ms raw while cycle count
#define TIMEOUT1_COUNT_50MS				50
#define TIMEOUT1_COUNT_150MS				150		// 150ms TimeoutCounter with 3ms delay
#define TIMEOUT3_COUNT_150MS				0x0030	// 150ms TimeoutCounter with 3ms delay

#define DEBOUNCE_COUNT						0x08		//  Delay count to change state

				

/////////////////////////////////////////////////////////////////////////////// 
// EXPORT VARIABLES, DEFINES, AND MACROS
extern unsigned char g_aDataBuffer[SERIAL_BUFFER_SIZE];		// General purpose buffer - do not use in interrupt
extern unsigned short g_aCoagualationSetPoints[6][3];

#define SAMPLE_FREQUENCY 					14			//  K Samples per Seconds * 10:  27 = 2.7KSps
extern unsigned long g_ulSampleSum;
extern unsigned short g_uSampleCount;
extern unsigned short g_ADCVoltage;
 #define POWER_ON_VOLTAGE					0x03B0	//  If below this value then cable is turned on
 #define PULSE_ON_VOLTAGE					0x0240   //  0x0300	//  If below this value then Fet is on.
 #define SHORT_CIRCUIT_VOLTAGE			0x0008	//  The Current Amplifier will saturate if a shorted cable is inserted 	

extern unsigned short g_ADCCurrent;
 #define POWER_ON_CURRENT					0x0008	//  If above this value then cable is inserted (about 130uA)
 #define PULSE_ON_CURRENT					0x0080	//  If above this value then Fet is on.
 #define SHORT_CIRCUIT_CURRENT			0x03F8	//  The Current Amplifier will saturate if a shorted cable is inserted 	
 

extern unsigned short g_ADCFlags;
 #define ADCFLG_CHANNEL1					BIT0		
 #define ADCFLG_SHORT_CIRCUIT				BIT1	
 #define ADCFLG_COLLECT_AVERAGE			BIT2			


// Formula for Current and Resistance:
//  Iadc = (ADC * Vrange) / (Rshunt * Gain * 1024)   Gain of amp is 100, Rshunt is 2 Ohms, and Vrange is 3.3V, multiply by 1,000,000 for uA.  16.5mA max, 16.11uA min
//  R = Vsupply / Iadc  = (VSupply * Rshunt * Gain * 1024) / (ADC * Vrange)	Supply is 1.2V and 9V 
#define AMPLIFIER_RSHUNT					2			// Shunt resistor for current sensor
#define AMPLIFIER_GAIN						100			// 83.33		// Gain of precision current sense amplifier
#define V_DIVIDER_GAIN						3			// Inverse Gain of voltage divider
#define MAX_ADC								0x03FF	// Max ADC value for 10 bit ADC
#define VRANGE_ADC							15
#define VSUPPLY_9V							9
#define VSUPPLY_5V							5
#define VSUPPLY_3_3V							3.3
#define VSUPPLY_1_2V							1.2
//#define VSUPPLY_VRANGE_CONSTANT_MR		((9L * 1024 * 100000) / 33726)// 9 Volt supply
//#define VSUPPLY_VRANGE_CONSTANT			((9L * 1024 * 10) / 33)		// 9 Volt supply

#define INIT_RES_GAIN						AMPLIFIER_RSHUNT * AMPLIFIER_GAIN * 100	// g_uResGain without calibration: * 100 for decimal precision
#define MIN_RESISTANCE_9V					((VSUPPLY_9V * AMPLIFIER_RSHUNT * AMPLIFIER_GAIN * MAX_ADC) / (VRANGE_ADC * SHORT_CIRCUIT_CURRENT))	// Minimum resistance that can be calculated when Current ADC is 0x3F8
#define MAX_RESISTANCE_9V					((VSUPPLY_9V * AMPLIFIER_RSHUNT * AMPLIFIER_GAIN * MAX_ADC) / (VRANGE_ADC))			// Maximum resistance that can be calculated when Current ADC is 0x001
#define INIT_CALIBRATION_RESISTANCE		995
#define MOSFET_RDS							3			// Mosfet resistance at 3V Vgs and 10mA
#define QUIESCENT_RESISTANCE				19948	
#define QUIESCENT_CURRENT					451		// Target qCurrenet in uA
#define QUIESCENT_ADC_300UA				((3L * 1024 * INIT_RES_GAIN) / (VRANGE_ADC * 100000))
#define QUIESCENT_ADC_200UA				((2L * 1024 * INIT_RES_GAIN) / (VRANGE_ADC * 100000))
#define QUIESCENT_ADC_100UA				((1L * 1024 * INIT_RES_GAIN) / (VRANGE_ADC * 100000))


#define NULL 				0x0000
#define LOBYTE(x)			((x) & 0x00FF)
#define HIBYTE(x)			(((x) >> 8) & 0x00FF)
#define MIN(x,y)			((x > y) ? y : x)
#define MAX(x,y)			((x > y) ? x : y)
#define ABS_DIFF(x, y)	((x > y) ? (x - y) : (y - x))
#define HEX_NIBBLE_TO_ASCII(x) ((x <= 0x09) ? (x += 0x30) : (x += 0x37))

/////////////////////////////////////////////////////////////////////////////// 
// EXPORT_FUNCTIONS
void delay_ms( unsigned short ); 				// Delays the number of miliseconds
void delay_seconds( unsigned short ); 			// Delays the number of seconds


#endif  //  HARDWARE_H