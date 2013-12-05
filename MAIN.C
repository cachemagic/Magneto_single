////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//							MAIN.C 
//
//		Arthrocare Magneto test fixture main processor program
//
//		This program controls the test process of the single position ArthroCare 
//		EULS test fixture.  This control process involves:
//		*  Respond to any communication from a PC to report test results or initiate
//				tests or functions.
//		*  Monitor start button to initiate testing.
//		*	Control the test sequence which includes:
//				*  Determine whether the DUT is shorted by measuring the current at 1.2V.
//				*	Record and report quescent current at 1.2V
//				*  If not shorted, power up unit at 9V and read the current pulses to determine the use count.
//				*  During pulses record max and min current for determining quiescent current and resistance.
//				*  Display the overall results with the LCD and LEDs and report to PC if connected.
//
//		The Modules used by this program are:
//		*  MAIN.C - Main program loop with the higher level control and test functions.
//		*  UART.C - Handles all of the register and interrupt level UART communication.
//		*  HARDWARE.C - Handles all of the processor dependant functions.
//		*  LCD.C - Handles all LCD Display functions.
//
//
//		Author:  David Smith: dsmith@digis.net or (801)420-8444 for questions or service.
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include "hardware.h"		// Pin, processor, and run state definitions
#include "uart.h"
#include "lcd.h"				// Interface to LCD display

#define FIXTURE_TYPE			0x21		// Magneto Fixture
#define FIRMWARE_VERSION	0x02		// Version 2 by Russell Anderson
#define FIRMWARE_REV			0x04

////////////////////////////////////////////////////////////////////////////////
//  GLOBAL VARIABLES AND DEFINES
unsigned short g_uRunFlags;
#define RFLG_CABLE_DETECTED				BIT1
#define RFLG_START_BUTTON_PRESSED		BIT2
#define RFLG_PROCESS_START_BUTTON		BIT3
#define RFLG_STATE_PULSE_LOW				BIT4
#define RFLG_BUTTON_INITIATED				BIT5
#define RFLG_DISPLAY_RESULTS				BIT6
#define RFLG_CALIBRATE_CURRENT			BIT7
#define RFLG_PC_CONNECTED					BIT8
#define RFLG_LEMO_POWER						BIT9
#define RFLG_DEBUG                     BITA
#define RFLG_PRODUCTION                BITB

#define FRAM_DATA_ADDRESS						0x1800	// Store persitent data in 256 byte information memory 0x1800 to 0x18FF also 0x1900 to 0x19FF
unsigned short g_uPassCount;					// Number of perfomed tests that have passed
unsigned short g_uFailCount;					// Number of perfomed tests that have failed
unsigned short g_uPowerupCount;				// Number of times unit has powered up
unsigned short g_uErrorCount;					// Generic Error Counter
unsigned short g_uResGain; 					// Calibrated Resistor Gain value for current calculation
unsigned short g_uCalibrationResistance;	// Precision resistor value used to calculate g_uResGain
unsigned short g_uSetPointIndex;				// Coagulation SetPoint Index

unsigned short g_uMaxSample;						
unsigned long g_ulMaxCurrent;
unsigned short g_uMaxCount;
unsigned short	g_uMinSample;
unsigned long g_ulMinCurrent;
unsigned short g_uMinCount;
unsigned long g_ulMaxVoltage;
unsigned short g_uMaxVoltageCount;
unsigned long g_ulMinVoltage;
unsigned short g_uMinVoltageCount;
unsigned long g_ulHMaxVoltage;
unsigned long g_uHMaxVoltageCount;
unsigned short g_uHActiveVoltage;					// Voltage while Mosfet Pulsed (High Voltage)


unsigned char g_ucErrorBlinkCount;					// Sets blink rate of led
unsigned char g_ucLivesUsed;							// Number of lives used determined from pulses
unsigned short g_uResistance;							// Calculated Resistance with Mosfet pulsed
unsigned short g_uQResistance;						// Calculated Resistance with Mosfet not pulsed
unsigned short g_uIDDQCurrent;						// Quiescent current while mosfet not pulsed
unsigned short g_uActiveCurrent;						// Active Current while Mosfet pulsed
unsigned short g_uActiveVoltage;						// Voltage while Mosfet Pulsed
unsigned short g_uMeanResistance;					// Calculated Resistance with Active and Quiescent Currents

unsigned short g_uLEDCount;
unsigned char g_uLCDCount;
unsigned char g_ucScreen;


////////////////////////////////////////////////////////////////////////////////
//  FUNCTION DEFINITIONS
void Initialize(void);
void CheckButtons(void);
void DetectCable(void);
void ProcessLED(void);
void ProcessLCD(void);

void ExecuteMagnetoTest(void);							// Runs the Magneto test.
unsigned char CountLivesUsed(void);						// Determines the number of lives used by magneto, also records quiescent current.
void PulseDevicePower(unsigned short uSeconds );	// Pulses DUT power at ucAddress for uSeconds.
void CalibrateCurrentSensor(void);						// Calculates the value for g_uResGain with precision resistor.
void ProcessUARTPacket(void);
void CalculateCurrent( void );
void DisplayCalibrationCurrent(void);
void SetCoagulationPoint(void);
void CheckCalibrationCable(void);

void ResetADC(void);
void ReadFRAMData(void);
void WriteFRAMData(void);

void DisplayDecimalCurrent(unsigned short uValue);
void DisplaySplashScreen(void);
void DisplayInstructions(void);
void DisplayResistance(void);
void DisplayCurrent(void);
void DisplayTestResults(void);
void DisplaySetPoint(void);


////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Main program loop 
////////////////////////////////////////////////////////////////////////////////
void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;					// Stop WDT
	
	Initialize();
	SET_DEVICE_POWER( OFF );					// Turn off device power

	g_uRunFlags = 0x00;							// Reset state machine
//   g_uRunFlags |= RFLG_DEBUG;     // Debug Mode uses first line of LCD
	g_ucErrorBlinkCount = 1;	
	g_uLEDCount = 1;
	g_uLCDCount = 1;
	g_ucScreen = 0;
	delay_ms( 100 );
	
	InitializeLcd();
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	
	ReadFRAMData();								// Load FRAM variables
	g_uPowerupCount++;							// Increment Power up counter
	
	CheckCalibrationCable();
 
	for (;;)		// Begin main polling loop
	{	// Main polling loop runs every 1ms 	
		delay_ms( 1 );
		
		CheckButtons();
		DetectCable();	

		if ( g_ucUARTFlags & SFLG_RX_DATA )	//  UART DATA RECEIVED
		{
			ProcessUARTPacket();
		}
			
		if ( (g_uRunFlags & RFLG_PROCESS_START_BUTTON) )
		{	// Execute test or calibration
			g_uRunFlags &= ~RFLG_PROCESS_START_BUTTON;
			
			ADC10CTL0 &= ~ADC10ENC;					// Stop Sampling and conversion	
			g_uRunFlags &= ~RFLG_LEMO_POWER;
			SET_DEVICE_POWER( OFF );	

			if ( g_uRunFlags & RFLG_CALIBRATE_CURRENT )
			{
				CalibrateCurrentSensor();
			}
			else 
			{
				ExecuteMagnetoTest();
			}
			g_uRunFlags |= RFLG_DISPLAY_RESULTS;
			g_ucScreen = 0;	// Set to immediately update screen with results
			g_uLEDCount = 1;
			g_uLCDCount = 1;
		}	
		
		if ( !(--g_uLEDCount) ) // Old code to blink LED -- not needed (?)
		{	// Timer for feedback LED
			g_uLEDCount = TIMEOUT1_COUNT_150MS;
			ProcessLED();

			if ( !(--g_uLCDCount) )
			{	// Update LCD Screen
				ADC10CTL0 &= ~ADC10ENC;						// Stop Sampling and conversion	
				g_uRunFlags &= ~RFLG_LEMO_POWER;
				SET_DEVICE_POWER( OFF );	

				g_uLCDCount = 15;
				RefreshLCD(g_uRunFlags & RFLG_DEBUG);
				if ( g_uRunFlags & RFLG_CALIBRATE_CURRENT )
					DisplayCalibrationCurrent();
				else 
					ProcessLCD();
			}
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
void Initialize(void)
{
	// Configure MCLK for 8MHz operation 
	CSCTL0_H = 0xA5;
	CSCTL1 |= DCOFSEL0 + DCOFSEL1;				// Set max. DCO setting = 8MHz
	CSCTL2 = SELA_3 + SELS_3 + SELM_3;			// set ACLK = SMCLK = MCLK = DCO
	CSCTL3 = DIVA_3 + DIVS_3 + DIVM_3;			// set all dividers to 1MHz x/8

	//	Configure UART 0
	InitUART();
	
	// Setup Port Pins		These are defined in the hardware.h file
	P1SEL0 = INIT_P1SEL0;
	P1SEL1 = INIT_P1SEL1;
	P1DIR = INIT_P1DIR;
	
	P2SEL0 = INIT_P2SEL0;			// Set port 2 pins to enable RX & TX
	P2SEL1 = INIT_P2SEL1;			// Set port 2 pins to enable RX & TX
	P2DIR = INIT_P2DIR;
	
	// Ports 3, 4 and J
	P3SEL0 = 0x00;
	P3SEL1 = 0x00;
	P3DIR = INIT_P3DIR;
	P3REN = INIT_P3REN;		
	P3OUT = 0x00;	// Pulldown

	P4DIR = INIT_P4DIR;
	PJDIR = INIT_PJDIR;

	
	// Setup Voltage Reference
	REFCTL0 = REFTCOFF | REFON;// | REFVSEL0;			// V reference on, 1.5V, Temp sensor off   2.5V = REFVSEL1,  2.0V = REFVSEL0, 1.5 = no REFVSEL
		
	// Setup ADC 10
	ADC10CTL0 = ADC10SHT_4 | ADC10MSC | ADC10ON;									// 64 cycle sample and hold time, multiple samples, ADC is On  
	ADC10CTL1 = ADC10SHP | ADC10SHS_0 | ADC10DIV_4 | ADC10SSEL_1 | ADC10CONSEQ_3;	// ACLK / 8, repeat sequence of channels mode
	ADC10CTL2 = ADC10RES | ADC10SR;													// 10 bit resolution, reduced sample rate
	ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_1;										// Vref is VREF and GND, single channel Pin #3
	ADC10IE |= ADC10IE0;																	// Enable sample interrupt to set g_ADCSample
	g_ADCFlags = 0;
		
	__bis_SR_register( GIE );	// Enable interrupt
}


////////////////////////////////////////////////////////////////////////////////
//  Reset all ADC variables
//------------------------------------------------------------------------------
void ResetADC( void )
{
	// Begin counting blinks until timeout is reached or three blinks are counted
	g_ulMinCurrent = 0;
	g_uMinSample = 0;
	g_uMinCount = 0;
	
	g_ulMaxCurrent = 0;
	g_uMaxSample = 0;
	g_uMaxCount = 0;
	
	g_ulMaxVoltage = 0;
	g_uMaxVoltageCount = 0;
	
	g_ulMinVoltage = 0;
	g_uMinVoltageCount = 0;
	
	g_ulSampleSum = 0;
	g_uSampleCount = 0;
	
	g_uSetPointIndex = 6;
   
	g_ulHMaxVoltage = 0;   
   g_uHMaxVoltageCount = 0;
}


////////////////////////////////////////////////////////////////////////////////
// Execute Test on Magneto cable
//------------------------------------------------------------------------------
void ExecuteMagnetoTest(void)
{
	ADC10CTL0 |= ADC10ENC + ADC10SC;			// Start Sampling and conversion
	ResetADC();										// Resets all Min Max Counts
	
	g_ucLivesUsed = CountLivesUsed();						
	g_ucErrorBlinkCount = g_ucLivesUsed;
			
	if ( g_uMaxCount ) g_ulMaxCurrent = g_ulMaxCurrent / g_uMaxCount;
	if ( g_uMaxVoltageCount ) g_ulMaxVoltage = g_ulMaxVoltage / g_uMaxVoltageCount;
	if ( g_uMinCount ) g_ulMinCurrent = g_ulMinCurrent / g_uMinCount;
	if ( g_uMinVoltageCount ) g_ulMinVoltage = g_ulMinVoltage / g_uMinVoltageCount;
	ADC10CTL0 &= ~ADC10ENC;						// Stop Sampling and conversion
	
	CalculateCurrent();
	SetCoagulationPoint();	// Set the coag index and g_uQResistance as optimal R

	if ( g_ucLivesUsed || (g_uSetPointIndex >= 6) 
		 || ((ROTARY_SWITCH_2C) && (g_uSetPointIndex != 5)) || ((ROTARY_SWITCH_3C) && (g_uSetPointIndex != 2)) )
	{
		SET_RED_LED( ON );
		SET_GREEN_LED( OFF );
		g_uFailCount++;
		g_uRunFlags |= RFLG_BUTTON_INITIATED;		// Forces a button press to reset
	}
	else
	{
		SET_RED_LED( OFF );
		SET_GREEN_LED( ON );
		g_uPassCount++;
	}
	WriteFRAMData();
}


////////////////////////////////////////////////////////////////////////////////
// Calculates current and resistance
// Formula for Current and Resistance:
//  Vadc = g_AdcVoltage * Vrange * DivGain / 1024:	Gain of divider is 3, Vrange is 1.5V
//  Iadc = (ADC * Vrange) / (Rshunt * Gain * 1024):  Gain of amp is 100, Rshunt is 2 Ohms, and Vrange is 3.3V, multiply by 1,000,000 for uAmps.
//  R = Vsupply / Iadc  = (VSupply * Rshunt * Gain * 1024) / (ADC * Vrange)	Supply is 1.2V and 9V 
//  1/Rr = 1/Ra - 1/Rq 		Ra is resistance with Active Current  Rq is resistance with IDDQ current 
//------------------------------------------------------------------------------
void CalculateCurrent( void )
{
	g_uActiveVoltage = ((g_ulMaxVoltage * 4500) >> 10);									// Voltage in mV 
	g_uActiveCurrent = ((((g_ulMaxCurrent * 1500000) / g_uResGain) * 100) >> 10);	// Current in micro Amps with 1.5V ADC range, g_uResGain * 100 for precision, bit shift 10 is same as divide by 1024	
	g_uIDDQCurrent = ((((g_ulMinCurrent * 1500000) / g_uResGain) * 100) >> 10);	// Current in micro Amps
	g_uResistance = ((3L * g_ulMaxVoltage * g_uResGain) / (g_ulMaxCurrent * 10));	// Resistance with a high current pulse and Divider gain of 3
	g_uQResistance = ((3L * g_ulMaxVoltage * g_uResGain) / ((g_ulMaxCurrent - g_ulMinCurrent) * 10));//  - QUIESCENT_ADC_100UA	
	g_uMeanResistance =  g_uResistance;
   
   g_uHActiveVoltage=((g_ulHMaxVoltage*682) >> 10);									// Voltage in mV ??

	g_uResistance = ((g_uResistance % 10) >= 5) ? (g_uResistance / 10) + 1 : (g_uResistance / 10); // Round resistor value
	g_uQResistance = ((g_uQResistance % 10) >= 5) ? (g_uQResistance / 10) + 1 : (g_uQResistance / 10); // Round resistor value
	g_uMeanResistance = ((g_uMeanResistance % 10) >= 5) ? (g_uMeanResistance / 10) + 1 : (g_uMeanResistance / 10); // Round resistor value
}


////////////////////////////////////////////////////////////////////////////////
//  	Powers up the DUT at location ucAddress just long enough to sense the current 
//  pulses (indicating the lives used) and returns this count.  This function also 
//  keeps track of the quiescent current as well as the pulse current for determining
//  the resistance between the power and ground pins of the DUT.
//------------------------------------------------------------------------------
unsigned char CountLivesUsed()
{	
	unsigned short uTimeoutCount = 500;	// 500 ms on Pulse
	unsigned char uDebounceCount = 3;	// 1 low adc readings before counting pulse
	unsigned char ucLastTransition = 0;	// Keeps track of the time between pulses
	unsigned short uPulseCount = 0;		// Initialize to 0
		
	g_uRunFlags &= ~RFLG_STATE_PULSE_LOW;	// Initialize in On to detect the low start bit	
	SET_GREEN_LED( ON );							// Feedback Led
	SET_DEVICE_POWER( ON );						// Turn on device power
	g_ADCFlags |= ADCFLG_COLLECT_AVERAGE;	// Set for more low pulse readings

	// Begin counting blinks until timeout is reached or three blinks are counted
	while ( uTimeoutCount )		
	{	// Timout must be less than 2 seconds to prevent using a life
		uTimeoutCount--;
		delay_ms(1);			

		if ( !(g_uRunFlags & RFLG_STATE_PULSE_LOW) )
		{	// Looking for a low pulse
			if ( g_ADCVoltage > PULSE_ON_VOLTAGE )
			{
				if ( uDebounceCount )
					uDebounceCount--;
				else
				{	// low pulse detected and debounced
					uDebounceCount = 3;//DEBOUNCE_COUNT;
					g_uRunFlags |= RFLG_STATE_PULSE_LOW;		
					if ( ucLastTransition < 15 )
					{
						ucLastTransition = 0;
						uPulseCount++;
					}
					SET_GREEN_LED( OFF );
				}
            if(g_uHMaxVoltageCount<100){
				   g_ulHMaxVoltage += g_ADCVoltage;
				   g_uHMaxVoltageCount++;
            }
            
			}
			else 
			{  // Legitimate high pulse - so take samples
				uDebounceCount = 3;//DEBOUNCE_COUNT;
				if ( uTimeoutCount < 100 )
				{	//  Only count samples for the last 100ms
					g_ulMaxCurrent += g_ADCCurrent;
					g_uMaxCount++;
					g_ulMaxVoltage += g_ADCVoltage;
					g_uMaxVoltageCount++;
				}
			}
		}
		else	// Current State = RFLG_STATE_PULSE_LOW
		{	// Looking for a high pulse
			if ( g_ADCVoltage < PULSE_ON_VOLTAGE )	// Debounce transition
			{
				if (uDebounceCount )
					uDebounceCount--;
				else
				{	
					uDebounceCount = 3;	//DEBOUNCE_COUNT;
					g_uRunFlags &= ~RFLG_STATE_PULSE_LOW;
					g_ADCFlags &= ~ADCFLG_COLLECT_AVERAGE;
					if ( ucLastTransition < 15 )
					{
						ucLastTransition = 0;
						uPulseCount++;
					}
					SET_GREEN_LED( ON );
				}
			}				
			else 
			{	// Legitimate low pulse
				uDebounceCount = 3;	//DEBOUNCE_COUNT;
				g_ulMinCurrent += g_ADCCurrent;
				g_uMinCount++;
				g_ulMinVoltage += g_ADCVoltage;
				g_uMinVoltageCount++;
TOGGLE_GREEN_LED;
			}
		}
		ucLastTransition++;
	}
//	SET_3V_POWER( OFF );	
	SET_DEVICE_POWER( OFF );						// Turn off device power
//DebugLCD("uPulseCnt= ",uPulseCount);
//delay_ms(1000);
	uPulseCount = (uPulseCount / 2) - 1;		// Subtract first pulse
	SET_GREEN_LED( OFF );
	return uPulseCount;
}

////////////////////////////////////////////////////////////////////////////////
// Checks to see if the calibration cable is inserted upon startup
//------------------------------------------------------------------------------
void CheckCalibrationCable(void)
{
	unsigned char ucLives;
	
	ADC10CTL0 |= ADC10ENC + ADC10SC;			// Start Sampling and conversion
	ResetADC();										// Resets all Min Max Counts
	ucLives = CountLivesUsed();						

	if ( g_uMaxCount ) g_ulMaxCurrent = g_ulMaxCurrent / g_uMaxCount;
	if ( g_uMaxVoltageCount ) g_ulMaxVoltage = g_ulMaxVoltage / g_uMaxVoltageCount;
	if ( g_uMinCount ) g_ulMinCurrent = g_ulMinCurrent / g_uMinCount;
	if ( g_uMinVoltageCount ) g_ulMinVoltage = g_ulMinVoltage / g_uMinVoltageCount;
	ADC10CTL0 &= ~ADC10ENC;						// Stop Sampling and conversion	
	CalculateCurrent();

//   DebugLCD("ACurrent= ",g_uActiveCurrent);
//   delay_ms(1000);
	if ( (ucLives == 0xFF) && (g_uIDDQCurrent == 0x00) && (g_uActiveCurrent > 1000) )
	{
 		g_uRunFlags |= RFLG_CALIBRATE_CURRENT;		
		g_uCalibrationResistance = 750;
	}
	SET_RED_LED( OFF );
	SET_GREEN_LED( OFF );

}


////////////////////////////////////////////////////////////////////////////////
// Calculates the value for g_uResGain to measure precisely set resistor value
//
// ResGain = (CalResistance * ADC_Current * 10) / (3L * ADC_Voltage)
//------------------------------------------------------------------------------
void CalibrateCurrentSensor( void )
{
	g_uResGain = (100L * g_uCalibrationResistance * g_ulMaxCurrent) / (g_ulMaxVoltage * V_DIVIDER_GAIN); 
//	g_uResGain = (((((g_ulMaxCurrent * 3300000)) / (9000000L / g_uCalibrationResistance)) * 100) >> 10);

	LcdSetCursorPosition(1, 0);
	DisplayString( "ResGain: " );
	DisplayDecimal ( g_uResGain );
		
	WriteFRAMData();								// Record new ResGain value to FRAM
	delay_seconds( 3 );
}
	

////////////////////////////////////////////////////////////////////////////////
// When in Calibration Mode, continuously checks the current and updates display
//------------------------------------------------------------------------------
void DisplayCalibrationCurrent(void)
{
	unsigned short uTimeoutCount;
	
	ResetADC();
	ADC10CTL0 |= ADC10ENC + ADC10SC;			// Start Sampling and conversion
	uTimeoutCount = 500;
	SET_DEVICE_POWER( ON );						// Turn on 9V device power
	while ( uTimeoutCount )		
	{	// Timout must be less than 2 seconds to prevent using a life
		uTimeoutCount--;
		delay_ms(1);	
		if ( uTimeoutCount < 100 )
		{
			g_ulMaxCurrent += g_ADCCurrent;
			g_uMaxCount++;
			g_ulMaxVoltage += g_ADCVoltage;
			g_uMaxVoltageCount++;
		}
	}		
	SET_DEVICE_POWER( OFF );		// Turn off device power
	ADC10CTL0 &= ~ADC10ENC;			// Stop Sampling and conversion

	g_ulMaxCurrent = g_ulMaxCurrent / g_uMaxCount;
	g_ulMaxVoltage = g_ulMaxVoltage / g_uMaxVoltageCount;
	SET_RED_LED( OFF );
	CalculateCurrent();

	g_uLCDCount = 5;
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	DisplayString( "V " );	
	DisplayDecimalCurrent( g_uActiveVoltage );

	DisplayString( " I " );	
	DisplayDecimalCurrent( g_uActiveCurrent );
			
	// Display resistance		
	LcdSetCursorPosition( 1, 0 );
	DisplayString( "Rs: " );	
	DisplayDecimal( g_uResistance );
	DisplayString( " Ohms    " );
}


////////////////////////////////////////////////////////////////////////////////
//  Powers up DUT at location ucAddress for uSeconds and then powers down.
//------------------------------------------------------------------------------
void PulseDevicePower( unsigned short uSeconds )
{
	unsigned long ulTimeoutCount;

	ulTimeoutCount = uSeconds * 100;	
	ResetADC();
	
	SET_DEVICE_POWER( ON );		// Turn on power
	for ( ; ulTimeoutCount; ulTimeoutCount--)
	{
		delay_ms( 10 );
		if ( g_uMaxCount++ > 90 )
		{
			if ( g_uMinSample == 0 ) 
				g_uMinSample = g_ADCCurrent;
		
			g_uMinSample = MIN( g_uMinSample, g_ADCCurrent );
			g_uMaxSample = MAX( g_uMaxSample, g_ADCCurrent );	
			g_ulMinCurrent += g_ADCCurrent;
			g_uMinCount++;
		}
	}						
	SET_DEVICE_POWER( OFF );		// Turn off power

	if ( g_uMinCount )g_ulMinCurrent = g_ulMinCurrent / g_uMinCount;
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	DisplayString( "MIN:" );
	DisplayDecimal(g_uMinSample );
	DisplayString( " MAX:" );
	DisplayDecimal(g_uMaxSample );
	LcdSetCursorPosition(1, 0);	
	DisplayString( "Ave:" );
	DisplayDecimal(g_ulMinCurrent );
	delay_ms( 500 );
}


////////////////////////////////////////////////////////////////////////////////
//  Checks and debounces the start button
//------------------------------------------------------------------------------
void CheckButtons(void)
{
	static unsigned char s_uDebounceCount = DEBOUNCE_COUNT; // Normally 3
	
	if ( START_BUTTON_PRESSED && !(g_uRunFlags & RFLG_START_BUTTON_PRESSED) )	// POLL BUTTON PRESSES
	{
		if ( s_uDebounceCount )
			s_uDebounceCount--;
		else
		{
			s_uDebounceCount = DEBOUNCE_COUNT;
			g_uRunFlags |= RFLG_START_BUTTON_PRESSED;
			if ( g_uRunFlags & RFLG_BUTTON_INITIATED )
			{
				g_uRunFlags &= ~RFLG_BUTTON_INITIATED;
				g_uRunFlags &= ~RFLG_DISPLAY_RESULTS;
				SET_RED_LED( OFF );
				SET_GREEN_LED( OFF );
				g_ucScreen = 0;
				g_uLCDCount = 1;
				g_uLEDCount = 1;
			}
			else
			{
				g_uRunFlags |= RFLG_PROCESS_START_BUTTON;
				g_uRunFlags |= RFLG_BUTTON_INITIATED;
			}
		}
	}
	else if ( !START_BUTTON_PRESSED )
	{
		if ( s_uDebounceCount )
				s_uDebounceCount--;
		else
		{
			s_uDebounceCount = DEBOUNCE_COUNT;
			g_uRunFlags &= ~RFLG_START_BUTTON_PRESSED;
		}
	}			
}


////////////////////////////////////////////////////////////////////////////////
//  Reads the current delivered through the 9V supply 
// Called once each milliseconds from Main loop
//------------------------------------------------------------------------------
void DetectCable(void)
{	
	static unsigned short s_uToggleCount = 150;
	static unsigned short s_uHighCurrentCount = 0;

	if ( !(--s_uToggleCount) )
	{
		s_uToggleCount = 150;               // Wait 150 mS before next measurement
		
		// Toggle power to lemo for cable detect
		if ( g_uRunFlags & RFLG_LEMO_POWER ) // If power is already on--
		{ // Toggle power off and see if we had no current on last pulse
			ADC10CTL0 &= ~ADC10ENC;						// Stop Sampling and conversion	
			g_uRunFlags &= ~RFLG_LEMO_POWER;       // Indicate that power is off
			SET_DEVICE_POWER( OFF );	            // Turn off power
			
			if ( g_uRunFlags & RFLG_CABLE_DETECTED ) // If Cable had been detected 
			{	// Cable was removed
				if ( s_uHighCurrentCount < 10 )       // But not long enough(?)
				{
					g_uRunFlags &= ~RFLG_CABLE_DETECTED; // Turn off Detected Flag
					if ( !(g_uRunFlags & RFLG_BUTTON_INITIATED) ) //If no Button Press
					{
						g_uRunFlags &= ~RFLG_DISPLAY_RESULTS;  // Turn off Display Results
						SET_RED_LED( OFF );                    // And turn off LEDs
						SET_GREEN_LED( OFF );
						g_ucScreen = 0;
						g_uLCDCount = 1;
						g_uLEDCount = 1;
					}
				}
				else  //We did have a current pulse
				{
					s_uToggleCount = 1000;        // 1 s delay before next detection
				}
			}
		}
		else 
		{	// Toggle power on
			ADC10CTL0 |= ADC10ENC + ADC10SC;			// Start Sampling and conversion
			g_uRunFlags |= RFLG_LEMO_POWER;
			SET_DEVICE_POWER( ON );	
			s_uHighCurrentCount = 0;
		}
	}
			
	if ( g_uRunFlags & RFLG_LEMO_POWER ) // Check for current only if power is on
	{
		if ( g_ADCCurrent > PULSE_ON_CURRENT ) // Then FET is on
		{
			s_uHighCurrentCount++;
		}
			
      // cable not detected and HighCurrentCount>10 then set CABLE_DETECTED
		if ( !(g_uRunFlags & RFLG_CABLE_DETECTED) && (s_uHighCurrentCount > 10) )
		{	// Cable is now detected
			s_uHighCurrentCount = 0;
			s_uToggleCount = 1000;

			ADC10CTL0 &= ~ADC10ENC;						// Stop Sampling and conversion	
			g_uRunFlags &= ~RFLG_LEMO_POWER;
			SET_DEVICE_POWER( OFF );	
	
			g_uRunFlags |= RFLG_CABLE_DETECTED;
			g_ucScreen = 1;
			g_uLCDCount = 1;
			g_uLEDCount = 1;
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
void ProcessUARTPacket(void)
{
	GetUARTReceived( g_aDataBuffer );  
	
	switch ( g_tDUTPacket.command )	
	{
	case SCMD_SET_STATUS:
		g_ucErrorBlinkCount = g_tDUTPacket.data1;
		break;
		
	case SCMD_GET_STATUS:
		g_aDataBuffer[0] = FIXTURE_TYPE;
		g_aDataBuffer[1] = FIRMWARE_VERSION;
		g_aDataBuffer[2] = FIRMWARE_REV;
		TransmitUART( 3, g_aDataBuffer );
		break;

	case SCMD_START_TEST:
		g_uRunFlags |= RFLG_PROCESS_START_BUTTON;
		break;
			
	case SCMD_GET_TEST_RESULTS:
		g_aDataBuffer[0] = g_ucLivesUsed;
		g_aDataBuffer[1] = HIBYTE(g_uMeanResistance);							
		g_aDataBuffer[2] = LOBYTE(g_uMeanResistance);		// Calculated Resistance with Mosfet pulsed
		g_aDataBuffer[3] = HIBYTE(g_uIDDQCurrent);	// Quiescent current while mosfet not pulsed
		g_aDataBuffer[4] = LOBYTE(g_uIDDQCurrent);	// Active Current while Mosfet pulsed
		TransmitUART( 5, g_aDataBuffer );
		break;

	case SCMD_SET_DEVICE_POWER:
		PulseDevicePower( g_tDUTPacket.data1 );
		break;

	case SCMD_SET_LED_STATE:
		SET_RED_LED( g_tDUTPacket.data1 );
		SET_GREEN_LED( g_tDUTPacket.data2 );
		break;
				
	case SCMD_GET_UART_DATA:
		TransmitUART( SERIAL_BUFFER_SIZE, g_aDataBuffer );
		break;
			
	case SCMD_SEND_UART_DATA:
		TransmitUART( SERIAL_BUFFER_SIZE, g_aDataBuffer );
		break;

	case SCMD_CALIBRATE_CURRENT:	
		g_uCalibrationResistance = (g_tDUTPacket.data1 << 8) +  g_tDUTPacket.data2;
		if ( g_uCalibrationResistance < 500 ) g_uRunFlags &= ~RFLG_CALIBRATE_CURRENT; else g_uRunFlags |= RFLG_CALIBRATE_CURRENT;	
		g_uLEDCount = 1;
		g_uLCDCount = 1;
		break;

	case SCMD_WRITE_FRAM:
		if ( g_tDUTPacket.data1 < 64 )
		{
			unsigned short* pData = (unsigned short*)FRAM_DATA_ADDRESS + g_tDUTPacket.data1;
			*pData = (g_tDUTPacket.data2 << 8) +  g_tDUTPacket.data3;
		}
		break;
		
	case SCMD_READ_FRAM:
		if ( g_tDUTPacket.data1 < 64 )
		{
			unsigned short* pData = (unsigned short*)FRAM_DATA_ADDRESS + g_tDUTPacket.data1;
			g_aDataBuffer[0] = HIBYTE( *pData );
			g_aDataBuffer[1] = LOBYTE( *pData++ );							
			g_aDataBuffer[2] = HIBYTE( *pData );					
			g_aDataBuffer[3] = LOBYTE( *pData++ );			
			g_aDataBuffer[4] = HIBYTE( *pData );					
			g_aDataBuffer[5] = LOBYTE( *pData++ );
			g_aDataBuffer[6] = HIBYTE( *pData );						
			g_aDataBuffer[7] = LOBYTE( *pData++ );				
			TransmitUART( 8, g_aDataBuffer );
		}
		break;

	case SCMD_PRINT_LCD:		// Displays 4 characters beginning at location checksum
		if (  g_tDUTPacket.checksum <= 31 )	// If greater than 31, then start where current cursor location is
		{	// The checksum in the data packet indicates starting cursor location
			if ( g_tDUTPacket.checksum <= 15 ) LcdSetCursorPosition( 0, g_tDUTPacket.checksum );
			else LcdSetCursorPosition( 1, g_tDUTPacket.checksum - 15 );
		}
		DisplayChar( g_tDUTPacket.data1 );
		DisplayChar( g_tDUTPacket.data2 );
		DisplayChar( g_tDUTPacket.data3 );
		DisplayChar( g_tDUTPacket.data4 );
		g_uLEDCount = TIMEOUT1_COUNT_150MS;		// Set to not refresh screen for quite a while
		g_uLCDCount = TIMEOUT1_COUNT_150MS;		// Set to not refresh screen for quite a while
		break;

	case SCMD_RESET_PROCESSOR:
		WDTCTL = WDTPW | WDTIS_7;
		while ( 1 ){}		// Park here until watchdog times out resetting the processor.
		break;
	
	default:
		break;
	}
}


////////////////////////////////////////////////////////////////////////////////
//  Controls the main processor LED blink rate
//------------------------------------------------------------------------------
void ProcessLED()
{
	static unsigned char s_ucBlinkCount;	
	
	if ( (s_ucBlinkCount <= 2) || (s_ucBlinkCount & 0x01) )  
	{
		SET_STATUS_LED( OFF );
	}
	else 
	{
		SET_STATUS_LED( ON );
	}
		
	if ( ++s_ucBlinkCount >= ((g_ucErrorBlinkCount + 2) * 2) )
	{
		SET_STATUS_LED( OFF );
		s_ucBlinkCount = 0;
	}
}


////////////////////////////////////////////////////////////////////////////////
//  Controls the LCD Display
//------------------------------------------------------------------------------
void ProcessLCD()
{
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	if ( g_uRunFlags & RFLG_DISPLAY_RESULTS )
	{
		if ( g_ucScreen == 0 )
		{
			g_ucScreen++;
			g_uLCDCount = 10;
			DisplayTestResults();			
		}
		else 
		if ( g_ucScreen == 1 )
		{	// Display resistance and current
			g_ucScreen++;
			DisplaySetPoint();
		}	
		else 
		{
			g_ucScreen = 0x00;
			DisplayCurrent();
		}
	}
	else
	{
		if ( g_ucScreen == 0 )
		{
			g_ucScreen++;
			if (  g_uRunFlags & RFLG_CABLE_DETECTED )
			{
				g_uRunFlags |= RFLG_PROCESS_START_BUTTON;
				if ( !( g_uRunFlags & RFLG_DEBUG ) ){
               DisplayString("TESTING...");	
   			   delay_ms(750);
            }
			}
			else DisplaySplashScreen();
		}
		else 
		{	// Display resistance and current
			g_ucScreen = 0x00;
			DisplayInstructions();
		}	
	}
}


////////////////////////////////////////////////////////////////////////////////
//  Displays the initial Splash Screen
//------------------------------------------------------------------------------
void DisplaySplashScreen(void)
{
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	if ( !( g_uRunFlags & RFLG_DEBUG ) )
     DisplayString("   ArthroCare    ");
	LcdSetCursorPosition(1, 0);
	DisplayString("EULS Tester v");			
	DisplayChar( (FIRMWARE_VERSION + 0x30) );				
	DisplayChar( '.' );				
	DisplayChar( (FIRMWARE_REV + 0x30) );				
}


////////////////////////////////////////////////////////////////////////////////
//  Displays the initial Splash Screen
//------------------------------------------------------------------------------
void DisplayInstructions(void)
{
	if ( g_uRunFlags & RFLG_CABLE_DETECTED )
	{
		if ( !( g_uRunFlags & RFLG_DEBUG ) )
        DisplayString(" Cable Detected");
		g_uLCDCount = 0x04;
	}
	else
	{
		if ( !( g_uRunFlags & RFLG_DEBUG ) )
        DisplayString("  Insert Cable");
		LcdSetCursorPosition(1, 0);
		DisplayString("  Press  Start");
	}
}


////////////////////////////////////////////////////////////////////////////////
// Displays Pass or fail results
//------------------------------------------------------------------------------
void DisplayTestResults(void)
{
	if ( g_ucLivesUsed || (g_uSetPointIndex >= 6) || ((ROTARY_SWITCH_2C) && (g_uSetPointIndex != 5)) || ((ROTARY_SWITCH_3C) && (g_uSetPointIndex != 2)) )
	{
		if ( !( g_uRunFlags & RFLG_DEBUG ) )
        DisplayString( "TEST:  FAIL" );
		
		LcdSetCursorPosition( 1, 0);
		if  ( g_ucLivesUsed && (g_ucLivesUsed < 4) )
		{
			DisplayChar( g_ucLivesUsed + 0x30 );				
			DisplayString( " LIVES USED" );
		}
		else if ( g_ucLivesUsed >= 4 )
		{
			DisplayString( "Invalid Cable" );
		}
		else if ( (g_uSetPointIndex >= 6) )
		{
			DisplayString( "No Setpoint" );			
		}
		else if ( ((ROTARY_SWITCH_2C) && (g_uSetPointIndex != 5)) || ((ROTARY_SWITCH_3C) && (g_uSetPointIndex != 2)) )
		{
			DisplayString( "Invalid Setpoint" );			
		}
	}
	else
	{
		if ( !( g_uRunFlags & RFLG_DEBUG ) )
        DisplayString( "TEST:  PASS    " );
		LcdSetCursorPosition( 1, 0);
		DisplayString( "0 LIVES USED   " );
	}	
}


////////////////////////////////////////////////////////////////////////////////
//  Determine SetPoint from resistance values
//unsigned short g_aCoagualationSetPoints[6][3] =
//	Nominal Resistance (ohms),	Min Resistance (ohms),	Max Resistance (ohms)
//			{ 	412, 391, 433,			// AP1	
//				576, 547, 605,			// SP3	
//				750, 713, 788,			// SP4
//				1050, 998, 1103,		// SP5
//				200, 190, 210,			// SP6
//				909, 864, 954 };		// SP7
//------------------------------------------------------------------------------
void SetCoagulationPoint(void)
{
	unsigned short* pResistanceLimit;
	unsigned short uIndex;
	
	for ( uIndex = 0; uIndex < 6; uIndex++ )
	{
		pResistanceLimit = &g_aCoagualationSetPoints[uIndex][1];
		if ( (g_uMeanResistance >= *pResistanceLimit++) && (g_uMeanResistance <= *pResistanceLimit) )
			break;
	}
// DebugLCD("Resistance= ",g_uMeanResistance);
// delay_ms(1000);
	g_uSetPointIndex = uIndex;
}


////////////////////////////////////////////////////////////////////////////////
//  Displays Coagulation set point
//------------------------------------------------------------------------------
void DisplaySetPoint(void)
{
	if ( !( g_uRunFlags & RFLG_DEBUG ) )
     DisplayString( "SET POINT: " );
   else
     return;
//	Nominal Resistance (ohms)	Min Resistance (ohms)	Max Resistance (ohms)
//			{ 	412, 391, 433,			// AP1	
//				576, 547, 605,			// SP3	
//				750, 713, 788,			// SP4
//				1050, 998, 1103,		// SP5
//				200, 190, 210,			// SP6
//				909, 864, 954 };		// SP7
	switch ( g_uSetPointIndex )
	{
	case 0:
		DisplayString( "AP1" );
		break;
		
	case 1:
		DisplayString( "SP3" );
		break;

	case 2:
		DisplayString( "SP4" );
		break;

	case 3:
		DisplayString( "SP5" );
		break;

	case 4:
		DisplayString( "SP6" );
		break;
		
	case 5:
		DisplayString( "SP7" );
		break;

	default:
		DisplayString( "NONE" );
		break;
	}
	
	LcdSetCursorPosition(1, 0);
	DisplayString( "R: " );	
	DisplayDecimal( g_uMeanResistance );
	DisplayString( " Ohms" );
}



////////////////////////////////////////////////////////////////////////////////
//  Displays the calculated resistance and current
//------------------------------------------------------------------------------
void DisplayResistance(void)
{
	LcdSetCursorPosition(1, 0);
	DisplayString( "R: " );	
	DisplayDecimal( g_uResistance );
	DisplayString( " Ohms " );
}

					 
////////////////////////////////////////////////////////////////////////////////
//  Display the current measurements
//------------------------------------------------------------------------------
void DisplayCurrent(void)
{
	// Now display digits for Quiescent Current
//xRA	DisplayString( "IDDQ: " );	
//xRA	DisplayDecimalCurrent( g_uIDDQCurrent );
//xRA	DisplayString( " mA    " );
  
  if ( !( g_uRunFlags & RFLG_DEBUG ) ){
     DisplayString( "VMax: " );	
	  DisplayDecimalCurrent( g_uHActiveVoltage ); 
     DisplayString( " V    " );
  }
  
	// Now display digits for ActiveCurrent
	LcdSetCursorPosition( 1, 0);
	DisplayString( "IMax: " );	
	DisplayDecimalCurrent( g_uActiveCurrent );
	DisplayString( " mA  " );		
}


////////////////////////////////////////////////////////////////////////////////
//  Display the decimal current measurements in mAmps
//------------------------------------------------------------------------------
void DisplayDecimalCurrent( unsigned short uValue )
{
	unsigned short uRemainder;
	unsigned char ucChar;

	uRemainder = uValue;
	if ( uValue >= 10000 )
	{
		ucChar = uValue / 10000;
		DisplayChar( (ucChar + 0x30)  );	
		uRemainder = uRemainder - (ucChar * 10000);
	}

	if ( uValue >= 1000 )
	{
		ucChar = uRemainder / 1000;
		DisplayChar( (ucChar + 0x30) );	
		uRemainder = uRemainder - (ucChar * 1000);
	}
	else DisplayChar( '0' );
	DisplayChar( '.' );	
		
	if ( uValue >= 100 )
	{
		ucChar = uRemainder / 100;
		DisplayChar( (ucChar + 0x30) );				
		uRemainder = uRemainder - (ucChar * 100);
	}
	else DisplayChar( '0' );

	ucChar = uRemainder / 10;
	DisplayChar( (ucChar + 0x30) );				
	ucChar = uRemainder - (ucChar * 10);		
	DisplayChar( (ucChar + 0x30) );		
}


////////////////////////////////////////////////////////////////////////////////
//  Reads data from Fram into buffer
//------------------------------------------------------------------------------
void ReadFRAMData(void)
{
	unsigned short* pData = (unsigned short*)FRAM_DATA_ADDRESS;

	g_uPassCount = *pData++;
	g_uFailCount = *pData++;
	g_uPowerupCount = *pData++;
	g_uErrorCount = *pData++;
	g_uResGain = *pData;			
	
	if ( ((g_uPassCount == 0xFFFF) && (g_uFailCount == 0xFFFF)) || (START_BUTTON_PRESSED) )
	{
		g_uPassCount = 0x0000;
		g_uFailCount = 0x0000;
		g_uPowerupCount = 0x0001;
		g_uErrorCount = 0x0000;
      if(g_uResGain<10000)
   		g_uResGain = INIT_RES_GAIN;			
		WriteFRAMData();
	}
	
	ClearLcdScreen(g_uRunFlags & RFLG_DEBUG ); 
	if ( !( g_uRunFlags & RFLG_DEBUG ) ){
	  LcdSetCursorPosition( 0, 0 );
     DisplayString( "Pass Count: " );
	  DisplayDecimal( g_uPassCount );
   }
	LcdSetCursorPosition( 1, 0 );
	DisplayString( "Fail Count: " );
	DisplayDecimal( g_uFailCount );
	
	delay_ms( 500 );
}


////////////////////////////////////////////////////////////////////////////////
//  Controls the LCD Display
//------------------------------------------------------------------------------
void WriteFRAMData(void)
{
	unsigned short* pData = (unsigned short*)FRAM_DATA_ADDRESS;

	*pData++ = g_uPassCount;
	*pData++ = g_uFailCount;
	*pData++ = g_uPowerupCount;
	*pData++ = g_uErrorCount;
	*pData = g_uResGain;
}

