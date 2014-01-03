////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//							HARDWARE.C 
//
//			This file contains all processor dependant code    
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include "hardware.h"


/////////////////////////////////////////////////////////////////////////////
// EXPORTED VARIABLES
//unsigned int g_ADCSample;
unsigned char g_uADCInch;
unsigned short g_ADCVoltage;
unsigned short g_ADCCurrent;
unsigned short g_ADCFlags;
unsigned long g_ulSampleSum;
unsigned short g_uSampleCount;
unsigned char g_aDataBuffer[SERIAL_BUFFER_SIZE];	// General purpose buffer - do not use in interrupt
unsigned short g_aCoagualationSetPoints[6][3] = { 412, 391, 433,		// AP1	Nominal Resistance (ohms)	Min Resistance (ohms)	Max Resistance (ohms)
																576, 547, 605,			// SP3	
//																750, 735, 765,			// SP4 Updated 11/13/2013
																770, 755, 785,			// SP4 Updated 12/09/2013
																1050, 998, 1103,		// SP5
																200, 190, 210,			// SP6
																930, 912, 948 };		// SP7 Updated 12/09/2013


////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
void delay_ms( unsigned short n ) 
{
	unsigned short msCount;

	for ( ; n; n--)
	{
		msCount = MS_COUNT;
		while ( msCount-- ){}
	}
}


////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
void delay_seconds( unsigned short n ) 
{
	for ( n *= 2; n; n--)
	{
		delay_ms( 500 );
	}
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//   INTERRUPT SERVICE ROUTINES
////////////////////////////////////////////////////////////////////////////////
// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	switch(__even_in_range(ADC10IV, 12))
	{
	case  0: break;                          // No interrupt
	case  2: break;                          // conversion result overflow
	case  4: break;                          // conversion time overflow
	case  6: break;                          // ADC10HI
	case  8: break;                          // ADC10LO
	case 10: break;                          // ADC10IN
	case 12:
		if ( (ADC10MCTL0 & 0x0F) == ADC10INCH_1 )//!(g_ADCFlags & ADCFLG_CHANNEL1) )
		{
			g_ADCCurrent = ADC10MEM0;  
			if (g_ADCCurrent > 0x3F8) 
			{
//				SET_DEVICE_POWER( OFF );	// Turn on DUT at ucAddress power
				g_ADCFlags |= ADCFLG_SHORT_CIRCUIT;
			}
			else 
			{
				if ( (g_ADCFlags & ADCFLG_COLLECT_AVERAGE) && ( g_ADCCurrent < PULSE_ON_CURRENT ) )
				{
					g_ulSampleSum += g_ADCCurrent;
					g_uSampleCount++;
				}
			}
		}
		else if ( (ADC10MCTL0 & 0x0F) == ADC10INCH_0 )
		{			
			g_ADCVoltage = ADC10MEM0;  
		}
		else g_uADCInch = ADC10MEM0;
		break;  
		
	default: break; 
	} 
}

#if 0  // These ISR's Are Not Used

#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case  0: break;                          // No interrupt
    case  2: 
      // 32 conversions complete
      break;                                 // DMA0IFG
    case  4: break;                          // DMA1IFG
    case  6: break;                          // DMA2IFG
    case  8: break;                          // Reserved
    case 10: break;                          // Reserved
    case 12: break;                          // Reserved
    case 14: break;                          // Reserved
    case 16: break;                          // Reserved
    default: break; 
  }   
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
	if ( P1IFG & PIN_SWITCH_STATE )
	{
		WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDTSSEL__ACLK | WDTIS_5;	//  32ms Debounce @ 4, 8ms @ 5
		SFRIFG1 &= ~WDTIFG;
		SFRIE1 |= WDTIE;

		TOGGLE_STATUS_LED;
		P1IE &= ~PIN_SWITCH_STATE;
	}
}


#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	SFRIE1 &= ~WDTIE;
	SFRIFG1 &= ~WDTIFG;
	WDTCTL = WDTPW + WDTHOLD;
	
	P1IFG = 0x00;
//	(P1IN & PIN_SWITCH_STATE)?SET_BOARD_LED( ON ):SET_BOARD_LED( OFF );
//	P1IES = P1IN & PIN_SWITCH_STATE;
	P1IE |= PIN_SWITCH_STATE;
	
//	TransmitData();
		
	TOGGLE_TEST_PIN1;
}
#endif

