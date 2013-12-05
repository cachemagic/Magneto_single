////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//					UART.C
//
//		Arthrocare Werewolf Tester
//
//		This module provides the interrupt and register level access to the UART.
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include "hardware.h"
#include "uart.h"




////////////////////////////////////////////////////////////////////////////////
// STRUCTURE AND TYPE DECLARATIONS
typedef struct		
{	// Serial control structure
	unsigned char ucAddress;		// 6-bit address 
	unsigned char* pRxData;			// serial RX data pointer 
	unsigned char* pTxData;			// serial TX data pointer 
	unsigned char ucRxCount;		// serial RX count 
	unsigned char ucTxCount;		// serial TX count 
} TYPE_SERIAL;



////////////////////////////////////////////////////////////////////////////////
// NON EXPORTED VARIABLE AND DEFINES
#define SERIAL_FRAME_TERMINATOR		0x0D	// Terrminator byte for all Tx and Rx
#define SERIAL_FRAME_HEADER			0x20	// Header byte for all Tx and Rx

TYPE_SERIAL		g_tUART;					// UART data structure
PACKET			g_tDUTPacket;			// Packet Frame from werewolf unit

unsigned char 	g_ucUARTFlags;			// UART state flags

unsigned char g_aUARTRxBuffer[SERIAL_BUFFER_SIZE];
unsigned char g_aUARTTxBuffer[SERIAL_BUFFER_SIZE];

unsigned char 	g_uHardwareStatus;
unsigned char 	g_uTestResults;


void TransferUARTData(void);



////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//	UART
//------------------------------------------------------------------------------
void InitUART(void)
{
	g_tUART.pRxData = g_aUARTRxBuffer;		
	g_tUART.pTxData = g_aUARTTxBuffer;		
	g_tUART.ucRxCount = 0x00;		 
	g_tUART.ucTxCount = 0x00;		 
	g_tUART.ucAddress = 0x05;
	g_ucUARTFlags = 0x00;

	UCA0CTL1 |= UCSWRST; 
	UCA0CTL1 = UCMODE_0 | UCSSEL_1 | UCBRKIE;		// UART Mode, ACLK = 1MHz, Break Character interrupt for EOT 
	UCA0BR0 = 6;											// 9600 baud
	UCA0BR1 = 0; 
	UCA0MCTLW = 0x2081;									// (See UG)
	UCA0IE = UCTXIE | UCRXIE;// | UCTXCPTIE;		// Enable Transmit, receive and break end transmit interrupts
	UCA0CTL1 &= ~UCSWRST;								// release from reset

	UCA1CTL1 |= UCSWRST; 
	UCA1CTL1 = UCMODE_0 | UCSSEL_1 | UCBRKIE;		// UART Mode, ACLK = 1MHz, Break Character interrupt for EOT 
	UCA1BR0 = 6;											// 9600 baud
	UCA1BR1 = 0; 
	UCA1MCTLW = 0x2081;									// (See UG)
	UCA1IE = UCTXIE | UCRXIE;// | UCTXCPTIE;		// Enable Transmit, receive and break end transmit interrupts
	UCA1CTL1 &= ~UCSWRST;								// release from reset
}

////////////////////////////////////////////////////////////////////////////////
// PrintUART Send string to UART
void PrintUART( unsigned char* pData)
{
  int i;
  
  i=0;
  while(*pData++){
    i++;
  }

  TransmitUART(i, pData); 
}

////////////////////////////////////////////////////////////////////////////////
// OutNum() Output number (0 to 255) to UART
void OutNum(char num)
{
  unsigned char atmp[5];
  char i;
  char tmp;

  i=0;
  if(num>99){
    tmp = num/100;
    atmp[i++]=tmp + 0x30;
    num = num - tmp*100;
    atmp[i]=0;
  }
  if(num>9){
    tmp = num/10;
    atmp[i++]=tmp + 0x30;
    num = num - tmp*10;
  }
  atmp[i++] = num + 0x30;
  atmp[i++]=0;
  delTransmitUART(i-1,atmp);
}
////////////////////////////////////////////////////////////////////////////////
//
void delTransmitUART( unsigned char ucBytes, unsigned char* pData )
{
  TransmitUART(ucBytes, pData );
  delay_ms(20);
}

////////////////////////////////////////////////////////////////////////////////
// PrintLn()
void PrintLn(unsigned char* pData)
{
  int i;
  
  i = 0;
  while (*pData++){
    i++;
  }
  *pData++ = 13;  // carriage return
  *pData++ = 10;  // Line Feed
  *pData = 0;
  delTransmitUART(i+2, pData);   
}

////////////////////////////////////////////////////////////////////////////////
// Print()
void Print(unsigned char* pData)
{
  int i;
  
  i = 0;
  while (*pData++){
    i++;
  }
  delTransmitUART(i, pData);   
}

////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
void TransmitUART( unsigned char ucBytes, unsigned char* pData )
{
	unsigned char index = 0;

	if ( ucBytes && !(g_ucUARTFlags & SFLG_TRANSMITTING) )			// Be sure there's data and not already rxing or txing
	{
		g_tUART.ucTxCount = ucBytes - 1;			// subtract first transmit byte sent in this function
		while ( index < ucBytes )					// Load transmit buffer
		{	
			g_aUARTTxBuffer[index++] = *pData++;
		}

		g_tUART.pTxData = g_aUARTTxBuffer;			// Reset data to beginning of buffer	 
		g_ucUARTFlags |= SFLG_TRANSMITTING;		// Set TX flag
		if ( g_ucUARTFlags & SFLG_USB_RX )
			UCA1TXBUF = *g_tUART.pTxData++;			// Load transmit shift register
		else
			UCA0TXBUF = *g_tUART.pTxData++;			// Load transmit shift register
	}
} 


////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
unsigned char GetUARTReceived( unsigned char* pBuffer )
{
	unsigned char index = 0;

	while ( index < g_tUART.ucRxCount )		// Load pBuffer with bytes recieved	
	{	
		*pBuffer++ = g_aUARTRxBuffer[index++];
	}
	g_tUART.pRxData = g_aUARTRxBuffer;	// Reset data pointer		 
	g_tUART.ucRxCount = 0x00;			// Reset counter
	g_ucUARTFlags &= ~SFLG_RX_DATA;				// Clear recieve flags
	return index;
}


////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	switch( __even_in_range(UCA0IV, 0x08) )
	{
	case 0x00: break;									// Vector 0 - no interrupt
	
	case 0x02: // RECEIVE FLAG						// Vector 2 - RXIFG
		if ( UCA0STATW & UCBRK )					// End of rx data, set flag to notify
		{
			UCA0STATW &= ~UCBRK;						// Clear break status -- This is cleared by reading RXBUF
			g_tUART.pRxData = g_aUARTRxBuffer;		// Do not reset ucCount - this is used in get data routine
			g_ucUARTFlags = SFLG_RX_DATA;			// Clears all Flags and sets RX_DATA
		}
		else
		{
			if ( (UCA0RXBUF == SERIAL_FRAME_HEADER) && !(g_ucUARTFlags & SFLG_RECEIVING) )
			{	// Reset buffer if first byte or buffer overrun
				g_ucUARTFlags &= ~SFLG_USB_RX;
				g_ucUARTFlags |= SFLG_RECEIVING;
				g_tUART.pRxData = g_aUARTRxBuffer;
				g_tUART.ucRxCount = 0x00;		
			}
			if ( g_ucUARTFlags & SFLG_RECEIVING )
			{
				*g_tUART.pRxData++ = UCA0RXBUF;		// stuff the buffer
				g_tUART.ucRxCount++;		
				
				if ( g_tUART.ucRxCount >= DUT_PACKET_SIZE )
				{
					g_ucUARTFlags &= ~SFLG_RECEIVING;
					g_ucUARTFlags |= SFLG_RX_DATA;	
					TransferUARTData();
				}
			}
		}
		break;

	case 0x04: // TRANSMIT FLAG				// Vector 4 - TXIFG  TX Buffer empty
		if ( (g_tUART.ucTxCount > 0) && (g_tUART.ucTxCount <= SERIAL_BUFFER_SIZE) )
		{	// Send next byte in frame
			g_tUART.ucTxCount--;
			UCA0TXBUF = *g_tUART.pTxData++; 
		}
		else if ( !(g_ucUARTFlags & SFLG_END_TRANSMISSION) )
		{	// End of transmission
			g_ucUARTFlags |= SFLG_END_TRANSMISSION;
			UCA0CTL1 |= UCTXBRK;					// Send Break character for EOT
			UCA0TXBUF = 0x00; 					// 0x55 if in auto BR detect mode
			g_tUART.pTxData = g_aUARTTxBuffer;	
			g_tUART.ucTxCount = 0x00;
		}
		else
			g_ucUARTFlags = 0x00;				// Clear all flags
			
		UCA0IFG &= ~UCTXIFG;
		break;

	case 0x06: break;						// Vector 6 - UCSTTIFG	 Start bit received	
	case 0x08: break;						// Vector 8 - UCTXCPTIFG  Entire byte shifted out
//		UCA0CTL1 &= ~UCTXBRK;		
//		UCA0IFG &= ~UCTXCPTIFG;
//		g_ucUARTFlags &= ~SFLG_TRANSMITTING;
//		UCA0IE = UCRXIE;					// Enable receive 
//
		break;

	default: break;  
	}
}



////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	switch( __even_in_range(UCA1IV, 0x08) )
	{
	case 0x00: break;									// Vector 0 - no interrupt
	
	case 0x02: // RECEIVE FLAG						// Vector 2 - RXIFG
		if ( UCA1STATW & UCBRK )					// End of rx data, set flag to notify
		{
			UCA1STATW &= ~UCBRK;						// Clear break status -- This is cleared by reading RXBUF
			g_tUART.pRxData = g_aUARTRxBuffer;		// Do not reset ucCount - this is used in get data routine
			g_ucUARTFlags = SFLG_RX_DATA;			// Clears all Flags and sets RX_DATA
		}
		else
		{
			if ( (UCA1RXBUF == SERIAL_FRAME_HEADER) && !(g_ucUARTFlags & SFLG_RECEIVING) )
			{	// Reset buffer if first byte or buffer overrun
				g_ucUARTFlags |= SFLG_USB_RX;
				g_ucUARTFlags |= SFLG_RECEIVING;
				g_tUART.pRxData = g_aUARTRxBuffer;
				g_tUART.ucRxCount = 0x00;		
			}
			if ( g_ucUARTFlags & SFLG_RECEIVING )
			{
				*g_tUART.pRxData++ = UCA1RXBUF;		// stuff the buffer
				g_tUART.ucRxCount++;		
				
				if ( g_tUART.ucRxCount >= DUT_PACKET_SIZE )
				{
					g_ucUARTFlags &= ~SFLG_RECEIVING;
					g_ucUARTFlags |= SFLG_RX_DATA;	
					TransferUARTData();
				}
			}
		}
		break;

	case 0x04: // TRANSMIT FLAG				// Vector 4 - TXIFG  TX Buffer empty
		if ( (g_tUART.ucTxCount > 0) && (g_tUART.ucTxCount <= SERIAL_BUFFER_SIZE) )
		{	// Send next byte in frame
			g_tUART.ucTxCount--;
			UCA1TXBUF = *g_tUART.pTxData++; 
		}
		else if ( !(g_ucUARTFlags & SFLG_END_TRANSMISSION) )
		{	// End of transmission
			g_ucUARTFlags |= SFLG_END_TRANSMISSION;
			UCA1CTL1 |= UCTXBRK;					// Send Break character for EOT
			UCA1TXBUF = 0x00; 					// 0x55 if in auto BR detect mode
			g_tUART.pTxData = g_aUARTTxBuffer;	
			g_tUART.ucTxCount = 0x00;
		}
		else
			g_ucUARTFlags = 0x00;				// Clear all flags
			
		UCA1IFG &= ~UCTXIFG;
		break;

	case 0x06: break;						// Vector 6 - UCSTTIFG	 Start bit received	
	case 0x08: break;						// Vector 8 - UCTXCPTIFG  Entire byte shifted out
//		UCA1CTL1 &= ~UCTXBRK;		
//		UCA1IFG &= ~UCTXCPTIFG;
//		g_ucUARTFlags &= ~SFLG_TRANSMITTING;
//		UCA1IE = UCRXIE;					// Enable receive 
//
		break;

	default: break;  
	}
}



////////////////////////////////////////////////////////////////////////////////
//  Called from ISR to transfer Rx Buffer to DUT data packet structure
//------------------------------------------------------------------------------
void TransferUARTData( void )
{	
	unsigned char* pBuffer;
	unsigned char index = 0;

	pBuffer = &g_tDUTPacket.head; //&g_aDataBuffer[1];
	if ( g_tUART.ucRxCount <= DUT_PACKET_SIZE )
	{
		while ( index < g_tUART.ucRxCount )		// Load pBuffer with bytes recieved	
		{	
			*pBuffer++ = g_aUARTRxBuffer[index++];
		}
//		g_tUART.pRxData = g_aUARTBuffer;	// Reset data pointer		 
//		g_tUART.ucRxCount = 0x00;			// Reset counter
		g_ucUARTFlags |= SFLG_RX_DATA;	
	}
}

