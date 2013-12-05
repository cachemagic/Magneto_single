////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//						UART.H
//
//		This module provides the interrupt and register level access to the UART.
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifndef UART_H
#define UART_H



/////////////////////////////////////////////////////////////////////////////
// Global Variables and defines
extern unsigned char 	g_ucUARTFlags;
	#define SFLG_RX_DATA						BIT0		// Receive data is available, cleared by user
	#define SFLG_RECEIVING					BIT1		// USCI module is receiving buffer frame, cleared when stop recieved
	#define SFLG_TRANSMITTING				BIT2		// USCI module is transmitting buffer frame, cleared when last byte sent
	#define SFLG_END_TRANSMISSION			BIT3		// Break character sent, cleanup transmit
	#define SFLG_SERIAL_ERROR				BIT4		// Serial error occured
	#define SFLG_USB_RX						BIT5		// Serial error occured


// SERIAL_COMMANDS
#define SCMD_GET_STATUS						0x81		//	0x01		
#define SCMD_SET_STATUS						0x82
#define SCMD_START_TEST						0x83
#define SCMD_GET_TEST_RESULTS				0x84
#define SCMD_SET_DEVICE_POWER				0x85
#define SCMD_SET_LED_STATE					0x86
#define SCMD_SEND_UART_DATA				0x87
#define SCMD_GET_UART_DATA					0x88		// 0x08
#define SCMD_SEND_I2C_DATA					0x89
#define SCMD_CALIBRATE_CURRENT			0x8A		// 0x0A
#define SCMD_WRITE_FRAM						0x8B
#define SCMD_READ_FRAM						0x8C
#define SCMD_PRINT_LCD						0x8D
#define SCMD_RESET_PROCESSOR				0x8E

#define SCMD_TX_BIT							0x80				

#define GREEN_LED_ON							0x01
#define RED_LED_ON							0x10


#define DUT_PACKET_SIZE				7
typedef struct
{
	unsigned char head;				// Head = 0x20
	unsigned char command;			// command
	unsigned char data1;				
	unsigned char data2;				
	unsigned char data3;				
	unsigned char data4;				
	unsigned char checksum;			// Checksum = (~checksum) + 1
} PACKET;

extern PACKET		g_tDUTPacket;


/////////////////////////////////////////////////////////////////////////////
// Export Functions
void InitUART(void);

void TransmitUART( unsigned char ucBytes, unsigned char* pData );
unsigned char GetUARTReceived( unsigned char* pBuffer );  		// user must make sure that pBuffer is at least SERIAL_BUFFER_SIZE


#endif  //  UART_H