
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//						LCD.H
//
//		This is the 2x16 LCD Control File. 
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include "lcd.h"
 

// 
// Define symbolic LCM - MCU pin mappings
// We've set DATA PIN TO 4,5,6,7 for easy translation
//

////////////////////////////////////////////////////////////////////////////////
// This is the function that must be called 
// whenever the LCM needs to be told to 
// scan it's data bus.
//
// Parameters: void.
//
// Return void. 
//------------------------------------------------------------------------------
void PulseLcd()
{
    //
    // pull EN bit low
    //
    LCD_OUT &= ~LCD_PIN_EN;
    __delay_cycles(600);

    //
    // pull EN bit high
    //
    LCD_OUT |= LCD_PIN_EN;
    __delay_cycles(600);
 
    //
    // pull EN bit low again
    // 
    LCD_OUT &= (~LCD_PIN_EN);
    __delay_cycles(600);
}



////////////////////////////////////////////////////////////////////////////////
// Send a byte on the data bus in the 4 bit mode
// This requires sending the data in two chunks.
// The high nibble first and then the low nible
// 
// Parameters:
//
//    ByteToSend - the single byte to send
//
//    IsData - set to TRUE if the byte is character data
//                  FALSE if its a command
//
// Return     void.
//------------------------------------------------------------------------------
void SendByte(char ByteToSend, int IsData)
{
    //
    // clear out all pins
    //
    LCD_OUT &= (~LCD_PIN_MASK);
    LCD_DOUT &= (~LCD_DPIN_MASK);
    //
    // set High Nibble (HN) - 
    // usefulness of the identity mapping
    // apparent here. We can set the 
    // DB7 - DB4 just by setting P1.7 - P1.4 
    // using a simple assignment
    //
    LCD_DOUT |= ((ByteToSend & 0xF0) >> 4);		// (ByteToSend & 0xF0);
 
	if (IsData == TRUE)
	{
        LCD_OUT |= LCD_PIN_RS;
	}
    else
    {
        LCD_OUT &= ~LCD_PIN_RS;
    }
 
    //
    // we've set up the input voltages to the LCM.
    // Now tell it to read them. 
    // 
    PulseLcd();
     //
    // set Low Nibble (LN) -
    // usefulness of the identity mapping
    // apparent here. We can set the 
    // DB7 - DB4 just by setting P1.7 - P1.4 
    // using a simple assignment
    //
    LCD_OUT &= (~LCD_PIN_MASK);
    LCD_DOUT &= (~LCD_DPIN_MASK);
    LCD_DOUT |= ((ByteToSend & 0x0F));		//  << 4

    if (IsData == TRUE)
    {
        LCD_OUT |= LCD_PIN_RS;
    }
    else
    {
        LCD_OUT &= ~LCD_PIN_RS;
    }
 
    //
    // we've set up the input voltages to the LCM.
    // Now tell it to read them.
    // 
    PulseLcd();
}
 


////////////////////////////////////////////////////////////////////////////////
// Set the position of the cursor on the screen
// 
// Parameters:
//
//     Row - zero based row number
//
//     Col - zero based col number
// 
// Return    void.
//------------------------------------------------------------------------------
void LcdSetCursorPosition(char Row, char Col)
{
    char address;

    // 
    // construct address from (Row, Col) pair 
    //
    if (Row == 0)
    {
        address = 0;
    }
    else
    {
        address = 0x40;
    }

    address |= Col; 
    SendByte(0x80 | address, FALSE);
}
 






////////////////////////////////////////////////////////////////////////////////
// Clear the screen data and return the
// cursor to home position
// 
// Parameters:    void.
// 
// Return    void.
//------------------------------------------------------------------------------
void ClearLcdScreen()
{
    //
    // Clear display, return home
    //
    SendByte(0x01, FALSE);
    SendByte(0x02, FALSE);
}
 






////////////////////////////////////////////////////////////////////////////////
// Initialize the LCM after power-up.
// 
// Note: This routine must not be called twice on the
//           LCM. This is not so uncommon when the power
//           for the MCU and LCM are separate.
// 
// Parameters:    void.
// 
// Return    void.
//------------------------------------------------------------------------------
void InitializeLcd(void)
{
    //
    // set the MSP pin configurations 
    // and bring them to low
    //
    LCD_DIR |= LCD_PIN_MASK;
    LCD_OUT &= ~(LCD_PIN_MASK);

    LCD_DDIR |= LCD_DPIN_MASK;
    LCD_DOUT &= ~(LCD_DPIN_MASK);
	 
    //
    // wait for the LCM to warm up and reach
    // active regions. Remember MSPs can power 
    // up much faster than the LCM.
    //
    __delay_cycles(100000);
    
 
    // 
    // initialize the LCM module
    //
    // 1. Set 4-bit input 
    //
    LCD_OUT &= ~LCD_PIN_RS;
    LCD_OUT &= ~LCD_PIN_EN;

    LCD_DOUT = 0x02;	// 0x20; 
    PulseLcd();
    __delay_cycles(10000);

    //
    // set 4-bit input - second time. 
    // (as reqd by the spec.)
    // 
    SendByte(0x28, FALSE);
 
    //
    // 2. Display on, cursor off, blink cursor
    //
    SendByte(0x0C, FALSE);
  
    // 
    // 3. Cursor move auto-increment
    //
    SendByte(0x06, FALSE);
}
 

////////////////////////////////////////////////////////////////////////////////
//  Refresh LCD to clear garbled display
//------------------------------------------------------------------------------
void RefreshLCD(void)
{
    //
    // set 4-bit input - second time. 
    // (as reqd by the spec.)
    // 
    SendByte(0x28, FALSE);
 
    //
    // 2. Display on, cursor off, blink cursor
    //
    SendByte(0x0C, FALSE);
  
    // 
    // 3. Cursor move auto-increment
    //
    SendByte(0x06, FALSE);
}



////////////////////////////////////////////////////////////////////////////////
//  Display the character at the current cursor position
//------------------------------------------------------------------------------
void DisplayChar(char cChar)
{
   SendByte(cChar, TRUE);
}


////////////////////////////////////////////////////////////////////////////////
// Print a string of characters to the screen
//
// Parameters:
// 
//    Text - null terminated string of chars
//
// Returns void.
//------------------------------------------------------------------------------
void DisplayString(char *Text)
{
    char *c;

    c = Text;
    while ((c != 0) && (*c != 0)) 
    {
        SendByte(*c, TRUE);
        c++;
    }
}
 

////////////////////////////////////////////////////////////////////////////////
//  Display an unsigned decimal value on the LCD
//------------------------------------------------------------------------------
void DisplayDecimal( unsigned short uNumber )
{
	unsigned short uRemainder;
	unsigned char ucChar;

	uRemainder = uNumber;
	if ( uNumber >= 10000 )
	{
		ucChar = uNumber / 10000;
		SendByte( (ucChar + 0x30), TRUE );	
		uRemainder = uRemainder - (ucChar * 10000);
	}
	if ( uNumber >= 1000 )
	{
		ucChar = uRemainder / 1000;
		SendByte( (ucChar + 0x30), TRUE );	
		uRemainder = uRemainder - (ucChar * 1000);
	}		
	if ( uNumber >= 100 )
	{
		ucChar = uRemainder / 100;
		SendByte( (ucChar + 0x30), TRUE );				
		uRemainder = uRemainder - (ucChar * 100);
	}	
	if ( uNumber >= 10 )
	{
		ucChar = uRemainder / 10;
		SendByte( (ucChar + 0x30), TRUE );				
		uRemainder = uRemainder - (ucChar * 10);
	}
	SendByte( (uRemainder + 0x30), TRUE );		
}


////////////////////////////////////////////////////////////////////////////////
//  Display an hexadecimal number on the LCD
//------------------------------------------------------------------------------
void DisplayHex( unsigned short uNumber )
{
	unsigned char ucChar;
	
//	ucChar = (uNumber & 0xF000) >> 12;
//	HEX_NIBBLE_TO_ASCII(ucChar);
//	DisplayChar( ucChar );

	ucChar = (uNumber & 0x0F00) >> 8;
	HEX_NIBBLE_TO_ASCII(ucChar);
	DisplayChar( ucChar );

	ucChar = (uNumber & 0x00F0) >> 4;
	HEX_NIBBLE_TO_ASCII(ucChar);
	DisplayChar( ucChar );
				
	ucChar = (uNumber & 0x000F);
	HEX_NIBBLE_TO_ASCII(ucChar);
	DisplayChar( ucChar );
}















