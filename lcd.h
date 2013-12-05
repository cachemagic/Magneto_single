////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//						LCD.H
//
//		This is the LCD Control File. 
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifndef LCD_H_
#define LCD_H_

#include "hardware.h"
 

#define     LCD_DIR               P2DIR 
#define     LCD_OUT               P2OUT
 
#define     LCD_DDIR               PJDIR 
#define     LCD_DOUT               PJOUT


// 
// Define symbolic LCM - MCU pin mappings
// We've set DATA PIN TO 4,5,6,7 for easy translation
//
 
#define     LCD_PIN_RS            BIT7          // P2.0
#define     LCD_PIN_EN            BIT4          // P2.1

#define     LCD_PIN_D7            BIT3          // P3.3
#define     LCD_PIN_D6            BIT2          // P3.2
#define     LCD_PIN_D5            BIT1          // P3.1
#define     LCD_PIN_D4            BIT0          // PJ.0

#define     LCD_PIN_MASK  ((LCD_PIN_RS | LCD_PIN_EN))
#define     LCD_DPIN_MASK  ((LCD_PIN_D7 | LCD_PIN_D6 | LCD_PIN_D5 | LCD_PIN_D4))

#define     FALSE                 0
#define     TRUE                  1
 
void PulseLcd();
//void SendByte(char ByteToSend, int IsData);
void LcdSetCursorPosition(char Row, char Col);
void ClearLcdScreen(); 
void InitializeLcd(void);
void DisplayChar(char cChar);
void DisplayString(char *Text);
void DisplayDecimal(unsigned short uNumber);
void DisplayHex(unsigned short uNumber);
void RefreshLCD(void); 

#endif /* LCD_H_ */
