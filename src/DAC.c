// DAC.c
extern unsigned char array[10];
#define SIZE 6

#include <stdio.h>
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"

extern void Delay(unsigned long ulCount);

#include "DAC.h"
#include "lm3s1968.h"



void SSI1Write(enum typeOfWrite type,unsigned char TxByte)
{
	if(type == COMMAND)
	{
    GPIO_PORTB_DATA_R &= ~GPIO_PIN_5;
  } 
	else
	{
    GPIO_PORTB_DATA_R |= GPIO_PIN_5;
  }
	
	GPIO_PORTB_DATA_R &= ~SCE; // SCE active low
	SSIDataPut(SSI1_BASE, TxByte);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= SCE; // SCE inactive high
}

void ResetPulseSSI1(void)
{
	GPIO_PORTB_DATA_R |= RST; // think it has to 1st be set high for start of pulse
	Delay(500);                // on logic analyzer its default was low meaning RST active
	GPIO_PORTB_DATA_R &= ~RST;// pulse it low (active)
	Delay(500);
	GPIO_PORTB_DATA_R |= RST; // pulse RST bit high (off)
}

void Nokia_LCD_InitSSI1(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	Delay(2);
	GPIO_PORTE_DIR_R |= 0x0F; // make PE0-3 all outputs
	
  GPIOPinConfigure(GPIO_PE0_SSI1CLK);
  GPIOPinConfigure(GPIO_PE1_SSI1FSS);
  GPIOPinConfigure(GPIO_PE2_SSI1RX);
  GPIOPinConfigure(GPIO_PE3_SSI1TX);
  GPIOPinTypeSSI(GPIO_PORTE_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; 
	Delay(2);
	
	// PB3 -  SCE pin 3 on LCD (active low)
	// PB4 -  RST pin 4 on LCD (active low)
	// PB5 -  D/C pin 5 on LCD (D=high, C=low)
	// PE3 - MOSI pin 6 on LCD
	// PE0 - SCLK pin 7 on LCD
	// initialize the DATA/CMD, RST, SCE outputs
	GPIO_PORTB_DIR_R |= (GPIO_PIN_5+RST+SCE); // PB5-3 outputs
	GPIO_PORTB_AFSEL_R &= ~(GPIO_PIN_5+RST+SCE); 
	GPIO_PORTB_DEN_R |= (GPIO_PIN_5+RST+SCE);
	GPIO_PORTB_DATA_R &= ~RST; // set RST bit low to be ready to be pulsed
	GPIO_PORTB_DATA_R |= SCE; // set SCE bit high to disable transmit
	
	// A RESET (RST) pulse must be applied. Attention should be paid to the
	// possibility that the device may be damaged if not properly reset.
	// The RST input must be 0.3*VDD when VDD reaches VDDmin
	// (or higher) within a maximum time of 100 ms after VDD
	// goes HIGH (see Fig.16). (from datasheet pg. 15)
	ResetPulseSSI1();
	
	//	After reset, the LCD driver has the following state:
	//· Power-down mode (bit PD = 1)
	//· Horizontal addressing (bit V = 0) normal instruction set (bit H = 0)
	//· Display blank (bit E = D = 0)
	//· Address counter X6 to X0 = 0; Y2 to Y0 = 0
	//· Temperature control mode (TC1 TC0 = 0)
	//· Bias system (BS2 to BS0 = 0)
	//· VLCD is equal to 0, the HV generator is switched off (VOP6 to VOP0 = 0)
	//· After power-on, the RAM contents are undefined
	//
  // Nokia 5110 specs
  // Max SPI Clock : 4.0 MHz
  // Data Order : MSB transmitted first
  // Clock Polarity: SPO = 1, start sending on falling clock edge
  // Clock Phase : SPH = 1, capture data on rising clock edge
  // Tx data only - no Rx
  // Send 8-bits data/cmd and controlled by the DATA/CMD bit (D=1, C=0)  
	//
  // We read and write 8 bits but we need the Stellaris to drive the clock.  SSI_FRF_MOTO_MODE_0
  // is full duplex.  When we do a write, all 8 bits are written.
  SSIEnable(SSI1_BASE);

	SSI1Write(COMMAND,LCD_EXTENDED_CMD); // 0x21
	SSI1Write(COMMAND,CONTRAST_DEFAULT); // 0xB8
	SSI1Write(COMMAND,TEMP_COEFF);       // 0x04
	SSI1Write(COMMAND,LCD_BIAS_MODE_0);  // 0x14
	SSI1Write(COMMAND,LCD_BASIC_CMD);    // 0x20
	//SSI1Write(COMMAND,ALL_ON);           // 0x09
	SSI1Write(COMMAND,NORMAL_MODE);
}

//--------Nokia5110_OutChar-----------------
// Print a character to the Nokia 5110 48x84 LCD.  The
// character will be printed at the current cursor position,
// the cursor will automatically be updated, and it will
// wrap to the next row or back to the top if necessary.
// One blank column of pixels will be printed on either side
// of the character for readability.  Since characters are 8
// pixels tall and 5 pixels wide, 12 characters fit per row,
// and there are six rows.
// inputs: data  character to print
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutChar(unsigned char data){
  int i;
  SSI1Write(DATA, 0x00);                 // blank vertical line padding
  for(i=0; i<5; i=i+1){
    SSI1Write(DATA, ASCII[data - 0x20][i]);
  }
  SSI1Write(DATA, 0x00);                 // blank vertical line padding
}

//--------Nokia5110_OutString-----------------
// Print a string of characters to the Nokia 5110 48x84 LCD.
// The string will automatically wrap, so padding spaces may
// be needed to make the output look optimal.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutString(char *ptr){
  while(*ptr){
    Nokia5110_OutChar((unsigned char)*ptr);
    ptr = ptr + 1;
  }
}

//--------Nokia5110_OutUDec-----------------
// Output a 16-bit number in unsigned decimal format with a
// fixed size of five right-justified digits of output.
// Inputs: n  16-bit unsigned number
// Outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutUDec(unsigned short n){
  if(n < 10){
    Nokia5110_OutString("    ");
    Nokia5110_OutChar(n+'0'); // n is between 0 and 9 
  } else if(n<100){
    Nokia5110_OutString("   ");
    Nokia5110_OutChar(n/10+'0'); // tens digit 
    Nokia5110_OutChar(n%10+'0'); // ones digit 
  } else if(n<1000){
    Nokia5110_OutString("  ");
    Nokia5110_OutChar(n/100+'0'); // hundreds digit 
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); // tens digit 
    Nokia5110_OutChar(n%10+'0'); // ones digit 
  }
  else if(n<10000){
    Nokia5110_OutChar(' ');
    Nokia5110_OutChar(n/1000+'0'); // thousands digit 
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); // hundreds digit
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); // tens digit 
    Nokia5110_OutChar(n%10+'0'); // ones digit 
  }
  else {
    Nokia5110_OutChar(n/10000+'0'); // ten-thousands digit 
    n = n%10000;
    Nokia5110_OutChar(n/1000+'0'); // thousands digit 
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); // hundreds digit 
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); // tens digit 
    Nokia5110_OutChar(n%10+'0'); // ones digit 
  }
}

//---------Nokia5110_SetCursor-----------------
// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=11)
//         newY  new Y-position of the cursor (0<=newY<=5)
// outputs: none
void Nokia5110_SetCursor(unsigned char newX, unsigned char newY){
  if((newX > 11) || (newY > 5)){        // bad input
    return;                             // do nothing
  }
  // multiply newX by 7 because each character is 7 columns wide
  SSI1Write(COMMAND, 0x80|(newX*7));     // setting bit 7 updates X-position
  SSI1Write(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
}

//--------Nokia5110_Clear-----------------
// Clear the LCD by writing zeros to the entire screen and
// reset the cursor to (0,0) (top left corner of screen).
// inputs: none
// outputs: none
void Nokia5110_Clear(void){
  int i;
  for(i=0; i<(MAX_X*MAX_Y/8); i++)
	{
    SSI1Write(DATA, 0x00);
  }
  Nokia5110_SetCursor(0, 0);
}

//--------Nokia5110_DrawFullImage-----------------
// Fill the whole screen by drawing a 48x84 bitmap image.
// inputs: ptr  pointer to 504 byte bitmap
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DrawFullImage(const char *ptr){
  int i;
  Nokia5110_SetCursor(0, 0);
  for(i=0; i<(MAX_X*MAX_Y/8); i=i+1){
    SSI1Write(DATA, ptr[i]);
  }
}

/*
void Nokia_LCD_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; 
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; 
	Delay(10);
	// initialize the DATA/CMD and RST outputs
	GPIO_PORTC_DIR_R |= (GPIO_PIN_5 + RST);     GPIO_PORTB_DIR_R |= 0x08; //PB3 will be used as SS for LCD
	GPIO_PORTC_AFSEL_R &= ~(GPIO_PIN_5 + RST);  GPIO_PORTB_AFSEL_R &= ~0x08; 
	GPIO_PORTC_DEN_R |= (GPIO_PIN_5 + RST);     GPIO_PORTB_DEN_R |= 0x08;
	GPIO_PORTC_DATA_R |= RST;           GPIO_PORTB_DATA_R |= 0x08;
	
	// A RESET (RST) pulse must be applied. Attention should be paid to the
	// possibility that the device may be damaged if not properly reset.
	// The RST input must be 0.3*VDD when VDD reaches VDDmin
	// (or higher) within a maximum time of 100 ms after VDD
	// goes HIGH (see Fig.16). (from datasheet pg. 15)
	
	ResetPulse();
	Delay(200);
	
	//	After reset, the LCD driver has the following state:
	//· Power-down mode (bit PD = 1)
	//· Horizontal addressing (bit V = 0) normal instruction set (bit H = 0)
	//· Display blank (bit E = D = 0)
	//· Address counter X6 to X0 = 0; Y2 to Y0 = 0
	//· Temperature control mode (TC1 TC0 = 0)
	//· Bias system (BS2 to BS0 = 0)
	//· VLCD is equal to 0, the HV generator is switched off (VOP6 to VOP0 = 0)
	//· After power-on, the RAM contents are undefined
	
	GPIO_PORTC_DATA_R &= ~GPIO_PIN_5; // set PA7 low for cmd  mode	
	
	// Initialize SSI1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinConfigure(GPIO_PE0_SSI1CLK);
  GPIOPinConfigure(GPIO_PE1_SSI1FSS);
  GPIOPinConfigure(GPIO_PE2_SSI1RX);
  GPIOPinConfigure(GPIO_PE3_SSI1TX);
  GPIOPinTypeSSI(GPIO_PORTE_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
	
	//
  // Nokia 5110 specs
  // Max SPI Clock : 4.0 MHz
  // Data Order : MSB transmitted first
  // Clock Polarity: SPO = 1, start sending on falling clock edge
  // Clock Phase : SPH = 1, capture data on rising clock edge
  // Tx data only - no Rx
  // Send 8-bits data/cmd and controlled by the DATA/CMD bit (D=1, C=0)  
	//
  // We read and write 8 bits but we need the Stellaris to drive the clock.  SSI_FRF_MOTO_MODE_0
  // is full duplex.  When we do a write, all 8 bits are written.
  // Since this mode is full duplex, we'll get 8 bits back but mask off the top 8, leaving only the read data we are
  // intSSI_FRF_MOTO_MODE_0
  SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
  SSIEnable(SSI1_BASE);
	//Delay(1000);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	SSIDataPut(SSI1_BASE, LCD_EXTENDED_CMD);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	Delay(10);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	SSIDataPut(SSI1_BASE, CONTRAST_DEFAULT);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	Delay(10);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	
	SSIDataPut(SSI1_BASE, TEMP_COEFF);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	Delay(10);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	SSIDataPut(SSI1_BASE, LCD_BIAS_MODE_0);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	Delay(10);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	SSIDataPut(SSI1_BASE, LCD_BASIC_CMD);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	Delay(10);
	
	GPIO_PORTB_DATA_R &= ~0x08;
	
	SSIDataPut(SSI1_BASE, ALL_ON);
	// Wait until SSI1 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{
	}
	GPIO_PORTB_DATA_R |= 0x08;
	
}
*/




