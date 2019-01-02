/***************************************************************
This program is written for the MCP23X08/17 GPIO Exp Keypad/LCD
demo board (Board Part #: GPIODM-KPLCD)

Written using Microchips C18 Toolsuite
****************************************************************/

//#include	"p18f4550.h"
#include	"I2C.H"
#include	"SPI.H"
#include	"MCP23X17.H"
#include	"MCP23X08.H"

//***FUNCTION PROTOTYPES************************
void isr(void);
void Init_PIC(void);
void Init_LCD(void);
void Init_Keypad(void); 
void Key_Scan(void);
void Key_Press(void);
void Change_Mode(void); 
void Clear_LCD(void);
unsigned char Check_Cursor(void);
unsigned char LCD_Write_INST(unsigned char display_value);
unsigned char LCD_Write_DATA(unsigned char display_value);
void Splash_Screen(void);
void Get_Device_8bit(void);
void Get_Device_16bit(void); 
void Write23X08_17(unsigned char reg, unsigned char data);
unsigned char Read23X08_17(unsigned char reg);
void SetSSP(unsigned char mode);
void I2CWriteByte(unsigned char reg, unsigned char data);
unsigned char I2CReadByte(unsigned char reg);
void SPIWriteByte(unsigned char reg, unsigned char data);
unsigned char SPIReadByte(unsigned char reg);
void Delay_ms(long num_ms);

//***DEFINES***********************************
#define I2CMODE 0
#define SPIMODE 1
#define Mode00  0
#define Mode11  1
#define CSL PORTBbits.RB3 = 0
#define CSH PORTBbits.RB3 = 1

// Address bits for the GPIOs 
#define MCP23S17  0x02   // SPI 16-bit
#define MCP23S08  0x04   // SPI 8-bit
#define MCP23017  0x06   // I2C 16-bit
#define MCP23008  0x08   // I2C 8-bit

// LCD control bit setup                          R/S  R/W   E  (N/A)
#define WRT_READ  0x60    // Read Data from LCD    0    1    1    0
#define WRT_INST  0x20    // Write Instr to LCD    0    0    1    0
#define WRT_DATA  0xA0    // Write DATA to LCD     1    0    1    0

// Button map
#define	Key_1		0x77
#define	Key_2		0x7B      
#define	Key_3		0x7D
#define	Key_M		0x7E   // Mode Select
#define	Key_4		0xB7
#define	Key_5		0xBB
#define	Key_6		0xBD
#define	Key_B		0xBE   // Back Space
#define	Key_7		0xD7
#define	Key_8		0xDB
#define	Key_9		0xDD
#define	Key_R		0xDE	 // CR/CLR
#define	Key_T		0xE7   // '*'
#define	Key_0		0xEB
#define	Key_LB	0xED   // '#'
#define	Key_S		0xEE	 // Space

//***GLOBALS**********************************
unsigned char	ButtonFlag = 0;
unsigned char	Key_Value;               
unsigned char	Current_Mode = I2CMODE;		// I2C for default startup
unsigned char	gAddrPins;               
unsigned char I2CBaudGenerator = 0x12;	//0x12 = 100kHz I2C
unsigned char SPIClkMode = Mode00;
unsigned char gControlByte = 0x40;
unsigned char Line = 0; 								// For checking cursor

//---------------------------------------------------------------------
// Interrupt Code
//---------------------------------------------------------------------
// Locate ISR handler code at interrupt vector
#pragma code isrcode=0x08
void isrhandler(void)	// This function directs execution to the
{								      // actual interrupt code										
_asm
goto isr
_endasm
}
#pragma code

/****************************************************************
	ISR() for keypress
****************************************************************/
void isr (void)
{
	unsigned char n;
  if(INTCON3bits.INT2IF)      // Interrupt from keypress
  {
    ButtonFlag = 1;
    INTCON3bits.INT2IE = 0;
    INTCON3bits.INT2IF = 0;
  }

  INTCONbits.GIE = 1;
}

/*****************************************************************
Main
  1.  Initialize the Pic
  2.  Setup default mode for GPIO's (I2C)
  3.  Initialize the LCD
  4.  Display the splash screen
  5.  Initialize the keypad
  6.  Wait for button press
         -Scan the keys when a button is pressed
         -Determine which key was hit and perform desired action
         -Reenable the interrupt
******************************************************************/
void main (void)
{	
  Init_PIC();
  SetSSP(I2CMODE);
  Init_LCD();
  Splash_Screen();
  Init_Keypad();

  while(1)
  {
    if(ButtonFlag)
    {
      Key_Scan();
      Key_Press();
      INTCON3bits.INT2IF = 0;       //Clear the interrupt flag
      INTCON3bits.INT2IE = 1;       //Enable interrupt
    }
  }  
}

/*************************************************************
   Function Name:  Init_PIC                                         
   Return Value:   void                                           
   Parameters:     void                     
   Description:    Initializes the PIC       
**************************************************************/
void Init_PIC (void)
{
  //Configure PORTA
  ADCON1 = 0x0F;            // Set all digital pins
  PORTA = 0x10;             // RA4 is HIGH on power up (SPI LED OFF)
  TRISAbits.TRISA1 = 1;     // USB_Attach detect (NOT USED)
  TRISAbits.TRISA2 = 1;     // Self_Power detect (NOT USED)
  TRISAbits.TRISA4 = 0;     // SPI LEDs
  PORTAbits.RA4 = 1;        // SPI LED = OFF
                            
  //Configure PORTB         
  PORTB = 0x00;             // 
  TRISBbits.TRISB1 = 0;		  // SCL/SCK
  TRISBbits.TRISB2 = 1;     // INT pin
  TRISBbits.TRISB3 = 0;		  // Chip Select
                            
  //Configure PORTC         
  PORTC = 0x00;             
  TRISCbits.TRISC0 = 0;     // Reset for SPI slaves
                            
  //Configure PORTD         
  PORTD = 0x00;             // RD4 is LOW on power up (I2C LED ON)
  TRISDbits.TRISD4 = 0;     // I2C LEDs
  TRISDbits.TRISD5 = 0;     // Reset for I2C slaves
                            
  //Configure Timers        
  T0CONbits.T0CS = 0;			  // Timer 0 source is CLK0
  T2CONbits.T2OUTPS2 = 1;   // Postcaler bit for Timer 2 1:7 Postscale
  T0CONbits.T0PS1 = 1;		  // Prescaler bit for Timer 0 1:16 prescale
  T0CONbits.T0PS0 = 1;		  // Prescaler bit for Timer 0 1:16 prescale

  //Other
  OSCCON = 0x72;	            // Set internal RC frequency  8MHz

  //Configure INTs
  INTCON2bits.INTEDG2 = 0;    // INT on falling edge
  INTCON3bits.INT2IP = 0;     // INT2 Priority
  INTCON3bits.INT2IF = 0;      
  INTCON3bits.INT2IE = 1;     // Enable INT pin
  INTCONbits.PEIE = 1;
  INTCONbits.GIE = 1;         // Global interrupt enable
}

/*************************************************************
   Function Name:  Init_LCD                                        
   Return Value:   void                                           
   Parameters:     void                     
   Description:    Initializes the LCD       
   
   1. Wait more than 15 ms after power up.
   2. Write 0x30 to LCD and wait 5 ms for the instruction to complete
   3. Write 0x30 to LCD and wait 1 ms for instruction to complete
   4. Write 0x30 to LCD 
   5. Set the operating Characteristics of the LCD
**************************************************************/
void Init_LCD (void)
{  
  Clear_LCD();		
  
  // Setup of the 16-bit GPIO for the LCD
  Write23X08_17(IODIRA,  0x00);    
  Write23X08_17(IODIRB,  0x00);
  	             
  Delay_ms (15);			        
  
  LCD_Write_INST(0x30);
 	Delay_ms (5);
 
  LCD_Write_INST(0x30);
 	Delay_ms (1);
 	
 	LCD_Write_INST(0x30);
 	Delay_ms (1);
 
  LCD_Write_INST(0x38);
  Delay_ms (1);
 
  // LCD operating characteristics
  LCD_Write_INST(0x18);       
  LCD_Write_INST(0x0D);       
  LCD_Write_INST(0x01);       
  LCD_Write_INST(0x06);       	
}

/******************************************************************
   Function Name: Init_Keypad                                          
   Return Value:  void                                           
   Parameters:    void                      
   Description:   Initializes the MCP23X08 devices for key scan  
                  ***The order in which the registers are enabled/set
                  is important for reliable and accurate operation 
*******************************************************************/
void Init_Keypad(void)
{
  if (Current_Mode == SPIMODE)
  {
    Get_Device_8bit();
			
		//(SPI ONLY) Enable hardware addressing on 8 and 16 bit SPI devices
    Write23X08_17(IOCON, 0x08);   // Sets 8 bit HAEN bit
    Write23X08_17(IOCONB, 0x08);  // Sets 16 bit HAEN bit
    
    Write23X08_17(GPPU, 0x0F);    // Set pull up resistors
    Write23X08_17(GPIO, 0x00);    // Row is kept LOW while changing I/O
    Write23X08_17(IODIR, 0xF0);   // Rows = inputs; Columns = outputs
    Write23X08_17(INTCON, 0xF0);  // Low nibble compares against DEFVAL
    Write23X08_17(DEFVAL, 0xF0);  // INT if any bits in low nibble = 0  
    Write23X08_17(GPINTEN, 0xF0); // Interrupt-on-change of Rows  	
	}
	
	else if (Current_Mode == I2CMODE)
	{
    Get_Device_8bit();
    
    Write23X08_17(GPPU, 0x0F);    // Set Pull-up resistors
    Write23X08_17(GPIO, 0x00);    // Row is kept LOW while changing I/O
    Write23X08_17(IODIR, 0xF0);   // Rows = inputs; Columns = outputs
    Write23X08_17(INTCON, 0xF0);  // High nibble compares against DEFVAL
    Write23X08_17(DEFVAL, 0xF0);  // INT if any bits in high nibble = 0
    Write23X08_17(GPINTEN, 0xF0); // Interrupt on change of Rows  
	}
}

/*****************************************************************
     Function Name:  Key_Scan                                 
     Return Value:                                          
     Parameters:       
     Description:   Scans keypad and places key value into global
                    variable Key_Value
                    1. Read INTCAP register to get Row value
                    2. Switch the I/O
                    3. Read the port pins to get Column value
                    4. Combine the Row and Column values
                    5. Place the result into Key_Value
                    6. Flip the I/O back to the default setup
                    7. Wait for the key to be released       
                    ***The order in which the registers are enabled/set
                    is important for reliable and accurate operation  
*******************************************************************/
void Key_Scan()
{
  unsigned char dummy;
	unsigned char r;
  unsigned char c;

  Get_Device_8bit();
		 
  // Read INTCAP to obtain the row value  
  r = Read23X08_17(INTCAP);       // Row Value 
  
  // Flip the I/O to scan for column value 
    Write23X08_17(IODIR, 0x0F);   // Rows = ouputs; Columns = inputs
    Write23X08_17(DEFVAL,0x0F);   // INT if any bits in low nibble = 0    
    Write23X08_17(INTCON, 0x0F);  // Low nibble compares against DEFVAL 
    Write23X08_17(GPINTEN, 0x0F); // INT-on-change for the Columns
    
    // Read what is on the port and combine with Row to get Key_Value  
    c = Read23X08_17(GPIO);  
  
    Key_Value = r | c;
    
    // Set the I/O back to it's orignal setup
    Write23X08_17(IODIR, 0xF0);   // Rows = inputs; Columns = outputs
    Write23X08_17(INTCON, 0xF0);  // High nibble compares against DEFVAL
    Write23X08_17(DEFVAL, 0xF0);  // INT if any bits in high nibble = 0  
    Write23X08_17(GPINTEN, 0xF0); // INT-on-change for the Rows    
  
    // Read the port then check to see if there is still an interrupt, 
    // keep checking until interrupt is cleared.
    // The interrupt corresponds to a key still depressed
    do                                   
    {                                    
      Delay_ms(5);                     
      dummy = Read23X08_17(INTCAP);   // Read INTCAP to clear interrupt
    }                                    
    while(!(PORTBbits.INT2));
    
   ButtonFlag = 0;
}

/*****************************************************************
   Function Name: Key_Press                                         
   Return Value:  void	                                            
   Parameters:    void                     
   Description:   Reads Key_Value and determines correct action
                  to take 
******************************************************************/
void Key_Press ()
{
  unsigned char n;
 	
switch (Key_Value)
{
	case Key_0:            
		LCD_Write_DATA('0');         //0x30
	break;
		
	case Key_1:
		LCD_Write_DATA('1');        //0x31
	break;
	
	case Key_2:	
		LCD_Write_DATA('2');	
	break;
	
	case Key_3:	
		LCD_Write_DATA('3');
	break;
	
	case Key_4:
		LCD_Write_DATA('4');
	break;
	
	case Key_5:
		LCD_Write_DATA('5');
	break;
	
	case Key_6:
		LCD_Write_DATA('6');	
	break; 

	case Key_7:	
		LCD_Write_DATA('7');	
	break;
	
	case Key_8:
		LCD_Write_DATA('8');
	break;
	
	case Key_9:	
		LCD_Write_DATA('9');
	break;
	
	case Key_M:                       // Switch Modes
	 	Change_Mode();	    
	break;
	
	case Key_B:                       // Back Space
    // Move back 1 space
		Write23X08_17(GPIOB, 0x80);       
		n = Check_Cursor();              
    LCD_Write_INST(0x80 | (n-1));
    
    // Write a blank space
		LCD_Write_DATA(0x20);
		
		// Move back 1 space
		Write23X08_17(GPIOB, 0x80);
		n = Check_Cursor();
    LCD_Write_INST(0x80 | (n-1));
	break;
	
	case Key_R:                 // Carriage return/Clear
    if (Line==0)              // If on line 1 go to line 2
    {     
		  LCD_Write_INST(0xC0);
		  Line=1;
		  } 
		else if (Line==1)         // If on line 2 clear screen
		{
  		LCD_Write_INST(0x01);
  		Line=0;
  		}
	break;
	
	case Key_S:                 // Space    
		LCD_Write_DATA(0x20);
	break;
	
	case Key_T:                 // '*'      
		LCD_Write_DATA('*');                        // 0x2A
	break;
	
	case Key_LB:                // '#' 	                0x23
		LCD_Write_DATA('#');
	break;
	
	default:                // '?' If 2 keys are simultaneously pressed
		LCD_Write_DATA('?');          //0x3F
	break;
	}	
	
  Check_Cursor();         //check cursor position,
                          //If at the end of a line go to the next line
}

/*************************************************************
   Function Name:  Change_Mode                                         
   Return Value:                                              
   Parameters:                          
   Description:   Switches devices on key press      
**************************************************************/
void Change_Mode (void)
{
  unsigned char dummy;
  unsigned char count;	
  unsigned char SPI_SS[] =	"    SPI Mode                            "
	                          "   MCP23S08/17   ";	
  unsigned char I2C_SS[] =	"    I2C Mode                            "
	                          "   MCP23008/17   ";
  
  // If Current_Device is set to SPI enable I2C
  if (Current_Mode == SPIMODE) 
 	 {			  		 
		SetSSP(I2CMODE);      // Change to I2C Mode
  	Init_LCD();	
			// Display mode selected to LCD
			for (count = 0; I2C_SS[count]; count++)
				{
				LCD_Write_DATA(I2C_SS[count]);
				}   
  	Delay_ms(2000);
		Init_Keypad();																			                              
  	Clear_LCD();        
	 }
	
	// Else if Current_Device is set to I2C enable SPI  
	else if (Current_Mode == I2CMODE)
	 {			  	
		SetSSP(SPIMODE);            // change to SPI Mode	       
		Init_LCD();
			// Display mode selected to LCD 
			for (count = 0; SPI_SS[count]; count++)  
				{
					LCD_Write_DATA(SPI_SS[count]);
				}
  	Delay_ms(2000);
		Init_Keypad();																		                              
  	Clear_LCD();	
  		 
  	Delay_ms(10);
  	Get_Device_8bit();           //	gAddrPins = SPI_23X08;
  	dummy = Read23X08_17(GPIO);  // Read the port to clear INTCAP
   }	
}

/*****************************************************************
     Function Name:    Clear_LCD                             
     Return Value:                                             
     Parameters:       
     Description:      This function clears the LCD and brings the
                       cursor to the top left position.
*******************************************************************/
void Clear_LCD(void)
{       	
  LCD_Write_INST(0x01);	
}	
  
/*****************************************************************
     Function Name:    Check_Cursor                             
     Return Value:                                             
     Parameters:       
     Description:      This function checks to see if the cursor is 
                       at the end of the screen and sends it to the 
                       next line if necessary
*******************************************************************/
unsigned char Check_Cursor(void)
{
  unsigned char n;
  Get_Device_16bit();
  
  Write23X08_17(IODIRA, 0xFF);    // Set port to input to read the LCD
  Write23X08_17(GPIOB, WRT_READ); // Sets LCD control bits for reading
  n = Read23X08_17(GPIOA);        // Puts the value read into n
  Write23X08_17(GPIOB, 0x40);     // Turn off LCDE
  Write23X08_17(IODIRA, 0x00);    // Reset the Port for output
  
  if (n>0x10)    // If the cursor has been moved to the second line 
    Line = 1;
  
  if (n == 0x10)
  {
   LCD_Write_INST(0xC0);
   Line = 1;
  }
    
  if (n == 0x50)
  {
   LCD_Write_INST(0x80);
   Line = 0;
  }             
  return(n);
}

/*****************************************************************
   Function Name:  LCD_Write_INST                                         
   Return Value:                                              
   Parameters:                         
   Description:    Writes an Instructin to the LCD
                     -Determine device to write to
                     -Setup GPIOB LCD control bits for Instruction
                     -Write the Instruction to GPIOA
                     -Disable the LCD control bits on GPIOB
******************************************************************/
unsigned char LCD_Write_INST(unsigned char display_value)
{
  Get_Device_16bit();

	Write23X08_17(GPIOB, WRT_INST);	 		      
	Write23X08_17(GPIOA, display_value); 
	Write23X08_17(GPIOB, 0x00);	 		   
}

/*****************************************************************
   Function Name:  LCD_Write_DATA                                         
   Return Value:   void                                           
   Parameters:     void                    
   Description:    Displays value onto LCD from Diplay_Value
                   -Determine device to write to
                   -Setup control bits on LCD for writing of Data
                   -Write the data to the LCD
                   -Disable the LCD control bits on GPIOB
******************************************************************/
unsigned char LCD_Write_DATA(unsigned char display_value)
{
  Get_Device_16bit();

  Write23X08_17(GPIOB, WRT_DATA);	 	                    
	Write23X08_17(GPIOA, display_value); 
	Write23X08_17(GPIOB, 0x80);	 		    
}
	
/*************************************************************
   Function Name: Splash_Screen                                         
   Return Value:  	                                            
   Parameters:                        
   Description:   Outputs a splash screen at start up 
**************************************************************/
void Splash_Screen ()
{
  unsigned char count;
  unsigned char S_S[] =  "    Microchip                           "
		                     "  GPIODM-KPLCD";

	for (count = 0; S_S[count]; count++)
	  LCD_Write_DATA(S_S[count]);
		
	Delay_ms(2000);
	Clear_LCD();
}

/*****************************************************************
     Function Name:    Get_Device_8bit                             
     Return Value:                                             
     Parameters:       
     Description:      This function returns the address for the 
                       appropriate 8 bit device to be written to 
                       or read from                   
*******************************************************************/
void Get_Device_8bit(void)
{
  if (Current_Mode == SPIMODE)
    gAddrPins = MCP23S08;
  else if (Current_Mode == I2CMODE)
    gAddrPins = MCP23008;
}
  
/*****************************************************************
     Function Name:    Get_Device_16bit                             
     Return Value:                                             
     Parameters:       
     Description:      This function returns the address for the 
                       appropriate 16 bit device to be written to
                        or read from                   
*******************************************************************/
void Get_Device_16bit(void)  
{
  if (Current_Mode == SPIMODE)
    gAddrPins = MCP23S17;
  else if (Current_Mode == I2CMODE)
    gAddrPins = MCP23017;
}

/******************************************************************
   Function Name:  Write23X08_17                                         
   Return Value:                                              
   Parameters:     Register address, Data                    
   Description:    Writes to a 23X08_17 register. I2C or SPI is in
                   global byte     
******************************************************************/
void Write23X08_17(unsigned char reg, unsigned char data)
{
	if(Current_Mode == I2CMODE)	
 	  I2CWriteByte(reg, data);  
	else											
	  SPIWriteByte(reg, data); 
}

/*****************************************************************
   Function Name:  Read23X08_17                                         
   Return Value:                                              
   Parameters:     Register address                    
   Description:    Reads a 23X08_17 register. I2C or SPI is in
                   global byte     
******************************************************************/
unsigned char Read23X08_17(unsigned char reg)
{
  unsigned char num;
	if(Current_Mode == I2CMODE)	
		num = I2CReadByte(reg);
	else											
		num = SPIReadByte(reg);
	return(num);
}

/*****************************************************************
	SetSSP(mode)
******************************************************************/
void SetSSP(unsigned char mode)
{  
	SSPCON1bits.SSPEN = 0;				      // disable other peripheral

  // Setup I2C
	if (mode == I2CMODE)
	{  
    //Sets up the indicator LED's and puts opposite devices in reset
	  PORTAbits.RA4 = 1;      // SPI LED's OFF
	  PORTDbits.RD4 = 0;      // I2C LED's ON
	  PORTDbits.RD5 = 1;	    // I2C devices active
	  PORTCbits.RC0 = 0;	    // Hold SPI devices in reset	

    OpenI2C(MASTER, SLEW_OFF);

		SSPCON1bits.CKP = 1;
		SSPADD = I2CBaudGenerator;
		DDRBbits.RB1 = 0;		    // Clear latch
		DDRBbits.RB4 = 0;		    // Clear latch
		TRISBbits.TRISB0 = 1; 	// SDA
		TRISBbits.TRISB1 = 1; 	// SCL
		CSH;                    // Raise SPI CS to deselect parts
		
		Current_Mode = I2CMODE;	// Write I2C as Current_Mode
  }

  // Setup SPI
  else if (mode == SPIMODE)
  {
    // Sets up the indicator LED's and puts opposite devices in reset
  	PORTAbits.RA4 = 0;     // SPI LED's ON
	  PORTDbits.RD4 = 1;	   // I2C LED's OFF			
  	PORTDbits.RD5 = 0;     // Hold I2C devices in reset
  	PORTCbits.RC0 = 1;     // SPI devices active
  	
    if(SPIClkMode == Mode11)        //Mode 11
  		OpenSPI( SPI_FOSC_4, MODE_11, SMPMID);
    else if (SPIClkMode == Mode00)  //Mode 00
  		OpenSPI( SPI_FOSC_4, MODE_00, SMPMID );

		TRISBbits.TRISB3 = 0;		// CS
		TRISCbits.TRISC7 = 0;		// SDO
		TRISBbits.TRISB0 = 1;		// SDI
		TRISBbits.TRISB1 = 0;		// SCK
		CSH;
	
		Current_Mode = SPIMODE; 					 //Write SPI as Current_Mode
  }
}

/*****************************************************************
	I2CWriteByte(unsigned char addr, unsigned char byte)
	Writes a byte to the 23017
*****************************************************************/
void I2CWriteByte(unsigned char reg, unsigned char data)
{
	I2CStart();
	WriteI2C( gControlByte | WrtCmd | gAddrPins );
	WaitForACK();
	WriteI2C( reg );
	WaitForACK();
	WriteI2C( data );
	WaitForACK();
	I2CStop();
}

/*****************************************************************
	int I2CReadByte(unsigned char addr))
	Reads a byte from the MCP23X08/17	
******************************************************************/
unsigned char I2CReadByte(unsigned char reg)
{
	unsigned char num;
	I2CStart();
	WriteI2C( gControlByte | WrtCmd | gAddrPins );
	WaitForACK();
	WriteI2C( reg );
	WaitForACK();
	I2CReStart();;
	WriteI2C( gControlByte | RdCmd | gAddrPins );
	WaitForACK();
	num = ReadI2C();
	nACK();
	I2CStop();
	return(num);
}

/*****************************************************************
     Function Name:    SPIWriteByte                                
     Return Value:                                             
     Parameters:       register address, and data.               
     Description:      This routine performs a byte write.         
*******************************************************************/
void SPIWriteByte(unsigned char reg, unsigned char data )
{
 
  CSL;  // Enable SPI Communication to MCP23S08/17
  while( WriteSPI(gControlByte | WrtCmd | gAddrPins) );
  while( WriteSPI(reg) );
  while( WriteSPI(data) );
  CSH;  // Disable SPI Communication to MCP23S08/17
}

/*****************************************************************
     Function Name:    SPIReadByte                                
     Return Value:     Data at register                                        
     Parameters:       Register
     Description:      This routine performs a sequential write.         
*******************************************************************/
unsigned char SPIReadByte(unsigned char reg)
{
  unsigned char n;
  
  CSL;  // Enable SPI Communication to  MCP23S08/17
  while( WriteSPI(gControlByte | RdCmd | gAddrPins) );
  while( WriteSPI(reg) );
  n = ReadSPI();
  CSH;  // Disable SPI Communication to  MCP23S08/17
  
  return n;
}

/*************************************************************
   Function Name:  Delay_ms                                         
   Return Value:   void                                           
   Parameters:     Number of milliseconds                    
   Description:    Delay function
                   TMR0 times out every ~100 us     
**************************************************************/
void Delay_ms(long num_ms)
{
  long n,m;
  
  m = num_ms*8;

	TMR0H = 0x00;
	TMR0L = 0xE0;
	INTCONbits.TMR0IF = 0;
  
  for(n = 0; n < m; n++)
  {
    while(!INTCONbits.TMR0IF);  // Wait for timer flag to set
    INTCONbits.TMR0IF = 0;      // Clear flag
  }
}
