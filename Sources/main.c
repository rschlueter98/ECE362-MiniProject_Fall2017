/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Spring 2017
***********************************************************************
	 	   			 		  			 		  		
 Team ID: < 34 >

 Project Name: < QuadCopter >

 Team Members:

   - Team/Doc Leader:   < Ryan >    Signature: ______________________
   
   - Software Leader:   < Ryan >    Signature: ______________________

   - Interface Leader:  < Ryan >    Signature: ______________________

   - Peripheral Leader: < Ryan >    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to .... < Make a motors spin SUPER fast. >


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. get motors

 2. make them spin fast

 3. ??????

 4. Profit

 5.

***********************************************************************

  Date code started: < ? >

  Update history (add an entry every time a significant change is made):

  Date: < ? >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >


***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void readInputs(void);
void motorStartup(void);
void motorShutdown(void);
void motorIdle(void);
void updateRotor1(int);
void updateRotor2(int);
void updateRotor3(int);
void updateRotor4(int);
int mapInput(int);
int checkBounds(int);
void updateLCD(void);





/* Variable declarations */
//9S12 Variables
#define RTICTLI 0x1F
int timerCounter = 0;
int timerFlag = 0;
int temp = 30;    ///Devlin code
int currpb1 = 0;
int currpb2 = 0;
int prevpb1 = 0;
int prevpb2 = 0;
int leftpb = 0;
int rghtpb = 0;


//LCD Variables
int shiftout_counter = 0;
int lcdwait_counter = 0;
int pmsglcd_counter = 0;
int cursorLoc;

//Drone Input Variables
char checker = 0x00;
char readOut = 0x00;

//Drone Control Variables
#define MAX_ROTOR_PWM 0x04      //////////////////////////////////////////////////////////////////////////Define this after testing
#define SHUTDOWN_PWMDTY 0x00    //////////////////////////////////////////////////////////////////////////Define this after testing
#define STARTUP_PWMDTY 0x01     //////////////////////////////////////////////////////////////////////////Define this after testing
#define IDLE_PWMDTY 0x02        //////////////////////////////////////////////////////////////////////////Define this after testing

//Motor PWM Setup Variables
#define MOTOR_PER 0xff
#define MOTOR_SCL 0x03
#define MOTOR_CLK 0x21


int rollInput=1500;     //Channel 1 input from FS-iA6 receiver
int pitchInput=1500;    //Channel 2 input from FS-iA6 receiver
int throttleInput=1000; //Channel 3 input from FS-iA6 receiver
int yawInput=1500;      //Channel 4 input from FS-iA6 receiver
int armInput=1000;      //Channel 5 input from FS-iA6 receiver
int modeInput=1000;     //Channel 6 input from FS-iA6 receiver

int roll=0;             //Current 9S12 roll value
int pitch=0;            //Current 9S12 pitch value
int throttle=0;         //Current 9S12 throttle value
int yaw=0;              //Current 9S12 yaw value
int armed=0;            //Current 9S12 armed value
int mode=0;             //Current 9S12 mode value

int speed=0;            //speed and out are for formatting PPM input to PWMDTY output
int out=0;

int rotor1Speed=0;      //These values are the final values for the PWMDTY for each of the motors
int rotor2Speed=0;      //make sure to pass through checkBounds() first to make sure they are within a
int rotor3Speed=0;      //good range and are not going to blow anything up
int q = 0;
int rotor4Speed=0;

   	   			 		  			 		       

/* Special ASCII characters */
#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10		// RS pin mask (PTT[4])
#define RW 0x20		// R/W pin mask (PTT[5])
#define LCDCLK 0x40	// LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 = 0x80	// LCD line 1 cursor position
#define LINE2 = 0xC0	// LCD line 2 cursor position

	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */

/* Initialize SPI for baud rate of 6 Mbs */
  DDRM = 0xff;
  SPICR1 = 0x50;
  SPICR2 = 0x00;
  SPIBR = 0x01;

// Other initializations  
  DDRT = 0xff; //sets up ddrt as output
  ATDDIEN = 0xC0; //program PAD7 and PAD6 pins as digital inputs


//PWM Initializations
  MODRR = 0x1F;				// Turn channels 0-4 to PWM output from regular digital output
  PWME = 0x1F;				// Turn on pwm output
  PWMPOL = 0x1F;			// Set active high polarity
  PWMCTL  = 0x00;         // no concatenate (8bit)
  PWMCAE  = 0x00;         // left-aligned output mode
  PWMPRCLK  = MOTOR_CLK;  // set Clock B = -- MHz (prescaler 128) rate


//Buzzer Setup
// initialize PWM Ch 0 (left aligned, positive polarity, max 8-bit period)
  PWMPER0 = 0xFF;           // movb	#$FF,PWMPER0	; set maximum 8-bit period                       //this already divides by 255
  PWMDTY0 = 0xaF;           // movb	#$00,PWMDTY0	; initially clear DUTY register
  PWMCLK = 0x01;            // movb	#$00,PWMCLK	; select Clock A for Ch 0
  PWMPRCLK = 0xf1;          // movb	#$01,PWMPRCLK	; set Clock A = 12 MHz (prescaler = 2) rate

//Motor 1 PWM Setup
  PWMPER1 = MOTOR_PER;    // set maximum 8-bit period
  PWMDTY1 = 0x00;         // initially clear Duty Register
  PWMSCLB = MOTOR_SCL;    // 
  PWMCLK_PCLK1  = 1;      // select Clock SB for Ch 1

//Motor 2 PWM Setup
  PWMPER2 = MOTOR_PER;    // set maximum 8-bit period
  PWMDTY2 = 0x7f;         // initially clear Duty Register
  PWMSCLB = MOTOR_SCL;    // 
  PWMCLK_PCLK2  = 1;      // select Clock SB for Ch 2
  
//Motor 3 PWM Setup
  PWMPER3 = MOTOR_PER;    // set maximum 8-bit period
  //PWMPER3 = 0x37;
  PWMDTY3 = 0x00;         // initially clear Duty Register
  PWMSCLB = MOTOR_SCL;    // 
  //PWMSCLB = 10;
  PWMCLK_PCLK3  = 1;      // select Clock SB for Ch 3

//Motor 4 PWM Setup
  PWMPER4 = MOTOR_PER;    // set maximum 8-bit period
  PWMDTY4 = 0x00;         // initially clear Duty Register
  PWMSCLB = MOTOR_SCL;    // 
  PWMCLK_PCLK4  = 1;      // select Clock SB for Ch 4
	            
/* Initialize interrupts */
	      
	//Timmer module initialization
	TSCR1_TEN = 1;
	TIOS = 0x80;
	TSCR2 = 0x0c;
	TC7 = 1500;
	TIE_C7I = 1;
	
	//RTI initialization 2.048ms or something like that
	RTICTL=RTICTLI;
  CRGINT = CRGINT | 0x80;
	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  DisableInterrupts
  initializations(); 		  			 		  		
  EnableInterrupts;
	
  motorStartup();
  motorIdle();

  for(;;) {
  
/* < start of your main loop > */ 

  //Devlin's code to test Adjusting PWM output to Rotors
  //if(timerFlag){
 	//  if(rghtpb){
 	//    rghtpb = 0;
 	//    temp++;
 	//  } 
 	//  else if(leftpb){
 	//    leftpb = 0;
 	//    temp--;
 	//  }
 	//  if(temp == 255){
 	//    temp = 0; 
 	//  }                              
 	//  
  //  PWMDTY3 = temp;
    
    //*****************************************************************
    //Start of Actual Code//
    //*****************************************************************
    
    readInputs();  //Reads in the inputs from the arduino
    
    //FAILSAFE
    //If data read in is not correct, out of value, shut down drone
    
    
    if(armed){
      updateRotor1(rotor1Speed);
      updateRotor2(rotor2Speed);
      updateRotor3(rotor3Speed);
      updateRotor4(rotor4Speed);  
    } else{
      motorShutdown();  
    }
    
    if(mode==2){
      //LED pin on, Speaker pwm on
    }else if(mode==1){
      //LED pin on, Speaker pwm off
    }else{
      //LED pin off, Speaker pwm off
    }
    
    //updateLCD();
  
  

  
   } /* loop forever */
   
}   /* do not leave main */


void readInputs(void){
  armed=0;                //If anything goes wrong in this loop, this should turn the rotor speed off upon completion
  
  checker=SPISR_SPIF;       //Readout extra variables until it gets back to the starting one
  while (!checker){
    checker=SPISR_SPIF;  
    
    for(q = 0; q < 2000; q++)
    {
    		
    }
  }
  
  PORTA=checker;  
  
  for(;;) {
  	asm
  	{
  		ldaa 0x69
  		staa 0x6969
  	}
  }
  
  readOut = inchar();
  rollInput = inchar();
  readOut = inchar();
  pitchInput = inchar();
  readOut = inchar();
  throttleInput = inchar();
  readOut = inchar();
  yawInput = inchar();
  readOut = inchar();
  armInput = inchar();
  readOut = inchar();
  modeInput = inchar();
  
  
  roll=mapInput(rollInput);
  pitch=mapInput(pitchInput);
  throttle=mapInput(throttleInput);
  yaw=mapInput(yawInput);
  
  
  if(armInput >= 1950 && armInput <= 2050){
    armed = 1;
  }else{
    armed = 0;
  }
  
  if(modeInput >= 1950 && modeInput <=2050){
    mode=2;  //Turn lights and buzzer on
  }else if(modeInput >= 1450 && modeInput <=1550){
    mode=1;  //Turn lights only on
  } else{
    mode=0;  //Turn everything off
  
  
  rotor1Speed=IDLE_PWMDTY + throttle+5 - roll + pitch;      //Starting with adding IDLEPWM_DTY gives an idle speed when sticks are rpy are centered and t is zeroed
  rotor1Speed=checkBounds(rotor1Speed);                      //+5 negated mapInput's lowest bound of -5. This way 1000 on the throttle will be 0, not -5
  
  rotor2Speed=IDLE_PWMDTY + throttle+5 - roll - pitch;
  rotor2Speed=checkBounds(rotor2Speed);
  
  rotor3Speed=IDLE_PWMDTY + throttle+5 + roll + pitch;
  rotor3Speed=checkBounds(rotor3Speed);
  
  rotor4Speed=IDLE_PWMDTY + throttle+5 + roll - pitch;
  rotor4Speed=checkBounds(rotor4Speed);
  }
}

int mapInput(speed){
  if(speed<1000){       //Check to make sure the value isn't too negative
    out=0;  
  }else if(speed<1100){ //Between 1000 and 1100  //All the way left/down
    out=-5;
  }else if(speed<1200){ //Between 1100 and 1200
    out=-4;
  }else if(speed<1300){ //Between 1200 and 1300
    out=-3;
  }else if(speed<1400){ //Between 1300 and 1400
    out=-2;
  }else if(speed<1450){ //Between 1400 and 1450
    out=-1;
  }else if(speed<1550){ //Between 1450 and 1550  //Middle Position
    out=0;
  }else if(speed<1600){ //Between 1550 and 1600
    out=1;
  }else if(speed<1700){ //Between 1600 and 1700
    out=2;
  }else if(speed<1800){ //Between 1700 and 1800
    out=3;
  }else if(speed<1900){ //Between 1800 and 1900
    out=4;
  }else if(speed<2000){ //Between 1900 and 2000  //All the way right/up
    out=5;
  }else{
    out=0;  
  }
  
}

int checkBounds(speed){
  if(speed>MAX_ROTOR_PWM){ speed=MAX_ROTOR_PWM; }
  if(speed<IDLE_PWMDTY){ speed=IDLE_PWMDTY; }
  return speed;
}
                             
void motorStartup(void){
  updateRotor1(STARTUP_PWMDTY);
  updateRotor2(STARTUP_PWMDTY);
  updateRotor3(STARTUP_PWMDTY);
  updateRotor4(STARTUP_PWMDTY);  
}

void motorShutdown(void){
  updateRotor1(SHUTDOWN_PWMDTY);
  updateRotor2(SHUTDOWN_PWMDTY);
  updateRotor3(SHUTDOWN_PWMDTY);
  updateRotor4(SHUTDOWN_PWMDTY);  
}

void motorIdle(void){
  updateRotor1(IDLE_PWMDTY);
  updateRotor2(IDLE_PWMDTY);
  updateRotor3(IDLE_PWMDTY);
  updateRotor4(IDLE_PWMDTY);  
}


void updateRotor1(int speed){
  PWMDTY0 = speed;  
}
void updateRotor2(int speed){
  PWMDTY1 = speed;  
}
void updateRotor3(int speed){
  PWMDTY2 = speed;  
}
void updateRotor4(int speed){
  PWMDTY3 = speed;  
}



/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
***********************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80;
  	
  	//***UPDATE PUSHBUTTONS***UPDATE PUSHBUTTONS***UPDATE PUSHBUTTONS**
  	//***UPDATE PUSHBUTTONS***UPDATE PUSHBUTTONS***UPDATE PUSHBUTTONS**
  	currpb1 = PTAD_PTAD7;    //Left
    currpb2 = PTAD_PTAD6;    //Right
    if(currpb1==0&&prevpb1==1){leftpb=1;}
    if(currpb2==0&&prevpb2==1){rghtpb=1;}	
    prevpb1 = currpb1;
    prevpb2 = currpb2;
    
    
}

/*
***********************************************************************                       
  TIM interrupt service routine	  		
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  	// clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80; 
 	
 	timerCounter++;
 	if(timerCounter >= 500){
 	  timerCounter = 0;
 	  timerFlag=1;
 	}


}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
 


}

//***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***
//***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***LCD***
/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  It should shift MSB first.  
             
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/
 
void shiftout(char x)
{
 if(SPISR_SPTEF==1){        // read the SPTEF bit, continue if bit is 1
    SPIDR=x;                // write data to SPI data register
    for (shiftout_counter=0;shiftout_counter<30;shiftout_counter++);     // wait for 30 cycles for SPI data to shift out
  }
}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
  for(lcdwait_counter=0;lcdwait_counter<8000;lcdwait_counter++){ }
}

/*
*********************************************************************** 
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
  shiftout(x);              //shift out character
  PTT_PTT4=0;               //pulse LCD clock line low->
  PTT_PTT4=1;               //                     high->
  PTT_PTT4=0;               //                     low
  lcdwait();                //wait 2 ms for LCD to process data
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD  
***********************************************************************
*/

void send_i(char x)
{
  PTT_PTT2=0;               // set the register select line low (instruction data)
  send_byte(x);             // send byte
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
  send_i(CURMOV);           //Tell the LCD to expect a cursor location
  send_i(x);                //Give the LCD the cursor location
  cursorLoc=x;              //Move the cursor variable to the same location
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
  PTT_PTT2=1;               // set the register select line high (character data)
  send_byte(x);             // send the byte of the input character
  cursorLoc++;;             // then increment cursorLoc so the next character gets put in the next location
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[], int length)
{
  for(pmsglcd_counter=0; pmsglcd_counter<length; pmsglcd_counter++){
    print_c(str[pmsglcd_counter]);
  }
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}