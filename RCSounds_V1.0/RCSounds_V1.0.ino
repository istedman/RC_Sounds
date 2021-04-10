/*
 * RC Sounds Software
 * Version          1.0
 * Date             28th February 2021
 * Target & config  ATTiny 85 with Optiboot bootloader, 16MHz PLL, B.O.D. disabled, Timer1 clock set to CPU.
 *                  LTO set to enabled, Millis()/Micros() set to enabled.
 *                  Progrmmer USBAsp
 * Fues             Low       0xE1    
 *                  High      0xDD for development but use 0x5D for final, this disables reset so need high voltage programming to remove.
 *                  Extended  0xFE
 *                  LB        0xFF
 * Pinout and I/O   PB0       Servo PWM input
 *                  PB1       Aux input
 *                  PB2       LED, active low
 *                  PB3       Serial TX pin to MP3 module
 *                  PB4       Serial RX pin from MP3 module
 *                  PB5/A0    Analgue input, reset disabled, also has setup switch.
 * 
 * Description
 *                  This software controls  DFRobot DFR0299 Mini MP3 player using a software serial port. It is used to create a sound effects module 
 *                  for radio control vehicles.
 *                  Currently used for boats but could be used for cars or planes. It is fixed pitch so cannot change as speed increases.
 *                  There is the option to have a horn sound effect. This is achieved by mixing the continuous engine sound and a pre-determined 
 *                  horn sample. To play the horn the software switches to an advert, this allows a quick selection of another audio track.
 *                  The analogue input connects to 4 switches via a resistive divider, this allows easy selection of the sound effect in the field.
 *                  
 *                  This used the DFPlayerMini_fast library as the stock DFRobot library did not switch neatly to adverts and back again.
 *                  
 *                  You can connect a TTL serial cable to the RX pin and GND, to monitor the debug messages.
 *                  The MP3 module ignores all text strings.
 *             
 */

//#include <DFPlayerMini_Fast.h>
#include "DFRobotDFPlayerMini.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(4, 3); // RX, TX for the MP3 module

//DFPlayerMini_Fast myMP3;
DFRobotDFPlayerMini myMP3;
#define LEDPIN        PB2
#define SEL_HORN      PB0
#define SELECTION_IN  A0
#define AUX_IN        PB1
#define LEDON         LOW
#define LEDOFF        HIGH
#define SWITCHCLOSED  0x77
#define SWITCHOPEN    0x66

#define DEBUG 33

// For EEPROM setup
#define E_INIT 511

// Variables for Sound effect selection these are the scaled voltages seen at the input
const float NAVY_DIESEL_LOWER =  4.12;    // All switches open
const float NAVY_DIESEL_UPPER =  4.30;    
const float TUG_DIESEL_LOWER  =  3.95;    // S4 only closed
const float TUG_DIESEL_UPPER  =  4.09;
const float STEAM_LOWER       =  3.60;    // S3 only closed
const float STEAM_UPPER       =  3.90;
const float SETUP_UPPER       =  1.60;
const float SETUP_LOWER        =  1.30;

enum Sound_Effect_Type{none=0,NavyDiesel=1,TugDiesel=2,SteamBoat=3};
enum Sound_Effect_Type Sounds=none;

//Globals to hold the ADC sampled value and the converted voltage.
float Scaled_V;
//Used when reading servo pulse input
unsigned int PulseLimit=0;
unsigned char GreatOrLessThan;

// Settings related to servo pulse positions
#define GREATERTHAN 0x33
#define LESSTHAN 0x22

#define TimerAdjustment 9   //9us to allow for servo pulse jitter.

#define DEBOUNCETIME 200

/* 
   This checks the PWM input pulse and will play the mixed horn sample if detected width matches the value in EEPROM
   The second method, will be used on the model, it reads width of the PWM pulse and if is within a window of values, 
   changes the MP3 track to an advert
*/
int PlayHorn()
{
  unsigned int PWMTime;
  // Use the servo PWM input
    PWMTime = pulseIn(SEL_HORN, HIGH);
   #ifdef DEBUG
    mySerial.print("PWMTime: ");
    mySerial.print(PWMTime);
    mySerial.print("\tPulseLimit: ");
    mySerial.println(PulseLimit);
    #endif
    
    if((GreatOrLessThan==GREATERTHAN && PWMTime > PulseLimit) || (GreatOrLessThan==LESSTHAN && PWMTime < PulseLimit))
    {
        switch(Sounds){
    
            case NavyDiesel:
            //myMP3.playAdvertisement(1);
            myMP3.advertise(1);
            break;
            case TugDiesel:
            //myMP3.playAdvertisement(2);
            myMP3.advertise(2);
            break;
            case SteamBoat:
            //myMP3.playAdvertisement(3);
            myMP3.advertise(3);
            break;
          }
        // Now switch the LED on to indicate we have detected a valid input.
      digitalWrite(LEDPIN,LEDON);
      /*The delay below is to allow the advert to play out uninterrupted.
       * DO NOT need to call stopAdvertisement as it resumes playback at the end where it left off.
      */
      delay(15000);
      digitalWrite(LEDPIN,LEDOFF);  // Signal end of HORN/Advert.
    }
} // End PlayHorn function

void setup()
{
  unsigned char LimLo, LimHi;
  mySerial.begin(9600);
  myMP3.begin(mySerial);
  delay(1000);
  myMP3.volume(28);
  delay(100);
  pinMode(SEL_HORN, INPUT_PULLUP); 
  pinMode(LEDPIN,OUTPUT);

 Scaled_V=AvgADCRead(SELECTION_IN);
  if (Scaled_V>SETUP_LOWER && Scaled_V<SETUP_UPPER)
  {
    DoOnOffSetup();
  }
  
  if (EEPROM.read(E_INIT) == 'T')
  {
      LimLo=EEPROM.read(0);
      LimHi=EEPROM.read(1);
      PulseLimit=LimHi<<8 | LimLo;
      GreatOrLessThan=EEPROM.read(2);
      #ifdef DEBUG
      mySerial.println("Use stored data");
      #endif
  }
  else{
    //First time, so store data in EEPROM, 1500us pulse width and Greaterthan
    EEPROM.write(0,0xDC);
    EEPROM.write(1,0x05);
    EEPROM.write(2,GREATERTHAN);
    EEPROM.write(E_INIT, 'T');
    PulseLimit=1500;
    GreatOrLessThan=GREATERTHAN;
    #ifdef DEBUG
    mySerial.print("INIT EEPROM");
    #endif
  }
  
       #ifdef DEBUG
      mySerial.print("Limlo: ");
      mySerial.print(LimLo);
      mySerial.print("\tLimhi: ");
      mySerial.print(LimHi);
      mySerial.print("Pulselimit=");
      mySerial.print(PulseLimit);
      mySerial.print("\t GreatOrLessThan 0x"); 
      mySerial.println(GreatOrLessThan,HEX);
      #endif 
  digitalWrite(LEDPIN,LEDON);
  delay(500);
  digitalWrite(LEDPIN,LEDOFF);
 
}

/*
 * Function to take 4 readings from the specified ADC channel, average them, conver to a voltage
 * and return it to the calling function.
 */

float AvgADCRead(unsigned char ADCInput)
{
  unsigned int AvgADC=0;
  unsigned char loopCNT=0;
  float AvgADCVal=0.0;
  
  for(loopCNT=0;loopCNT<4;loopCNT++)
  {
    AvgADC=AvgADC+analogRead(ADCInput);
    delay(2);   // ADC settle time
  }
  AvgADCVal= (float)((AvgADC*4.88E-3)/4);   // Calculate average of 4 readings
  return(AvgADCVal);
}

/*
 * Function to read the analogue input and see if the voltage matches the expected range of the setup switch.
 * Will return SWITCHPRESSED or SWITCHOPEN in response
 * SETUP_UPPER       =  1.60;
 * SETUP_LOWER       =  1.30;
 */

unsigned char ReadSetupSwitch()
{
  float ADCReading;
  ADCReading=AvgADCRead(SELECTION_IN);
  if (ADCReading >SETUP_LOWER && ADCReading < SETUP_UPPER)
  {
    return(SWITCHCLOSED);
  }
  else
  {
    return(SWITCHOPEN);
  }
  
  
}
/* Function to do the setup for a simple on/off switch.
 * Hold down the setup switch to enter, release it to start the setup.
 * Use the pulseIN() function to read the PWM input
 * It can work with greater than a pulse limit or less than a pulse limit. This is stored in EEPROM
 * As we are reading an analogue input, which uses a push switch, looking for >SETUP_LOWER & <SETUP_HIGHER, 
 * we still need to debounce.
 *
 */ 
int DoOnOffSetup()
{
    unsigned int Pulse1, Pulse2;
    unsigned char Byte1, Byte2;
    unsigned char Dbounce1=0, Dbounce2=0;
    unsigned char LED1=0;
    unsigned char i=0;
    float SetupSWVal=0.0;
    digitalWrite(LEDPIN,LEDON);
    Dbounce1=ReadSetupSwitch();
    //Wait for switch release
    do
    {
        delay(DEBOUNCETIME);
        Dbounce1=ReadSetupSwitch();
        
    }while(Dbounce1!=SWITCHCLOSED);
    
    delay(1000);
    
    // Now we need the off position, wait for user to press button
    do
    {
        Dbounce1=ReadSetupSwitch();
        LED1= !LED1; //Toggle LED1
        digitalWrite(LEDPIN,LED1);
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );

    Pulse1=pulseIn(SEL_HORN, HIGH);

    digitalWrite(LEDPIN,LEDON);
    delay(1000);
    digitalWrite(LEDPIN,LEDOFF);
 
    //Now we need the full extent/on value, let user release switch
    do
    {
        Dbounce1=ReadSetupSwitch();
        LED1=!LED1;
        digitalWrite(LEDPIN,LED1);
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHCLOSED && Dbounce2==SWITCHCLOSED );      
    
    digitalWrite(LEDPIN,LEDON);
    //Now move to final value and close switch.
    // Now we need the off position, wait for user to press button
    
     do
    {
        Dbounce1=ReadSetupSwitch();
        LED1=!LED1;
        digitalWrite(LEDPIN,LED1);
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );  
 
    
     Pulse2 = pulseIn(SEL_HORN, HIGH);   // get the timer value

     digitalWrite(LEDPIN,LEDOFF);
    delay(1000);
  
    /* We should have two pulse limits in pulse1 & pulse2
     * If we subtract pulse2 from pulse 1 and it is positive, we are looking for greater than pulse2-TimerAdjustment
     * so we store pulse2 and GREATERTHAN to EEPROM
     * otherwise if pulse2-pulse1 is negative, we are looking for less than pulse 2 so we store pulse2+TimerAdjustmen
     * and LESSTHAN to EEPROM
     */ 
 
    if(Pulse2>Pulse1)
    {
        Pulse2=Pulse2-TimerAdjustment;
        Byte1=(unsigned char)(Pulse2>>8);
        Byte2=(unsigned char)(Pulse2&0xFF);
        EEPROM.write(0,Byte1);
        EEPROM.write(1,Byte2);
        EEPROM.write(2,GREATERTHAN);
    }
    else if (Pulse1>Pulse2)
    {
        Pulse2=Pulse2+TimerAdjustment;
        Byte1=(unsigned char)(Pulse2>>8);
        Byte2=(unsigned char)(Pulse2&0xFF);
        EEPROM.write(0,Byte1);
        EEPROM.write(1,Byte2);
        EEPROM.write(2,LESSTHAN);
    }
    //Now flash LEDs to indicate it is done
    for (i=0;i<4;i++)
    {
        LED1=!LED1;
        digitalWrite(LEDPIN,LED1);
        delay(DEBOUNCETIME);
    }
      // Back to main routine
    return(23);
}

void loop()
{
 //First check the switch input to know what sample to play
  
 Scaled_V=AvgADCRead(SELECTION_IN);
 #ifdef DEBUG
 mySerial.print("Voltage: ");
 mySerial.println(Scaled_V);
 #endif
 delay(1000);

if (myMP3.available()) 
  {
      printDetail(myMP3.readType(), myMP3.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

// Change the sample if needed
  if (Scaled_V>NAVY_DIESEL_LOWER & Scaled_V<NAVY_DIESEL_UPPER & Sounds!=NavyDiesel)
  {
      myMP3.loop(1); // 
      Sounds=NavyDiesel;
      #ifdef DEBUG
      mySerial.print("Navy Diesel");
      #endif
  }
  else if (Scaled_V>TUG_DIESEL_LOWER & Scaled_V<TUG_DIESEL_UPPER & Sounds!=TugDiesel)
  {
      myMP3.loop(2); // 
      Sounds=TugDiesel;
      #ifdef DEBUG
      mySerial.print("Tug");
      #endif
  }
  
   else if (Scaled_V>STEAM_LOWER & Scaled_V<STEAM_UPPER& Sounds!=SteamBoat)
  {
      myMP3.loop(3); // 
      Sounds=SteamBoat;
      #ifdef DEBUG
      mySerial.println("Steam Engine");
      #endif
  }
    PlayHorn();   // See if we need to change to a horn sound effect.

}// End loop()

void BlinkLED(byte numTimes)
{
  byte numblinks=0;
  while(numblinks<numTimes)
  {
      digitalWrite(LEDPIN,LEDON);
      delay(300);
      digitalWrite(LEDPIN,LEDOFF);
      delay(700);
      numblinks++;
  }    
  
}

void printDetail(uint8_t type, int value){
  switch (type) {
     case DFPlayerPlayFinished:
     BlinkLED(1);
     break;
    case TimeOut:
      BlinkLED(2);
      break;
    case WrongStack:
      BlinkLED(3);
      break;
    case DFPlayerCardInserted:
      BlinkLED(4);
      break;
    case DFPlayerCardRemoved:
      BlinkLED(5);
      break;
    case DFPlayerCardOnline:
      BlinkLED(6);
      break;

    case DFPlayerError:
     BlinkLED(15);
      switch (value) {
        case Busy:
          BlinkLED(7);
          break;
        case Sleeping:
          BlinkLED(8);;
          break;
        case SerialWrongStack:
          BlinkLED(9);
          break;
        case CheckSumNotMatch:
          BlinkLED(10);
          break;
        case FileIndexOut:
          BlinkLED(11);
          break;
        case FileMismatch:
          BlinkLED(12);
          break;
        case Advertise:
          BlinkLED(13);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
