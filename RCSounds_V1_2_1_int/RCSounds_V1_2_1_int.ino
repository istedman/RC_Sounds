/*
 * RC Sounds Software
 * Version          1.2.1
 * Date             7th August 2023
 * Target & config  ATTiny 85 with no bootloader, 16MHz PLL, B.O.D. disabled, Timer1 clock set to CPU.
 *                  LTO set to enabled, Millis()/Micros() set to enabled.Save EEPROM set to 'Not retained'
 *                  Progrmmer USBAsp
 * Fues             Low       0xD1   /F1 Need 64ms startup timer when RSTDISBL turned on to allow for PSU 
 *                                   slow ramp. Fixed startup issues I experienced.
 *                  High      0xDF for development but use 0x5F for final, this disables reset so need high voltage programming to remove.
 *                  Extended  0xFF  Don't need self programming as there is no bootloader.
 *                  LB        0xFF
 * Pinout and I/O   PB0       Servo PWM input
 *                  PB1       Throttle input
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
 *                  You can connect a TTL serial cable to the RX pin and GND, to monitor the debug messages.
 *                  The MP3 module ignores all text strings.
 *             
 */

#include <DFPlayerMini_Fast.h>
#include "RC_Sounds.h"
#include <EEPROM.h>
#include <SoftSerial.h>     /* Allows Pin Change Interrupt Vector Sharing */

SoftSerial mySerial(4, 3); // RX, TX for the MP3 module

/*  
  SoftRcPulseIn HornIn;
  SoftRcPulseIn ThrotIn;
  */

/*
 * ALL SETUP IS DONE IN RC_Sounds.h, edit that to define the configuration
 */

//DFRobotDFPlayerMini myMP3;
DFPlayerMini_Fast myMP3;

//Best keeping this here as it's easy to change.
//#define DEBUG 33
//#define DEBUGPROG 1
//#define ERASESETTINGS 2
/*
 * Intialise the hardware, configuring the I/O as required 
 * Moved MP3 player init to the end to allow for other setup first.
 * 
 */

void setup()
{
  pinMode(LEDPIN,OUTPUT);
  digitalWrite(LEDPIN,LEDON);       // Switch LED on until MP3 initialised and playing
  mySerial.begin(9600);             // My debug software serial.
  myMP3.begin(mySerial);
  delay(1000);                      // Give it time to initialise
  myMP3.volume(28);                 // Maximum value without distortion
  pinMode(HORN_IN, INPUT_PULLUP); 
  pinMode(THROTTLE_IN,INPUT_PULLUP);

  Scaled_V=AvgADCRead(SELECTION_IN);
  if (Scaled_V>SETUP_LOWER && Scaled_V<SETUP_UPPER)
  {
    DoSetup();
  }
  /*
   * See if the letter 'T' has been stored, indicating valid data has been set by the user.
   * If it has, recall the settings to globals.
   */
  #ifndef ERASESETTINGS
  if (EEPROM.read(E_INIT) == 'T')
  {
    RecallSettings();
    CalculateGears();
  }
  #endif
  else{  /* First run through of the software, reset to default values by calling the function below */
      ResetSettings();
      RecallSettings();
      CalculateGears();
   }
    digitalWrite(LEDPIN,LEDOFF);       // Switch LED off now init is complete
    PWMFound=CheckforPWMInputs();      // Save unnecessary waits for PWM inputs later on.
    // For maintenance only and not in debug mode
    #ifndef DEBUG
    GetCurrentPWMVals();
    mySerial.println(F("\nEEPROM Settings:"));
    mySerial.print(F("Forward Throttle limit "));
    mySerial.println(ForwardThrot);
    mySerial.print(F("Neutral "));
    mySerial.println(NeutralThrot);
    mySerial.print(F("Reverse Throttle limit "));
    mySerial.println(ReverseThrot);
    mySerial.print(F("Throttle is "));
    if (ThrotNormReverse==NORMAL_THROT)
    {
      mySerial.println(F("normal."));
    }
    else
    {
      mySerial.println(F("reversed."));
    }
    mySerial.print(F("Horn limit "));
    mySerial.print(HornPulseLimit);
    mySerial.print(F(", must be "));
    if (HornGreatOrLessThan == GREATERTHAN)
    {
      mySerial.println(F(" > to operate."));
    }
    else
    {
      mySerial.println(F(" < to operate"));
    }
    mySerial.print(F("PWM detected:"));
    if (PWMFound==1 || PWMFound== 11)
    {
      mySerial.println(F("Horn OK "));
      mySerial.print(F("Current horn "));
      mySerial.println(HornPWM);      
    }
    if (PWMFound==10 || PWMFound== 11)
    {
      mySerial.println(F("Throttle OK "));
      mySerial.print(F("Current throttle "));
      mySerial.println(ThrottlePWM);
    }
    else
    {
      mySerial.println(F("No PWM found"));
    }
    #endif
    RunTime=millis();
}

byte CalculateGears(void)
{
   unsigned int GearCalc=0;
   if(ThrotNormReverse==NORMAL_THROT)
      {
          GearCalc=(ForwardThrot-NeutralThrot)/4;    // Get the difference and divide by 4
          FirstGear=NeutralThrot+GearCalc; 
          SecondGear=FirstGear+GearCalc;      
          ThirdGear=SecondGear+GearCalc;
          FourthGear=ThirdGear+GearCalc;
      }
      else if (ThrotNormReverse==REVERSE_THROT)
      {
          GearCalc=(NeutralThrot-ForwardThrot)/4;    // Get the difference and divide by 4
          FirstGear=NeutralThrot-GearCalc; 
          SecondGear=FirstGear-GearCalc;      
          ThirdGear=SecondGear-GearCalc;
          FourthGear=ThirdGear-GearCalc;
      }
      #ifdef DEBUG
      mySerial.print(F("Gears 1: "));
      mySerial.print(FirstGear);
      mySerial.print(F(" 2: "));
      mySerial.print(SecondGear);
      mySerial.print(F(" 3: "));
      mySerial.print(ThirdGear);
      mySerial.print(F(" 4: "));
      mySerial.print(FourthGear);
      mySerial.print(F(" Max: "));
      mySerial.print(ForwardThrot);
      mySerial.print(F(" Neutral: "));
      mySerial.print(NeutralThrot);
      mySerial.print(F(" Reverse: "));
      mySerial.println(ReverseThrot);
      #endif
      CurrentGear=0; // For initialisation to be sure in case PWM count is way off.
}

/*
 * Function to recall all settings from EEPROM and initilise the global variables.
 *  Easier to do this as I only have to concatenate two
 * bytes from EEPROM once. If read from EEPROM, would do this every time around loop.
 */
byte RecallSettings(void)
{
      byte LimLo,LimHi;
      
      LimLo=EEPROM.read(HORN_EEPROM_LO);
      LimHi=EEPROM.read(HORN_EEPROM_HI);
      HornPulseLimit=LimHi<<8 | LimLo;
      HornGreatOrLessThan=EEPROM.read(HORN_EEPROM_GT_LT);
      //Now read the throttle settings from EEPROM
      LimLo=EEPROM.read(THROT_EEPROM_FORWARD_LO);
      LimHi=EEPROM.read(THROT_EEPROM_FORWARD_HI);
      ForwardThrot=LimHi<<8 | LimLo;
      LimLo=EEPROM.read(THROT_EEPROM_NEUTRAL_LO);
      LimHi=EEPROM.read(THROT_EEPROM_NEUTRAL_HI);
      NeutralThrot=LimHi<<8 | LimLo;
      LimLo=EEPROM.read(THROT_EEPROM_REVERSE_LO);
      LimHi=EEPROM.read(THROT_EEPROM_REVERSE_HI);
      ReverseThrot=LimHi<<8 | LimLo;     
      ThrotNormReverse=EEPROM.read(THROT_EEPROM_DIRECTION);
}

     /*
     * For a first time, so store data in EEPROM, 1500us pulse width and Greater than for Horn. This gets around the issue
     * that the Arduino IDE/USBAsp cannot init EEPROM by default when programming a fresh device.
     *  Also store default throttle settings, idle of 1500us, full throttle of 2000us and reverse of 1000us.
     *  Also called if the user holds the setup button for 10 seconds
      */
int ResetSettings(void)
{
  
    EEPROM.write(HORN_EEPROM_LO,0xD0);
    EEPROM.write(HORN_EEPROM_HI,0x07); // 0x7d0 = 2000 (us)
    EEPROM.write(HORN_EEPROM_GT_LT,GREATERTHAN);
    //Now store the default throttle settings
   
    EEPROM.write(THROT_EEPROM_FORWARD_LO,0xD0);
    EEPROM.write(THROT_EEPROM_FORWARD_HI,0x07); // 0x7D0 = 2000 (us)
    EEPROM.write(THROT_EEPROM_NEUTRAL_LO,0xDC);
    EEPROM.write(THROT_EEPROM_NEUTRAL_HI,0x05); // 0x5DC = 1500 (us)
    EEPROM.write(THROT_EEPROM_REVERSE_LO,0xE8);
    EEPROM.write(THROT_EEPROM_REVERSE_HI,0x03); // 0x3E8 = 1000 (us)
    EEPROM.write(THROT_EEPROM_DIRECTION,NORMAL_THROT);
    EEPROM.write(E_INIT,'T');
    //A little LED flash or 4 to indicate settings reset
    BlinkLED(4, 700, 300);
}
/* 
   This checks the PWM input pulse and will play the mixed horn sample if detected width matches the value in EEPROM
   The second method, will be used on the model, it reads width of the PWM pulse and if is within a window of values, 
   changes the MP3 track to an advert
*/
int PlayHorn()
{
    unsigned int HornTime;
    HornTime=HornPWM;
    CurrentAdvert=SpeedIndex[Sounds][CurrentGear];

    if(HornTime >500 && HornTime <2500) // To detect lack of signal or seriously large signal and not toot your trumpet
    {
      
        if((HornGreatOrLessThan==GREATERTHAN && HornTime > HornPulseLimit) || (HornGreatOrLessThan==LESSTHAN && HornTime < HornPulseLimit))
        {
          if (CurrentAdvert != PreviousAdvert)
          {
              myMP3.playAdvertisement(CurrentSound);
              AdvertPlaying=1;    // Used to stop advert interruption
              AdTime=millis();
              digitalWrite(LEDPIN,LEDON);   // Now switch the LED on to indicate we have detected a valid input.
          }
              PreviousAdvert=CurrentAdvert;    
        }
    }
    /* After 13 seconds, we need to assume the horn input has gone and switch off the LED
     * and reset previous advert.
     */
     /*TimeSinceAd=millis();
       if (TimeSinceAd > AdTime+ 3000)
       */
       if(millis() >(AdTime+13000))
       {
          digitalWrite(LEDPIN,LEDOFF);  // Signal end of HORN/Advert.
          PreviousAdvert=0;
          AdvertPlaying=0;
       }
} // End PlayHorn function



/*
 * Function to take 4 readings from the specified ADC channel, average them, convert to a voltage
 * and return it to the calling function.
 */

//float AvgADCRead(byte ADCInput)
unsigned int AvgADCRead(byte ADCInput)
{
  unsigned int AvgADC=0;
  byte loopCNT=0;
  //float AvgADCVal=0.0;
  unsigned int AvgADCVal=0;
  
  for(loopCNT=0;loopCNT<4;loopCNT++)
  {
    AvgADC=AvgADC+analogRead(ADCInput);
    delay(2);   // ADC settle time
  }
  AvgADCVal= AvgADC>>2;   // Calculate average of 4 readings
  return(AvgADCVal);
}

/*
 * Function to read the analogue input and see if the voltage matches the expected range of the setup switch.
 * Will return SWITCHPRESSED or SWITCHOPEN in response
 */

byte ReadSetupSwitch()
{
  unsigned int ADCReading;
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

/*
 * Function to check if one or both PWM inputs are active in the setup phase.
 * It calls pulsein() on both potential inputs, looks for non-zero response and then returns 1 of 4 codes 
 * Return codes
 * 0    No pwm input
 * 1    Horn input only
 * 10   Throttle input only
 * 11   Horn and throttle present
 * ONLY USED IN SETUP NOW
 */

byte CheckforPWMInputs(void)
{
  int pwm1,pwm2;
  byte pwmfound=0;

  pwm1=pulseIn(HORN_IN,HIGH);
  pwm2=pulseIn(THROTTLE_IN,HIGH);
/*
 pwm1=HornIn.width_us();
 pwm2=ThrotIn.width_us();
 */
  if(pwm1>800 && pwm1 <2500)
  {
    pwmfound=1;   // Found horn PWM
  }
  if (pwm2>800 && pwm2 <2500)
  {
    pwmfound+=10; // Found throttle PWM
  }
  return(pwmfound); // Return result. Is initialised to zero in case no signals found
}

/* Function to do the setup for a simple on/off switch.
 * Hold down the setup switch to enter, release it to start the setup.
 * Use the pulseIN() function to read the PWM inputs
 * It can work with greater than a pulse limit or less than a pulse limit. This is stored in EEPROM
 * As we are reading an analogue input, which uses a push switch, looking for >SETUP_LOWER & <SETUP_HIGHER, 
 * we still need to debounce.
 *
 */ 
int DoSetup()
{
    unsigned long Timeout1=0, Timeout2=0;
    byte Dbounce1=0, Dbounce2=0;
    byte PWMAvailable=0;
    //float SetupSWVal=0.0;
    Dbounce1=ReadSetupSwitch();
    Timeout1=millis(); // read current time
    BlinkLED(6,300,300);
    //Wait for switch release
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();
        Timeout2=millis();
        if (Timeout2-Timeout1 >10000)
        {
          #ifdef DEBUG
          mySerial.println(F("Reset settings in Dosetup"));
          #endif
          ResetSettings();
          return(77); // Exit out now.
        }
        
    }while(Dbounce1==SWITCHCLOSED && Dbounce1==SWITCHCLOSED);
    
    PWMAvailable=CheckforPWMInputs();

    switch (PWMAvailable){

      case 1:             // Horn only
        DoHornSetup();
      break;

      case 10:            // ThrottleOnly
        DoThrottleSetup();
      break;
      
      case 11:            // Both
        DoHornSetup();
        DoThrottleSetup();
      break;

      case 0:               // No PWM detected so abort
      default:
            BlinkLED(6, 200, 800);
            return(67);
            #ifdef DEBUG
            mySerial.print(F("No PWM in setup"));
            #endif
     
        break;

    }// End case
    // Back to main routine
    return(23);
}

/*
 * Function to read the PWM input from the horn and store the values in EEPROM.
 * Can handle normal or reverse PWM inputs
 */
byte DoHornSetup(void)
{
    unsigned int Pulse1, Pulse2;
    byte Byte1, Byte2;
    byte Dbounce1=0, Dbounce2=0;
    byte LED1=0;
    byte i=0;
    
    // First we need the off position, wait for user to press button
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );

    Pulse1=pulseIn(HORN_IN, HIGH,22000);
    //Pulse1=HornIn.width_us();
    BlinkLED(1, 300, 300);

    //Now we need the full extent/on value, let user release switch
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHCLOSED && Dbounce2==SWITCHCLOSED );      
 
    //Now move to final value and close switch.
    // Now we need the off position, wait for user to press button
    
     do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );  
    
     Pulse2 = pulseIn(HORN_IN, HIGH,22000);   // get the timer value
     BlinkLED(2, 300, 300);
  
    /* We should have two pulse limits in pulse1 & pulse2
     * If we subtract pulse2 from pulse 1 and it is positive, we are looking for greater than pulse2-TimerAdjustment
     * so we store pulse2 and GREATERTHAN to EEPROM
     * otherwise if pulse2-pulse1 is negative, we are looking for less than pulse 2 so we store pulse2+TimerAdjustmen
     * and LESSTHAN to EEPROM
     */ 
 
    if(Pulse2>Pulse1)
    {
        Pulse2=Pulse2-TimerAdjustment;
        Byte1=(byte)(Pulse2&0xFF);
        Byte2=(byte)(Pulse2>>8);
        EEPROM.write(HORN_EEPROM_LO,Byte1);
        EEPROM.write(HORN_EEPROM_HI,Byte2);
        EEPROM.write(HORN_EEPROM_GT_LT,GREATERTHAN);
    }
    else if (Pulse1>Pulse2)
    {
        Pulse2=Pulse2+TimerAdjustment;
        Byte1=(byte)(Pulse2&0xFF);
        Byte2=(byte)(Pulse2>>8);
        EEPROM.write(HORN_EEPROM_LO,Byte1);
        EEPROM.write(HORN_EEPROM_HI,Byte2);
        EEPROM.write(HORN_EEPROM_GT_LT,LESSTHAN);
    }
     return(1);
} // End DoHornSetup()

/*
 * Function to read the PWM input from the throttle and store the values in EEPROM.
 * Can handle normal or reverse PWM inputs. We have to detect and store neutral, full forward throttle and full reverse throttle
 */

byte DoThrottleSetup(void)
{
    unsigned int NeutralPulse, ForwardPulse, ReversePulse;
    byte Byte1, Byte2;
    byte Dbounce1=0, Dbounce2=0;
    byte LED1=0;
    byte i=0;
    
    // First we need the neutral position, wait for user to press button
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );

    NeutralPulse=pulseIn(THROTTLE_IN, HIGH,22000);
    //NeutralPulse=ThrotIn.width_us();
    BlinkLED(3, 300, 300);
 
    //Now we need the full extent/on value, let user release switch
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHCLOSED && Dbounce2==SWITCHCLOSED );      
    
    //Now move to final value and close switch.
    // Now we need the off position, wait for user to press button
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );  
    BlinkLED(4, 300, 300);
    ForwardPulse = pulseIn(THROTTLE_IN, HIGH,22000);   // get the timer value


  //Now we need the reverse value, let user release switch
    do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHCLOSED && Dbounce2==SWITCHCLOSED );      
 
    //Now move to final value and close switch.
    // Now we need the off position, wait for user to press button
    
     do
    {
        Dbounce1=ReadSetupSwitch();
        delay(DEBOUNCETIME);
        Dbounce2=ReadSetupSwitch();        
    } while(Dbounce1==SWITCHOPEN && Dbounce2==SWITCHOPEN );  
    
     ReversePulse = pulseIn(THROTTLE_IN, HIGH,22000);   // get the timer value
     //ReversePulse=ThrotIn.width_us();
     BlinkLED(5, 300, 300);
    /* We should have three pulse limits in ForwardPulse, NeutralPulse & ReversePulse
     * If we subtract ForwardPulse from ReversePulse and it is positive, we are looking at a normal throtle so
     * we store NORMAL_THROT in THROT_EEPROM_DIRECTION
     * If we subtract ForwardPulse from ReversePulse and it is negative, we are looking at a reverse throtle so
     * we store REVERSE_THROT in THROT_EEPROM_DIRECTION
     * We store neutral position as-is regardl;ess of direction
     */ 
 
    if(ForwardPulse>ReversePulse)
    {
        ForwardPulse+=TimerAdjustment;    // Add PWM neutral adjustment
        ReversePulse-=TimerAdjustment;
        EEPROM.write(THROT_EEPROM_DIRECTION ,NORMAL_THROT);
    }
    else if (ReversePulse>ForwardPulse)
    {
        ForwardPulse-=TimerAdjustment;    // Add PWM neutral adjustment
        ReversePulse+=TimerAdjustment;
        EEPROM.write(THROT_EEPROM_DIRECTION,REVERSE_THROT);
    }

        Byte1=(byte)(ForwardPulse&0xFF);
        Byte2=(byte)(ForwardPulse>>8);
        EEPROM.write(THROT_EEPROM_FORWARD_LO,Byte1);
        EEPROM.write(THROT_EEPROM_FORWARD_HI,Byte2);
        Byte1=(byte)(NeutralPulse&0xFF);
        Byte2=(byte)(NeutralPulse>>8);
        EEPROM.write(THROT_EEPROM_NEUTRAL_LO,Byte1);
        EEPROM.write(THROT_EEPROM_NEUTRAL_HI,Byte2);
        Byte1=(byte)(ReversePulse&0xFF);
        Byte2=(byte)(ReversePulse>>8);
        EEPROM.write(THROT_EEPROM_REVERSE_LO,Byte1);
        EEPROM.write(THROT_EEPROM_REVERSE_HI,Byte2);     
      
    return(1);
  
}

/*
 * Function to read the throttle input, if used and return a number from 0-3
 * If ForwardThrot, NeutralThrot and ReverseThrot settings match the defaults, always return 0 as throttle is not in use
 */
byte CheckCurrentGear(void)
{
    byte PWMPresent;
    byte loopy;
    PWMPresent=PWMFound;	// Copy global

    if((PWMPresent==0)||(PWMPresent==1))  //Either no PWM inputs or just horn
    {
      return(0);
    }
    else
    {
       ThrottlePosition=ThrottlePWM;
       //Averaging filter
       total=total-readings[readIndex];
       readings[readIndex]=ThrottlePWM;
      // add the reading to the total
      total = total +readings[readIndex];
      readIndex++;
      if(readIndex>=numReadings)
      {
        readIndex=0; 
      }
      //Now calculate the all important average
      //ThrottlePosition=total/numReadings;
      ThrottlePosition=total>>2;
       
        if(ThrotNormReverse==NORMAL_THROT)
        {
                if ((ThrottlePosition > NeutralThrot-TimerAdjustment) && (ThrottlePosition < NeutralThrot+TimerAdjustment))
                {
                  return(0);  //Idle +/- timer adjustment
                }
           
                else if ((ThrottlePosition > NeutralThrot+TimerAdjustment+GearHysteresis) && (ThrottlePosition < FirstGear-GearHysteresis))
                //if ((ThrottlePosition > NeutralThrot+TimerAdjustment) && (ThrottlePosition < FirstGear))
                {
                  return(1); // First gear
                }
            
                else if((ThrottlePosition > FirstGear+GearHysteresis) && (ThrottlePosition < SecondGear-GearHysteresis))
                {
                    return(2); // Second gear
                }

                else if((ThrottlePosition > SecondGear+GearHysteresis) && (ThrottlePosition < ThirdGear-GearHysteresis))
                {
                    return(3); // Third gear
                }
           
                //else if((ThrottlePosition > ThirdGear) && (ThrottlePosition < FourthGear))
                else if(ThrottlePosition > ThirdGear+GearHysteresis)
                {
                    return(4); // Fourth gear
                }
                          
            if ((ThrottlePosition < NeutralThrot-TimerAdjustment) && (ThrottlePosition > ReverseThrot))    // Reverse always second gear
            {
                return(2); 
             }
        }
    
        else  // Reversed throttle channel from receiver
        {
            if ((ThrottlePosition < NeutralThrot+TimerAdjustment) && (ThrottlePosition > NeutralThrot-TimerAdjustment))
            {
              return(0);  //Idle
            }
            if ((ThrottlePosition < NeutralThrot-TimerAdjustment-GearHysteresis) &&  (ThrottlePosition > FirstGear+GearHysteresis))
            {
                return(1);
            }
            
            else if((ThrottlePosition < FirstGear-GearHysteresis) && (ThrottlePosition > SecondGear+GearHysteresis))
            {
                return(2);
            }
            else if((ThrottlePosition < SecondGear-GearHysteresis) && (ThrottlePosition > ThirdGear+GearHysteresis))
            {
                return(3);
            }
            else if(ThrottlePosition < ThirdGear)
            {
                return(4);
            }
            else if ((ThrottlePosition > NeutralThrot-TimerAdjustment-GearHysteresis) && (ThrottlePosition < ReverseThrot))    // Reverse always second gear
            {
                return(2); 
            }
        }
    }

} // End CheckCurrentGear()

/*
*   One function to read the current PWM inputs just once per loop and store them in two globals
*   ThrottlePWM and HornPWM. This is due to PulseIn() variants screwing up interrupt handling and
*   Software serial causing much grief.
    Now also contains a check for PWM inputs, used by setup functions.
* 	Possible  codes
* 	0    No pwm input
* 	1    Horn input only
* 	10   Throttle input only
* 	11   Horn and throttle present
*/
byte GetCurrentPWMVals()
{
  if (PWMFound==1 || PWMFound == 11);     // Looking for just the HORN or HORN + Throttle
	{
    HornPWM=pulseIn(HORN_IN,HIGH,22000);
	}
	if(PWMFound==10 || PWMFound==11)
  {
		ThrottlePWM=pulseIn(THROTTLE_IN,HIGH,22000);
  }
}

void loop()
{
    //First check the switches input to know what sample to play
    
    Scaled_V=AvgADCRead(SELECTION_IN);
    GetCurrentPWMVals();
    CurrentGear=CheckCurrentGear();
      //Now we now what type of sound to play and at what speed.
// Change the sample if needed

    if (AdvertPlaying==0)     // If no advert/horn then we chan change gear, else skip until finished
    {
      if (Scaled_V>NAVY_DIESEL_LOWER && Scaled_V<NAVY_DIESEL_UPPER)
      {
          Sounds=NavyDiesel;
          CurrentSound=SpeedIndex[Sounds][CurrentGear];
      
          if(CurrentSound!=PreviousSound)
          {
            myMP3.loop(CurrentSound);
          } 
          PreviousSound=CurrentSound;
      }
      
      else if (Scaled_V>TUG_DIESEL_LOWER && Scaled_V<TUG_DIESEL_UPPER)
      {
          Sounds=TugDiesel;
          CurrentSound=SpeedIndex[Sounds][CurrentGear];
        
          if(CurrentSound!=PreviousSound)
          {
            myMP3.loop(CurrentSound);
          }
          PreviousSound=CurrentSound;
      }
  
      else if (Scaled_V>STEAM_1_LOWER && Scaled_V<STEAM_1_UPPER)
      {
          Sounds=SteamBoat1;
          CurrentSound=SpeedIndex[Sounds][CurrentGear];
       
          if(CurrentSound!=PreviousSound)
          {  
              myMP3.loop(CurrentSound);
          }
          PreviousSound=CurrentSound;          
      }
      
      else if (Scaled_V>STEAM_2_LOWER && Scaled_V<STEAM_2_UPPER)
      {
          Sounds=SteamBoat2;
          CurrentSound=SpeedIndex[Sounds][CurrentGear];
       
          if(CurrentSound!=PreviousSound)
          {  
              myMP3.loop(CurrentSound);
          }
          PreviousSound=CurrentSound;      
      }
      
    }// End if Advertplaying==0
    PlayHorn();   // See if we need to change to a horn sound effect.
    //RunTime=millis();
    #ifdef DEBUG
    mySerial.print("millis=");
    mySerial.print(millis());
    mySerial.print("RunTime= ");
    mySerial.println(RunTime);
    #endif
    if (millis()>RunTime + 1000)  // >1000ms since last check.
    {
      PWMFound=CheckforPWMInputs();        // Update current PWM inputs.
      RunTime=millis();           // Set new runtime expiry
    }
    #ifdef DEBUG
    delay(350);
    mySerial.print(F("AdvertPlaying= "));
    mySerial.println(AdvertPlaying);
    mySerial.print(F("Throttle= "));
    mySerial.println(ThrottlePWM);
    mySerial.print(F("Horn= "));
    mySerial.println(HornPWM);
    #endif


}// End loop()

void BlinkLED(byte numTimes, word ontime, word offtime)
{
  byte numblinks=0;
  while(numblinks<numTimes)
  {
      digitalWrite(LEDPIN,LEDON);
      delay(ontime);
      digitalWrite(LEDPIN,LEDOFF);
      delay(offtime);
      numblinks++;
  }    
  
}
