/*
 * RC_Sounds.h
 * This provides all the configuration for the RC_Sounds software.
 */


#define LEDPIN        PB2
#define HORN_IN       PB0
#define SELECTION_IN  A0
#define THROTTLE_IN   PB1
#define LEDON         LOW
#define LEDOFF        HIGH
#define SWITCHCLOSED  0x77
#define SWITCHOPEN    0x66


/*
 * These defines might seem extraneous but they allow easier configuration if the code is used for Boats, Cars/Trucks
 * or trains. They are compile time directives and only the code defined gets built.
 */

#define BOAT
//#define CAR
/* Constants for Sound effect selection these are the scaled voltages seen at the input with +/-0.15V tolerance.
 *  All calculated values for reference
 *  Switch(es)  Equivalent resistance   Upper limit   Nominal   Lower limit
 *  None        100K                    4.69V         4.54V     4.39V
 *  S4          31.972K                 4.05V         3.90V     3.75V
 *  S3          15.27K                  3.50V         3.35V     3.20V
 *  S2          9.09K                   2.65V         2.50V     2.35V            
 *  S1          4.48K                   1.85V         1.70V     1.55V
 *  S5          2.15K                   1.03V         0.88V     0.73V
 *  S4+S3                               3.15V         3.00V     2.85V    
 *  S4+S2                               2.50V         2.35V     2.20V
 *  S3+S2                               2.30V         2.15V     2.00V       
 *  
 */
#ifdef BOAT
const int NAVY_DIESEL_UPPER =  961;        
const int NAVY_DIESEL_LOWER =  899;      // 4.69 to 4.39V All switches open
const int TUG_DIESEL_UPPER  =  829;    // S4 only closed 4.05V to 3.75V
const int TUG_DIESEL_LOWER  =  780;    
const int STEAM_1_UPPER     =  717;    // S3 only closed
const int STEAM_1_LOWER     =  655;    
const int STEAM_2_UPPER     =  543;    // S2 only closed
const int STEAM_2_LOWER     =  481;

//const int MIN_PULSE=750;
//const int MAX_PULSE=2500;

enum Sound_Effect_Type{NavyDiesel=0,TugDiesel=1,SteamBoat1=2,SteamBoat2=3,None=99};
/*
 * Multi-dimensional array to pick the sound effect played. 
 * Column 0 is the default set of sounds i.e. idle, column 1 is 1st gear, column 2 is 2nd gear, column 3 is 3rd gear, column 4 is 4th gear
 * Row 0 is Navy diesel, Row 1 is Tug diesel, Row 2 is Steam boat, Row 3 is Steam launch.
 * This same array, used as a lookup table, also selects the horn sound.
 * So Navy diesel sound #5, for example, would select Advert #5 if needed, which blends sample #5 with the horn
 */
byte SpeedIndex[4][5]={
                        {1,2,3,4,5},        // Navy diesel sounds
                        {6,7,8,9,10},       // Tug diesel sounds
                        {11,12,13,14,15},   // Steam boat 1 sounds
                        {16,17,18,19,20}    // Steam boat 2 sounds
};
#endif
/*
 * This sets the samples/type for Car/truck sounds
 */
#ifdef CAR


enum Sound_Effect_Type{none=0,RegularCar,V8Car,MuscleCar,DieselTruck};

#endif 


/* The setup switch, S5 is regardless of output type and the values shown allow for S1-S4 closed in paralllel
   or just S5 closed on its own.
*/
#ifdef DEBUGPROG
//Test settings using S4 closed and push switch across S3, save changing fuses for testing.
const int SETUP_UPPER       =  645;   // 3.15V to 2.85V
const int SETUP_LOWER       =  584;
#else
const int SETUP_UPPER       =  211;       //1.03V to 0.49V
const int SETUP_LOWER       =  100;
#endif


enum Sound_Effect_Type Sounds=None;
//Globals to hold the ADC sampled value and the converted voltage.
unsigned int Scaled_V;
//Used when reading servo pulse input
unsigned int HornPulseLimit=0;
unsigned int FirstGear,SecondGear,ThirdGear,FourthGear;
unsigned int ForwardThrot, NeutralThrot, ReverseThrot;
byte HornGreatOrLessThan, ThrotNormReverse;
byte CurrentGear;
// Set the hysteresis of the gear change
const byte GearHysteresis=25;


// To control the Advert timing
byte CurrentSound,PreviousSound;
byte CurrentAdvert=0,PreviousAdvert=0;
byte AdvertPlaying=0;
unsigned long AdTime=0;
// Globals that hold the current PWM values
unsigned int ThrottlePWM,HornPWM;
//Global that denotes what signals are present
byte PWMFound;

const byte numReadings=4;
unsigned int readings[numReadings];
byte readIndex=0;
unsigned int total=0;

unsigned long RunTime=0;

//Throttle globals
unsigned int ThrottlePosition=0;
// Settings related to servo pulse positions
#define GREATERTHAN             0x33
#define LESSTHAN                0x22
#define NORMAL_THROT            0x34
#define REVERSE_THROT           0x23
#define HORN_EEPROM_LO          0
#define HORN_EEPROM_HI          1
#define HORN_EEPROM_GT_LT       2
#define THROT_EEPROM_FORWARD_LO 3
#define THROT_EEPROM_FORWARD_HI 4
#define THROT_EEPROM_NEUTRAL_LO 5
#define THROT_EEPROM_NEUTRAL_HI 6
#define THROT_EEPROM_REVERSE_LO 7
#define THROT_EEPROM_REVERSE_HI 8
#define THROT_EEPROM_DIRECTION  9
// For EEPROM setup
#define E_INIT 256



#define TimerAdjustment 27   //27us to allow for servo pulse jitter and detent of radio

#define DEBOUNCETIME 200
