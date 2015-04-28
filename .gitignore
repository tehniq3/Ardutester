/* ARDUTESTER v0.X 25/04/2013
 
 Original Source from:        http://www.mikrocontroller.net/articles/AVR-Transistortester
 Original Software:           by Karl-Heinz Kübbeler (kh_kuebbeler@web.de)
 LiquidCrystal_I2C library:   by Mario H. (http://www.xs4all.nl/~hmario/arduino/LiquidCrystal_I2C/LiquidCrystal_I2C.zip)
 
 Schematic & Home Page:       http://www.pighixxx.com/lavori/ardutester/ 
 Arduino version:             by PighiXXX (info@pighixxx.com)
 Contributors:                PaoloP (http://www.arduino.cc/forum/index.php?action=profile;u=58300)
 
                              - ONLY TTL COMPONENTS! -
 
 TODO:
 - Check Resistor Function (>47K)
 - Inductance Function
 - Detailed Component Analysis
 
 CHANGELOG:
 - 01/05/2013 v06e - Waitus Function, String to Flash Functions, Killed 3 Goto :-), Code Cleanup - PighiXXX 
 - 01/05/2013 v06f - Killed all Goto (Thanks to PaoloP), Implemented Button
 - 01/05/2013 v06g - Code Cleanup
 - 02/05/2013 v06h - Code Cleanup, SERIAL-LCD Flag, I2C LCD Functions
 - 02/05/2013 v06i - Button Flag, Button Function
 - 02/05/2013 v06j - PowerSave Function, Code Cleanup, Flag only when more info
 - 02/05/2013 v06k - Some fix (By PaoloP)
 - 03/05/2013 v06l - Disabled digital input on analog pins (By PaoloP), Minor fixes
 - 04/05/2013 v06m - ShowFET() fixed, Code Cleanup, Short Circuit Ok
 
 */
 
//Ardutester Features
#define BUTTON_INST                              //Button Installed
#define LCD_PRINT                                //Print on I2C LCD 
#define DEBUG_PRINT                              //Print on Serial Port
#define DET_COMP_ANALYSIS                        //Detailed Component Analysis (Soon)
#define LCD_ADDRESS      0x20                    //I2C LCD Address
#define TIMEOUT_BL       600                     //LCD Backlight Timeout
#define TIMEOUT_TK       800                     //Thanks Timeout

//Includes
#include <avr/wdt.h>

#ifdef LCD_PRINT
  #include <Wire.h> 
  #include <LiquidCrystal_I2C.h>
#endif

//Arduino Leonardo WorkAround
#ifndef atmega32u4
  #define ADCW ADC
#endif

//UINT32_MAX
#define UINT32_MAX  ((uint32_t)-1)

//ARDUTESTER PARAMETERS
//Maximum time to wait after a measurement in continous mode (in ms).
#define CYCLE_DELAY        3000
//Maximum number of measurements without any components found.
#define CYCLE_MAX             5
//ADC voltage reference based on Vcc (in mV). 
#define UREF_VCC           5001                  //5001
/* Offset for the internal bandgap voltage reference (in mV): -100 up to 100
 - To compensate any difference between real value and measured value.
 - The ADC has a resolution of about 4.88mV for V_ref = 5V (Vcc) and
 1.07mV for V_ref = 1.1V (bandgap).
 - Will be added to measured voltage of bandgap reference.
 */
#define UREF_OFFSET           0                  //0
//Rl in Ohms
#define R_LOW               680                  //680
//Rh in Ohms
#define R_HIGH           470000                  //470000
//Offset for systematic error of resistor measurement with RH (470k) in Ohms
#define RH_OFFSET           700                  //700
//Resistance of probe leads (in 0.01 Ohms).
#define R_ZERO               20                  //20
//Capacitance of the wires between PCB and terminals (in pF).
#define CAP_WIRES             9                  //2
//Capacitance of the probe leads connected to the tester (in pF).
/*   examples
 3pF      about 10cm
 9pF      about 30cm
 15pF      about 50cm
 */
#define CAP_PROBELEADS        9
//Maximum voltage at which we consider a capacitor being discharged (in mV)
#define CAP_DISCHARGED        2
//Number of ADC samples to perform for each mesurement. Valid values are in the range of 1 - 255.
#define ADC_SAMPLES          25
//Estimated internal resistance of port to GND (in 0.1 Ohms) 
#define R_MCU_LOW           200                   //209
//Estimated internal resistance of port to VCC (in 0.1 Ohms) 
#define R_MCU_HIGH          220                   //235
//Voltage offset of µCs analog comparator (in mV): -50 up to 50 
#define COMPARATOR_OFFSET    15
//Capacitance of the probe tracks of the PCB and the µC (in pF) 
#define CAP_PCB              32
//Total default capacitance (in pF): max. 255 
#define C_ZERO              CAP_PCB + CAP_WIRES + CAP_PROBELEADS
//Define clock divider
#define CPU_FREQ    F_CPU
#define ADC_CLOCK_DIV (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)

//Component IDs
#define COMP_NONE             0
#define COMP_ERROR            1
#define COMP_MENU             2
#define COMP_RESISTOR        10
#define COMP_CAPACITOR       11
#define COMP_DIODE           20
#define COMP_BJT             21
#define COMP_FET             22
#define COMP_TRIAC           24
#define COMP_THYRISTOR       25
#define COMP_INDUCTANCE      30
#define FLAG_PULLDOWN        0b00000000
#define FLAG_PULLUP          0b00000001
#define FLAG_1MS             0b00001000
#define FLAG_10MS            0b00010000

#define R_DDR DDRB
#define R_PORT PORTB

//FET type bit masks (also used for IGBTs) 
#define TYPE_N_CHANNEL       0b00000001          //n channel
#define TYPE_P_CHANNEL       0b00000010          //p channel 
#define TYPE_ENHANCEMENT     0b00000100          //enhancement mode 
#define TYPE_DEPLETION       0b00001000          //depletion mode 
#define TYPE_MOSFET          0b00010000          //MOSFET 
#define TYPE_JFET            0b00100000          //JFET 
#define TYPE_IGBT            0b01000000          //IGBT (no FET)

//BJT (bipolar junction transistor)  
#define TYPE_NPN              1    
#define TYPE_PNP              2    

//Multiplicator tables
#define TABLE_SMALL_CAP       1
#define TABLE_LARGE_CAP       2

//Maximum voltage at which we consider a capacitor being discharged (in mV)
#define CAP_DISCHARGED        2

//Error type IDs
#define TYPE_DISCHARGE        1                  //Discharge error

//Test probes:
#define ADC_PORT         PORTC                   //ADC port data register
#define ADC_DDR          DDRC                    //ADC port data direction register
#define ADC_PIN          PINC                    //port input pins register
#define TP1              0                       //test pin 1 (=0)
#define TP2              1                       //test pin 2 (=1)
#define TP3              2                       //test pin 3 (=2)
#define pushButton       2                       //PushButton

#define OSC_STARTUP      16384

#define LCD_CHAR_UNSET    0                      //Just a place holder 
#define LCD_CHAR_DIODE1   1                      //Diode icon '>|' 
#define LCD_CHAR_DIODE2   2	                 //Diode icon '|<' 
#define LCD_CHAR_CAP      3                      //Capacitor icon '||' 
#define LCD_CHAR_FLAG     4                      //Flag Icon
#define LCD_CHAR_RESIS1   6                      //Resistor left icon '[' 
#define LCD_CHAR_RESIS2   7                      //Resistor right icon ']' 
#define LCD_CHAR_OMEGA  244                      //Default 244 
#define LCD_CHAR_MICRO  228

//Probes
byte             Probe1_Pin;         
byte             Probe2_Pin;        
byte             Probe3_Pin;     

//Probing
byte             CompDone;                       //Flag for component detection done
byte             CompFound;                      //Component type which was found 
byte             CompType;                       //Component specific subtype
byte             ResistorsFound;                 //Number of resistors found
byte             DiodesFound;                    //Number of diodes found

//Bit masks for switching probes and test resistors
byte             Probe1_Rl;                      //Rl mask for probe-1 
byte             Probe1_Rh;                      //Rh mask for probe-1
byte             Probe2_Rl;                      //Rl mask for probe-2 
byte             Probe2_Rh;                      //Rh mask for probe-2
byte             Probe3_Rl;                      //Rl mask for probe-3 
byte             Probe3_Rh;                      //Rh mask for probe-3
byte             Probe1_ADC;                     //ADC mask for probe-1
byte             Probe2_ADC;                     //ADC mask for probe-2 

//Constant custom characters for LCD (stored EEPROM)
//Diode icon with anode at left side 
byte DiodeIcon1[8]  = {
  0x11, 0x19, 0x1d, 0x1f, 0x1d, 0x19, 0x11, 0x00};
//Diode icon with anode at right side 
byte DiodeIcon2[8]  = {
  0x11, 0x13, 0x17, 0x1f, 0x17, 0x13, 0x11, 0x00};
//Capacitor icon 
byte CapIcon[8]  = {
  0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x00};
//Resistor icon #1 (left part) 
byte ResIcon1[8]  = {
  0x00, 0x0f, 0x08, 0x18, 0x08, 0x0f, 0x00, 0x00};
//Resistor icon #2 (right part) 
byte ResIcon2[8] = {
  0x00, 0x1e, 0x02, 0x03, 0x02, 0x1e, 0x00, 0x00};
//Flag Icon
byte FlagIcon[8] = {
  0x15, 0x1a, 0x15, 0x1a, 0x10, 0x10, 0x10, 0x00};

//Bitmasks for Rl probe resistors based on probe ID
const char Rl_table[] = {
  (1 << (TP1 * 2)), (1 << (TP2 * 2)), (1 << (TP3 * 2))};
//Bitmasks for ADC pins based on probe ID */
const char ADC_table[] = {
  (1 << TP1), (1 << TP2), (1 << TP3)};
//Voltage based factors for small caps (using Rh)
const unsigned int  SmallCap_table[] = {
  954, 903, 856, 814, 775, 740, 707, 676, 648};
//Voltage based factors for large caps (using Rl)
const unsigned int  LargeCap_table[] = {
  23022, 21195, 19629, 18272, 17084, 16036, 15104, 14271, 13520, 12841, 12224, 11660, 11143, 10668, 10229, 9822, 9445, 9093, 8765, 8458, 8170, 7900, 7645, 7405, 7178, 6963, 6760, 6567, 6384, 6209, 6043, 5885, 5733, 5589, 5450, 5318, 5191, 5069, 4952, 4839, 4731, 4627, 4526, 4430, 4336};
//Interpolation Table for Inductance 
const unsigned int Log_table[]  = {
0, 20, 41, 62, 83, 105, 128, 151, 174, 198, 223, 248, 274, 301, 329, 357, 386, 416, 446, 478, 511, 545, 580, 616, 654, 693, 734, 777, 821, 868, 916, 968, 1022, 1079, 1139, 1204, 1273, 1347, 1427, 1514, 1609, 1715, 1833, 1966, 2120, 2303, 2526 };

//Unit prefixes
const char Prefix_table[] = {
  'p', 'n', LCD_CHAR_MICRO, 'm', 0, 'k', 'M'}; 
//Output buffer
char                OutBuffer[12]; 
char                PRGBuffer[16];

//Magic Store String to Flash Functions :-)
class __FlashStringHelper;
#define X(str) (strcpy_P(PRGBuffer, PSTR(str)), PRGBuffer)
#define Y(x) (__FlashStringHelper*)(x)


//Resistor
typedef struct
{
  byte              A;                           //Probe pin #1
  byte              B;                           //Probe pin #2
  byte              HiZ;                         //Probe pin in HiZ mode
  byte              Scale;                       //Exponent of factor (value * 10^x)
  unsigned long     Value;                       //Resistance
} 
Resistor_Type;

//Capacitor
typedef struct
{
  byte              A;                           //Probe pin #1
  byte              B;                           //Probe pin #2 
  signed char       Scale;                       //Exponent of factor (value * 10^x)
  unsigned long     Value;                       //Capacitance incl. zero offset 
  unsigned long     Raw;                         //Capacitance excl. zero offset 
} 
Capacitor_Type;

//Diode
typedef struct
{
  byte              A;                           //Probe pin connected to anode 
  byte              C;                           //Probe pin connected to cathode 
  unsigned int      V_f;                         //Forward voltage in mV (high current)
  unsigned int      V_f2;                        //Forward voltage in mV (low current) 
} 
Diode_Type;

//Bipolar junction transistor
typedef struct
{
  byte              B;                           //Probe pin connected to base 
  byte              C;                           //Probe pin connected to collector
  byte              E;                           //Probe pin connected to emitter 
  unsigned long     hfe;                         //Current amplification factor 
  //BE voltage
} 
BJT_Type;

//FET
typedef struct
{
  byte              G;                           //Test pin connected to gate 
  byte              D;                           //Test pin connected to drain 
  byte              S;                           //Test pin connected to source 
  unsigned int      V_th;                        //Threshold voltage of gate in mV 
} 
FET_Type;

//Offsets and values
typedef struct
{
  byte              Samples;                     //Number of ADC samples
  byte              AutoScale;                   //Flag to disable/enable ADC auto scaling
  byte              RefFlag;                     //Internal control flag for ADC
  unsigned int      U_Bandgap;                   //Voltage of internal bandgap reference (mV)
  unsigned int      RiL;                         //Internal pin resistance in low mode (0.1 Ohms)
  unsigned int      RiH;                         //Internal pin resistance in high mode (0.1 Ohms)
  unsigned int      RZero;                       //Resistance of probe leads (0.01 Ohms)
  byte              CapZero;                     //Capacity zero offset (input + leads) (pF)
  signed char       RefOffset;                   //Voltage offset of bandgap reference (mV)
  signed char       CompOffset;                  //Voltage offset of analog comparator (mV)
} 
Config_Type;

//Error (failed discharge)
typedef struct
{
  byte              Probe;                       //Probe pin
  unsigned int       U;                          //Voltage left in mV
} 
Error_Type;

Config_Type    Config;                           //Offsets and values
Error_Type     Error;                            //Error
Resistor_Type  Resistors[3];                     //Resistors (3 combinations)
Capacitor_Type Caps[3];                          //Capacitors (3 combinations)
Diode_Type     Diodes[6];                        //Diodes (3 combinations in 2 directions)
BJT_Type       BJT;                              //Bipolar junction transistor
FET_Type       FET;                              //FET

const unsigned int    NV_RiL = R_MCU_LOW;
const unsigned int    NV_RiH = R_MCU_HIGH;
const unsigned int    NV_RZero = R_ZERO;
const byte            NV_CapZero = C_ZERO;
const signed char     NV_RefOffset = UREF_OFFSET;
const signed char     NV_CompOffset = COMPARATOR_OFFSET;

//Function prototype
byte SmallCap(Capacitor_Type *Cap);
byte LargeCap(Capacitor_Type *Cap);

//Program control
byte           RunsPassed;                       //Counter for successful measurements
byte           RunsMissed;                       //Counter for failed/missed measurements
byte           ErrFnd;                           //An Error is occured

//String
const char Running_str[] PROGMEM = "Probing...";
const char Failed1_str[] PROGMEM = "No component";
const char Failed2_str[] PROGMEM = "found!";
const char Failed3_str[] PROGMEM = "Component?";
const char Failed5_str[] PROGMEM = "Discharge Failed";
const char Failed6_str[] PROGMEM = "Short Circuit";
const char Thyristor_str[] PROGMEM = "SCR";
const char Triac_str[] PROGMEM = "Triac";
const char GAK_str[] PROGMEM = "GAC=";
const char Error_str[] PROGMEM = "Error!";
const char MOS_str[] PROGMEM = "MOS";
const char FET_str[] PROGMEM = "FET";
const char Channel_str[] PROGMEM = "-ch";
const char Enhancement_str[] PROGMEM = "enh.";
const char Depletion_str[] PROGMEM = "dep.";
const char GateCap_str[] PROGMEM = "Cgs=";
const char GDS_str[] PROGMEM = "GDS=";
const char EBC_str[] PROGMEM = "EBC=";
const char hfe_str[] PROGMEM ="B=";
const char Vf_str[] PROGMEM = "Vf=";
const char DiodeCap_str[] PROGMEM = "C=";
const char Vth_str[] PROGMEM = "Vth=";
const char Version_str[] PROGMEM = "v0.6m";
const char NPN_str[] = "NPN";
const char PNP_str[] = "PNP";
//LCD Char
const char Cap_str[] = {
  '-',LCD_CHAR_CAP, '-',0};
const char Diode_AC_str[] = {
  '-', LCD_CHAR_DIODE1, '-', 0};
const char Diode_CA_str[] = {
  '-', LCD_CHAR_DIODE2, '-', 0};
const char Diodes_str[] = {
  '*', LCD_CHAR_DIODE1, ' ', ' ', 0};
const char Resistor_str[] = {
  '-', LCD_CHAR_RESIS1, LCD_CHAR_RESIS2, '-', 0};
boolean nosplash=true;

//Set the LCD address to LCD_ADDRESS for a 16 chars and 2 line display
#ifdef LCD_PRINT
  LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);
#endif

//Setup
void setup() {
  #ifdef DEBUG_PRINT
    Serial.begin(9600);
    Serial.print("ArduTester ");
    Serial.println(Y(Version_str));
  #endif
  //LCD Init
  #ifdef LCD_PRINT
    lcd.init();
    lcd.backlight();
    //Symbols for components
    //Diode symbol |>|
    lcd.createChar(LCD_CHAR_DIODE1,DiodeIcon1);
    //Diode symbol |<|
    lcd.createChar(LCD_CHAR_DIODE2,DiodeIcon2);
    //Capacitor symbol ||
    lcd.createChar(LCD_CHAR_CAP,CapIcon);   
    //Resistor symbol [  
    lcd.createChar(LCD_CHAR_RESIS1,ResIcon1);
    //Resistor symbol ] 
    lcd.createChar(LCD_CHAR_RESIS2,ResIcon2); 
    //Flag symbol ] 
    lcd.createChar(LCD_CHAR_FLAG,FlagIcon); 
    lcd.home();
    lcd.print("Ardutester ");
    lcd.print(Y(Version_str));
    lcd_line(2);
    lcd.print("by PighiXXX");
    delay(2000);                                 //Read Splash Screen :-)
  #endif
  //MCUCR |= (1 << PUD);                         //Disable pull-up resistors globally
  ADCSRA = (1 << ADEN) | ADC_CLOCK_DIV;          //Enable ADC and set clock divider
  MCUSR &= ~(1 << WDRF);                         //Reset watchdog flag
  wdt_disable();                                 //Disable watchdog
  //Cycling
  RunsMissed = 0;
  RunsPassed = 0;
  //Default offsets and values
  Config.Samples = ADC_SAMPLES;                  //Number of ADC samples
  Config.AutoScale = 1;                          //Enable ADC auto scaling
  Config.RefFlag = 1;                            //No ADC reference set yet
  analogReference(EXTERNAL);                     //Set Analog Reference to External
  pinMode(pushButton, INPUT);                    //Set pin to input
  digitalWrite(pushButton, HIGH);                //Turn on pullup resistors
  LoadAdjust();                                  //Load adjustment values
  delay(100);
}

//Main Loop
void loop() {  
  #ifdef BUTTON_INST
    CheckButton(false);                          //Wait User  
  #endif
  lcd_clear();                                   //Show Probing...
  lcd_print(Y(Running_str));
  #ifdef WDT_enabled
    wdt_enable(WDTO_2S);	                 //Enable watchdog (timeout 4s)
  #endif
  //Reset variabels
  CompFound = COMP_NONE;
  CompType = 0;
  CompDone = 0;
  DiodesFound = 0;
  ResistorsFound = 0;
  ErrFnd = 0;
  BJT.hfe = 0;
  //Reset hardware
  SetADCHiZ();                                   //Set ADC port to HiZ mode
  //Internal bandgap reference
  Config.U_Bandgap = ReadU(0x0e);                //Dummy read for bandgap stabilization 
  Config.Samples = 200;                          //Do a lot of samples for high accuracy 
  Config.U_Bandgap = ReadU(0x0e);                //Get voltage of bandgap reference 
  Config.Samples = ADC_SAMPLES;                  //Set samples back to default 
  Config.U_Bandgap += Config.RefOffset;          //Add voltage offset 
  //Try to discharge any connected component
  DischargeProbes();
  if (CompFound == COMP_ERROR)                   //Discharge failed
  {
    ErrFnd=4;                                    //Skip all other checks
  }
  //Short-circuit
  if (AllProbesShorted() == 3)
  {
    ErrFnd=3;                                    //New cycle after job is is done
  }
  else
  {
    //Check all 6 combinations of the 3 probes
    CheckProbes(TP1, TP2, TP3);
    CheckProbes(TP2, TP1, TP3);
    CheckProbes(TP1, TP3, TP2);
    CheckProbes(TP3, TP1, TP2);
    CheckProbes(TP2, TP3, TP1);
    CheckProbes(TP3, TP2, TP1);
    //If component might be a capacitor
    if ((CompFound == COMP_NONE) ||
      (CompFound == COMP_RESISTOR) ||
      (CompFound == COMP_DIODE))
    {
      //Check all possible combinations
      MeasureCap(TP3, TP1, 0);
      MeasureCap(TP3, TP2, 1);
      MeasureCap(TP2, TP1, 2);
    }
    //Call output function based on component type
    wdt_disable();
    #ifdef LCD_PRINT
        lcd.init();
        lcd.print("");
    #endif
    #ifdef DEBUG_PRINT
        Serial.println("");  
        Serial.print("Component Found:"); 
        Serial.println(CompFound);
    #endif
    delay(100);
    switch (CompFound)
    {
    case COMP_ERROR:
      ErrFnd=1;
      ShowError(Y(Failed3_str));
      break;
    case COMP_DIODE:
      ShowDiode();
      break;
    case COMP_BJT:
      ShowBJT();
      break;
    case COMP_FET:
      ShowFET();
      break;
    case COMP_THYRISTOR:
      ShowSpecial();
      break;
    case COMP_TRIAC:
      ShowSpecial();
      break;
    case COMP_RESISTOR:
      ShowResistor();
      break;
    case COMP_CAPACITOR:
      ShowCapacitor();
      break;
    default:                                   //No component found
      ShowFail();
      ErrFnd=2;
    }
  }
  if (ErrFnd==0)
  {
    //Component was found
    RunsMissed = 0;                            //Reset counter
    RunsPassed++;                              //Increase counter
    delay(1000);  
  }
  else
  {
    //Error
    switch (ErrFnd)
    {
    case 3:
      //Short Circuit
      ShowError(Y(Failed6_str));
      break;
    case 4:
      //Discharge failed
      ShowError(Y(Failed5_str));
      break;
    }
  }
}

//Check Button
boolean CheckButton(boolean showFlag)
{
  if(showFlag==true)
  {
  #ifdef LCD_PRINT
      lcd.setCursor(15, 1);                    //Show Flag 
      lcd.write(LCD_CHAR_FLAG);
  #endif
  }
  wdt_disable();                               //Disable watchdog
  int waituser=      0;
  int waitthnk=      0;
  int buttonState=   0;
  do
  {
    buttonState = digitalRead(pushButton);
    //Turn Off Backlight
    if (waituser<TIMEOUT_BL) waituser++; 
    else 
    {
      #ifdef LCD_PRINT
        lcd.noBacklight();
      #endif
      if (showFlag==false)
      {
      #ifdef LCD_PRINT
        lcd.clear();
        lcd.print("Ardutester ");
        lcd.print(Y(Version_str));
        lcd_line(2);
        lcd.print("by PighiXXX");
        nosplash=true;
      #endif
      }  
    }
    if (waitthnk<TIMEOUT_TK) waitthnk++;
    else
    {
      if (nosplash==true & showFlag==false)
      {
        //Show Thanks!
        #ifdef LCD_PRINT
          lcd_clear_line(2);
          lcd.print("Thanks to PaoloP");
          lcd.backlight();
          waituser=0;
          showFlag=true;
          nosplash=false;
        #endif
      }
    }
    delay(20);
  } 
  while (buttonState==HIGH);                   //Wait button pressed
  #ifdef LCD_PRINT
    lcd.setCursor(15, 1);                      //Erase Flag 
    lcd.print(" ");
    lcd.backlight();
  #endif
}

//Set ADC port low
void SetADCLow(void)
{
  digitalWrite(A0,LOW);                          
  digitalWrite(A1,LOW);
  digitalWrite(A2,LOW);
}

//Set ADC port to HiZ mode 
void SetADCHiZ(void)
{
  ADC_DDR &= ~(1<<TP1);                                                           
  ADC_DDR &= ~(1<<TP2);
  ADC_DDR &= ~(1<<TP3);
}

//Fail
void ShowFail(void)
{
  //Display info
  lcd_print(Y(Failed1_str));                     //Display: No component
  lcd_line(2);                                   //Move to line #2
  lcd_print(Y(Failed2_str));                     //Display: found!
  //Display numbers of diodes found
  if (DiodesFound > 0)                           //Diodes found
  {
    lcd_space();                                 //Display space
    lcd_data(DiodesFound + '0');                 //Display number of diodes found
    lcd_fix_string(Diode_AC_str);                //Display: -|>|-   
  }
  RunsMissed++;                                  //Increase counter
  RunsPassed = 0;                                //Reset counter
}

//Diode
void ShowDiode(void)
{
  Diode_Type        *D1;                         //Pointer to diode #1 
  Diode_Type        *D2 = NULL;                  //Pointer to diode #2 
  byte              CFlag = 1;                   //Capacitance display flag 
  byte              A = 5;                       //ID of common anode 
  byte              C = 5;                       //ID of common cothode 
  D1 = &Diodes[0];                               //Pointer to first diode 
  if (DiodesFound == 1)                          //Single diode 
  {
    C = D1->C;                                   //Make anode first pin 
  }
  else if (DiodesFound == 2)                     //Two diodes 
  {
    D2 = D1;
    D2++;                                        //Pointer to second diode 
    if (D1->A == D2->A)                          //Common anode 
    {
      A = D1->A;                                 //Save common anode 
    }
    else if (D1->C == D2->C)                     //Common cathode 
    {
      C = D1->C;                                 //Save common cathode 
    }
    else if ((D1->A == D2->C) && (D1->C == D2->A))
    {                                            //Anti-parallel
      A = D1->A;                                 //Anode and cathode are the same, disable display of capacitance
      C = A;                           
      CFlag = 0;                   
    } 
  }
  else if (DiodesFound == 3)                     //Three diodes
  {
    byte            n;
    byte            m;
    //Two diodes in series are additionally detected as third big diode:
    for (n = 0; n <= 2; n++)                     //Loop for first diode
    {
      D1 = &Diodes[n];                           //Get pointer of first diode
      for (m = 0; m <= 2; m++)                   //Loop for second diode
      {
        D2 = &Diodes[m];                         //Get pointer of second diode
        if (n != m)                              //Don't check same diode :-) 
        {
          if (D1->C == D2->A)                    //Got match 
          {
            n = 5;                               //End loops
            m = 5;
          }
        }
      }
    }
    if (n < 5) D2 = NULL;                        //No match found 
    C = D1->C;                                   //Cathode of first diode in series mode
    A = 3;                         
  }
  else                                           //To much diodes
  {
    D1 = NULL;                                   //Don't display any diode and tell user
    ShowFail();                    
  }
  //Display pins 
  if (D1)                                        //First Diode
  {
    if (A < 3) lcd_testpin(D1->C);               //Common anode
    else lcd_testpin(D1->A);                     //Common cathode
    if (A < 3) lcd_fix_string(Diode_CA_str);     //Common anode
    else lcd_fix_string(Diode_AC_str);           //Common cathode 
    if (A < 3) lcd_testpin(A);                   //Common anode 
    else lcd_testpin(C);                         //Common cathode 
  }
  if (D2)                                        //Second diode
  {
    if (A <= 3) lcd_fix_string(Diode_AC_str);    //Common anode or in series 
    else lcd_fix_string(Diode_CA_str);           //Common cathode
    if (A == C) lcd_testpin(D2->A);              //Anti parallel 
    else if (A <= 3) lcd_testpin(D2->C);         //Common anode or in series 
    else lcd_testpin(D2->A);                     //Common cathode 
  }
  //Display Uf (forward voltage) and capacitance
  if (D1)                                        //First diode
  {
    //Uf
    lcd_line(2);                                 //Go to line #2
    lcd_print(Y(Vf_str));                        //Display: Vf= 
    DisplayValue(D1->V_f, -3, 'V');              //Display Vf 
    if (D2)                                      //Second diode 
    {
      lcd_space();
      DisplayValue(D2->V_f, -3, 'V');            //Display Vf 
    }
    //Capacitance
    if (CFlag == 1)
    {
      #ifdef BUTTON_INST
        CheckButton(true);                      //Wait User
      #else
        delay(3000);                            //Wait 3sec  
      #endif    
      lcd_clear_line(2);
      lcd_print(Y(DiodeCap_str));                //Display: C= 
      //Get capacitance (opposite of flow direction)
      MeasureCap(D1->C, D1->A, 0);
      //and show capacitance 
      DisplayValue(Caps[0].Value, Caps[0].Scale, 'F');
      if (D2)                                    //Second diode
      {
        lcd_space();
        MeasureCap(D2->C, D2->A, 0);
        DisplayValue(Caps[0].Value, Caps[0].Scale, 'F');
      }
    }
  }
}

//BJT
void ShowBJT(void)
{
  Diode_Type        *Diode;                      //Pointer to diode
  char              *String;                     //Display string pointer 
  byte              Counter;                     //Counter 
  unsigned int      Vf;                          //Forward voltage U_be 
  signed int        Slope;                       //Slope of forward voltage 
  //Display type
  if (CompType == TYPE_NPN)                      //NPN 
      String = (char *)NPN_str;
  else                                           //PNP 
  String = (char *)PNP_str;
  lcd_fix_string(String);                        //Display: NPN / PNP 
  //Protections diodes 
  if (DiodesFound > 2)                           //Transistor is a set of two diodes :-) 
  {
    lcd_space();
    if (CompType == TYPE_NPN)                    //NPN 
        String = (char *)Diode_AC_str;
    else                                         //PNP 
    String = (char *)Diode_CA_str;
    lcd_fix_string(String);                      //Display: -|>|- / -|<|- 
  }
  //Display pins
  lcd_space();
  lcd_print(Y(EBC_str));                         //Display: EBC= 
  lcd_testpin(BJT.E);                            //Display emitter pin 
  lcd_testpin(BJT.B);                            //Display base pin 
  lcd_testpin(BJT.C);                            //Display collector pin 
  //Display hfe
  lcd_line(2);                                   //Move to line #2 
  lcd_print(Y(hfe_str));                         //Display: B= 
  DisplayValue(BJT.hfe, 0, 0);
  //Display Uf (forward voltage)
  Diode = &Diodes[0];                            //Get pointer of first diode   
  Counter = 0;
  while (Counter < DiodesFound)                  //Check all diodes 
  {
    //If the diode matches the transistor 
    if (((Diode->A == BJT.B) &&
      (Diode->C == BJT.E) &&
      (CompType == TYPE_NPN)) ||
      ((Diode->A == BJT.E) &&
      (Diode->C == BJT.B) &&
      (CompType == TYPE_PNP)))
    {
      //Not enough space on LCD for large hfe and Vf
      if (BJT.hfe < 1000)                        //Small hfe 
      {
        lcd_space();                             //Display space 
      }
      else                                       //Line to short 
      {
        #ifdef BUTTON_INST
          CheckButton(true);                     //Wait User
        #else
          delay(3000);                           //Wait 3sec  
        #endif   
        lcd_clear_line(2);
      }
      lcd_print(Y(Vf_str));                      //Display: Vf=
      /*
         Vf is quite linear for a logarithmicly scaled I_b.
       So we may interpolate the Vf values of low and high test current
       measurements for a virtual test current. Low test current is 10µA
       and high test current is 7mA. That's a logarithmic scale of
       3 decades.
       */
      //Calculate slope for one decade
      Slope = Diode->V_f - Diode->V_f2;
      Slope /= 3;
      //Select Vf based on hfe
      if (BJT.hfe < 100)                         //Low hfe
      {
        /*
           BJTs with low hfe are power transistors and need a large I_b
         to drive the load. So we simply take Vf of the high test current
         measurement (7mA). 
         */
        Vf = Diode->V_f;
      }
      else if (BJT.hfe < 250)                    //Mid-range hfe
      {
        /*
           BJTs with a mid-range hfe are signal transistors and need
         a small I_b to drive the load. So we interpolate Vf for
         a virtual test current of about 1mA.
         */
        Vf = Diode->V_f - Slope;
      }
      else                                       //High hfe
      {
        /*
           BJTs with a high hfe are small signal transistors and need
         only a very small I_b to drive the load. So we interpolate Vf
         for a virtual test current of about 0.1mA.
         */
        Vf = Diode->V_f2 + Slope;
      }
      DisplayValue(Vf, -3, 'V');
      Counter = DiodesFound;                     //End loop
    }
    else
    {
      Counter++;                                 //Increase counter
      Diode++;                                   //Next one
    }
  }
}

//Fet
void ShowFET(void)
{
  byte           Data;                           //Temp. data
  //Display type
  if (CompType & TYPE_MOSFET)                    //MOSFET
    lcd_print(Y(MOS_str));                       //Display: MOS 
  else                                           //JFET 
  lcd_data('J');                                 //Display: J 
  lcd_print(Y(FET_str));                         //Display: FET 
  //Display channel type
  lcd_space();
  if (CompType & TYPE_N_CHANNEL)                 //n-channel 
    Data = 'N';
  else                                           //p-channel 
  Data = 'P';
  lcd_data(Data);                                //Display: N / P 
  lcd_print(Y(Channel_str));                     //Display: -ch 
  //Display mode
  if (CompType & TYPE_MOSFET)                    //MOSFET 
  {
    lcd_space();
    if (CompType & TYPE_ENHANCEMENT)             //Enhancement mode 
      lcd_print(Y(Enhancement_str));
    else                                         //Depletion mode 
    lcd_print(Y(Depletion_str));
  }
  //Pins
  lcd_line(2);                                   //Move to line #2  
  lcd_print(Y(GDS_str));                         //Display: GDS= 
  lcd_testpin(FET.G);                            //Display gate pin 
  lcd_testpin(FET.D);                            //Display drain pin 
  lcd_testpin(FET.S);                            //Display source pin 
  //Extra data for MOSFET in enhancement mode
  if (CompType & (TYPE_ENHANCEMENT | TYPE_MOSFET))
  {
    //Protection diode
    if (DiodesFound > 0)
    {
      lcd_space();                               //Display space 
      lcd_data(LCD_CHAR_DIODE1);                 //Display diode symbol 
    }
    #ifdef BUTTON_INST
      CheckButton(true);                         //Wait User
    #else
      delay(3000);                               //Wait 3sec  
    #endif                          
    lcd_clear();
    //Gate threshold voltage 
    lcd_print(Y(Vth_str));                       //Display: Vth 
    DisplayValue(FET.V_th, -3, 'V');             //Display V_th in mV    
    lcd_line(2);
    //Display gate capacitance
    lcd_print(Y(GateCap_str));                   //Display: Cgs= 
    MeasureCap(FET.G, FET.S, 0);                 //Measure capacitance 
    //Display value and unit 
    DisplayValue(Caps[0].Value, Caps[0].Scale, 'F');
  }
}

//Special
void ShowSpecial(void)
{
  //Display component type
  if (CompFound == COMP_THYRISTOR)
  {
    lcd_print(Y(Thyristor_str));                 //Display: thyristor 
  }
  else if (CompFound == COMP_TRIAC)
  {
    lcd_print(Y(Triac_str));                     //Display: triac 
  }
  //Display pins
  lcd_line(2);                                   //Move to line #2 
  lcd_print(Y(GAK_str));                         //Display: GAK 
  lcd_testpin(BJT.B);                            //Display gate pin 
  lcd_testpin(BJT.C);                            //Display anode pin 
  lcd_testpin(BJT.E);                            //Display cathode pin 
}

//Error
void ShowError( __FlashStringHelper* data)
{
  //Display info
  lcd_clear();
  lcd_fix_string("Error:");                      //Display Error:
  lcd_line(2);                                   //Move to line #2
  lcd_print(data);                               //Display ErrorText
  RunsMissed++;                                  //Increase counter
  RunsPassed = 0;   
}

//Capacitor
void ShowCapacitor(void)
{
  Capacitor_Type    *MaxCap;                     //Pointer to largest cap
  Capacitor_Type    *Cap;                        //Pointer to cap
  byte              Counter;                     //Loop counter
  //Find largest cap
  MaxCap = &Caps[0];                             //Pointer to first cap
  Cap = MaxCap;
  for (Counter = 1; Counter <= 2; Counter++) 
  {
    Cap++;                                       //Next cap
    if (CmpValue(Cap->Value, Cap->Scale, MaxCap->Value, MaxCap->Scale) == 1)
    {
      MaxCap = Cap;
    }
  }
  //Display largest cap
  lcd_testpin(MaxCap->A);                        //Display pin #1 
  lcd_fix_string(Cap_str);                       //Display capacitor symbol 
  lcd_testpin(MaxCap->B);                        //Display pin #2 
  lcd_line(2);                                   //Move to line #2 
  //and show capacitance 
  DisplayValue(MaxCap->Value, MaxCap->Scale, 'F');
}

//Resistor
void ShowResistor(void)
{
  Resistor_Type     *R1;                         //Pointer to resistor #1 
  Resistor_Type     *R2;                         //Pointer to resistor #2 
  byte              Pin;                         //ID of common pin 
  R1 = &Resistors[0];                            //Pointer to first resistor 
  if (ResistorsFound == 1)                       //Single resistor 
  {
    R2 = NULL;                                   //Disable second resistor 
    Pin = R1->A;                                 //Make B the first pin 
  }
  else                                           //Multiple resistors 
  {
    R2 = R1;
    R2++;                                        //Pointer to second resistor 
    if (ResistorsFound == 3)                     //Three resistors 
    {
      Resistor_Type     *Rmax;                   //Pointer to largest resistor     
      /*
         3 resistors mean 2 single resistors and both resitors in series.
       So we have to single out that series resistor by finding the
       largest resistor.
       */
      Rmax = R1;                                 //Starting point
      for (Pin = 1; Pin <= 2; Pin++)
      {
        if (CmpValue(R2->Value, R2->Scale, Rmax->Value, Rmax->Scale) == 1)
        {
          Rmax = R2;                             //Update largest one
        }
        R2++;                                    //Next one
      }
      //Get the two smaller resistors
      if (R1 == Rmax) R1++;
      R2 = R1;
      R2++;
      if (R2 == Rmax) R2++;
    }
    //Find common pin of both resistors
    if ((R1->A == R2->A) || (R1->A == R2->B)) Pin = R1->A;
    else Pin = R1->B;
  }
  //First resistor */
  if (R1->A != Pin) lcd_testpin(R1->A);
  else lcd_testpin(R1->B);
  lcd_fix_string(Resistor_str);
  lcd_testpin(Pin);
  if (R2)                                        //Second resistor
  {
    lcd_fix_string(Resistor_str);
    if (R2->A != Pin) lcd_testpin(R2->A);
    else lcd_testpin(R2->B);
  }
  //First resistor
  lcd_line(2);
  DisplayValue(R1->Value, R1->Scale, LCD_CHAR_OMEGA);
  if (R2)                                        //Second resistor
  {
    lcd_space();
    DisplayValue(R2->Value, R2->Scale, LCD_CHAR_OMEGA);
  }
}

//Wait Functions
void waitus(byte microsec) {
  delayMicroseconds(microsec);
}

//Pull probe up/down via probe resistor for 1 or 10 ms
void PullProbe(byte Mask, byte Mode)
{
  //Set pull mode
  if (Mode & FLAG_PULLUP) R_PORT |= Mask;         //Pull-up
  else R_PORT &= ~Mask;                           //Pull-down
  R_DDR |= Mask;                                  //Enable pulling 
  if (Mode & FLAG_1MS) delay(1);                  //Wait 1ms
  else delay(10);                                 //Wait 10ms
  //Reset pulling
  R_DDR &= ~Mask;                                 //Set to HiZ mode
  R_PORT &= ~Mask;                                //Set 0
}

//Setup probes, bitmasks for probes and test resistors
void UpdateProbes(byte Probe1, byte Probe2, byte Probe3)
{
  //DSt probe IDs
  Probe1_Pin = Probe1;
  Probe2_Pin = Probe2;
  Probe3_Pin = Probe3;
  //Setup masks using bitmask tables
  Probe1_Rl = Rl_table[Probe1];
  Probe1_Rh = Probe1_Rl + Probe1_Rl;
  Probe1_ADC = ADC_table[Probe1];
  Probe2_Rl = Rl_table[Probe2];
  Probe2_Rh = Probe2_Rl + Probe2_Rl;
  Probe2_ADC = ADC_table[Probe2];
  Probe3_Rl = Rl_table[Probe3];
  Probe3_Rh = Probe3_Rl + Probe3_Rl;
}

//Check for a short circuit between two probes
byte ShortedProbes(byte Probe1, byte Probe2)
{
  byte              Flag = 0;                     //return value
  unsigned int      U1;                           //voltage at probe #1 in mV
  unsigned int      U2;                           //voltage at probe #2 in mV
  /*
     Set up a voltage divider between the two probes:
   - Probe1: Rl pull-up
   - Probe2: Rl pull-down
   - third probe: HiZ
   */
  R_PORT = Rl_table[Probe1];
  R_DDR = Rl_table[Probe1] | Rl_table[Probe2];
  //Read voltages
  U1 = ReadU(Probe1);
  U2 = ReadU(Probe2);
  /*
   We expect both probe voltages to be about the same and
   to be half of Vcc (allowed difference +/- 30mV).
   */
  if ((U1 > UREF_VCC/2 - 30) && (U1 < UREF_VCC/2 + 30))
  { 
    if ((U2 > UREF_VCC/2 - 30) && (U2 < UREF_VCC/2 + 30))
    {
      Flag = 1;
    }    
  }
  //Reset port
  R_DDR = 0;
  return Flag;
}

//Check for a short circuit between all probes
byte AllProbesShorted(void)
{
  byte              Flag = 0;                     //Return value
  //Check all possible combinations
  Flag = ShortedProbes(TP1, TP2);
  Flag += ShortedProbes(TP1, TP3);
  Flag += ShortedProbes(TP2, TP3);
  return Flag;  
}

//Read ADC and return voltage in mV (GOTO Free - Thanks to PaoloP)
unsigned int ReadU(byte Probe)
{
  unsigned int      U;                            //Return value (mV)
  byte              Counter;                      //Loop counter
  unsigned long     Value;                        //ADC value
  Probe |= (1 << REFS0);                          //Use internal reference anyway
  boolean cycle;
  do {
    cycle = false;                                // One Time cycle
    ADMUX = Probe;                                //Set input channel and U reference
    //If voltage reference has changed run a dummy conversion
    Counter = Probe & (1 << REFS1);               //Get REFS1 bit flag
    if (Counter != Config.RefFlag)
    {
      waitus(100);                                //Time for voltage stabilization
      ADCSRA |= (1 << ADSC);                      //Start conversion
      while (ADCSRA & (1 << ADSC));               //Wait until conversion is done
      Config.RefFlag = Counter;                   //Update flag
    }
    //Sample ADC readings
    Value = 0UL;                                  //Reset sampling variable
    Counter = 0;                                  //Reset counter
    while (Counter < Config.Samples)              //Take samples
    {
      ADCSRA |= (1 << ADSC);                      //Start conversion
      while (ADCSRA & (1 << ADSC));               //Wait until conversion is done
      Value += ADCW;                              //Add ADC reading
      //Auto-switch voltage reference for low readings
      if ((Counter == 4) && ((unsigned int)Value < 1024) && !(Probe & (1 << REFS1)) && (Config.AutoScale == 1))
      {
        Probe |= (1 << REFS1);                    //Select internal bandgap reference
        cycle = true;
        break;                                    //Re-run sampling 
      }
      Counter++;                                  //One less to do
    }
  } 
  while (cycle);
  //Convert ADC reading to voltage
  if (Probe & (1 << REFS1)) U = Config.U_Bandgap; //Bandgap reference
  else U = UREF_VCC;                              //Vcc reference   
  //Convert to voltage
  Value *= U;                                     //ADC readings U_ref 
  //Value += 511 * Config.Samples;                //Automagic rounding
  Value /= 1024;                                  // / 1024 for 10bit ADC
  //De-sample to get average voltage
  Value /= Config.Samples;
  U = (unsigned int)Value;
  return U; 
}

//Try to discharge any connected components, e.g. capacitors
void DischargeProbes(void)
{
  byte              Counter;                      //Loop control
  byte              Limit = 40;                   //Sliding timeout (2s)
  byte              ID;                           //Test pin
  byte              DischargeMask;                //Bitmask
  unsigned int      U_c;                          //Current voltage
  unsigned int      U_old[3];                     //Old voltages
  //Set probes to a save discharge mode (pull-down via Rh) 
  //Set ADC port to HiZ input
  SetADCHiZ();                                    //Set ADC port to HiZ mode
  SetADCLow();                                    //Set ADC port low
  //All probe pins: Rh and Rl pull-down
  R_PORT = 0;
  R_DDR = (2 << (TP1 * 2)) | (2 << (TP2 * 2)) | (2 << (TP3 * 2));
  R_DDR |= (1 << (TP1 * 2)) | (1 << (TP2 * 2)) | (1 << (TP3 * 2));
  //Get current voltages
  U_old[0] = ReadU(TP1);
  U_old[1] = ReadU(TP2);
  U_old[2] = ReadU(TP3);
  /*
     try to discharge probes
   - We check if the voltage decreases over time.
   - A slow discharge rate will increase the timeout to support
   large caps.
   - A very large cap will discharge too slowly and an external voltage
   maybe never :-)
   */
  Counter = 1;
  ID = 2;
  DischargeMask = 0;
  while (Counter > 0)
  {
    ID++;                                         //Next probe
    if (ID > 2) ID = 0;                           //Start with probe #1 again
    if (DischargeMask & (1 << ID))                //Skip discharged probe
      continue;
    U_c = ReadU(ID);                              //Get voltage of probe 
    if (U_c < U_old[ID])                          //Voltage decreased 
    {
      U_old[ID] = U_c;                            //Update old value
      //Adapt timeout based on discharge rate
      if ((Limit - Counter) < 20)
      {
        //Increase timeout while preventing overflow
        if (Limit < (255 - 20)) Limit += 20;
      }
      Counter = 1;                                //Reset no-changes counter 
    }
    else                                          //Voltage not decreased
    {
      //Increase limit if we start at a low voltage
      if ((U_c < 10) && (Limit <= 40)) Limit = 80;
      Counter++;                                  //Increase no-changes counter
    }

    if (U_c <= CAP_DISCHARGED)                    //Seems to be discharged
    {
      DischargeMask |= (1 << ID);                 //Set flag
    }
    else if (U_c < 800)                           //Extra pull-down
    {
      //It's save now to pull-down probe pin directly
      ADC_DDR |= ADC_table[ID];
    }
    if (DischargeMask == 0b00000111)              //All probes discharged
    {
      Counter = 0;                                //End loop
    }
    else if (Counter > Limit)                     //No decrease for some time
    {
      //Might be a battery or a super cap
      CompFound = COMP_ERROR;                     //Report error
      CompType = TYPE_DISCHARGE;                  //Discharge problem
      Error.Probe = ID;                           //Save probe
      Error.U = U_c;                              //Save voltage
      Counter = 0;                                //End loop
    }
    else                                          //Go for another round
    {
      wdt_reset();                                //Reset watchdog
      delay(50);                                  //Wait for 50ms
    }
  }
  //Reset probes
  R_DDR = 0;                                      //Set resistor port to input mode
  SetADCHiZ();                                    //Set ADC port to HiZ mode
}

//Wait 5ms and then read ADC
unsigned int ReadU_5ms(byte Probe)
{ 
  delay(5);                                       //Wait 5ms
  return (ReadU(Probe));
}

//Wait 20ms and then read ADC
unsigned int ReadU_20ms(byte Probe)
{
  delay(20);                                      //Wait 20ms
  return (ReadU(Probe));
}

//Interpolate value from table based on voltage
unsigned int GetFactor(unsigned int U_in, byte ID)
{
  unsigned int      Factor;                       //Return value
  unsigned int      U_Diff;                       //Voltage difference to table start
  unsigned int      Fact1, Fact2;                 //Table entries
  unsigned int      TabStart;                     //Table start voltage 
  unsigned int      TabStep;                      //Table step voltage
  unsigned int      TabIndex;                     //Table entries (-2)
  unsigned int      *Table;
  byte              Index;                        //Table index
  byte              Diff;                         //Difference to next entry
  //Setup table specific stuff
  if (ID == TABLE_SMALL_CAP)
  {
    TabStart = 1000;                              //Table starts at 1000mV 
    TabStep = 50;                                 //50mV steps between entries 
    TabIndex = 7;                                 //Entries in table - 2 
    Table = (unsigned int *)&SmallCap_table[0];   //Pointer to table start 
  }
  else if (ID == TABLE_LARGE_CAP)
  {
    TabStart = 300;                               //Table starts at 1000mV 
    TabStep = 25;                                 //50mV steps between entries 
    TabIndex = 42;                                //Entries in table - 2 
    Table = (unsigned int *)&LargeCap_table[0];   //Pointer to table start 
  }
  else
  {
    return 0;
  }
  //Difference to start of table
  if (U_in >= TabStart) U_Diff = U_in - TabStart;  
  else U_Diff = 0;
  //Calculate table index
  Index = U_Diff / TabStep;                       //Index (position in table) 
  Diff = U_Diff % TabStep;                        //Difference to index 
  Diff = TabStep - Diff;                          //Difference to next entry 
  //Prevent index overflow
  if (Index > TabIndex) Index = TabIndex;
  //Get values for index and next entry
  Table += Index;                                 //Advance to index
  //MEM_read_word(Table);
  Fact1 = *(Table);
  Table++;          //Next entry
  //MEM_read_word(Table);
  Fact2 = *(Table);                               
  //Interpolate values based on difference
  Factor = Fact1 - Fact2;
  Factor *= Diff;
  Factor += TabStep / 2;
  Factor /= TabStep;
  Factor += Fact2;
  return Factor;
}

//Get number of digits of a value
byte NumberOfDigits(unsigned long Value)
{
  byte           Counter = 1;
  while (Value >= 10)
  {
    Value /= 10;
    Counter++;
  }
  return Counter;
}

//Compare two scaled values
signed char CmpValue(unsigned long Value1, signed char Scale1, unsigned long Value2, signed char Scale2)
{
  signed char     Flag;                           //Return value
  signed char     Len1, Len2;                     //Length
  //Determine virtual length
  Len1 = NumberOfDigits(Value1) + Scale1;
  Len2 = NumberOfDigits(Value2) + Scale2;
  if ((Value1 == 0) || (Value2 == 0))             //Special case
  {
    Flag = 10;                                    //Perform direct comparison
  }
  else if (Len1 > Len2)                           //More digits -> larger
  {
    Flag = 1;
  }
  else if (Len1 == Len2)                          //Same length
  {
    //Re-scale to longer value
    Len1 -= Scale1;
    Len2 -= Scale2;
    while (Len1 > Len2)                           //Up-scale Value #2
    {
      Value2 *= 10;
      Len2++;
      //Scale2--
    }
    while (Len2 > Len1)                           //Up-scale Value #1
    {
      Value1 *= 10;
      Len1++;
      //Scale1--
    }   
    Flag = 10;                                    //Perform direct comparison 
  }
  else                                            //Less digits -> smaller
  {
    Flag = -1;
  }

  if (Flag == 10)                                 //Perform direct comparison
  {
    if (Value1 > Value2) Flag = 1;
    else if (Value1 < Value2) Flag = -1;
    else Flag = 0;
  }
  return Flag;
}

//Measure capacitance between two probe pins
void MeasureCap(byte Probe1, byte Probe2, byte ID)
{
  byte              TempByte;                     //Temp. value
  Capacitor_Type    *Cap;                         //Pointer to cap data structure
  Diode_Type        *Diode;                       //Pointer to diode data structure
  Resistor_Type     *Resistor;                    //Pointer to resistor data structure
  //Reset cap data */
  Cap = &Caps[ID];
  Cap->A = 0;
  Cap->B = 0;
  Cap->Scale = -12;                               //pF by default
  Cap->Raw = 0;
  Cap->Value = 0;
  if (CompFound == COMP_ERROR) return;            //Skip check on any error
  //Check for a resistor < 10 Ohm. Might be a large cap.
  if (CompFound == COMP_RESISTOR)
  {
    //Check for matching pins
    Resistor = &Resistors[0];                     //Pointer to first resistor
    TempByte = 0;
    while (TempByte < ResistorsFound)
    {
      //Got matching pins
      if (((Resistor->A == Probe1) && (Resistor->B == Probe2)) ||
        ((Resistor->A == Probe2) && (Resistor->B == Probe1)))
      {
        if (CmpValue(Resistor->Value, Resistor->Scale, 10UL, 0) == -1)
          TempByte = 99;                          //Signal low resistance and end loop
      }
      TempByte++;                                 //Next one
      Resistor++;                                 //Next one  
    }
    //We got a valid resistor 
    if (TempByte != 100) return;                  //Skip this one
  }
  //Skip measurement for "dangerous" diodes
  Diode = &Diodes[0];                             //Pointer to first diode
  for (TempByte = 0; TempByte < DiodesFound; TempByte++)
  {
    //Got matching pins and dangerous threshold voltage
    if ((Diode->C == Probe2) &&
      (Diode->A == Probe1) &&
      (Diode->V_f < 1500))
    {
      return;
    }
    Diode++;                                      //Next one
  }
  //Run measurements
  UpdateProbes(Probe1, Probe2, 0);                //Update bitmasks and probes
  //First run measurement for large caps  
  TempByte = LargeCap(Cap);
  //If cap is too small run measurement for small caps 
  if (TempByte == 2)
  {
    TempByte = SmallCap(Cap);
  }
  //Check for plausibility
  if (DiodesFound == 0)
  {
    //Low resistance might be a large cap 
    if (CompFound == COMP_RESISTOR)
    {
      //Report capacitor for large C (> 4.3uF)
      if (Cap->Scale >= -6) CompFound = COMP_CAPACITOR;
    }
    //We consider values below 5pF being just ghosts
    else if ((Cap->Scale > -12) || (Cap->Value >= 5UL))
    {
      CompFound = COMP_CAPACITOR;                 //Report capacitor
    }
  }
  //Clean up
  DischargeProbes();                              //Discharge DUT
  //Reset all ports and pins
  SetADCHiZ();                                    //Set ADC port to HiZ mode
  SetADCLow();                                    //Set ADC port low
  R_DDR = 0;                                      //Set resistor port to input
  R_PORT = 0;                                     //Set resistor port low
}

//Measure cap <4.7uF between two probe pins
byte SmallCap(Capacitor_Type *Cap)
{
  byte              Flag = 3;                     //Return value
  byte              TempByte;                     //Temp. value 
  signed char       Scale;                        //Capacitance scale
  unsigned int      Ticks;                        //Timer counter
  unsigned int      Ticks2;                       //Timer overflow counter
  unsigned int      U_c;                          //Voltage of capacitor
  unsigned long     Raw;                          //Raw capacitance value
  unsigned long     Value;                        //Corrected capacitance value
  Ticks2 = 0;                                     //Reset timer overflow counter
  //Init hardware, prepare probes
  DischargeProbes();                              //Try to discharge probes 
  if (CompFound == COMP_ERROR) return 0;          //Skip on error
  //Set probes: Gnd -- all probes / Gnd -- Rh -- probe-1
  R_PORT = 0;                                     //Set resistor port to low 
  //Set ADC probe pins to output mode
  ADC_DDR = (1 << TP1) | (1 << TP2) | (1 << TP3);
  SetADCLow();                                    //Set ADC port low
  R_DDR = Probe1_Rh;                              //Pull-down probe-1 via Rh
  //Setup analog comparator
  ADCSRB = (1 << ACME);                           //Use ADC multiplexer as negative input 
  ACSR =  (1 << ACBG) | (1 << ACIC);              //Use bandgap as positive input, trigger timer1
  ADMUX = (1 << REFS0) | Probe1_Pin;              //Switch ADC multiplexer to probe 1 and set AREF to Vcc
  ADCSRA = ADC_CLOCK_DIV;                         //Disable ADC, but keep clock dividers
  waitus(200);
  //Setup timer
  TCCR1A = 0;                                     //Set default mode
  TCCR1B = 0;                                     //Set more timer modes
  //Timer stopped, falling edge detection, noise canceler disabled
  TCNT1 = 0;                                      //Set Counter1 to 0
  //Clear all flags (input capture, compare A & B, overflow
  TIFR1 = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);
  R_PORT = Probe1_Rh;                             //Pull-up probe-1 via Rh                               
  //Enable timer
  if (CompFound == COMP_FET)
  {
    //Keep all probe pins pulled down but probe-1
    TempByte = (((1 << TP1) | (1 << TP2) | (1 << TP3)) & ~(1 << Probe1_Pin));    
  }
  else
  {
    TempByte = Probe2_ADC;                        //Keep just probe-1 pulled down
  }
  //Start timer by setting clock prescaler (1/1 clock divider)
  TCCR1B = (1 << CS10);
  ADC_DDR = TempByte;                             //Start charging DUT
  //Timer loop
  while (1)
  {
    TempByte = TIFR1;                            //Get timer1 flags 
    //End loop if input capture flag is set (= same voltage)
    if (TempByte & (1 << ICF1)) break;
    //Detect timer overflow by checking the overflow flag
    if (TempByte & (1 << TOV1))
    {
      //Happens at 65.536ms for 1MHz or 8.192ms for 8MHz
      TIFR1 = (1 << TOV1);                       //Reset flag
      wdt_reset();                               //Reset watchdog
      Ticks2++;                                  //Increase overflow counter
      //End loop if charging takes too long (13.1s) 
      if (Ticks2 == (CPU_FREQ / 5000)) break;
    }
  }
  //Stop counter
  TCCR1B = 0;                                     //Stop timer
  TIFR1 = (1 << ICF1);                            //Reset Input Capture flag
  Ticks = ICR1;                                   //Get counter value
  //Disable charging
  R_DDR = 0;                                      //Set resistor port to HiZ mode
  //Catch missed timer overflow
  if ((TCNT1 > Ticks) && (TempByte & (1 << TOV1)))
  {
    TIFR1 = (1 << TOV1);                          //Reset overflow flag
    Ticks2++;                                     //Increase overflow counter
  }
  //Enable ADC again
  ADCSRA = (1 << ADEN) | (1 << ADIF) | ADC_CLOCK_DIV;
  //Get voltage of DUT
  U_c = ReadU(Probe1_Pin);                        //Get voltage of cap
  //Start discharging DUT 
  R_PORT = 0;                                     //Pull down probe-2 via Rh
  R_DDR = Probe1_Rh;                              //Enable Rh for probe-1 again
  //Skip measurement if charging took too long
  if (Ticks2 >= (CPU_FREQ / 5000)) Flag = 1;
  //Calculate capacitance (<50uF)
  if (Flag == 3)
  {
    //Combine both counter values 
    Raw = (unsigned long)Ticks;                   //Set lower 16 bits
    Raw |= (unsigned long)Ticks2 << 16;           //Set upper 16 bits 
    //Subtract processing time overhead 
    if (Raw > 2) Raw -= 2;
    Scale = -12;                                  //Factor is for pF scale
    if (Raw > (UINT32_MAX / 1000))                //Prevent overflow (4.3*10^6)
    {
      Raw /= 1000;                                //Scale down by 10^3
      Scale += 3;                                 //Add 3 to the exponent
    }
    //Multiply with factor from table
    Raw *= GetFactor(Config.U_Bandgap + Config.CompOffset, TABLE_SMALL_CAP);
    //Divide by CPU frequency to get the time and multiply with table scale
    Raw /= (CPU_FREQ / 10000);
    Value = Raw;                                  //Take raw value
    //Take care about zero offset if feasable
    if (Scale == -12)                             //pF scale
    {
      if (Value >= Config.CapZero)                //If value is larger than offset
      {
        Value -= Config.CapZero;                  //substract offset
      }
      else                                        //If value is smaller than offset
      {
        //We have to prevent a negative value
        Value = 0;                                //Set value to 0
      }
    }
    //Copy data
    Cap->A = Probe2_Pin;                          //Pull-down probe pin 
    Cap->B = Probe1_Pin;                          //Pull-up probe pin 
    Cap->Scale = Scale;                           // -12 or -9 
    Cap->Raw = Raw;
    Cap->Value = Value;                           // max. 5.1*10^6pF or 125*10^3nF 
    //Self-adjust 
    if (((Scale == -12) && (Value >= 100000)) ||
      ((Scale == -9) && (Value <= 20000)))
    {
      signed int         Offset;
      signed long        TempLong;
      while (ReadU(Probe1_Pin) > 980)
      {
        //Keep discharging
      }
      R_DDR = 0;                                 //Stop discharging 
      Config.AutoScale = 0;                      //Disable auto scaling 
      Ticks = ReadU(Probe1_Pin);                 //U_c with Vcc reference 
      Config.AutoScale = 1;                      //Enable auto scaling again 
      Ticks2 = ReadU(Probe1_Pin);                //U_c with bandgap reference 
      R_DDR = Probe1_Rh;                         //Resume discharging 
      Offset = Ticks - Ticks2;
      //Allow some offset caused by the different voltage resolutions (4.88 vs. 1.07)
      if ((Offset < -4) || (Offset > 4))         //Offset too large 
      {
        //Calculate total offset: 
        TempLong = Offset;
        TempLong *= Config.U_Bandgap;            // * U_ref
        TempLong /= Ticks2;                      // / U_c 
        Config.RefOffset = (signed char)TempLong;
      }
      Offset = U_c - Config.U_Bandgap;
      //Limit offset to a valid range of -50mV - 50mV
      if ((Offset > -50) && (Offset < 50)) Config.CompOffset = Offset;
    }
  }
  return Flag;
}

//Measure cap >4.7uF between two probe pins (GOTO Free - Thanks to PaoloP)
byte LargeCap(Capacitor_Type *Cap)
{
  byte              Flag = 3;                     //Return value
  byte              TempByte;                     //Temp. value 
  byte              Mode;                         //Measurement mode 
  signed char       Scale;                        //Capacitance scale 
  unsigned int      TempInt;                      //Temp. value 
  unsigned int      Pulses;                       //Number of charging pulses 
  unsigned int      U_Zero;                       //Voltage before charging 
  unsigned int      U_Cap;                        //Voltage of DUT 
  unsigned int      U_Drop = 0;                   //Voltage drop 
  unsigned long     Raw;                          //Raw capacitance value 
  unsigned long     Value;                        //Corrected capacitance value 
  //Setup mode
  Mode = FLAG_10MS | FLAG_PULLUP;                 //Start with large caps 
  boolean rerun;
  do { 
    rerun = false;                                // One-Time
    //Prepare probes
    DischargeProbes();                            //Try to discharge probes
    if (CompFound == COMP_ERROR) return 0;        //Skip on error 
    //Setup probes: Gnd -- probe 1 / probe 2 -- Rl -- Vcc 
    SetADCLow();                                  //Set ADC port low
    ADC_DDR = Probe2_ADC;                         //Pull-down probe 2 directly 
    R_PORT = 0;                                   //Set resistor port to low
    R_DDR = 0;                                    //Set resistor port to HiZ 
    U_Zero = ReadU(Probe1_Pin);                   //Get zero voltage (noise)
    //Charge DUT with up to 500 pulses until it reaches 300mV 
    Pulses = 0;
    TempByte = 1;
    while (TempByte)
    {
      Pulses++;
      PullProbe(Probe1_Rl, Mode);                 //Charging pulse
      U_Cap = ReadU(Probe1_Pin);                  //Get voltage
      U_Cap -= U_Zero;                            //Zero offset
      //Eend loop if charging is too slow
      if ((Pulses == 126) && (U_Cap < 75)) TempByte = 0;
      //End loop if 300mV are reached
      if (U_Cap >= 300) TempByte = 0;
      //End loop if maximum pulses are reached
      if (Pulses == 500) TempByte = 0;
      wdt_reset();                                //Reset watchdog
    }
    //If 300mV are not reached DUT isn't a cap or much too large (>100mF)we can ignore that for mid-sized caps */
    if (U_Cap < 300)
    {
      Flag = 1;
    }
    //If 1300mV are reached with one pulse we got a small cap 
    if ((Pulses == 1) && (U_Cap > 1300))
    {
      if (Mode & FLAG_10MS)                       // <47uF 
      {
        Mode = FLAG_1MS | FLAG_PULLUP;            //Set mode (1ms charging pulses)
        rerun = true;
      }
      else                                        // <4.7uF 
      {
        Flag = 2;
      }
    }
  } 
  while (rerun);
  //Check if DUT sustains the charge and get the voltage drop
  if (Flag == 3)
  {
    //Check self-discharging
    TempInt = Pulses;
    while (TempInt > 0)
    {
      TempInt--;                                  //Descrease timeout
      U_Drop = ReadU(Probe1_Pin);                 //Get voltage
      U_Drop -= U_Zero;                           //Zero offset
      wdt_reset();                                //Reset watchdog
    }
    //Calculate voltage drop
    if (U_Cap > U_Drop) U_Drop = U_Cap - U_Drop;
    else U_Drop = 0;
    //If voltage drop is too large consider DUT not to be a cap
    if (U_Drop > 100) Flag = 0;
  }
  //Calculate capacitance
  if (Flag == 3)
  {
    Scale = -9;                                   //Factor is scaled to nF
    //Get interpolated factor from table
    Raw = GetFactor(U_Cap + U_Drop, TABLE_LARGE_CAP);
    Raw *= Pulses;                                //C = pulses * factor 
    if (Mode & FLAG_10MS) Raw *= 10;              // *10 for 10ms charging pulses 

    if (Raw > UINT32_MAX / 1000)                  //Scale down if C >4.3mF
    {
      Raw /= 1000;                                //Scale down by 10^3
      Scale += 3;                                 //Add 3 to the exponent
    }
    Value = Raw;                                  //Copy raw value
    //It seems that we got a systematic error
    Value *= 100;
    if (Mode & FLAG_10MS) Value /= 109;           // -9% for large cap 
    else Value /= 104;                            // -4% for mid cap 
    //Copy data
    Cap->A = Probe2_Pin;                          //Pull-down probe pin 
    Cap->B = Probe1_Pin;                          //Pull-up probe pin 
    Cap->Scale = Scale;                           // -9 or -6 
    Cap->Raw = Raw;
    Cap->Value = Value;                           // max. 4.3*10^6nF or 100*10^3uF  
  }
  return Flag;
}

//Check for diode
void CheckDiode(void)
{
  Diode_Type        *Diode;                       //Pointer to diode 
  unsigned int      U1_Rl;                        //Vf #1 with Rl pull-up 
  unsigned int      U2_Rl;                        //Vf #2 with Rl pull-up 
  unsigned int      U1_Rh;                        //Vf #1 with Rh pull-up 
  unsigned int      U2_Rh;                        //Vf #2 with Rh pull-up 
  wdt_reset();                                    //Reset watchdog 
  DischargeProbes();                              //Try to discharge probes 
  if (CompFound == COMP_ERROR) return;            //Skip on error
  SetADCLow();                                    //Set ADC port low
  ADC_DDR = Probe2_ADC;                           //Pull down cathode directly
  //Vf #1, supporting a possible p-channel MOSFET
  //Measure voltage across DUT (Vf) with Rl
  R_DDR = Probe1_Rl;                              //Enable Rl for probe-1
  R_PORT = Probe1_Rl;                             //Pull up anode via Rl 
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLUP);  //Discharge gate 
  U1_Rl = ReadU_5ms(Probe1_Pin);                  //Get voltage at anode 
  U1_Rl -= ReadU(Probe2_Pin);                     //Substract voltage at cathode 
  //Measure voltage across DUT (Vf) with Rh
  R_DDR = Probe1_Rh;                              //Enable Rh for probe-1 
  R_PORT = Probe1_Rh;                             //Pull up anode via Rh 
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLUP);  //Discharge gate 
  U1_Rh = ReadU_5ms(Probe1_Pin);                  //Get voltage at anode, neglect voltage at cathode
  //Vf #2, supporting a possible n-channel MOSFET
  //Measure voltage across DUT (Vf) with Rl
  R_DDR = Probe1_Rl;                              //Enable Rl for probe-1 
  R_PORT = Probe1_Rl;                             //Pull up anode via Rl    
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLDOWN);//Discharge gate 
  U2_Rl = ReadU_5ms(Probe1_Pin);                  //Get voltage at anode 
  U2_Rl -= ReadU(Probe2_Pin);                     //Substract voltage at cathode 
  //Measure voltage across DUT (Vf) with Rh 
  R_DDR = Probe1_Rh;                              //Enable Rh for probe-1 
  R_PORT = Probe1_Rh;                             //Pull up anode via Rh 
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLDOWN);//Discharge gate 
  U2_Rh = ReadU_5ms(Probe1_Pin);                  //Get voltage at anode, neglect voltage at cathode
  //Process results, choose between measurements of p and n channel setup
  if (U1_Rl > U2_Rl)                              //The higher voltage wins 
  {
    U2_Rl = U1_Rl;
    U2_Rh = U1_Rh;
  }
  //Vf is between 0.15V and 4.64V and within limits for high and low test currents
  if ((U2_Rl > 150) &&
    (U2_Rl < 4640) &&
    (U2_Rl > (U2_Rh + (U2_Rh / 8))) &&
    (U2_Rl < (U2_Rh * 8)))
  {
    //If we haven't found any other component yet
    if ((CompFound == COMP_NONE) ||
      (CompFound == COMP_RESISTOR))
    {
      CompFound = COMP_DIODE;
    }
    //Save data 
    Diode = &Diodes[DiodesFound];
    Diode->A = Probe1_Pin;
    Diode->C = Probe2_Pin;
    Diode->V_f = U2_Rl;                           //Vf for high measurement current */
    Diode->V_f2 = U2_Rh;                          //Vf for low measurement current */
    DiodesFound++;
  }
}

//Measure a resistor with low resistance (< 100 Ohms)
unsigned int SmallResistor(void)
{
  unsigned int      R = 0;                        //Return value
  byte              Probe;                        //Probe ID 
  byte              Mode;                         //Measurement mode 
  byte              Counter;                      //Sample counter 
  unsigned long     Value;                        //ADC sample value 
  unsigned long     Value1 = 0;                   //U_Rl temp. value 
  unsigned long     Value2 = 0;                   //U_R_i_L temp. value 
  DischargeProbes();                              //Try to discharge probes 
  if (CompFound == COMP_ERROR) return R;          //Skip on error 
  //Charge: GND -- probe 2 / probe 1 -- Rl -- 5V - discharge: GND -- probe 2 / probe 1 -- Rl -- GND 
  SetADCLow();                                    //Set ADC port low
  ADC_DDR = Probe2_ADC;                           //Pull-down probe 2 directly 
  R_PORT = 0;                                     //Low by default 
  R_DDR = Probe1_Rl;                              //Enable resistor 
#define MODE_HIGH        0b00000001
#define MODE_LOW         0b00000010
  //Monster loop
  Mode = MODE_HIGH;
  while (Mode > 0)
  {
    //Setup measurement
    if (Mode & MODE_HIGH) Probe = Probe1_Pin;
    else Probe = Probe2_Pin;
    wdt_reset();                                  //Reset watchdog
    Counter = 0;                                  //Reset loop counter
    Value = 0;                                    //Reset sample value
    //Set ADC to use bandgap reference and run a dummy conversion 
    Probe |= (1 << REFS0) | (1 << REFS1);
    ADMUX = Probe;                                //Set input channel and U reference
    waitus(100);                                  //Time for voltage stabilization 
    ADCSRA |= (1 << ADSC);                        //Start conversion 
    while (ADCSRA & (1 << ADSC));                 //Wait until conversion is done 
    //Measurement loop (about 1ms per cycle)  
    while (Counter < 100)
    {
      //Create short charging pulse
      ADC_DDR = Probe2_ADC;                       //Pull-down probe 2 directly
      R_PORT = Probe1_Rl;
      //Start ADC conversion
      ADCSRA |= (1 << ADSC);           
      //Wait 20us
      waitus(20);
      //Start discharging
      R_PORT = 0;
      ADC_DDR = Probe2_ADC | Probe1_ADC;
      //Get ADC reading (about 100us)
      while (ADCSRA & (1 << ADSC));               //Wait until conversion is done
      Value += ADCW;                              //Add ADC reading
      //Wait
      waitus(900);
      Counter++;                                  //Next round
    }
    //Convert ADC reading to voltage
    Value *= Config.U_Bandgap;
    Value /= 1024;                                // / 1024 for 10bit ADC 
    Value /= 10;                                  //De-sample to 0.1mV
    //Loop control
    if (Mode & MODE_HIGH)                         //Probe #1 / Rl 
    {
      Mode = MODE_LOW;
      Value1 = Value;
    }
    else                                          //Probe #2 / R_i_L 
    {
      Mode = 0;
      Value2 = Value;
    }
  }
  //Process measurement
  if (Value1 > Value2)                            //Sanity check 
  {
    // I = U/R = (5V - U_Rl)/(Rl + R_i_H)
    Value = 10UL * UREF_VCC;                      //in 0.1 mV 
    Value -= Value1;
    Value *= 1000;                                //Scale to µA
    Value /= ((R_LOW * 10) + Config.RiH);         //in 0.1 Ohms
    // U = U_Rl - U_R_i_L = U_Rl - (R_i_L * I) 
    // U = U_probe1 - U_probe2 
    Value1 -= Value2;                             //in 0.1 mV
    Value1 *= 10000;                              //Scale to 0.01 µV 
    //R = U/I (including R of probe leads)
    Value1 /= Value;                              //in 0.01 Ohms */
    R = (unsigned int)Value1;
  }
#undef MODE_LOW
#undef MODE_HIGH
  //Update Uref flag for next ADC run 
  Config.RefFlag = (1 << REFS1);                  //Set REFS1 bit flag */
  return R;
}

//Check for resistor
void CheckResistor(void)
{
  Resistor_Type     *Resistor;                    //Pointer to resistor 
  unsigned long     Value1;                       //Resistance of measurement #1 
  unsigned long     Value2;                       //Resistance of measurement #2 
  unsigned long     Value;                        //Resistance value
  unsigned long     Temp;                         //Temp. value 
  signed char       Scale;                        //Resistance scaling 
  byte              n;                            //Counter 
  //Voltages 
  unsigned int      U_Rl_H;                       //Voltage #1 
  unsigned int      U_Ri_L;                       //Voltage #2 
  unsigned int      U_Rl_L;                       //Voltage #3 
  unsigned int      U_Ri_H;                       //Voltage #4 
  unsigned int      U_Rh_H;                       //Voltage #5 
  unsigned int      U_Rh_L;                       //Voltage #6 
  wdt_reset();                                    //Reset watchdog
  //Resistor measurement
  SetADCLow();                                    //Set ADC port low
  ADC_DDR = Probe2_ADC;                           //Pull down probe-2 directly
  R_DDR = Probe1_Rl;                              //Enable Rl for probe-1
  R_PORT = Probe1_Rl;                             //Pull up probe-1 via Rl
  U_Ri_L = ReadU_5ms(Probe2_Pin);                 //Get voltage at internal R
  U_Rl_H = ReadU(Probe1_Pin);                     //Get voltage at Rl pulled up
  //Check for a capacitor, set probes: Gnd -- probe-2 / Gnd -- Rh -- probe-1
  R_PORT = 0;                                     //Set resistor port low 
  R_DDR = Probe1_Rh;                              //Pull down probe-1 via Rh
  U_Rh_L = ReadU_5ms(Probe1_Pin);                 //Get voltage at probe 1
  //We got a resistor if the voltage is near Gnd
  if (U_Rh_L <= 20)
  {
    //Set probes: Gnd -- probe-2 / probe-1 -- Rh -- Vcc 
    R_PORT = Probe1_Rh;                           //Pull up probe-1 via Rh
    U_Rh_H = ReadU_5ms(Probe1_Pin);               //Get voltage at Rh pulled up
    //Set probes: Gnd -- Rl -- probe-2 / probe-1 -- Vcc
    ADC_DDR = Probe1_ADC;                         //Set probe-1 to output 
    ADC_PORT = Probe1_ADC;                        //Pull up probe-1 directly 
    R_PORT = 0;                                   //Set resistor port to low  
    R_DDR = Probe2_Rl;                            //Pull down probe-2 via Rl 
    U_Ri_H = ReadU_5ms(Probe1_Pin);               //Get voltage at internal R  
    U_Rl_L = ReadU(Probe2_Pin);                   //Get voltage at Rl pulled down 
    //Set probes: Gnd -- Rh -- probe-2 / probe-1 -- Vcc
    R_DDR = Probe2_Rh;                            //Pull down probe-2 via Rh 
    U_Rh_L = ReadU_5ms(Probe2_Pin);               //Get voltage at Rh pulled down 
    //If voltage breakdown is sufficient
    if ((U_Rl_H >= 4400) || (U_Rh_H <= 97))       //R >= 5.1k / R < 9.3k 
    {
      if (U_Rh_H < 4972)                          //R < 83.4M & prevent division by zero 
      {
        //Voltage breaks down with low test current and it is not nearly shorted => resistor 
        Value = 0;                                //Reset value of resistor 
        if (U_Rl_L < 169)                         //R > 19.5k 
        {
          //Use measurements done with Rh, esistor is less 60MOhm
          if (U_Rh_L >= 38)                       //R < 61.4M & prevent division by zero
          {
            Value1 = R_HIGH * U_Rh_H;
            Value1 /= (UREF_VCC - U_Rh_H);
            Value2 = R_HIGH * (UREF_VCC - U_Rh_L);
            Value2 /= U_Rh_L;
            //Calculate weighted average of both measurements
            if (U_Rh_H < 990)                     //Below bandgap reference
            {
              //Weighted average for U_Rh_H
              Value = (Value1 * 4);
              Value += Value2;
              Value /= 5;
            }
            else if (U_Rh_L < 990)                //Below bandgap reference
            {
              //Weighted average for U_Rh_L
              Value = (Value2 * 4);
              Value += Value1;
              Value /= 5;
            }
            else                                  //Higher than bandgap reference
            {
              //Classic average
              Value = (Value1 + Value2) / 2;
            }

            Value += RH_OFFSET;                   //Add offset value for Rh
            Value *= 10;                          //Upscale to 0.1 Ohms
          }
        }
        else                                      //U_Rl_L: R <= 19.5k
        {
          //Use measurements done with Rl
          if ((U_Rl_H >= U_Ri_L) && (U_Ri_H >= U_Rl_L))
          {
            //Prevent division by zero
            if (U_Rl_H == UREF_VCC) U_Rl_H = UREF_VCC - 1;   
            Value1 = (R_LOW * 10) + Config.RiH;   //Rl + RiH in 0.1 Ohm
            Value1 *= (U_Rl_H - U_Ri_L);
            Value1 /= (UREF_VCC - U_Rl_H);
            Value2 = (R_LOW * 10) + Config.RiL;   //Rl + RiL in 0.1 Ohms
            Value2 *= (U_Ri_H - U_Rl_L);
            Value2 /= U_Rl_L;
            //Calculate weighted average of both measurements
            if (U_Rl_H < 990)                     //Below bandgap reference
            {
              //Weighted average for U_Rh_H
              Value = (Value1 * 4);
              Value += Value2;
              Value /= 5;
            }
            else if (U_Rl_L < 990)                //Below bandgap reference
            {
              //Weighted average for U_Rh_L
              Value = (Value2 * 4);
              Value += Value1;
              Value /= 5;
            }
            else                                  //Higher than bandgap reference
            {
              //Classic average
              Value = (Value1 + Value2) / 2;
            }
          }
          else      
          {
            if (U_Rl_L > 4750) Value = 1;         //U_Rl_L: R < 15 Ohms
            //This will trigger the low resistance measurement below
          }
        }
        //Process results of the resistance measurement
        if (Value > 0)                            //Valid resistor
        {
          Scale = -1;                             //0.1 Ohm by default
          //Meassure small resistor <10 Ohm with special method
          if (Value < 100UL)
          {
            Value = (unsigned long)SmallResistor();
            Scale = -2;                           //0.01 Ohm
            //Auto-zero
            if (Value > Config.RZero) Value -= Config.RZero;
            else Value = 0;
          }
          //Check for measurement in reversed direction
          n = 0;
          while (n < ResistorsFound)
          {
            Resistor = &Resistors[n];
            if (Resistor->HiZ == Probe3_Pin)      //Same HiZ probe
            {
              //Check if the reverse measurement is within a specific tolerance
              if (CmpValue(Value, Scale, 1, 0) == -1)  
              {
                Temp = Value / 2;                 //50%
              }
              else                                // >= 1 Ohm
              {
                Temp = Value / 20;                //5%
              }

              Value1 = Value - Temp;              //95% or 50%
              Value2 = Value + Temp;              //105% or 150% 
              //Special case for very low resistance
              if (CmpValue(Value, Scale, 1, -1) == -1) 
              {
                Value1 = 0;                       //0
                Value2 = Value * 5;               //500%
                if (Value2 == 0) Value2 = 5;      //Special case
              }
              //Check if value matches given tolerance
              if ((CmpValue(Resistor->Value, Resistor->Scale, Value1, Scale) >= 0) &&
                (CmpValue(Resistor->Value, Resistor->Scale, Value2, Scale) <= 0))
              {
                n = 100;                          //End loop and signal match
              }
              else                                //No match
              {
                n = 200;                          //End loop and signal mis-match 
              }
            }
            else                                  //No match
            {
              n++;                                //Next one 
            }
          }
          //We got a new resistor
          if (n != 100)
          {
            CompFound = COMP_RESISTOR;
            //Save data
            Resistor = &Resistors[ResistorsFound];
            Resistor->A = Probe2_Pin;
            Resistor->B = Probe1_Pin;
            Resistor->HiZ = Probe3_Pin;
            Resistor->Value = Value;
            Resistor->Scale = Scale;
            //Another one found
            ResistorsFound++;           
            //Prevent array overflow            
            if (ResistorsFound > 6) ResistorsFound--;
          }
        }
      }
    }
  }
}

//Check for depletion mode FET
unsigned int CheckDepModeFET(void)
{
  unsigned int      U_Rl_L;                       //Return value / voltage across Rl 
  unsigned int      U_1;                          //Voltage #1
  unsigned int      U_2;                          //Voltage #2
  //Setup probes, - Gnd -- Rl -- probe-2 / probe-1 -- Vcc
  R_PORT = 0;                                     //Set resistor port to low
  R_DDR = Probe2_Rl;                              //Pull down probe-2 via Rl
  ADC_DDR = Probe1_ADC;                           //Set probe-1 to output
  ADC_PORT = Probe1_ADC;                          //Pull-up probe-1 directly
  //Some FETs require the gate being discharged
  //Try n-channel
  //We assume: probe-1 = D / probe-2 = S / probe-3 = G 
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLDOWN);//Discharge gate via Rl
  U_Rl_L = ReadU_5ms(Probe2_Pin);                 //Get voltage at source
  //For n channel we would expect a low voltage
  if (U_Rl_L >= 977)                              //This might by a p-channel
  {
    //We assume: probe-1 = S / probe-2 = D / probe-3 = G
    PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLUP);//Discharge gate via Rl
    U_Rl_L = ReadU_5ms(Probe2_Pin);               //Get voltage at drain
  }
  //Other possibilities: diode or resistor
  if (U_Rl_L > 92)
  {
    //Check if we got a n-channel JFET or depletion-mode MOSFET
    if (CompDone == 0)                            //No component found yet
    {
      //We assume: probe-1 = S / probe-2 = D / probe-3 = G
      R_DDR = Probe2_Rl | Probe3_Rh;              //Pull down gate via Rh
      U_1 = ReadU_20ms(Probe2_Pin);               //Voltage at source

      R_PORT = Probe3_Rh;                         //Pull up gate via Rh 
      U_2 = ReadU_20ms(Probe2_Pin);               //Voltage at source 
      if (U_2 > (U_1 + 488))
      {
        //Compare gate voltage to distinguish JFET from MOSFET
        //Set probes: Gnd -- probe-2 / probe-1 -- Rl -- Vcc 
        SetADCLow();                              //Set ADC port low
        ADC_DDR = Probe2_ADC;                     //Pull down source directly
        R_DDR = Probe1_Rl | Probe3_Rh;            //Enable Rl for probe-1 & Rh for probe-3
        R_PORT = Probe1_Rl | Probe3_Rh;           //Pull up drain via Rl / pull up gate via Rh
        U_2 = ReadU_20ms(Probe3_Pin);             //Get voltage at gate
        if (U_2 > 3911)                           //MOSFET
        {
          //n channel depletion-mode MOSFET 
          CompType = TYPE_N_CHANNEL | TYPE_DEPLETION | TYPE_MOSFET;
        }
        else                                      //JFET
        {
          //n channel JFET (depletion-mode only)
          CompType = TYPE_N_CHANNEL | TYPE_JFET;
        }
        //Save data
        CompFound = COMP_FET;
        CompDone = 1;
        FET.G = Probe3_Pin;
        FET.D = Probe1_Pin;
        FET.S = Probe2_Pin;
      }
    }
    //Check if we got a p-channel JFET or depletion-mode MOSFET
    if (CompDone == 0)                            //No component found yet
    {
      //We assume: probe-1 = S / probe-2 = D / probe 3 = G
      //Set probes: Gnd -- probe-2 / probe-1 -- Rl -- Vcc 
      SetADCLow();                                //Set ADC port low
      ADC_DDR = Probe2_ADC;                       //Pull down drain directly
      R_DDR = Probe1_Rl | Probe3_Rh;              //Enable Rl for probe-1 & Rh for probe-3
      R_PORT = Probe1_Rl | Probe3_Rh;             //Pull up source via Rl / pull up gate via Rh
      U_1 = ReadU_20ms(Probe1_Pin);               //Get voltage at source
      R_PORT = Probe1_Rl;                         //Pull down gate via Rh
      U_2 = ReadU_20ms(Probe1_Pin);               //Get voltage at source
      if (U_1 > (U_2 + 488))
      {
        //Compare gate voltage to distinguish JFET from MOSFET
        //Set probes: probe-2 = HiZ / probe-1 -- Vcc
        ADC_PORT = Probe1_ADC;                    //Pull up source directly 
        ADC_DDR = Probe1_ADC;                     //Enable pull up for source 
        //Gate is still pulled down via Rh
        U_2 = ReadU_20ms(Probe3_Pin);             //Get voltage at gate 
        if (U_2 < 977)                            //MOSFET
        {
          //p channel depletion-mode MOSFET
          CompType =  TYPE_P_CHANNEL | TYPE_DEPLETION | TYPE_MOSFET;
        }
        else                                      //JFET
        {
          //p channel JFET (depletion-mode only)
          CompType = TYPE_P_CHANNEL | TYPE_DEPLETION | TYPE_JFET;
        }
        //Save data
        CompFound = COMP_FET;
        CompDone = 1;
        FET.G = Probe3_Pin;
        FET.D = Probe2_Pin;
        FET.S = Probe1_Pin;
      }
    }
  }
  return U_Rl_L;
}

//Measure hfe of BJT in common collector circuit (emitter follower)
unsigned long Get_hfe_c(byte Type)
{
  unsigned long     hfe;                          //Return value 
  unsigned int      U_R_e;                        //Voltage across emitter resistor
  unsigned int      U_R_b;                        //Voltage across base resistor
  unsigned int      Ri;                           //Internal resistance 
  //Measure hfe for a BJT in common collector circuit
  //Setup probes and get voltages
  if (Type == TYPE_NPN)                           //NPN
  {
    //We assume: probe-1 = C / probe-2 = E / probe-3 = B
    //Set probes: Gnd -- Rl -- probe-2 / probe-1 -- Vcc
    ADC_DDR = Probe1_ADC;                         //Set probe 1 to output
    ADC_PORT = Probe1_ADC;                        //Pull up collector directly
    R_DDR = Probe2_Rl | Probe3_Rl;                //Select Rl for probe-2 & Rl for probe-3 
    R_PORT = Probe3_Rl;                           //Pull up base via Rl 
    U_R_e = ReadU_5ms(Probe2_Pin);                //U_R_e = U_e 
    U_R_b = UREF_VCC - ReadU(Probe3_Pin);         //U_R_b = Vcc - U_b 
  }
  else                                            //PNP 
  {
    //We assume: probe-1 = E / probe-2 = C / probe-3 = B 
    //Set probes: Gnd -- probe-2 / probe-1 -- Rl -- Vcc 
    SetADCLow();                                  //Set ADC port low
    ADC_DDR = Probe2_ADC;                         //Pull down collector directly 
    R_PORT = Probe1_Rl;                           //Pull up emitter via Rl 
    R_DDR = Probe1_Rl | Probe3_Rl;                //Pull down base via Rl 
    U_R_e = UREF_VCC - ReadU_5ms(Probe1_Pin);     //U_R_e = Vcc - U_e 
    U_R_b = ReadU(Probe3_Pin);                    //U_R_b = U_b 
  }
  if (U_R_b < 10)                                 //I_b < 14µA = Darlington 
  {
    //Change base resistor from Rl to Rh and measure again 
    if (Type == TYPE_NPN)                         //NPN 
    {    
      R_DDR = Probe2_Rl | Probe3_Rh;              //Select Rl for probe-2 & Rh for probe-3 
      R_PORT = Probe3_Rh;                         //Pull up base via Rh 
      U_R_e = ReadU_5ms(Probe2_Pin);              //U_R_e = U_e 
      U_R_b = UREF_VCC - ReadU(Probe3_Pin);       //U_R_b = Vcc - U_b 
      Ri = Config.RiL;                            //Get internal resistor 
    }
    else                                          //PNP 
    {
      R_DDR = Probe1_Rl | Probe3_Rh;              //Pull down base via Rh 
      U_R_e = UREF_VCC - ReadU_5ms(Probe1_Pin);   //U_R_e = Vcc - U_e 
      U_R_b = ReadU(Probe3_Pin);                  //U_R_b = U_b 
      Ri = Config.RiH;                            //Get internal resistor
    }
    if (U_R_b < 1) U_R_b = 1;                     //Prevent division by zero 
    hfe =  U_R_e * R_HIGH;                        //U_R_e * R_b 
    hfe /= U_R_b;                                 // / U_R_b 
    hfe *= 10;                                    //Upscale to 0.1 
    hfe /= (R_LOW * 10) + Ri;                     // / R_e in 0.1 Ohm 
  }
  else                                            //I_b > 14µA = standard 
  {
    //Both resistors are the same (R_e = R_b): 
    hfe = (unsigned long)((U_R_e - U_R_b) / U_R_b);
  }
  return hfe;
}

//Check for thyristor and triac
byte CheckThyristorTriac(void)
{
  byte              Flag = 0;                     //Return value
  unsigned int      U_1;                          //Voltage #1
  unsigned int      U_2;                          //Voltage #2
  //We assume: probe-1 = A / probe-2 = C / probe-3 = G
  PullProbe(Probe3_Rl, FLAG_10MS | FLAG_PULLDOWN);//Discharge gate
  U_1 = ReadU_5ms(Probe1_Pin);                    //Get voltage at anode
  R_PORT = 0;                                     //Pull down anode
  delay(5);
  R_PORT = Probe1_Rl;                             //and pull up anode again
  U_2 = ReadU_5ms(Probe1_Pin);                    //Get voltage at anode (below Rl)
  //Voltages match behaviour of thyristor or triac
  if ((U_1 < 1600) && (U_2 > 4400))
  {
    CompFound = COMP_THYRISTOR;                   //If not detected as a triac below
    CompDone = 1;
    //We assume: probe-1 = MT2 / probe-2 = MT1 / probe-3 = G
    R_DDR = 0;                                    //Disable all probe resistors 
    R_PORT = 0;
    ADC_PORT = Probe2_ADC;                        //Pull up MT1 directly
    delay(5);
    R_DDR = Probe1_Rl;                            //Pull down MT2 via Rl 
    //Probe-3/gate is in HiZ mode 
    //triac shouldn't conduct without a triggered gate  
    U_1 = ReadU_5ms(Probe1_Pin);                  //Get voltage at MT2 
    //Voltage of MT2 is low (no current)
    if (U_1 <= 244)
    {
      //Trigger gate for reverse direction 
      R_DDR = Probe1_Rl | Probe3_Rl;              //and pull down gate via Rl 
      U_1 = ReadU_5ms(Probe3_Pin);                //Get voltage at gate 
      U_2 = ReadU(Probe1_Pin);                    //Get voltage at MT2   
      if ((U_1 >= 977) && (U_2 >= 733))
      {
        //Check if triac still conducts without triggered gate 
        R_DDR = Probe1_Rl;                        //Set probe3 to HiZ mode
        U_1 = ReadU_5ms(Probe1_Pin);              //Get voltage at MT2 
        //Voltage at MT2 is still high (current = triac is conducting)
        if (U_1 >= 733)
        {
          //Check if triac stops conducting when load current drops to zero
          R_PORT = Probe1_Rl;                     //Pull up MT2 via Rl
          delay(5);
          R_PORT = 0;                             //and pull down MT2 via Rl
          U_1 = ReadU_5ms(Probe1_Pin);            //Get voltage at MT2
          //Voltage at MT2 is low (no current = triac is not conducting)
          if (U_1 <= 244)
          {
            //Now we are pretty sure that the DUT is a triac
            CompFound = COMP_TRIAC;
          }
        }
      }
    }
    //Save data (we misuse BJT)
    BJT.B = Probe3_Pin;
    BJT.C = Probe1_Pin;
    BJT.E = Probe2_Pin;
    Flag = 1;                                     //Signal that we found a component
  }
  return Flag;
}

//Measure the gate threshold voltage of a depletion-mode MOSFET
void GetGateThreshold(byte Type)
{
  unsigned long     Uth = 0;                      //Gate threshold voltage
  byte              Drain_Rl;                     //Rl bitmask for drain
  byte              Drain_ADC;                    //ADC bitmask for drain
  byte              PullMode;
  byte              Counter;                      //Loop counter
  //Init variables
  if (Type & TYPE_N_CHANNEL)                      //n-channel
  {
    //We assume: probe-1 = D / probe-2 = S / probe-3 = G
    Drain_Rl =  Probe1_Rl;
    Drain_ADC = Probe1_ADC;
    PullMode = FLAG_10MS | FLAG_PULLDOWN;
  }
  else                                            //p-channel
  {
    //We assume: probe-1 = S / probe-2 = D / probe-3 = G
    Drain_Rl =  Probe2_Rl;
    Drain_ADC = Probe2_ADC;
    PullMode = FLAG_10MS | FLAG_PULLUP;
  }
  //For low reaction times we use the ADC directly.
  //Sanitize bit mask for drain to prevent a never-ending loop
  Drain_ADC &= 0b00000111;                        //drain
  ADMUX = Probe3_Pin | (1 << REFS0);              //Select probe-3 for ADC input
  //Sample 10 times
  for (Counter = 0; Counter < 10; Counter++) 
  {
    wdt_reset();                                  //Reset watchdog
    //Discharge gate via Rl for 10 ms
    PullProbe(Probe3_Rl, PullMode);
    //Pull up/down gate via Rh to slowly charge gate
    R_DDR = Drain_Rl | Probe3_Rh;
    //Wait until FET conducts
    if (Type & TYPE_N_CHANNEL)                    //n-channel
    {
      //FET conducts when the voltage at drain reaches low level
      while (ADC_PIN & Drain_ADC);
    }
    else                                          //p-channel
    {
      //FET conducts when the voltage at drain reaches high level
      while (!(ADC_PIN & Drain_ADC));             
    }
    R_DDR = Drain_Rl;                             //Set probe-3 to HiZ mode
    //Get voltage of gate
    ADCSRA |= (1 << ADSC);                        //Start ADC conversion
    while (ADCSRA & (1 << ADSC));                 //Wait until conversion is done
    //Add ADC reading 
    if (Type & TYPE_N_CHANNEL)                    //n-channel
    {
      Uth += ADCW;                                //U_g = U_measued
    }
    else                                          //p-channel
    {
      Uth += (1023 - ADCW);                       //U_g = Vcc - U_measured
    }
  }
  //Calculate V_th
  Uth /= 10;                                      //Average of 10 samples
  Uth *= UREF_VCC;                                //Convert to voltage
  Uth /= 1024;                                    //Using 10 bit resolution
  //Save data
  FET.V_th = (unsigned int)Uth;
}

//Check for BJT or depletion-mode MOSFET
void CheckBJTorDepMOSFET(byte BJT_Type, unsigned int U_Rl)
{
  byte              FET_Type;                     //MOSFET type
  unsigned int      U_R_c;                        //Voltage across collector resistor
  unsigned int      U_R_b;                        //Voltage across base resistor
  unsigned int      BJT_Level;
  unsigned int      FET_Level;
  unsigned long     hfe_c;                        //hfe (common collector)
  unsigned long     hfe_e;                        //hfe (common emitter)
  //Init, set probes and measure
  if (BJT_Type == TYPE_NPN)                       //NPN
  {
    BJT_Level = 2557;
    FET_Level = 3400;
    FET_Type = TYPE_N_CHANNEL;
    //We assume: probe-1 = C / probe-2 = E / probe-3 = B
    R_DDR = Probe1_Rl | Probe3_Rh;                //Enable Rl for probe-1 & Rh for probe-3
    R_PORT = Probe1_Rl | Probe3_Rh;               //Pull up collector via Rl and base via Rh
    delay(50);                                    //Wait to skip gate charging of a FET
    U_R_c = UREF_VCC - ReadU(Probe1_Pin);         //U_R_c = Vcc - U_c 
    U_R_b = UREF_VCC - ReadU(Probe3_Pin);         //U_R_b = Vcc - U_b
  }
  else                                            //PNP
  {
    BJT_Level = 977;
    FET_Level = 2000;
    FET_Type = TYPE_P_CHANNEL;
    //Drive base/gate via Rh instead of Rl
    //We assume: probe-1 = E / probe-2 = C / probe-3 = B
    R_DDR = Probe2_Rl | Probe3_Rh;                //Pull down base via Rh
    U_R_c = ReadU_5ms(Probe2_Pin);                //U_R_c = U_c
    U_R_b = ReadU(Probe3_Pin);                    //U_R_b = U_b
  }
  //Distinguish BJT from depletion-mode MOSFET
  if (U_R_b > BJT_Level)                          //U_R_b matches minimum level of BJT
  {
    //Two test runs needed at maximium to get right hfe & pins
    if (CompFound == COMP_BJT) CompDone = 1;
    CompFound = COMP_BJT;
    CompType = BJT_Type;
    //Calculate hfe via voltages and known resistors:
    hfe_e = U_R_c * R_HIGH;                       //U_R_c * R_b 
    hfe_e /= U_R_b;                               // / U_R_b 
    hfe_e *= 10;                                  //Upscale to 0.1 
    if (BJT_Type == TYPE_NPN)                     //NPN 
        hfe_e /= (R_LOW * 10) + Config.RiH;         // / R_c in 0.1 Ohm 
    else                                          //PNP 
    hfe_e /= (R_LOW * 10) + Config.RiL;         // / R_c in 0.1 Ohm 
    //Get hfe for common collector circuit
    hfe_c = Get_hfe_c(BJT_Type);
    //Keep largest hfe
    if (hfe_c > hfe_e) hfe_e = hfe_c;
    //Only update data if hfe is larger than old one 
    if (hfe_e > BJT.hfe)
    {
      //Save data
      BJT.hfe = hfe_e;
      BJT.B = Probe3_Pin;
      if (BJT_Type == TYPE_NPN)                   //NPN
      {
        BJT.C = Probe1_Pin;
        BJT.E = Probe2_Pin;
      }
      else                                        //PNP
      {
        BJT.C = Probe2_Pin;
        BJT.E = Probe1_Pin;
      }
    }
#if 0
    //Check for proper emitter and collector:
    SetADCHiZ();                                  //Set ADC port to HiZ mode
    R_DDR = 0;                                    //Set resistor port to HiZ mode
    if (BJT_Type == TYPE_NPN)                     //NPN
    {
      //We assume: probe-1 = E / probe-2 = C / probe-3 = B
      SetADCLow();                                //Set ADC port low
      ADC_DDR = Probe1_ADC;                       //Pull-down emitter directly
      R_PORT = Probe2_Rl | Probe3_Rh;             //Pull-up base via Rh
      R_DDR = Probe2_Rl | Probe3_Rh;              //Enable probe resistors
      U_R_b = UREF_VCC - ReadU_5ms(Probe2_Pin);   //U_R_c = Vcc - U_c        
    }
    else                                          //PNP
    { 
      //We assume: probe-1 = C / probe-2 = E / probe-3 = B 
      R_PORT = 0;
      R_DDR = Probe1_Rl | Probe3_Rh;              //Pull down base via Rh 
      ADC_DDR = Probe2_ADC;
      ADC_PORT = Probe2_ADC;                      //Pull-up emitter directly 
      U_R_b = ReadU_5ms(Probe1_Pin);              //U_R_c = U_c 
    }
    //If not reversed, BJT is identified
    //U_R_b *= 10;                                //Be much larger 
    if (U_R_c > U_R_b)                            //I_c > I_c_reversed 
    {
      //Move other stuff here: save data & Comp
      CompDone = 1;
    }
#endif
  }
  else if ((U_Rl < 97) && (U_R_c > FET_Level))    //no BJT
  {
    //We got a FET.
    CompFound = COMP_FET;
    CompType = FET_Type | TYPE_ENHANCEMENT | TYPE_MOSFET;
    CompDone = 1;
    //Measure gate threshold voltage
    GetGateThreshold(FET_Type);
    //Save data
    FET.G = Probe3_Pin;
    FET.D = Probe2_Pin;
    FET.S = Probe1_Pin;
  }
}

//Probe connected component and try to identify it
void CheckProbes(byte Probe1, byte Probe2, byte Probe3)
{
  byte              Flag;                         //Temporary value
  unsigned int      U_Rl;                         //Voltage across Rl (load)
  unsigned int      U_1;                          //Voltage #1
  //Init
  if (CompFound == COMP_ERROR) return;            //Skip check on any error
  wdt_reset();                                    //Reset watchdog 
  UpdateProbes(Probe1, Probe2, Probe3);           //Update bitmasks 
  //Check for depletion mode FET and get U_Rl
  U_Rl = CheckDepModeFET();
  /*
     If there's nearly no conduction (just a small leakage current) between
   probe-1 and probe-2 we might have a semiconductor:
   - BJT
   - enhancement mode FET
   - Thyristor or Triac
   or a large resistor
   */
  if (U_Rl < 977)                                 //Load current < 1.4mA
  {
    /*
       check for:
     - PNP BJT (common emitter circuit)
     - p-channel MOSFET (low side switching circuit)
     */
    if (CompDone == 0)                           //Not sure yet
    {
      //We assume: probe-1 = E / probe-2 = C / probe-3 = B */
      R_DDR = Probe2_Rl;                         //Enable Rl for probe-2 
      R_PORT = 0;                                //Pull down collector via Rl 
      ADC_DDR = Probe1_ADC;                      //Set probe 1 to output
      ADC_PORT = Probe1_ADC;                     //Pull up emitter directly 
      delay(5);
      R_DDR = Probe2_Rl | Probe3_Rl;             //Pull down base via Rl 
      U_1 = ReadU_5ms(Probe2);                   //Get voltage at collector  
      //If DUT is conducting we might have a PNP BJT or p-channel FET.
      if (U_1 > 3422)                            //Detected current > 4.8mA
      {
        //Distinguish PNP BJT from p-channel MOSFET 
        CheckBJTorDepMOSFET(TYPE_PNP, U_Rl);
      }
    }
    /*
       check for
     - NPN BJT (common emitter circuit)
     - Thyristor and Triac
     - n-channel MOSFET (high side switching circuit)
     */
    if (CompDone == 0)                           //Not sure yet
    {
      //We assume: probe-1 = C / probe-2 = E / probe-3 = B 
      ADC_DDR = Probe2_ADC;                      //Set probe-2 to output mode 
      SetADCLow();                               //Set ADC port low
      R_DDR = Probe1_Rl | Probe3_Rl;             //Select Rl for probe-1 & Rl for probe-3 
      R_PORT = Probe1_Rl | Probe3_Rl;            //Pull up collector & base via Rl 
      U_1 = ReadU_5ms(Probe1);                   //Get voltage at collector 
      //If DUT is conducting we might have a NPN BJT, something similar or a n-channel MOSFET.
      if (U_1 < 1600)                            //Detected current > 4.8mA
      {
        //First check for thyristor and triac
        Flag = CheckThyristorTriac();
        if (Flag == 0)                           //No thyristor or triac
        {
          //It seems that we got a NPN BJT or a n-channel MOSFET.
          CheckBJTorDepMOSFET(TYPE_NPN, U_Rl);
        }
      }
    }
  }
  /*
     If there's conduction between probe-1 and probe-2 we might have a
   - diode (conducting)
   - small resistor (checked later on)
   */
  else                                           //Load current > 1.4mA
  {
    //We check for a diode even if we already found a component to get Vf. There might be a body/protection diode of a semiconductor
    CheckDiode();
  }
  //Check for a resistor.
  if ((CompFound == COMP_NONE) ||
    (CompFound == COMP_RESISTOR))
  {
    CheckResistor();
  }
  //Clean up
  SetADCHiZ();                                   //Set ADC port to HiZ mode
  SetADCLow();                                   //Set ADC port low
  R_DDR = 0;                                     //Set resistor port to HiZ mode */
  R_PORT = 0;                                    //Set resistor port low */
}

//Set default values
void LoadAdjust(void)
{
  Config.RiL = R_MCU_LOW;
  Config.RiH = R_MCU_HIGH;
  Config.RZero = R_ZERO;
  Config.CapZero = C_ZERO;
  Config.RefOffset = UREF_OFFSET;
  Config.CompOffset = COMPARATOR_OFFSET;
  DIDR0 = 0b00111111;			         //DIDR0 – Digital Input Disable Register 0 - Disable digital input on analog pins
}

//Display signed value and unit
void DisplaySignedValue(signed long Value, signed char Exponent, unsigned char Unit)
{
  //Take care about sign
  if (Value < 0)                                 //Negative value
  {
  #ifdef LCD_PRINT
      lcd.write('-');
  #endif
  #ifdef DEBUG_PRINT
      Serial.write('-');                         //Display: "-"
  #endif
  Value = -Value;                                //Make value positive
  }
  //and display unsigned value
  DisplayValue((signed long)Value, Exponent, Unit);
}

//Display Value
void DisplayValue(unsigned long Value, signed char Exponent, unsigned char Unit)
{
  unsigned char     Prefix = 0;                  //Prefix character 
  byte              Offset = 0;                  //Exponent offset to next 10^3 step 
  byte              Index;                       //index ID
  byte              Length;                      //String length
  //Scale value down to 4 digits
  while (Value >= 10000)
  {
    Value += 5;                                  //For automagic rounding
    Value = Value / 10;                          //scale down by 10^1
    Exponent++;                                  //increase exponent by 1
  } 
  //Determine prefix and offset (= number of digits right of dot)
  if (Exponent >= -12)                           //Prevent index underflow 
  {
    Exponent += 12;                              //Shift exponent to be >= 0 
    Index = Exponent / 3;                        //Number of 10^3 steps 
    Offset = Exponent % 3;                       //Offset to lower 10^3 step 

    if (Offset > 0)                              //Dot required 
    {
      Index++;                                   //Upscale prefix 
      Offset = 3 - Offset;                       //Reverse value (1 or 2) 
    }    
    //Look up prefix in table (also prevent array overflow)
    if (Index <= 6) Prefix = *(&Prefix_table[Index]);
  }
  //Convert value into string
  utoa((unsigned int)Value, OutBuffer, 10);
  Length = strlen(OutBuffer);
  //We misuse Exponent for the dot position
  Exponent = Length - Offset;                    //Calculate position 
  if (Exponent <= 0)                             //We have to prepend "0."
  {
    //0: factor 10 / -1: factor 100
    #ifdef DEBUG_PRINT 
      Serial.write('0');
      Serial.write('.');
      if (Exponent < 0) Serial.write('0');       //Extra 0 for factor 100
    #endif
  #ifdef LCD_PRINT
    lcd.write('0');
    lcd.write('.');
    if (Exponent < 0) lcd.write('0');            //Extra 0 for factor 100    
  #endif
  }
  if (Offset == 0) Exponent = -1;                //Disable dot if not needed 
  //Adjust position to match array or disable dot if set to 0 
  Exponent--;
  //Display value and add dot if requested
  Index = 0;
  while (Index < Length)                         //Loop through string
  {
    #ifdef DEBUG_PRINT
      Serial.write(OutBuffer[Index]);            //Display char
      if (Index == Exponent) Serial.write('.');  //Display dot
    #endif
    #ifdef LCD_PRINT
      lcd.write(OutBuffer[Index]);               //Display char
      if (Index == Exponent) lcd.write('.');     //Display dot
    #endif
    Index++;                                     //Next one 
  }
  //Display prefix and unit
  #ifdef DEBUG_PRINT
    if (Prefix) Serial.write(Prefix);
    if (Unit) Serial.write(Unit);
  #endif
  #ifdef LCD_PRINT
    if (Prefix) lcd.write(Prefix);
    if (Unit) lcd.write(Unit);  
  #endif
}

//LCD Send
void lcd_send(unsigned char Byte)
{
  #ifdef LCD_PRINT
    lcd.write((char)Byte);
  #endif 
  #ifdef DEBUG_PRINT
    Serial.write(Byte);
  #endif
}

//LCD Send Data
void lcd_data(unsigned char Data)
{
  lcd_send(Data);
}

//LCD Clear
void lcd_clear(void)
{
  #ifdef LCD_PRINT
    lcd.clear();
  #endif
  #ifdef DEBUG_PRINT
    Serial.println("");
  #endif
}

//LCD Select Line
void lcd_line(char Line)
{
  #ifdef LCD_PRINT
    lcd.setCursor(0, Line-1);
  #endif
  #ifdef DEBUG_PRINT
    Serial.println("");
  #endif
}

//LCD Clear Line
void lcd_clear_line(unsigned char Line)
{
  unsigned char     Pos;
  lcd_line(Line);                                //Go to beginning of line
  for (Pos = 0; Pos < 16; Pos++)                 //For 16 times
  {
    lcd_data(' ');                               //Send space
  }
  lcd_line(Line);                                //Go back to beginning of line
}

//LCD TestPin
void lcd_testpin(char Probe)
{
  //Since TP1 is 0 we simply add the value to '1'
  #ifdef LCD_PRINT
    lcd.write('1' + Probe);                      //Send data
  #endif
  #ifdef DEBUG_PRINT
    Serial.write('1' + Probe);                   //Send data
  #endif
}
//LCD Space
void lcd_space(void)
{
  #ifdef LCD_PRINT
    lcd.print(" ");
  #endif
  #ifdef DEBUG_PRINT
    Serial.print(" ");
  #endif
}

//LCD Fix String
void lcd_fix_string(const char *String)
{
  unsigned char     Char;
  while (1)
  {
    Char = *(String);                            //Read character
    //Check for end of string
    if ((Char == 0) || (Char == 128)) break;
    #ifdef LCD_PRINT
      lcd.write((char)Char);
    #endif
    #ifdef DEBUG_PRINT
      Serial.write(Char);                        //Send character
    #endif
    String++;                                    //Next one
  }
}

//LCD Print with Flash Var
void lcd_print( __FlashStringHelper* data)
{
  #ifdef LCD_PRINT
    lcd.print(data);
  #endif
  #ifdef DEBUG_PRINT
    Serial.print(data);
  #endif
}


////////////////////
//WORK IN PROGRESS//
////////////////////

