/*-----------------------------------------------------------------

The BierBot mini Firmware liscense v.1.0

BierBot mini Firmware v.1.1.
(c) 2014-2016 Bernhard Schlegel, omni Technologie UG & Co. KG
(called omni in the following)

Redistribution and use in source and binary forms, with or without
modification, is permitted free of charge provided that the
following conditions are met if not otherwise specified:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

3. The name "BierBot" or "BierBot mini" must not be used to endorse
or promote products derived from this software without prior written
permission. For written permission, please contact hello@bierbot.com.

4. Products derived from this software may not be called "BierBot"
or "BierBot mini", nor may "BierBot" or "BierBot mini" appear in
their name, without prior written permission from hello@bierbot.com.

5. omni may publish revised and/or new versions of the license from
time to time. Each version will be given a distinguishing version
number.
Once covered code has been published under a particular version
of the license, you may always continue to use it under the terms
of that version. You may also choose to use such covered code
under the terms of any subsequent version of the license
published by omni. No one other than omni has
the right to modify the terms applicable to covered code created
under this License.

7. Redistributions of any form whatsoever must retain the BierBot
mini splash screen. The splash screen includes the black mashing pan
icon with the bolt in the middle, the text saying "BierBot mini" as
well as the current software version.

6. Redistributions of any form whatsoever must retain the following
acknowledgment as well as the BierBot mini splash screen:
"This product is powered by BierBot mini, freely available from <https://bierbot.de/>".


THIS SOFTWARE IS PROVIDED BY THE omni DEVELOPMENT TEAM ``AS IS'' AND
ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE omni
DEVELOPMENT TEAM OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.


This software consists of voluntary contributions made by many
individuals on behalf of omni.

omni can be contacted via Email at hello@bierbot.com.

For more information on tje BierBot please see <https://bierbot.de>.
*/

// EEPROM adresses for storing settings persistently
#define EEPROM_INITIALIZED				       	0
#define EEPROM_KDHEAT					          	1
#define EEPROM_KDCOOL					            2
#define EEPROM_HYSTERESE				        	3
#define EEPROM_BOILING_THRESHOLD			    22
#define EEPROM_DELAYCOOL					        82
#define EEPROM_DELAYHEAT					        86

 // defines for all supported modes, a mode defines whats being displayed
#define MODE_MAIN							            0		  // display main menu
#define MODE_COOL							            1		  // display cool mode
#define MODE_NACHGUSS						          2		  // displa sparging
#define MODE_ENTER_REST_COUNT				      19    // enter number of rests
#define MODE_ENTER_EINMAISCHTEMP			    20		// temperature for mixing grain with water
#define MODE_ENTER_RAST_TEMP				      21		// enter temperature of rest
#define MODE_ENTER_RAST_TIME				      22		// enter time of rest
#define MODE_SOUND							          24		// Beep On/Off
#define MODE_ENTERABMAISCH				      	25		// Lautering temperature
#define MODE_STARMASH						          26		// Start mash
#define MODE_AUTO							            27		// Start recipe in automatic mode
#define MODE_AUTO_TEMP						        28		// temperature reached?
#define MODE_AUTO_TIME						        29		// after temperature is reached: display remaining time and hold temp 
#define MODE_AUTO_FINISH					        30		// hold temperature until programm is ended by user
#define MODE_ALARM							          31		// make noise and display alarm text after finished
#define MODE_CALL							            32		// make noise and ask for continue
#define MODE_NUMBEROFHOPS					        38		// enter number of hops
#define MODE_HOP						            	39		// set time of hop
#define MODE_BOILING_TIME				        	40		// set boiling time
#define MODE_BOILING_INIT					        41		// boiling start screen
#define MODE_BOILING_STARTED				      42		// main display for cooking
#define MODE_STIRRINTERVALL					      230		// not used
#define MODE_STIRRINTERVALL_ON		    		231		// not used
#define MODE_STIRRINTERVALL_OFF   				232		// not used
#define MODE_SETTINGS                     235		// display settings
#define MODE_SETTINGS_WAIT_COOL				    237		// display settings for turn on delay cooling
#define MODE_SETTINGS_WAIT_HEAT				    238		// display settings for turn on delay heating
#define MODE_SETTINGS_BOILING_TRHESHOLD		239		// display settings for boiling threshold
#define MODE_SETTINGS_HYSTERESIS			    240		// display settings for hysteresis
#define MODE_SETTINGS_KD_HEAT				      241		// display settings for kd heating
#define MODE_SETTINGS_KD_COOL				      242		// display settings for kd cool
#define MODE_ABORT							          255		// call jimmy hendrix

// general settings, e.g. limits
#define SETTINGS_MS_LAST_TURN_MS			    250		// if last turn was less than SETTINGS_MS_LAST_TURN_MS ago, turn will be considered "fast"
#define SETTINGS_TURN_FAST_INCREMENT      5		  // what's the increment when turning "fast"
#define SETTINGS_TEMP_MIN					        1		  // when setting temperatures - what is the minimum temperature that can be set
#define SETTINGS_TEMP_MAX					        105		// when setting temperatures - what is the maximum temperature that can be set
#define SETTINGS_TIME_MIN					        1		  // what is the minimum time that can be set for rests
#define SETTINGS_TIME_MAX					        99		// what is the maximum time that can be set for rests
#define SETTINGS_LIM_DELAY_COOL_MIN		  	0		  // what is the minimum number for setting cooling delay
#define SETTINGS_LIM_DELAY_COOL_MAX			  30		// what is the maximum number for setting cooling delay
#define SETTINGS_LIM_DELAY_HEAT_MIN			  0		  // what is the minimum number for setting heating delay
#define SETTINGS_LIM_DELAY_HEAT_MAX			  120		// what is the maximum number for setting heating delay
#define SETTINGS_LIM_BOILING_MIN			    20		// what is the minimum boiling temperature
#define SETTINGS_LIM_BOILING_MAX			    99		// what is the maximum boiling temperature
#define SETTINGS_LIM_KD_HEAT_MIN			    5		  // min value for kd heat (factor 10)
#define SETTINGS_LIM_KD_HEAT_MAX			    24		// max value for kd heat (factor 10)
#define SETTINGS_LIM_KSCHWELLE_MIN			  85		// min value for boiling threshold
#define SETTINGS_LIM_KSCHWELLE_MAX			  110		// max value for boiling threshold
#define SETTINGS_LIM_KD_COOL_MIN			    5		  // min value for kd cool (factor 10)
#define SETTINGS_LIM_KD_COOL_MAX			    24		// max value for kd cool (factor 10)
#define SETTINGS_LIM_HYSTERESIS_MIN			  5		  // min number for hystersis
#define SETTINGS_LIM_HYSTERESIS_MAX			  40		// max number for hystersis
#define SETTINGS_TEMPHIST_NSAMPLE			    12		// number of samples
#define SETTINGS_TEMPHIST_SAMPLEDIST_S		5		  // Zeitdifferent in sekunden zwischen zwei temperaturhistory samples
#define SETTINGS_DISP_WHAT_X				      0		  // Displaying what happens, e.g. Mashing. Kind of the headline. X coord
#define SETTINGS_DISP_WHAT_Y				      0		  // Displaying what happens, e.g. Mashing. Kind of the headline. Y coord
#define SETTINGS_DISP_TIME_REMAIN_X			  0 	  // Where to plot remaining time x
#define SETTINGS_DISP_TIME_REMAIN_Y			  2		  // Where to plot remaining time y
#define SETTINGS_DISP_TIME_TARGET_X			  0		  // Where to plot remaining time difference x
#define SETTINGS_DISP_TIME_TARGET_Y			  1		  // Where to plot remaining time difference y
#define SETTINGS_DISP_DT_X					      10		// Where to plot current temperature changing rate x
#define SETTINGS_DISP_DT_Y					      1		  // Where to plot current temperature changing rate y
#define SETTINGS_DISP_TGT_X					      9		  // Where to plot target temperature x
#define SETTINGS_DISP_TGT_Y					      2		  // Where to plot target temperature y
#define SETTINGS_DISP_IST_X					      9		  // Where to plot current temperature x
#define SETTINGS_DISP_IST_Y					      3		  // Where to plot current temperature y
#define SETTINGS_DISP_RUF_X					      0		  // Where to plot the alert that something needs attention x
#define SETTINGS_DISP_RUF_Y					      1		  // Where to plot the alert that something needs attention y
#define SETTINGS_DISP_CONTINUE_X			    0	 	  // Where to plot confirmation message to continue x
#define SETTINGS_DISP_CONTINUE_Y			    2		  // Where to plot confirmation message to continue y
#define SETTINGS_RELAY_OFF					      LOW		// use this (1) to invert relais logic (depending on your PCB).
#define SETTINGS_RELAY_ON					        HIGH	// use this (2) to invert relais logic (depending on your PCB).
                                                // if you want to invert it, set SETTINGS_RELAY_OFF to HIGH and
                                                // SETTINGS_RELAY_ON to LOW respectively

// GPIO Pin Settings
#define SETTINGS_PIN_RELAIS					      6     // GPIO for heating/cooling relais
#define SETTINGS_PIN_SOUND					      7		  // GPIO for accustic signal
#define SETTINGS_PIN_ROTARY_A				      2		  // GPIO for rotary encoder ISR A
#define SETTINGS_PIN_ROTARY_B			      	3		  // GPIO for rotary encoder ISR B
#define SETTINGS_PIN_BUTTON					      4		  // GPIO for Button
#define SETTINGS_OIN_ONE_WIRE_BUS_TEMP		5		  // Data wire is plugged into port 5 on the Arduino

// LCD Init 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Encoder Init

volatile byte number = 0;							          // raw rotary values
volatile bool halfleft = false;					        // Used in both interrupt routines
volatile bool halfright = false;					      // Used in both interrupt routines
byte oldnumber = 0;									            // old rotary state, for determinining new "number" values
byte buttonState = 0;								            // raw button state after GPIO read
int buttonPressed = 0;								          // is used in programm logic, validated button state
byte turn = 0;									                // number is used in programm logic, validated rotary value	
byte oldTurn;										                // old value of turn. Used to detect a changed "turn", e.g. to update the display only when necessary
byte turnFast = 0;									            // setting how much the turn is incremented when turning fast
unsigned long lastTurn = 0;							        // last turn of rotary
bool lastTurnWasUp = true;							        // for determining changing directions

// Tempsens init 
#include <OneWire.h>
#include <DallasTemperature.h>

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(SETTINGS_OIN_ONE_WIRE_BUS_TEMP);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

// Time measurement
#include <Time.h>

// EEPROM 
#include <EEPROM.h>

// Pseudo-PDControl
static float kpHeat = 1.0;
static float kpCool = 1.0;

// Setting up of global variables
byte initDisplay = 0;								            // initializing the display after screen is changed
unsigned long altsekunden;							        // for remembering ms for blinking texts on the display
byte controlActivated = 0;							        // indicates, if the automatic pd-control is activated or not				
byte relaisState = 0;								            // indicating current relais state
byte relaisRequest = 0;								          // target relais state of the PD-control, without paying attention to the turning on delay
byte sensorError = 0;								            // indicating a sensor error (e.g. when a value like 0, 85 or -128 is received after startup)
float kdHeat;										                // the differential factors for the pseudo pd-control when mashing
float kdCool;										                // the differential factors for the pseudo pd-control when using the cool mode for controlling a fridge
float hystereses;									              // determines the band, where no relais state change is done and as well when a temperature is being regarded as "reached"
byte kdHeatEEPROM;									            // holds the value from 0-255, needs to be diveded by ten, valid values range from 5 = (0.5) to 30 (3.0)
byte kdCoolEEPROM;									            // holds the value from 0-255, needs to be diveded by ten, valid values range from 5 = (0.5) to 30 (3.0)
byte hystereseEEPROM;								            // holds the value from 0-255, needs to be diveded by ten
byte delayHeatEEPROM;								            // holding the EEPROM value
byte delayCoolEEPROM;								            // holding the EEPROM value
bool tempReached = false;							          // indicating if the targettemperature is reached - only relevant for stirring
unsigned long lastRelaisChangeTS = 0;				    // timestamp for last relais change (used for turn on/off delays)
float sensorvalue;									            // holding the sensor value
float currentTemp = 20;								          // current temperature, initialized with 20 that the sensor error check does not trigger
bool currentTempValid = false;						      // indicating if the currentTemp value is holding a sensed value or the 20 from the initialization
int mode = MODE_MAIN;								            // defining the mode and therefore the displayed menu
int rufmodus = MODE_MAIN;							          // defining the demanded mode (e.g. where is the pointer, which mode is called next upon button press)
unsigned long lastSignalTS = 0;						      // holding the timestamp of the last signal
bool spargingSignal = false;						        // Signal on spargingtemp (Nachgusstemp) reached
byte x = 1;                                     // current rest number
byte h = 0;                                     // current hop
byte y = 1;                                     // passed value for brewmeistercall
byte n = 0;                                     // Counter for avoiding temperature errors
byte pause = 0;                                 // Counter f�r signalcount
unsigned long startButtonPressTS;					      // timestamp for detecting a long button press
bool buttonPress = false;							          // Detecting a release

byte seconds = 0;									              // counting time in auto mode
byte minutes = 0;                               // counting time in auto mode
byte minutesValue = 0;								          // counting time in auto mode
byte hours = 0;										              // counting time in auto mode

// default settings after start, these are overwritten by the user during runtime when entering the programme
float targetTemp = 20;								          // targettemp for display
float mashTemp = 45;                            // preset mashtemp
byte numberOfRests = 3;								          // present number of rests
byte restTempsArray[] = {							          // preset rest temperatures
  0, 55, 64, 72, 72, 72
};
byte restTimesArray[] = {							          // preset rest times
  0, 15, 40, 25, 20, 20
};
byte brewmeisterArray[] = {							        // preset braumeister calls
  0, 0, 0, 0, 0, 0
};
byte endtemp = 78;                              // preset end temperature
byte boilingTime = 90;								          // preset boiling time
byte numberOfHops = 3;								          // Number of times hop is being added during boiling
byte boilingTempTarget = 97;						        // preset, when is the mash regarded as "boiling"

// hopa
byte const maxNumberOfHops = 6;						      // can be changed, also check hopsArray
byte hopsArray[maxNumberOfHops] = { 0, 0, 0, 0, 0, 0 }; // times for hops

// boiling
bool boilingTempReached = false;

// delays for heating and cooling
unsigned long delayCoolMS = 15 * 60 * 1000;			// delay in ms for easier comparision #TODO if RAM is full
unsigned long delayHeatMS = 10000;					    // delay in ms for easier comparision #TODO if RAM is full

// temperature gradient / changing rate
unsigned long temphistLastSampleMillis = millis();	// last temperature history
float tempHistory[SETTINGS_TEMPHIST_NSAMPLE] = { 20,20,20,20,20,20,20,20,20,20,20,20 }; // holding last values, distance of SETTINGS_TEMPHIST_SAMPLEDIST_S
byte tempHistoryNextWriteIndex = 0;					    // next write index in temphistory fifo buffer
bool tempHistReady = false;							        // is the temperature history array complete filled (and the value plausible)
const unsigned long temphistSampleDistance = SETTINGS_TEMPHIST_SAMPLEDIST_S * 1000; // sampledistance between two samples
float gradient = 0.0;								            // current temperature gradient

// logview / COM transfer
bool logviewTransferEnabled = false;				    // transfer data to the logview using the COM port, atm. not supported in factory
                                                // version of the BierBot mini. The connectors are on the board though (-> custom mod ;))

/// <summary>
/// Adds a temperature to the temperature history. Temperature history is used
/// for precise calulcation of the current temperature changing rate (gradient)
/// </summary>
/// <param name="temp">the temperature, that is going to be added to history</param>
void temphist_Add(float temp) {
  if (!currentTempValid) {
    return;
  }

  temphistLastSampleMillis = millis();
  tempHistory[tempHistoryNextWriteIndex] = temp;
  tempHistoryNextWriteIndex++;

  if (tempHistoryNextWriteIndex >= SETTINGS_TEMPHIST_NSAMPLE) {
    tempHistoryNextWriteIndex = 0;
    tempHistReady = true;	// temperature history is only ready, after buffer was filled up once
  }
}

/// <summary>
/// Gets the last temperature added to the history from the history. 
/// </summary>
/// <returns>the last temperature</returns>
float temphist_GetNewest() {
  byte newestIndex;
  if (tempHistoryNextWriteIndex == 0) {
    newestIndex = SETTINGS_TEMPHIST_NSAMPLE - 1;
  }
  else {
    newestIndex = tempHistoryNextWriteIndex - 1;
  }

  return tempHistory[newestIndex];
}

/// <summary>
/// Gets the oldest temperature from the history. In other words: 
/// the temperature thats in there for the longest time
/// </summary>
/// <returns>the oldest temprature</returns>
float temphist_GetOldest() {
  if (tempHistReady) {
    // return element that is overwritten next
    return tempHistory[tempHistoryNextWriteIndex];
  }
  else {
    return tempHistory[0];
  }
}

/// <summary>
/// As you might have guessed: a short beep.
/// </summary>
void beep_Short() {

  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(100);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);

}

/// <summary>
/// As you might have guessed: a long beep.
/// </summary>
void beep_LongShort() {

  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(200);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);
  delay(50);
  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(100);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);

}

/// <summary>
/// A beautiful combination of a short and a long beep
/// </summary>
void beep_ShortLong() {

  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(100);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);
  delay(50);
  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(200);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);

}

/// <summary>
/// A extra long beep
/// </summary>
void beep_ShortXLong() {

  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(100);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);
  delay(50);
  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(600);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);

}

// marker characters
byte markerChar[8] = {0b00000, 0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b01110, 0b00000};
int markerCharNum = 0;
byte markerCharBorder[8] = { 0b00000, 0b00000, 0b00100, 0b11111, 0b00100, 0b00000, 0b00000, 0b00000};
int markerCharBorderNum = 1;

/// <summary>
/// setup routine is called once at startup. Used to setup the BierBot. 
/// </summary>
void setup() {
  Serial.begin(9600);							// Init the COM-Port at 9600 Baud

  turn = targetTemp;

  // init the GPIOs, set direction first
  pinMode(SETTINGS_PIN_RELAIS, OUTPUT);		    // init relais
  digitalWrite(SETTINGS_PIN_RELAIS, SETTINGS_RELAY_OFF);   // and turn off
  pinMode(SETTINGS_PIN_SOUND, OUTPUT);		    // init the sound pin
  pinMode(SETTINGS_PIN_BUTTON, INPUT);        // init buttton pin
  digitalWrite(SETTINGS_PIN_BUTTON, HIGH);	  // Turn on internal pullup resistor

  pinMode(SETTINGS_PIN_ROTARY_A, INPUT);		  // pin for rotary encoder channel a
  digitalWrite(SETTINGS_PIN_ROTARY_A, HIGH);	// Turn on internal pullup resistor
  pinMode(SETTINGS_PIN_ROTARY_B, INPUT);		  // pin for rotary encoder channel b
  digitalWrite(SETTINGS_PIN_ROTARY_B, HIGH);	// Turn on internal pullup resistor
  attachInterrupt(0, isr_2, CHANGE);			    // Call isr_2 when digital pin 2 goes LOW

  // init LCD
  lcd.begin(20, 4);							              // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.backlight();							              // backlight on

  // print info
  lcd.setCursor(15, 0); 
  lcd.print("V1.1b");
  lcd.setCursor(4, 2);
  lcd.print("BierBot mini");

  // The BierBot mini logo :)
  int pos = 8;
  int charNum = 0;
  byte customChar0[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00011, 0b00111};
  lcd.createChar(charNum, customChar0);
  lcd.setCursor(pos, 0);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 1
  byte customChar1[8] = { 0b00011, 0b00011, 0b00011, 0b00011, 0b01111, 0b11111, 0b11110, 0b11110};
  lcd.createChar(charNum, customChar1);
  lcd.setCursor(pos, 0);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 2
  byte customChar2[8] = { 0b11000, 0b11000, 0b11000, 0b11000, 0b11110, 0b11111, 0b11111, 0b11111};
  lcd.createChar(charNum, customChar2);
  lcd.setCursor(pos, 0);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 3
  byte customChar3[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11000, 0b11100};
  lcd.createChar(charNum, customChar3);
  lcd.setCursor(pos, 0);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 4
  pos = 8;
  byte customChar4[8] = { 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b00111};
  lcd.createChar(charNum, customChar4);
  lcd.setCursor(pos, 1);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 5
  byte customChar5[8] = { 0b11100, 0b11000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
  lcd.createChar(charNum, customChar5);
  lcd.setCursor(pos, 1);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 6
  byte customChar6[8] = { 0b11111, 0b00011, 0b00111, 0b01111, 0b01111, 0b01111, 0b11111, 0b11111};
  lcd.createChar(charNum, customChar6);
  lcd.setCursor(pos, 1);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;
  // 7
  byte customChar7[8] = { 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11110, 0b11100};
  lcd.createChar(charNum, customChar7);
  lcd.setCursor(pos, 1);
  lcd.write((uint8_t)charNum);
  pos++;
  charNum++;

  // show logo for three seconds
  delay(3000);

  // make some noise
  beep_LongShort();

  x = 1;

  // clear LCD again
  lcd.clear();

  // create markerChar for later use
  lcd.createChar(markerCharNum, markerChar);
  lcd.write((uint8_t)markerCharNum);
  lcd.createChar(markerCharBorderNum, markerCharBorder);
  lcd.write((uint8_t)markerCharBorderNum);

  // init Tempsensors
  sensors.getAddress(insideThermometer, 0);
  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);


  // Get settings from EEPROM
  kdHeatEEPROM = EEPROM.read(EEPROM_KDHEAT);
  kdHeat = 0.1 * float(kdHeatEEPROM);	// for C beginners: starting with 0.1 will cast everything that follows to float
  kdCoolEEPROM = EEPROM.read(EEPROM_KDCOOL);
  kdCool = 0.1 * float(kdCoolEEPROM); 
  hystereseEEPROM = EEPROM.read(EEPROM_HYSTERESE);
  hystereses = 0.1 * float(hystereseEEPROM);
  delayHeatEEPROM = EEPROM.read(EEPROM_DELAYHEAT);
  delayHeatMS = 1000 * (unsigned long)delayHeatEEPROM;
  delayCoolEEPROM = EEPROM.read(EEPROM_DELAYCOOL);
  delayCoolMS = 60000 * (unsigned long)delayCoolEEPROM;
  boilingTempTarget = EEPROM.read(EEPROM_BOILING_THRESHOLD);

  // detect frist start, and init settings if it is the first start
  byte eepromInitialized = EEPROM.read(EEPROM_INITIALIZED);
  if (eepromInitialized != 117)                 // was john already here?
  {
    // beep
    beep_Short();

    // init kdHeat
    kdHeatEEPROM = 13;
    EEPROM.write(EEPROM_KDHEAT, kdHeatEEPROM);
    kdHeat = 0.1 * float(kdHeatEEPROM);
    lcd.setCursor(0, 0);
    lcd.print("kdHeat = 1,3");

    // init kdCool
    kdCoolEEPROM = 10;
    EEPROM.write(EEPROM_KDCOOL, kdCoolEEPROM);
    kdCool = 0.1 * float(kdCoolEEPROM);
    lcd.setCursor(0, 1);
    lcd.print("kdCool = 1,0");

    // init hysterese
    hystereseEEPROM = 5;
    EEPROM.write(EEPROM_HYSTERESE, hystereseEEPROM);
    hystereses = 0.1 * float(hystereseEEPROM);
    lcd.setCursor(0, 2);
    lcd.print("hyst = 0,5K");

    // init heat delay
    delayHeatEEPROM = 10;
    EEPROM.write(EEPROM_DELAYHEAT, delayHeatEEPROM);
    delayHeatMS = (unsigned long)delayHeatEEPROM * 1000;

    // init cool delay
    delayCoolEEPROM = 15;
    EEPROM.write(EEPROM_DELAYCOOL, delayCoolEEPROM);
    delayCoolMS = 60000 * (unsigned long)delayCoolEEPROM;
    lcd.setCursor(0, 3);
    lcd.print("10s(heat), 15m(cool)");

    // init kschwelle
    boilingTempTarget = 99;
    EEPROM.write(EEPROM_BOILING_THRESHOLD, boilingTempTarget);
    lcd.setCursor(13, 0);
    lcd.print("koch:99");

    // store for next start that values have been initialized    
    EEPROM.write(EEPROM_INITIALIZED, 117);    // john was here!

    // display and delay
    delay(2000);
    lcd.clear();
  }

  // Add temperature to history
  temphist_Add(currentTemp);
}


/// <summary>
/// This method is executed endlessly
/// </summary>
void loop() {
  if (logviewTransferEnabled) print_logview();

  // Getting the time
  seconds = second();			                    // save current second
  minutesValue = minute();	                  // save current minute
  hours = hour();				                      // save current hour

  // Temperaturesensor DS1820
  // call sensors.requestTemperatures() to issue a global temperature
  sensors.requestTemperatures();						  // Send the command to get temperatures
  sensorvalue = sensors.getTempC(insideThermometer);
  if ((sensorvalue != currentTemp) && (n < 10)) {	// avoid measurement errors of value after
    n++;											                // multiple measurements
  }
  else {
    currentTemp = sensorvalue;
    n = 0;
    currentTempValid = true;
  }

  // common sensor errors:
  // -127	: VCC missing / no supply voltage
  // 85	: internal sensor error, maybe the cable is too long
  // 0	: Dataline or GND missing
  if (controlActivated == 1) // control only activated in modes with heating
  {
    if ((int)currentTemp == -127 || (int)currentTemp == 0)
      if (sensorError == 0)
      {
        rufmodus = mode;
        lcd.clear();
        lcd.setCursor(8, 0);
        lcd.print("Sensorfehler");
        controlActivated = 0;
        relaisState = 0;

        sensorError = 1;
        mode = MODE_ALARM;
      }
  }

  // detect encoder turning
  if (number != oldnumber)
  {
    {
      if (number > oldnumber)   // to adjust the direction of the encoder switch < or >
      {
        if (lastTurnWasUp && ((lastTurn + SETTINGS_MS_LAST_TURN_MS) > millis()) && mode != MODE_MAIN && mode != MODE_SETTINGS) {
          // same direction && within timeframe
          turnFast = SETTINGS_TURN_FAST_INCREMENT;

        }
        else {
          turnFast = 0;
        }

        turn = turn + 1 + turnFast;
        lastTurnWasUp = true;
        lastTurn = millis();
      }
      else
      {
        if (!lastTurnWasUp && ((lastTurn + SETTINGS_MS_LAST_TURN_MS) > millis()) && mode != MODE_MAIN && mode != MODE_SETTINGS) {
          // same direction && within timeframe
          turnFast = SETTINGS_TURN_FAST_INCREMENT;
        }
        else {
          turnFast = 0;
        }

        turn = turn - 1 - turnFast;
        lastTurnWasUp = false;
        lastTurn = millis();
      }
      oldnumber = number;
    }
  }


  // show current temperature
  lcd.setCursor(SETTINGS_DISP_IST_X, SETTINGS_DISP_IST_Y);
  lcd.print("ist");
  float sensorwertRounded = 0.1 * round(sensorvalue * 10);

  if (sensorwertRounded < 100) {
    lcd.print(" ");
  }
  if (sensorwertRounded < 10) {
    lcd.print(" ");
  }
  if (sensorwertRounded > 0) {
    lcd.print(" ");
  }

  lcd.print(float(sensorwertRounded), 1);
  lcd.print(char(0xDF));
  lcd.print("C");	

  // Heating control
  if (controlActivated == 1)
  {
    if ((temphistLastSampleMillis + temphistSampleDistance) < millis()) {
      temphist_Add(currentTemp);

      gradient = (temphist_GetNewest() - temphist_GetOldest()) / (SETTINGS_TEMPHIST_SAMPLEDIST_S * SETTINGS_TEMPHIST_NSAMPLE / 60);

      lcd.setCursor(SETTINGS_DISP_DT_X, SETTINGS_DISP_DT_Y);
      if (gradient >= 0.0) {
        lcd.print(" ");
      }
      if (gradient < 10.0 && gradient > -10.0) {
        lcd.print(" ");
      }
      lcd.print((float)(0.1 * round(gradient * 10)),1); lcd.print("K/min");
    }

    // pseudo PD Control, (c) by Bernhard Schlegel, 2015
    if (mode != 1) {	// heating

      if (currentTemp >= (kpHeat * targetTemp - kdHeat * gradient)) {
        relaisRequest = 0;
      }

      if (currentTemp <= (kpHeat * targetTemp - kdHeat * gradient - hystereses)) {
        relaisRequest = 1;
      }

      if (relaisRequest != relaisState) {
        // delay only when turning on
        if (relaisRequest == 1 && (millis() >= (lastRelaisChangeTS + delayHeatMS))) {
          if (relaisState != 1) {
            lastRelaisChangeTS = millis();
          }
          relaisState = 1;

          lastRelaisChangeTS = millis();
        }
        else {
          if (relaisState != 0) {
            lastRelaisChangeTS = millis();
          }
          relaisState = 0;
        }
      }
    }
    else { // cooling
      if (currentTemp <= (kpCool * targetTemp + kdCool * gradient)) {
        relaisRequest = 0;
      }
      if (currentTemp >= (kpCool * targetTemp + kdCool * gradient + hystereses)) {
        relaisRequest = 1;
      }

      if (relaisRequest != relaisState) {
        // delay only when turning on
        if (relaisRequest == 1 && (millis() >= (lastRelaisChangeTS + delayCoolMS))) {
          if (relaisState != 1) {
            lastRelaisChangeTS = millis();
          }
          relaisState = 1;
          lastRelaisChangeTS = millis();
        }
        else {
          if (relaisState != 0) {
            lastRelaisChangeTS = millis();
          }
          relaisState = 0;
        }
      }
    }
    // end pseude PD Control

    // target temperature
    lcd.setCursor(SETTINGS_DISP_TGT_X, SETTINGS_DISP_TGT_Y);
    lcd.print("soll");
    float sollwertRounded = 0.1 * round(targetTemp * 10);

    if (targetTemp < 100) {
      lcd.print(" ");
    }
    if (targetTemp < 10) {
      lcd.print(" ");
    }

    lcd.print(float(sollwertRounded), 1);
    lcd.print(char(0xDF)); lcd.print("C");

    // counts for both
    if ((currentTemp >= (targetTemp - hystereses) && currentTemp <= (targetTemp + hystereses)))
    {
      tempReached = true;
    }
    else {
      tempReached = false;
    }
  }

  // shows "Heizen" or "Kuehlen" according to current mode
  if (relaisState == 1)
  {
    lcd.setCursor(0, 3);
    if (mode != 1) {
      lcd.print("Heizen");
    }
    else {
      lcd.print("K");
      lcd.print(char(0xF5));
      lcd.print("hlen");
    }
    digitalWrite(SETTINGS_PIN_RELAIS, SETTINGS_RELAY_ON);
  }
  else
  {
    if (mode < MODE_SETTINGS) {
      lcd.setCursor(0, 3);
      lcd.print("      "); // Clear "Heizen" or "K�hlen" display
    }

    digitalWrite(SETTINGS_PIN_RELAIS, SETTINGS_RELAY_OFF);
  }

  // get button State
  getButton();  

  // get mode
  switch (mode) {
    case MODE_MAIN: {
      controlActivated = 0;
      showMain();
      break;
    }
    case MODE_COOL: {
      controlActivated = 1;
      showTemperatureControl();
      break;
    }
    case MODE_NACHGUSS: {
      controlActivated = 1;
      showTemperatureControl();
      break;
    }
    case MODE_SETTINGS: {
      controlActivated = 0;
      showSetup();
      break;
    }
    case MODE_SETTINGS_HYSTERESIS: {
      controlActivated = 0;
      rufmodus = MODE_MAIN;
      display_settings_hystereses();
      break;
    }
    case MODE_ENTER_REST_COUNT: {
      controlActivated = 0;
      showSetNumberOfRests();
      break;
    }
    case MODE_ENTER_EINMAISCHTEMP: {
      controlActivated = 0;
      showInputMashTemp();
      break;
    }
    case MODE_ENTER_RAST_TEMP: {
      controlActivated = 0;
      showRestTemp();
      break;
    }
    case MODE_ENTER_RAST_TIME: {
      controlActivated = 0;
      showRestTime();
      break;
    }
    case MODE_STIRRINTERVALL: {
      // TODO
      break;
    }
    case MODE_STIRRINTERVALL_ON: {
      // TODO
      break;
    }
    case MODE_STIRRINTERVALL_OFF: {
      // TODO
      break;
    }
    case MODE_SETTINGS_WAIT_COOL: {
      controlActivated = 0;
      display_settings_waitCool();
      break;
    }

    case MODE_SETTINGS_WAIT_HEAT: {
      controlActivated = 0;
      display_settings_waitHeat();
      break;
    }
    case MODE_SETTINGS_BOILING_TRHESHOLD: {
      controlActivated = 0;
      display_settings_boilingThreshold();
      break;
    }
    case MODE_SETTINGS_KD_HEAT: {
      controlActivated = 0;
      rufmodus = MODE_MAIN;
      display_settings_kdheat();
      break;
    }
    case MODE_SETTINGS_KD_COOL: {
      controlActivated = 0;
      rufmodus = MODE_MAIN;
      display_settings_kdcool();
      break;
    }
    case MODE_SOUND: {
      controlActivated = 0;
      funktion_braumeister();
      break;
    }
    case MODE_ENTERABMAISCH: {
      controlActivated = 0;
      display_auto_lastMashTemp();
      break;
    }
    case MODE_STARMASH: {
      controlActivated = 0;
      display_auto_start();
      break;
    }
    case MODE_AUTO: {
      controlActivated = 1;
      display_auto_mashing();
      logviewTransferEnabled = true;
      break;
    }
    case MODE_AUTO_TEMP: {
      controlActivated = 1;
      display_auto_temp();
      break;
    }
    case MODE_AUTO_TIME: {
      controlActivated = 1;
      display_auto_time();
      break;
    }
    case MODE_AUTO_FINISH: {
      controlActivated = 1;
      display_auto_tempLast();
      break;
    }
    case MODE_ALARM: {
      controlActivated = 0;
      funktion_braumeisterrufalarm();
      break;
    }
    case MODE_CALL: {
      controlActivated = 0;
      funktion_braumeisterruf();
      break;
    }
    case MODE_NUMBEROFHOPS: {
      controlActivated = 0;
      display_boil_numberOfHops();
      break;
    }
    case MODE_HOP: {
      controlActivated = 0;
      display_boil_hopTimes();
      break;
    }
    case MODE_BOILING_TIME: {
      controlActivated = 0;
      display_boil_boilingTime();
      break;
    }
    case MODE_BOILING_INIT: {
      controlActivated = 0;
      display_boiling_boilTemperature();
      break;
    }
    case MODE_BOILING_STARTED: {
      controlActivated = 0;
      display_boiling_main();
      break;
    }
    case MODE_ABORT: {
      controlActivated = 0;
      main_abort();
      break;
    }
  }
}

/// <summary>
/// ISR for detecting rotary encoder turns
/// </summary>
void isr_2() {
  bool Aread = false;
  bool Bread = false;
  delay(50); //test
  Aread = digitalRead(SETTINGS_PIN_ROTARY_A);
  Bread = digitalRead(SETTINGS_PIN_ROTARY_B);
  if ((Aread == true && Bread == false) || (Aread == false && Bread == true)) {
    number--;
  }
  if ((Aread == false && Bread == false) || (Aread == true && Bread == true)) {
    number++;
  }
}

/// <summary>
/// Getting the button state
/// </summary>
/// <returns>if buttons is pressed (0/1)</returns>
int getButton() {
  buttonState = digitalRead(SETTINGS_PIN_BUTTON);
  if (buttonState == HIGH)
  {
    buttonPressed = 0;
    startButtonPressTS = millis();
  }

  else if (buttonState == LOW)
  {
    buttonPressed = 1;

    // if button is pressed for more than 2500ms abort is called
    if (millis() >= (startButtonPressTS + 2500)) {
      mode = MODE_ABORT;
    }	                             
  }
  return buttonPressed;
}

/// <summary>
/// display main menu, mode 0
/// </summary>
void showMain() {

  if (initDisplay == 0)

  {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Maischen");
    lcd.setCursor(1, 1);
    lcd.print("Nachguss");
    lcd.setCursor(1, 2);
    lcd.print("Kochen");
    lcd.setCursor(11, 0);
    lcd.print("K"); lcd.print(char(0xF5)); lcd.print("hlen");
    lcd.setCursor(11, 1);
    lcd.print("Setup");
    turn = 0;
    oldTurn = 0;
    initDisplay = 1;
  }



  if (turn == 255)
  {
    turn = 4;
}	
  if (turn > 4)
  {
    turn = 0;
  }

  if (turn != oldTurn)
  {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(10, 0);
    lcd.print(" ");
    lcd.setCursor(10, 1);
    lcd.print(" ");
  }

  if (turn == 0)
  {
    oldTurn = turn;
    rufmodus = MODE_ENTER_REST_COUNT;
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 1)
  {
    oldTurn = turn;
    rufmodus = MODE_NACHGUSS;
    lcd.setCursor(0, 1);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 2)
  {
    oldTurn = turn;
    rufmodus = MODE_BOILING_TIME;
    lcd.setCursor(0, 2);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 3)
  {
    oldTurn = turn;
    rufmodus = MODE_COOL;
    lcd.setCursor(10, 0);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 4)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS;
    lcd.setCursor(10, 1);
    lcd.write((uint8_t)markerCharNum);
  }

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode = rufmodus;
      if (mode == MODE_COOL) {
        turn = 10;
      }
      if (mode == MODE_NACHGUSS) {
        turn = 78;
      }
      initDisplay = 0;
      lcd.clear();
    }

}

/// <summary>
/// Show setup menu
/// </summary>
void showSetup() {

  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Schwelle");
    lcd.setCursor(1, 1);
    lcd.print("Hyst.");
    lcd.setCursor(1, 2);
    lcd.print("kd-Heiz");
    lcd.setCursor(1, 3);
    lcd.print("kd-K"); lcd.print(char(0xF5)); lcd.print("hl");
    lcd.setCursor(11, 0);
    lcd.print("ESVK"); lcd.print(char(0xF5)); lcd.print("h"); lcd.print("len");
    lcd.setCursor(11, 1);
    lcd.print("ESVHeizen");
    lcd.setCursor(11, 2);
    lcd.print("Zur"); lcd.print(char(0xF5)); lcd.print("ck");
    turn = 0;
    oldTurn = 0;
    initDisplay = 1;
  }


  if (turn == 255)
  {
    turn = 6;
  }
  if (turn > 6)
  {
    turn = 0;
  }

  if (turn != oldTurn)
  {
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print(" ");
    lcd.setCursor(10, 0);
    lcd.print(" ");
    lcd.setCursor(10, 1);
    lcd.print(" ");
    lcd.setCursor(10, 2);
    lcd.print(" ");
    oldTurn = turn;
  }

  if (turn == 0)
  {
    oldTurn = 0;
    rufmodus = MODE_SETTINGS_BOILING_TRHESHOLD;
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)markerCharNum);
  }
  if (turn == 1)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS_HYSTERESIS;
    lcd.setCursor(0, 1);
    lcd.write((uint8_t)markerCharNum);
  }
  if (turn == 2)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS_KD_HEAT;
    lcd.setCursor(0, 2);
    lcd.write((uint8_t)markerCharNum);
  }
  if (turn == 3)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS_KD_COOL;
    lcd.setCursor(0, 3);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 4)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS_WAIT_COOL;
    lcd.setCursor(10, 0);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 5)
  {
    oldTurn = turn;
    rufmodus = MODE_SETTINGS_WAIT_HEAT;
    lcd.setCursor(10, 1);
    lcd.write((uint8_t)markerCharNum);
  }

  if (turn == 6)
  {
    oldTurn = turn;
    rufmodus = MODE_MAIN;
    lcd.setCursor(10, 2);
    lcd.write((uint8_t)markerCharNum);
  }

  if (buttonPressed == 0) {
    buttonPress = true;
  }

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode = rufmodus;
      initDisplay = 0;
    }

}

/// <summary>
/// Display temperature automatic, modes = 1,2
/// </summary>
void showTemperatureControl() {

  if (mode == MODE_COOL)
  {
    lcd.setCursor(0, 0);
    lcd.print("K"); lcd.print(char(0xF5)); lcd.print("hlen");
  }

  if (mode == MODE_NACHGUSS)
  {
    lcd.setCursor(0, 0);
    lcd.print("Nachguss");
  }


  // apply limits
  if (turn < SETTINGS_TEMP_MIN)
    turn = SETTINGS_TEMP_MAX;
  if (turn > SETTINGS_TEMP_MAX)
    turn = SETTINGS_TEMP_MIN;

  targetTemp = turn;

  if ((mode == MODE_NACHGUSS) && (currentTemp >= targetTemp) && (spargingSignal == false))
  {
    spargingSignal = true;
    rufmodus = mode;          // make some noise and halt
    mode = MODE_ALARM;
    y = 0;
    brewmeisterArray[y] = 2;  // just make some noise and continue
  }
}

/// <summary>
/// set number of rests, mode 19
/// </summary>
void showSetNumberOfRests() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Eingabe");
    lcd.setCursor(12, 1);
    lcd.print("Rasten");

    turn = numberOfRests;
    initDisplay = 1;
  }

  // predefined values for different number of rests
  if (numberOfRests != turn)
  {
    if ((int)turn == 1)
    {
      restTempsArray[1] = 67;
      restTimesArray[1] = 60;
      mashTemp = 65;
    }
    if (turn == 2)
    {
      restTempsArray[1] = 62;
      restTimesArray[1] = 30;
      restTempsArray[2] = 72;
      restTimesArray[2] = 35;
      mashTemp = 62;
    }
    if (turn == 3)
    {
      restTempsArray[1] = 55;
      restTimesArray[1] = 10;
      restTempsArray[2] = 62;
      restTimesArray[2] = 35;
      restTempsArray[3] = 72;
      restTimesArray[3] = 25;
      mashTemp = 55;
    }
    if (turn == 4)
    {
      restTempsArray[1] = 40;
      restTimesArray[1] = 20;
      restTempsArray[2] = 55;
      restTimesArray[2] = 15;
      restTempsArray[3] = 64;
      restTimesArray[3] = 35;
      restTempsArray[4] = 72;
      restTimesArray[4] = 25;
      mashTemp = 35;
    }
    if (turn == 5)
    {
      restTempsArray[1] = 35;
      restTimesArray[1] = 20;
      restTempsArray[2] = 40;
      restTimesArray[2] = 20;
      restTempsArray[3] = 55;
      restTimesArray[3] = 15;
      restTempsArray[4] = 64;
      restTimesArray[4] = 35;
      restTempsArray[5] = 72;
      restTimesArray[5] = 25;
      mashTemp = 30;
    }
  }

  numberOfRests = turn;


  if (numberOfRests <= 1)
  {
    numberOfRests = 1;
    turn = 1;
  }
  if (numberOfRests >= 5)
  {
    numberOfRests = 5;
    turn = 5;
  }

  lcd.setCursor(19, 1);
  lcd.print(numberOfRests);

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode++;
      initDisplay = 0;
    }
}

/// <summary>
/// Prompts the user for a mashtemp, mode 20 (or 29)
/// </summary>
void showInputMashTemp() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y );
    lcd.print("Eingabe");
    turn = mashTemp;
    initDisplay = 1;
  }
  
  // apply limits
  if (turn < SETTINGS_TEMP_MIN)
    turn = SETTINGS_TEMP_MAX;
  if (turn > SETTINGS_TEMP_MAX)
    turn = SETTINGS_TEMP_MIN;

  mashTemp = turn;

  lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y + 1);
  lcd.print("Einmaischen");
  lcd.setCursor(15, 1);
  if (mashTemp < 100) {
    lcd.print(" ");
  }
  if (mashTemp < 10) {
    lcd.print(" ");
  }
  lcd.print((int)mashTemp);
  lcd.print(char(0xDF)); lcd.print("C");

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode++;
      initDisplay = 0;
    }
}

/// <summary>
/// Prompts user for rest temp, mode 21
/// </summary>
void showRestTemp() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y);
    lcd.print("Eingabe");
    turn = restTempsArray[x];
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_TEMP_MIN)
    turn = SETTINGS_TEMP_MAX;
  if (turn > SETTINGS_TEMP_MAX)
    turn = SETTINGS_TEMP_MIN;
  restTempsArray[x] = turn;

  lcd.setCursor(14, 0);
  lcd.print(x);
  lcd.print(".Rast");

  lcd.setCursor(15, 1);
  if (restTempsArray[x] < 100) {
    lcd.print(" ");
  }
  if (restTempsArray[x] < 10) {
    lcd.print(" ");
  }
  lcd.print(int(restTempsArray[x]));
  lcd.print(char(0xDF)); lcd.print("C ");


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true) {
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode++;
      initDisplay = 0;
    }
  }
}

/// <summary>
/// Show rest time, mode 22
/// </summary>
void showRestTime() {
  if (initDisplay == 0)
  {
    turn = restTimesArray[x];
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_TIME_MIN)
    turn = SETTINGS_TIME_MAX;
  if (turn > SETTINGS_TIME_MAX)
    turn = SETTINGS_TIME_MIN;
  restTimesArray[x] = turn;

  if (restTimesArray[x] < 10)
  {
    lcd.setCursor(14, 2);
    lcd.print(" ");
    lcd.print(int(restTimesArray[x]));
    lcd.print(" min");
  }
  else
  {
    lcd.setCursor(14, 2);
    lcd.print(int(restTimesArray[x]));
    lcd.print(" min");
  }

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true) {
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode = MODE_SOUND;
      initDisplay = 0;
    }
  }
}

/// <summary>
/// show setup page for delay cooling, MODE_SETTINGS_WAIT_COOL.
/// </summary>
void display_settings_waitCool() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Wartezeit K"); lcd.print(char(0xF5));  lcd.print("hlen");

    delayCoolEEPROM = EEPROM.read(EEPROM_DELAYCOOL);
    delayCoolMS = 60000 * (unsigned long)delayCoolEEPROM;

    turn = delayCoolEEPROM;
    initDisplay = 1;
  }
  else
  {
    if (turn < SETTINGS_LIM_DELAY_COOL_MIN || turn == 255) {
      turn = SETTINGS_LIM_DELAY_COOL_MAX;
    }
    if (turn > SETTINGS_LIM_DELAY_COOL_MAX) {
      turn = SETTINGS_LIM_DELAY_COOL_MIN;
    }

    delayCoolEEPROM = turn;


    lcd.setCursor(0, 1); lcd.print(delayCoolEEPROM); lcd.print(" min  ");


    if (buttonPressed == 0)
      buttonPress = true;

    if (buttonPress == true)
      if (buttonPressed == 1)
      {
        buttonPress = false;
        if (EEPROM.read(EEPROM_DELAYCOOL) != delayCoolEEPROM) {
          EEPROM.write(EEPROM_DELAYCOOL, delayCoolEEPROM);
          delayCoolMS = 60000 * (unsigned long)delayCoolEEPROM;
          display_settings_saving();
        }

        mode = MODE_SETTINGS;
        initDisplay = 0;
      }
  }
}

/// <summary>
/// show setup page for delay heating, MODE_SETTINGS_WAIT_HEAT.
/// </summary>
void display_settings_waitHeat() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Verz. Heizrelais");
    delayHeatEEPROM = EEPROM.read(EEPROM_DELAYHEAT);
    delayHeatMS = 1000 * (unsigned long)delayHeatEEPROM;
    turn = delayHeatEEPROM;
    initDisplay = 1;
  }
  else
  {

    if (turn < SETTINGS_LIM_DELAY_HEAT_MIN || turn == 255) {
      turn = SETTINGS_LIM_DELAY_HEAT_MAX;
    }
    if (turn > SETTINGS_LIM_DELAY_HEAT_MAX) {
      turn = SETTINGS_LIM_DELAY_HEAT_MIN;
    }

    delayHeatEEPROM = turn;


    lcd.setCursor(0, 1); lcd.print(delayHeatEEPROM); lcd.print(" s  ");


    if (buttonPressed == 0)
      buttonPress = true;

    if (buttonPress == true)
      if (buttonPressed == 1)
      {
        buttonPress = false;
        if (EEPROM.read(EEPROM_DELAYHEAT) != delayHeatEEPROM) {
          EEPROM.write(EEPROM_DELAYHEAT, delayHeatEEPROM);
          delayHeatMS = (long)delayHeatEEPROM * 1000;
          display_settings_saving();
        }
        mode = MODE_SETTINGS;
        initDisplay = 0;
      }
  }
}

/// <summary>
/// Display the menu for setting the bioling threshold, MODE_SETTINGS_BOILING_TRHESHOLD.
/// </summary>
void display_settings_boilingThreshold() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Kochschwelle");
    lcd.setCursor(0, 1); lcd.print("Ruf bei:");
    turn = EEPROM.read(EEPROM_BOILING_THRESHOLD);
    initDisplay = 1;
  }
  else
  {


    if (turn < SETTINGS_LIM_BOILING_MIN) {
      turn = SETTINGS_LIM_BOILING_MAX;
    }
    if (turn > SETTINGS_LIM_BOILING_MAX) {
      turn = SETTINGS_LIM_BOILING_MIN;
    }


    boilingTempTarget = turn;


    lcd.setCursor(9, 1); lcd.print(boilingTempTarget);


    if (buttonPressed == 0)
      buttonPress = true;

    if (buttonPress == true)
      if (buttonPressed == 1)
      {
        buttonPress = false;
        if (EEPROM.read(EEPROM_BOILING_THRESHOLD) != boilingTempTarget) {
          EEPROM.write(EEPROM_BOILING_THRESHOLD, boilingTempTarget);
          display_settings_saving();
        }

        mode = MODE_SETTINGS;
        initDisplay = 0;
      }
  }
}

/// <summary>
/// Displays the input for the last mash temperature
/// </summary>
void display_auto_lastMashTemp() {

  if (initDisplay == 0)
  {
    lcd.clear();
    turn = endtemp;
    initDisplay = 1;
  }

  endtemp = turn;

  if (endtemp <= 10)
    endtemp = 10;

  if (endtemp >= 80)
    endtemp = 80;

  lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y);
  lcd.print("Abmaischen");
  lcd.setCursor(16, 1);
  lcd.print(int(endtemp));
  lcd.print(char(0xDF)); lcd.print("C ");

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode++;
      initDisplay = 0;
    }
}

/// <summary>
/// Displays the dialog. If users confirmes, mashing starts
/// </summary>
void display_auto_start() {
  if (initDisplay == 0)
  {
    lcd.clear();
    initDisplay = 1;
    altsekunden = millis();
  }

  if (millis() >= (altsekunden + 1000))
  {
    lcd.setCursor(13, 1);
    lcd.print("       ");
    if (millis() >= (altsekunden + 1500))
      altsekunden = millis();
  }
  else
  {
    lcd.setCursor(13, 1);
    lcd.print("Start ?");
  }

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      initDisplay = 0;
      mode++;
    }
}

/// <summary>
/// Displays the informations throughout mashing
/// </summary>
void display_auto_mashing() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y);
    lcd.print("Maischen");
    turn = mashTemp;
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_TEMP_MIN)
    turn = SETTINGS_TEMP_MAX;
  if (turn > SETTINGS_TEMP_MAX)
    turn = SETTINGS_TEMP_MIN;

  mashTemp = turn;

  targetTemp = mashTemp;

  if ((currentTemp >= (targetTemp - hystereses) && currentTemp <= (targetTemp + hystereses)))  // targettemperature reached?
  {
    mode++;
    rufmodus = mode;
    y = 0;
    brewmeisterArray[y] = 1;
    mode = MODE_ALARM;
  }
}

/// <summary>
/// temperature automatic. This is active when the target temperature is not
/// yet reached. Once that is reached and the time needs to be counted, function 
/// display_auto_time is called
/// </summary>
void display_auto_temp() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(SETTINGS_DISP_WHAT_X, SETTINGS_DISP_WHAT_Y);
    lcd.print(x);
    lcd.print(".Rast");

    turn = restTempsArray[x];
    initDisplay = 1;
  }

  restTempsArray[x] = turn;

  targetTemp = restTempsArray[x];


  if ((currentTemp >= (targetTemp - hystereses) && currentTemp <= (targetTemp + hystereses))) // targettemperature reached?
  {
    mode++;
    initDisplay = 0;
  }
}

/// <summary>
/// During mashing, after targettemperature was reached, this function is used
/// to display the current time left in the current mash step.
/// </summary>
void display_auto_time() {
  if (initDisplay == 0)
  {
    turn = restTimesArray[x];
    beep_ShortLong();
  }


  lcd.setCursor(SETTINGS_DISP_TIME_TARGET_X, SETTINGS_DISP_TIME_TARGET_Y);
  if (restTimesArray[x] < 10)
  {
    lcd.print(" ");
    lcd.print(int(restTimesArray[x]));
    lcd.print(" min");
  }
  else
  {
    lcd.print(int(restTimesArray[x]));
    lcd.print(" min");
  }


  // couting time
  if (initDisplay == 0)
  {
    setTime(00, 00, 00, 00, 01, 01);	    // set seconds to zero

    seconds = second();					          // save the current second
    minutesValue = minute();			        // save current minutes for couting time
    hours = hour();						            // save current hours for couting time

    lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
    lcd.print("        ");
    initDisplay = 1;
  }

  lcd.setCursor(0, 2);

  if (seconds < 10)
  {
    if (minutes == 0)
    {
      lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
      lcd.print("00:0"); lcd.print(seconds);
    }
  }
  else
  {
    if (minutes == 0)
    {
      lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
      lcd.print("00:"); lcd.print(seconds);
    }
  }


  if (hours == 0)
    minutes = minutesValue;
  else
    minutes = ((hours * 60) + minutesValue);

  if ((minutes < 10) && (minutes > 0))
  {
    lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
    lcd.print("0"); lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  if ((minutes >= 10) && (minutes < 100))
  {
    lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
    lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  if (minutes >= 100)
  {
    lcd.setCursor(SETTINGS_DISP_TIME_REMAIN_X, SETTINGS_DISP_TIME_REMAIN_Y);
    lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  restTimesArray[x] = turn;

  if (minutes >= restTimesArray[x])   // target temperature of current step reached?
  {
    initDisplay = 0;
    y = x;
    if (x < numberOfRests)
    {
      mode--;						              // go to temperature control
      x++;						                // next step
    }
    else
    {
      x = 1;						              // last temp
      mode++;						              // Endtemperatur
    }

    if (brewmeisterArray[y] > 0)
    {
      rufmodus = mode;
      mode = MODE_ALARM;
    }
  }
}

/// <summary>
/// Same as display_auto_temp - only for the last temperature
/// </summary>
void display_auto_tempLast() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Abmaischen");

    turn = endtemp;
    initDisplay = 1;
  }

  endtemp = turn;

  targetTemp = endtemp;

  if ((currentTemp >= (targetTemp - hystereses) && currentTemp <= (targetTemp + hystereses)))  // targettemperature reached ?
  {
    mode = MODE_ABORT;
    rufmodus = mode;
    mode = MODE_ALARM;
    controlActivated = 0;
    relaisState = 0;
    y = 0;
    brewmeisterArray[y] = 1;
  }
}

/// <summary>
/// 
/// </summary>
void funktion_braumeister() {

  brewmeisterArray[x] = 0;
  if (x < numberOfRests)
  {
    x++;
    mode = MODE_ENTER_RAST_TEMP;
    initDisplay = 0;
  }
  else
  {
    x = 1;
    mode++;
    initDisplay = 0;
  }
}

/// <summary>
/// 
/// </summary>
void funktion_braumeisterrufalarm() {
  if (initDisplay == 0)
  {
    lastSignalTS = millis();
    altsekunden = millis();
    initDisplay = 1;
  }

  // Blink display and call braumeister
  if (millis() >= (altsekunden + 1000))
  {
    lcd.setCursor(17, 2);
    lcd.print("   ");
    digitalWrite(SETTINGS_PIN_SOUND, LOW);

    if (millis() >= (altsekunden + 1500))
    {
      altsekunden = millis();
      pause++;
    }
  }
  else
  {
    lcd.setCursor(SETTINGS_DISP_RUF_X, SETTINGS_DISP_RUF_Y);
    lcd.print("Ruf");
    if (pause <= 4)
    {
      digitalWrite(SETTINGS_PIN_SOUND, HIGH);

    }
    if (pause > 8)
      pause = 0;
  }

  // after twenty seconds, turn of alarm
  if (brewmeisterArray[y] == 2 && millis() >= (lastSignalTS + 20000))
  {
    initDisplay = 0;
    pause = 0;
    digitalWrite(SETTINGS_PIN_SOUND, LOW);

    mode = rufmodus;
    buttonPress = false;
    lcd.setCursor(SETTINGS_DISP_RUF_X, SETTINGS_DISP_RUF_Y);
    lcd.print("   ");
  }

  // continue with programm
  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      pause = 0;
      initDisplay = 0;
      digitalWrite(SETTINGS_PIN_SOUND, LOW);   // turn of alarm

      if (brewmeisterArray[y] == 2)
      {
        lcd.setCursor(17, 2);
        lcd.print("   ");
        if (sensorError == 1) {
          sensorError = 0;
          lcd.setCursor(8, 0);
          lcd.print("            ");
        }
        mode = rufmodus;
      }
      else
        mode++;
    }
}

/// <summary>
/// 
/// </summary>
void funktion_braumeisterruf() {
  if (initDisplay == 0)
  {
    initDisplay = 1;
  }


  if (millis() >= (altsekunden + 1000))
  {
    lcd.setCursor(12, 2);
    lcd.print("        ");
    if (millis() >= (altsekunden + 1500))
      altsekunden = millis();
  }
  else
  {
    lcd.setCursor(0, 2); lcd.print("            ");
    lcd.setCursor(SETTINGS_DISP_CONTINUE_X, SETTINGS_DISP_CONTINUE_Y);
    lcd.print("weiter ?");
  }


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;

      lcd.setCursor(SETTINGS_DISP_CONTINUE_X, SETTINGS_DISP_CONTINUE_Y);
      lcd.print("        ");					// clear text "weiter ?"

      lcd.setCursor(0, 3);	
      lcd.print("             ");				// clear text
      if (sensorError == 1) {
        sensorError = 0;  
        lcd.setCursor(8, 0);
        lcd.print("            ");
      }

      initDisplay = 0;
      mode = rufmodus;
      if (rufmodus == 42) {
        initDisplay = 1;					// add hops
        lcd.setCursor(0, 2);
        lcd.print("                    ");
      }

      // short delay, to avoid unintentional changing of the
      // next value (in next step)
      delay(500);
    }
}

/// <summary>
/// set the number of hops
/// </summary>
void display_boil_numberOfHops() {

  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hopfengaben");
    lcd.setCursor(5, 1);
    lcd.print("Anzahl");
    initDisplay = 1;
    numberOfHops = 3;
    turn = numberOfHops;
    lcd.setCursor(14, 1);
    lcd.print(numberOfHops);
  }


  if (numberOfHops < 9)
  {
    lcd.setCursor(14, 1); lcd.print(numberOfHops);
  }
  else
  {
    lcd.setCursor(13, 1); lcd.print(numberOfHops);
  }

  numberOfHops = turn;

  if (turn >= maxNumberOfHops)
  {
    numberOfHops = maxNumberOfHops; turn = maxNumberOfHops;
  }

  if (turn < 2)
  {
    numberOfHops = 1; turn = 1;
  }


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode = MODE_HOP;
      initDisplay = 0;
    }


}


/// <summary>
///  Set times for hop
/// </summary>
void display_boil_hopTimes() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hopfengabe:"); lcd.print(h + 1);
    lcd.setCursor(3, 1);
    lcd.print("min vor Ende");

    if (boilingTime >= 60)
    {
      if (h == 0) hopsArray[h] = boilingTime;  // typical times for hop boiling
      if (h == 1) hopsArray[h] = boilingTime - 20;
      if (h == 2) hopsArray[h] = 10;
    }

    turn = hopsArray[h];

    initDisplay = 1;
  }

  // apply limits
  if (turn == 251)
  {
    turn = boilingTime;
  }
  if (turn > boilingTime)
  {
    turn = 0;
  }

  // display
  lcd.setCursor(0, 1);
  if (turn < 100)
  {
    lcd.print(" ");
  }
  if (turn < 10)
  {
    lcd.print(" ");
  }
  lcd.print(turn);

  hopsArray[h] = turn;


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      if (h == (numberOfHops - 1)) {
        lcd.clear();
        mode = MODE_BOILING_INIT;

      }
      else {
        h++;
      }
      initDisplay = 0;
    }
}

/// <summary>
/// Displays the menu for setting the boiling time
/// </summary>
void display_boil_boilingTime() {

  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Aufheizen");
    lcd.setCursor(4, 1);
    lcd.print("Kochzeit");
    lcd.setCursor(17, 1);
    lcd.print("min");

    lcd.setCursor(2, 2);
    lcd.print("Dr"); lcd.print(char(0xF5)); lcd.print("cken = weiter");

    turn = boilingTime;

    relaisState = 1; // turn on heating instantanious

    initDisplay = 1;
  }

  boilingTime = turn; // 5 minute jumps

  if (boilingTime <= 20)
  {
    boilingTime = 20;
    turn = 20;
  }
  if (boilingTime >= 180)
  {
    boilingTime = 180;
    turn = 180;
  }

  if (boilingTime < 100)
  {
    lcd.setCursor(12, 1);
    lcd.print("  ");
    lcd.print(boilingTime);
  }
  else
  {
    lcd.setCursor(12, 1);
    lcd.print(" ");
    lcd.print(boilingTime);
  }

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      mode = MODE_NUMBEROFHOPS;
      initDisplay = 0;
    }
}

/// <summary>
/// Gives the user the opportunity to set the boiling temperature again
/// </summary>
void display_boiling_boilTemperature() {
  if (initDisplay == 0)
  {
    lcd.setCursor(0, 0); lcd.print("Ruf bei "); 
    initDisplay = 1;
    altsekunden = millis();
    turn = boilingTempTarget;
    initDisplay = 1;

    lcd.setCursor(8, 0);
    lcd.print(boilingTempTarget); lcd.print(char(0xDF)); lcd.print("C"); lcd.print("  ");
  }


  // apply limits
  if (turn < SETTINGS_LIM_KSCHWELLE_MIN)
    turn = SETTINGS_LIM_KSCHWELLE_MAX;
  if (turn > SETTINGS_LIM_KSCHWELLE_MAX)
    turn = SETTINGS_LIM_KSCHWELLE_MIN;

  if (turn != boilingTempTarget) {
    boilingTempTarget = turn;

    lcd.setCursor(8, 0);
    lcd.print(boilingTempTarget); lcd.print(char(0xDF)); lcd.print("C"); lcd.print("  ");
  }

  if (millis() >= (altsekunden + 1500))
  {
    if (millis() >= (altsekunden + 1500)) altsekunden = millis();
    if (currentTemp >= boilingTempTarget)
    {
      boilingTempReached = true;
      lcd.setCursor(0, 2);
      lcd.print("Start Zeitz");
      lcd.print(char(0xE1));
      lcd.print("hlung?");
      sound_duration(300);  // Ruf boiling threshold reached
    }
    else
    {
      lcd.setCursor(0, 2);
      lcd.print("Warten!");
      boilingTempReached = false;
    }
  }
  else
  {
    lcd.setCursor(0, 2);
    lcd.print("                   ");
  }

  if (boilingTempReached)
  {
    if (buttonPressed == 0)

      buttonPress = true;

    if (buttonPress == true)
      if (buttonPressed == 1)
      {
        buttonPress = false;
        initDisplay = 0;
        lcd.setCursor(0, 2); lcd.print("               ");
        mode = MODE_BOILING_STARTED;
      }
  }
}

/// <summary>
/// display when boiling is in progress
/// </summary>
void display_boiling_main() {

  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kochen");
    lcd.setCursor(4, 1);
    lcd.print("Kochzeit");
    lcd.setCursor(17, 1);
    lcd.print("min");

    // TODO: merge with time automatic (display_auto_time())
    setTime(00, 00, 00, 00, 01, 01); 

    delay(400);

    seconds = second(); 
    minutesValue = minute();
    hours = hour();   

    initDisplay = 1;
    h = 0;
    turn = boilingTime;
  }

  if (boilingTime < 100)
  {
    lcd.setCursor(12, 1);
    lcd.print("  ");
    lcd.print(boilingTime);
  }
  else
  {
    lcd.setCursor(12, 1);
    lcd.print(" ");
    lcd.print(boilingTime);
  }



  lcd.setCursor(14, 0);


  if (seconds < 10)
  {
    if (minutes == 0)
    {
      lcd.setCursor(14, 0);
      lcd.print("00:0"); lcd.print(seconds);
    }
  }
  else
  {
    if (minutes == 0)
    {
      lcd.setCursor(14, 0);
      lcd.print("00:"); lcd.print(seconds);
    }
  }

  minutes = ((hours * 60) + minutesValue);

  if ((boilingTime - minutes == hopsArray[h]) && (h < numberOfHops))
  {
    lcd.setCursor(0, 2); lcd.print(h + 1); lcd.print(". H-Gabe");
    h++;
    sound_duration(2000);
  }

  if ((seconds + 10) % 30 == 0) { lcd.setCursor(0, 2); lcd.print("          "); }


  if ((minutes < 10) && (minutes > 0))
  {
    lcd.setCursor(14, 0);
    lcd.print("0"); lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  if ((minutes >= 10) && (minutes < 100))
  {
    lcd.setCursor(14, 0);
    lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  if (minutes >= 100)
  {
    lcd.setCursor(13, 0);
    lcd.print(minutes); lcd.print(":");
    if (seconds < 10) {
      lcd.print("0");
      lcd.print(seconds);
    }
    else lcd.print(seconds);
  }

  boilingTime = turn; // 5min jumps

  if (boilingTime <= 20)
  {
    boilingTime = 20;
    turn = 20;
  }
  if (boilingTime >= 180)
  {
    boilingTime = 180;
    turn = 180;
  }

  if (minutes >= boilingTime)
  {
    mode = MODE_ABORT; 
    rufmodus = mode;
    mode = MODE_ALARM;
    controlActivated = 0;
    relaisState = 0;
    y = 0;
    brewmeisterArray[y] = 2;
    initDisplay = 0;
  }

}

/// <summary>
/// settings menu for setting kdheat
/// </summary>
void display_settings_kdheat() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("kd Heizen = ");
    lcd.setCursor(0, 2);
    lcd.print("sp"); lcd.print(char(0xE1));  lcd.print("ter   aus  fr"); lcd.print(char(0xF5));  lcd.print("her");

    turn = kdHeatEEPROM;
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_LIM_KD_HEAT_MIN)
    turn = SETTINGS_LIM_KD_HEAT_MAX;
  if (turn > SETTINGS_LIM_KD_HEAT_MAX)
    turn = SETTINGS_LIM_KD_HEAT_MIN;

  kdHeatEEPROM = turn;

  // clear display
  for (int i = 0; i < 20; ++i) {
    lcd.setCursor(i, 1);
    lcd.write((uint8_t)markerCharBorderNum);
  }
  // set double
  lcd.setCursor(kdHeatEEPROM - 5, 1);
  lcd.write((uint8_t)markerCharNum);

  // set value
  lcd.setCursor(12, 0);
  lcd.print(0.1 * float(kdHeatEEPROM));


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true) {
    if (buttonPressed == 1)
    {
      buttonPress = false;
      if (EEPROM.read(EEPROM_KDHEAT) != kdHeatEEPROM) {
        EEPROM.write(EEPROM_KDHEAT, kdHeatEEPROM);
        kdHeat = 0.1 * float(kdHeatEEPROM);
        display_settings_saving();
      }

      mode = MODE_SETTINGS;
      initDisplay = 0;
      lcd.clear();
    }
  }
}

/// <summary>
/// settings menu for setting kdcool
/// </summary>
void display_settings_kdcool() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("kd K"); lcd.print(char(0xF5));  lcd.print("hlen = ");
    lcd.setCursor(0, 2);
    lcd.print("sp"); lcd.print(char(0xE1));  lcd.print("ter   aus  fr"); lcd.print(char(0xF5));  lcd.print("her");

    turn = kdCoolEEPROM;
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_LIM_KD_COOL_MIN)
    turn = SETTINGS_LIM_KD_COOL_MAX;
  if (turn > SETTINGS_LIM_KD_COOL_MAX)
    turn = SETTINGS_LIM_KD_COOL_MIN;
  kdCoolEEPROM = turn;

  // clear display
  for (int i = 0; i < 20; ++i) {
    lcd.setCursor(i, 1);
    lcd.write((uint8_t)markerCharBorderNum);
  }
  // set booble
  lcd.setCursor(kdCoolEEPROM - 5, 1);
  lcd.write((uint8_t)markerCharNum);

  // set value
  lcd.setCursor(12, 0);
  lcd.print(0.1 * float(kdCoolEEPROM));


  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true) {
    if (buttonPressed == 1)
    {
      buttonPress = false;
      if (EEPROM.read(EEPROM_KDCOOL) != kdCoolEEPROM) {
        EEPROM.write(EEPROM_KDCOOL, kdCoolEEPROM);
        kdCool = 0.1 * float(kdCoolEEPROM);
        display_settings_saving();
      }
      mode = MODE_SETTINGS;
      initDisplay = 0;
      lcd.clear();
    }
  }
}

/// <summary>
/// settings menu for setting hystereses
/// </summary>
void display_settings_hystereses() {
  if (initDisplay == 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Zieltemp erreicht");
    lcd.setCursor(0, 1);
    lcd.print("ab Temp. Differenz");
    lcd.setCursor(0, 2);
    lcd.print("zu Zieltemp.");

    turn = hystereseEEPROM;
    initDisplay = 1;
  }

  // apply limits
  if (turn < SETTINGS_LIM_HYSTERESIS_MIN)
    turn = SETTINGS_LIM_HYSTERESIS_MAX;
  if (turn > SETTINGS_LIM_HYSTERESIS_MAX)
    turn = SETTINGS_LIM_HYSTERESIS_MIN;
  hystereseEEPROM = turn;


  lcd.setCursor(13, 2);
  lcd.print(0.1 * float(hystereseEEPROM));
  lcd.print(char(0xDF)); lcd.print("C ");

  if (buttonPressed == 0)
    buttonPress = true;

  if (buttonPress == true)
    if (buttonPressed == 1)
    {
      buttonPress = false;
      if (EEPROM.read(EEPROM_HYSTERESE) != hystereseEEPROM) {
        // only write if value changed
        EEPROM.write(EEPROM_HYSTERESE, hystereseEEPROM);
        hystereses = 0.1 * float(hystereseEEPROM);
        display_settings_saving();
      }
      mode = MODE_SETTINGS;
      initDisplay = 0;
      lcd.clear();
    }
}


/// <summary>
/// prints temperatures to the COM port - suitable for logview
/// studio download avaiable under
/// http://www.logview.info/forum/index.php?resources/
/// </summary>
void print_logview() {
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    switch (c)
    {
    case 83:                                   // "S" -> Start
      Serial.println("Start");
      // Send Header information
      Serial.print("$N$;Maische Temp Logging\r\n");
      Serial.print("$C$;Sollwert[C];IstTemp [C]\r\n");
      logviewTransferEnabled = true;
      break;
    case 69:                                   // "E" -> End
      Serial.println("Log ende");
      logviewTransferEnabled = false;
      break;
    }
  }
  if (logviewTransferEnabled)
  {
    Serial.print("$");
    Serial.print(targetTemp);
    Serial.print(";");
    Serial.print(currentTemp);
    Serial.print("\r\n");
  }
}

/// <summary>
/// Makes a sound of duration
/// </summary>
/// <param name="duration">duration in ms</param>
void sound_duration(int duration) {
  digitalWrite(SETTINGS_PIN_SOUND, HIGH);
  delay(duration);
  digitalWrite(SETTINGS_PIN_SOUND, LOW);
}

/// <summary>
/// for writing to the EEPROM (does the byte wrangling 
/// for you - splits a long into different bytes)
/// </summary>
/// <param name="address">where to write</param>
/// <param name="value">what to write</param>
void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);

  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
}

/// <summary>
/// for reading a long from the EEPROM
/// </summary>
/// <param name="address">the adress where to put data</param>
/// <returns>the long</returns>
long EEPROMReadlong(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF);
}

/// <summary>
/// displays "saving" when save settings
/// </summary>
void display_settings_saving() {
  lcd.setCursor(0, 3);
  lcd.print("speichern");
  delay(500);
}

/// <summary>
/// Aborts everything and resets everything
/// </summary>
void main_abort() {
  controlActivated = 0;
  relaisState = 0;

  lastRelaisChangeTS = 0;
  digitalWrite(SETTINGS_PIN_RELAIS, SETTINGS_RELAY_OFF);
  digitalWrite(SETTINGS_PIN_SOUND, LOW); 

  lcd.clear();

  x = 1;              
  h = 0;
  y = 1;
  n = 0;
  buttonPress = false;
  spargingSignal = false;
  logviewTransferEnabled = false;  // close logview
  pause = 0;
  turn = targetTemp;

  boilingTempReached = false;

  delayCoolMS = 0;
  initDisplay = 0;
  rufmodus = MODE_MAIN;
  mode = MODE_MAIN;
}
