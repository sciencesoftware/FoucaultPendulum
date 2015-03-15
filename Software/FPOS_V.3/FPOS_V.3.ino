#include "defs.h"
#include "TimerOne.h"
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//Open source Foucault Pendulum program
//this program will drive the portable foucault pendulum

//Global Variable Declarations
unsigned int	curSample;	//the current a/d sample of the pendulum output
unsigned int	masterClock; //the master clock counter, count timed interupts of 100 uS each,
//in sample mode it measures the length of the present period, from last zero crossing
//in Driver mode it is the number of periods since the last sample mode
long	        sumOfPeriods;  //sum of all the last periodCount periods

char	        periodCount; 	//used for avg period calculation
char 	        xcount;		//crossing count, 4 is a full period

unsigned int	periodLength;	//length of a period calculated from samples, in driver mode
//unsigned int	driverCount;	//countdown until next speaker driver event;
int 	        throwawayCnt;
int		switchTimer;	//used as a countdown timer for when to check a switch

enum	        switchState {swOFF, swON1, swON2, waitToSave, adjust, flashLED} swUpState; //switchState ;
//swON - Switch is on, most of the time it's here
//swOFF1 - first off state, happened, do the function
//swOFF2 - did function, waiting to turn back on
enum	        switchState 	swDownState;  //same as above

signed 	 char	saveAdjustTimer;
char* 	        eesaveptr;		//pointer to take apart an Integer into bytes

int		waveCounter;		//counts the current period for the DAC output


boolean		speakerMode;	//DRIVER=0 or SAMPLE=1 (period calculation
boolean		timerFlag;
boolean		direction;		//driver direction, 0 down, 1 up
boolean		Xflag = CLEAR;			//zerocrossing flag
boolean		upDown = UP;			//flag if the signal is up or down
//i.e. above or below the zero crossing
boolean 		mode;
boolean		outputFlag;		//time output a new value for the DAC, 0-511
boolean		startupflag;	//set only when starting up

long startAD = 0;
long endAD = 0;
signed int y, t, position, quad;
int    count = 0;
long   waveFraction = 0;  //used to store the fraction part of the interupt counter
long   constFraction = 0; //added to waveFraction on each interupt
int    scaleFactors[NUMSCALEFACTORS] = {SF0, SF1, SF2, SF3, SF4, SF5, SF6, SF7}; //scale is 0-5
unsigned int    currentScale; //fullscale
long currentTime = 0;

char* eeScalePtr; //needed to point to the bytes of currentScale
int maxValue = 0;
int minValue = 1024;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup()
{
  initFP();
  currentTime = millis();
}

void loop()
{
  /*find period from this sample and others.*/
  if (timerFlag)	//timer has just gone off
  {

    //digitalWrite(SCOPEOUT, HIGH);
    //delayMicroseconds(1);
    //digitalWrite(SCOPEOUT, LOW);

    if ( speakerMode == SAMPLE )
    {
      atod();		//do and A/D and wait until it's done, about 20 uS
      digitalWrite(LED, !upDown); //LED=!upDown;

      if ( calcPeriod() == DRIVER )	//change to driver mode?
      {
        //change to driver mode
        //start in up position, otherwise the speaker may go pop
        direction = UP;
        Serial.println("Start Wave output " + String(periodLength));
        wait4Sync(); //make sure the bob is in the right position before starting the driver
        digitalWrite(RELAYPORT, OFF);	//RELAYPORT=OFF;

        startDriving(periodLength);
      }
    }
    timerFlag = CLEAR;
  }

  if (outputFlag) //happens about every 2ms
  {
    //digitalWrite(SCOPEOUT,ON);
    nextDacValue();
    outputFlag = CLEAR;
    //digitalWrite(SCOPEOUT,OFF);
    if ( (millis() - currentTime) > SWITCHDELAY)
    {
      checkSwitches();
      currentTime = millis();
      //Serial.println("Checked switch " + String(currentTime));
    }

  }
  if (stringComplete )
  {
    inputString.toLowerCase();
    if (inputString.equals("save"))
    {
      saveEEPROM();
    }
    if (inputString.equals("erase"))
    {
      eraseEEPROM();
    }
    if (inputString.equals("reboot"))
    {
      wdt_enable(WDTO_30MS); //enable the watchdog timer and wait for a reset
      while (1) {};
    }
    if (inputString.equals("help"))
    {
      printHelp();
    }
    long value = inputString.toInt();
    if ( value != 0 )
    {
      //the number is the period
      if (value > 9000 && value < 11000)
      {
        periodLength = value; //it's the new period, recalculate the driver wave
        recalcWave(periodLength);
        Serial.println("Using new Period " + String(periodLength));
      }
    }
    inputString = ""; //clear the string
    stringComplete = false;
  }
}
//initialize the Arduino
//set all three of the interrupts
void initFP()
{
  cli();//stop interrupts
  //set timer2 interrupt at 10kHz

  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = TIMER2REG;   // = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);



  pinMode(RELAYPORT, OUTPUT);
  pinMode(DRIVERPORT, OUTPUT);
  pinMode(SCOPEOUT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(SWITCH_LED, OUTPUT);
  pinMode(SWITCHUP,  INPUT_PULLUP);
  pinMode(SWITCHDOWN,  INPUT_PULLUP);

  swUpState = swOFF;
  swDownState = swOFF;
  Serial.begin(BAUDRATE); // opens serial port, sets data rate to 115200 bps
  inputString.reserve(200); //for serial input
  stringComplete = false;

  Serial.println("Program Start");
  printHelp();

  eesaveptr = (char*)&periodLength; //needed to point to the bytes of periodLength
  eeScalePtr = (char*)&currentScale; //needed to point to the bytes of currentScale

  digitalWrite(LED, LEDOFF); //green on
  digitalWrite(SWITCH_LED, LEDOFF);  //red LED off

  clearCurrentPeriod();

  //get the period in eeprom
  for ( int i = 0; i < 2; i++)
  {
    eesaveptr[i] = EEPROM.read(i);
  }
  //get Scale in eeprom
  for ( int i = 2; i < 4; i++)
  {
    eeScalePtr[i - 2] = EEPROM.read(i);
  }

  Serial.println("EEPROM period " + String((long)periodLength));
  Serial.println("EEPROM scale " + String((long)currentScale));


  if ( periodLength > (MAXPERIOD >> 1) || periodLength <  (MINPERIOD >> 1))
  {
    eraseEEPROM();
    currentScale = 0;
  }
  digitalWrite(RELAYPORT, ON);
  //clearPeriods();

  //change the ADC prescaler to speed up the ADC, prescaler 4 or clock/16, 1Mhz
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;

  startupflag = SET;

  waitForStableData();

  Serial.println("Continue Sampling");
  sei();//allow interrupts
}

//used to sampple the input read of the speaker in Sample mode
//in driver mode it triggers a change to duty cycle for the PWM output
ISR(TIMER2_COMPA_vect) { //timer0 interrupt 10kHz

  if ( speakerMode == SAMPLE )
  {
    timerFlag = SET;
    masterClock++;
    if (switchTimer > 0) 	//timer for debounce of switches
    {
      --switchTimer;
    }
  }
  else
  {
    //the outputFlag is only set when the fraction being added reaches one
    waveFraction += constFraction;  //used to store the fraction part of the interupt counter
    if (waveFraction >= ONESCALED) //is the fraction greater than one
    {
      outputFlag = SET;
      waveFraction -= ONESCALED; //get rid of the interger part, but leave the fraction
    }
  }
}

//when a byte becomes available, it is read into the buffer
void serialEvent() {

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n' || inChar == '#') {
      stringComplete = true;
    }
    else
    {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
/*always returns after a period has been calculated and the drive just needs to go down

	 Must throw out the first output of the filter because the first
		period starts in an unknown place, probably not at an edge
*/
boolean calcPeriod()
{
  signed int		temp;
  /*The main stategy is as follows:
  The transition is very sharp and the xflag is set when a transition
  happens, the period is 4 transitions
  Period counts will be
  added to the sumOfPeriods
  */

  mode = SAMPLE;
  //did a zero crossing just happen
  if ( Xflag )
  {
    Xflag = 0;
    //end of good cycle or half cycle,
    //start of a good one
    //masterClock has he period
    ++xcount;
    if (xcount < CROSSINGSPERPERIOD)	//if 4 crossings continue
      return mode;
    xcount = 0;

    //inital throwaway of periods, helps stablize the system
    if (throwawayCnt > 0)
    {
      throwawayCnt--;
      clearCurrentPeriod();
      return mode;
    }

    if ((masterClock > MAXPERIOD) || (masterClock < MINPERIOD))	//period too long or too short
    {
      clearCurrentPeriod();
      return mode;
    }
    temp = (unsigned int)masterClock;
    //when the first period comes out of the filter,
    //the max and min period limits must be set
    if (temp != -1)
    {
      sumOfPeriods += temp;
      periodCount++;
    }

    /*if the periodLength is already set, then wait for 2 periods and then
    switch to driver mode, it will save the person doing the
    swinging from having to sync the pendulum to the speaker pulse
    */
    if ((periodLength > 0) && (periodCount > 2))	//driver mode?
    {
      mode = DRIVER;	//go directly into driver mode, the period is already set
    }
    //normal calculation of period
    if (periodCount >= PERIODSTOCOUNT)
    { //STOP measuring
      periodLength = sumOfPeriods >> PERIODSHIFT;
      periodLength += PERIODADJUST; //changes the natural frequency to the driver one
      //set the count ofr a 1/4 period, when it will
      //Serial.println("Driver Mode, the period is " + String(periodLength));
      mode = DRIVER;	//use this
    }
    else
    { //start a new period
      clearCurrentPeriod();
    }
  }
  return mode;
}

void clearCurrentPeriod()
{
  //initialize the period measurement and caluclation system
  speakerMode = SAMPLE;
  masterClock = 0;
  xcount = 0;
  return;
}

//***************************************************************************
//atod() 	- A to D Routine with setting of states
//
//***************************************************************************
//startup the cycle with the signal going up
void atod()
{

  curSample = analogRead(ATODPORT);
  if (startupflag)  //if startup, wait for a defined state
  {
    if (curSample > HIGHTRIG)
    {
      upDown = UP;
      Xflag = CLEAR;
      startupflag = CLEAR;
    }
    if (curSample < LOWTRIG)
    {
      upDown = DOWN;
      Xflag = CLEAR;
      startupflag = CLEAR;
    }
  }
  else
  {
    if ((curSample > HIGHTRIG) && (upDown == DOWN))
    {
      Xflag = SET;
      upDown = UP;
    }
    if ((curSample < LOWTRIG) && (upDown == UP))
    {
      Xflag = SET;
      upDown = DOWN;
    }
  }
  return;
}


/* Checks the state of the 2 switches, if pressed, the
	switch is read again to debounce, the period to drive the speaker
	is incremented or decremented (up or down).  If up is held for
	3 or more seconds, then the period adjust value is saved in eeprom
	When a normal press occurs, the the switch led is lite for 1 period
	If it's save it flashes on/off for 1 second
*/

void checkSwitches()
{
  bool sd = digitalRead(SWITCHUP);
  char i;
  if (saveAdjustTimer > 0)
    --saveAdjustTimer;

  //for the up switch, if it's pressed, increment amplitude
  switch (swUpState)
  {
    case swOFF:
      if (!sd)
        swUpState = swON1;
      break;
    case swON1:
      if (!sd)
      {
        swUpState = waitToSave;
        saveAdjustTimer = SAVE_ADJUST_TIMER;
        digitalWrite(SWITCH_LED, LEDON);		//turns on, ack
      }
      else
        swUpState = swOFF;
      break;
    case waitToSave:
      digitalWrite(SWITCH_LED, LEDOFF);		//turns off ack
      if (!sd)
      {
        //if a period of time has passed, then save the period adjust number
        if (saveAdjustTimer == 0)
        {
          saveEEPROM();
          Serial.println("Saved to eeprom");
          swUpState = flashLED;
          saveAdjustTimer = FLASH_LED_TIMER; //time to flash LED
        }
      }
      else
        swUpState = adjust;
      break;
    case adjust:
      if (sd)		//turned off
      {
        if (currentScale > 0)
        {
          currentScale--;
          Serial.println("currentScale=  " + String(currentScale));
        }
        swUpState = swON2;
      }
      break;
    case flashLED:
      digitalWrite(SWITCH_LED, !digitalRead(SWITCH_LED));	//flash LED on and off
      if (saveAdjustTimer == 0)
        swUpState = swON2;
      break;
    case swON2:
      digitalWrite(SWITCH_LED, LEDOFF);		//turns off flash
      if (sd)		//turned off
        swUpState = swOFF;
      break;
  }

  //for the down switch, if it's pressed, increment period length
  sd = digitalRead(SWITCHDOWN);

  switch (swDownState)
  {
    case swOFF:
      if (!sd)
      {
        swDownState = swON1;
      }
      break;
    case swON1:
      if (!sd)
      {
        swDownState = waitToSave;
        saveAdjustTimer = SAVE_ADJUST_TIMER;
        digitalWrite(SWITCH_LED, LEDON);		//turns on, ack
      }
      else
      {
        swDownState = swOFF;
      }
      break;
    case waitToSave:
      digitalWrite(SWITCH_LED, LEDOFF);		//turns off ack
      if (!sd)
      {
        //if a period of time has passed, then save the default period adjust number
        if (saveAdjustTimer == 0)
        {
          eraseEEPROM();
          Serial.println("Cleared eeprom");
          swDownState = flashLED;
          saveAdjustTimer = FLASH_LED_TIMER; //time to flash LED
        }
      }
      else
      {
        swDownState = adjust;
      }
      break;
    case adjust:
      if (sd)		//turned off
      {
        if (currentScale < NUMSCALEFACTORS - 1)
        {
          currentScale++;
          Serial.println("currentScale=  " + String(currentScale));
        }
        swDownState = swON2;
      }
      break;
    case flashLED:
      digitalWrite(SWITCH_LED, !digitalRead(SWITCH_LED));	//flash LED on and off
      if (saveAdjustTimer == 0)
      {
        swDownState = swON2;
      }
      break;
    case swON2:
      digitalWrite(SWITCH_LED, LEDOFF);		//turns off flash
      swDownState = swOFF;
      break;
  }
}


//start sending out the cos wave, the period is "steps" long
//setup timer1 for PWM mode, duty cycle and start the interupts
//at this moment the bob should be on a half way between up and down and moving downward
//so start the bob 90 degrees into the cycle from dropping the bob
//which is 3/4 into this wave
//interupt time 2 is used at the same 10k rate but in driver mode is used
//to calculate when to change the duty cycle to create a wave
void	startDriving(int steps)
{
  //cli();//stop interrupts
  Timer1.initialize(HALFPERIODWAVELENGTH * 2);
  Timer1.pwm(DRIVERPORT, INITDUTYCYCLE);  //start the PWM

  //set up the counter for timer2 to use
  //this works by using a scaled integer instead of floating point
  //above 1e6 (ONESCALED) is a whole number and time to change the duty cycle
  //belose 1e6 is a fraction, the SCALEFACTOR is used to convert the period (steps) to
  //the CONSTFRACTION to add to the waveFraction
  //During each interupt constFraction is added to waveFraction, when it value reaches
  //ONESCALED, the flag is triggered to change the duty cycle and the
  //waveFraction is reduced by ONESCALED

  waveFraction = 0;
  constFraction = SCALEFACTOR / steps;
  Serial.println("constFraction " + String(constFraction));

  waveCounter = STEPSINSAMPLE / 2;	//500 samples make up one quarter period,  3/4 into the cycle
  speakerMode = DRIVER;
  //sei();//start interrupts

}
//if in driver mode, then recalculate the contFraction to change the wavelength of the driver
void recalcWave(int steps)
{
  constFraction = SCALEFACTOR / steps;
  Serial.println("New constFraction " + String(constFraction));
}

void	stopWave(void)
{
  Timer1.disablePwm(DRIVERPORT);
}

//get the square wave but scaled it scaleFactor is 0-5,scales are predefined
signed int getSquare(int x, int scaleFactorLevel)
{
  int level = scaleFactors[scaleFactorLevel];
  if ( x <= STEPSINSAMPLE / 2 )
  {
    return level;
  }
  else
  {
    return 0;  //real systme

  }
}

//when called send out one sample of the wave out the PWM output
void	nextDacValue()
{
  int pwm2;
  //digitalWrite(SCOPEOUT,ON);

  signed int	pwm1 = getSquare(STEPSINSAMPLE - waveCounter, currentScale);

  if (waveCounter == STEPSINSAMPLE || waveCounter == STEPSINSAMPLE / 2)
  {
    output_toggle(LED);
  }
  pwm2 = (int)(pwm1 + OUTPUTOFFSET);
  Timer1.setPwmDuty(DRIVERPORT, pwm2);
  waveCounter--;
  if (waveCounter <= 0)	//if the period is over, start it again
  {
    waveCounter = STEPSINSAMPLE;
    waveFraction = 0; //get rid of any accumulated error and start over faction adding
  }
  //digitalWrite(SCOPEOUT,OFF);
  return;
}
//flip the output of the pin
void output_toggle(int pin)
{
  //get current state of the output pin
  short input_data = PORTD;
  if (bitRead(input_data, pin) == 1) //read the bit and flip the output
  {
    digitalWrite(pin, LOW);
  }
  else
  {
    digitalWrite(pin, HIGH);
  }

}
//Wait unil the bob signal is exactly at a zero crossing (512) after a high
void wait4Sync()
{
  startupflag = HIGH; //resync the bob with the read states
  while (curSample < 900)
  {
    atod();  //keep reading the atod untilo the bob is UP
    delayMicroseconds(500);
  }
  upDown == UP;
  //the bob is up
  //wait for bob to be down
  while (curSample > LOWTRIG)
  {
    atod();  //keep reading the atod untilo the bob is DOWN
    delayMicroseconds(500);
  }
  upDown == DOWN;
  //done, reached zero crossing after an UP
  Serial.println("Reached zero crossing, start driving");
}
void eraseEEPROM()
{
  for ( int i = 0; i < 4; i++)
  {
    EEPROM.write(i, 0);
  }
}
void waitForStableData()
{
  Serial.println("Wait for Stable Data");
  Serial.flush();

  //for real system delay to allow the electionics to stablize.
  for (int i = WAITFORSTABLEDDATA ; i > 0 ; i--)
  {
    for (int j = 0; j < 1000; j++)
    {
      delayMicroseconds(1000) ; //1 millisecond
    }
    Serial.println(String(i));
    Serial.flush();
  }
}
void saveEEPROM()
{
  Serial.println("Saving EEPROM period " + String(periodLength));
  for ( int i = 0; i < 2; i++)
  {
    EEPROM.write(i, eesaveptr[i]);
  }
  //save Scale in eeprom
  Serial.println("Saving EEPROM amplitude " + String(currentScale));
  for ( int i = 2; i < 4; i++)
  {
    EEPROM.write(i, eeScalePtr[i - 2]);
  }
}
void       printHelp()
{
  Serial.println();
  Serial.println("Commands");
  Serial.println("save - to save period and amplitude to eeprom");
  Serial.println("erase - to clear period and amplitude in eeprom");
  Serial.println("reboot - restart the arduino");
  Serial.println("A number between 9000 and 11000 for the period");
  Serial.println("help - this menu");
  Serial.println("Use '#' or CR as the final char");
  Serial.println();
}
