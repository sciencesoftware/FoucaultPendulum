#define SAMPLESPERSEC           10000
#define SWITCHDELAY		50		      //Number of interupts that are needed to get 50 mS
#define ONEMILLISECCNT		SAMPLESPERSEC/1000		//number of interupts to get on 1 mS
#define SAVE_ADJUST_TIMER	60				//holding the button for 3 seconds will save the period adjust
#define FLASH_LED_TIMER		20				//100 ms cycles to flash the LED to indicate that the adjust value was saved, 10 flashes
#define THROWAWAY_PERIOD_COUNT  3				//number of inital periods to throw away 
#define WAITFORSTABLEDDATA      7                             //Seconds wait for pendulum to have stable data 

#define	SET			1
#define	CLEAR			0
//#define	TRUE			1
//#define	FALSE			0
#define	ON			1
#define	OFF			0
#define	LEDON			0
#define	LEDOFF			1


#define ATODPORT		A0

#define	RELAYPORT		7 	
#define SERIALPORT		13 	
#define DRIVERPORT		9  	
#define LED			4
#define SCOPEOUT		12
#define SWITCHUP		2
#define SWITCHDOWN		3
#define SWITCH_LED		5

		//BLINKS when a switch is pushed and flashes when saving a period adjust

//speaker modes
#define	DRIVER			1
#define	SAMPLE			0
#define DOWN			1
#define UP			0

//period calculation values

#define DEBUG  0
#define MINPERIOD		19000	//in timer0 periods, initial
#define MAXPERIOD		20500	//in timer0 periods, initial

//the range on this program is that the pendulum must be between .25m and 1.5m
#define PERIODSTOCOUNT	32		//how many periods are added together


#define PERIODSHIFT		6		//used to divide the periods sum
#define ZEROXINGINIT	1023/2   		
#define PERIODADJUST	0		//add to the period length to adjust period
#define	LOWTRIG			ZEROXINGINIT-20
#define	HIGHTRIG		ZEROXINGINIT+20
#define	CROSSINGSPERPERIOD	4
#define TIMER2REG  198    //the register value to get timer 2 to be 10000 interupts per sec

#define BAUDRATE  115200  // baud rate for serial port


//eeprom addresses
//#define CHANGEADDRESS 0
#define SAVEDPERIODADDRESS 0
#define MAXPERIOD_ADDRESS 1
#define MINPERIOD_ADDRESS 2

//PWM which is really the DAC output
//#define CCP1CON_VALUE 0x0A	//single PWM output 

//for calculatiing Cosine or any wave from the table
#define QUADEND 125
#define VALSEND 125
#define	OUTPUTOFFSET	0
#define	STEPSINSAMPLE	500

//for PWM wave timing at 5102 hz square wave
#define HALFPERIODWAVELENGTH 98   //uSec half period
#define T2PRESCALER          128  //prescaler for 500 samples/period
#define SYSCLOCK             16000000
#define INITDUTYCYCLE        512  //50% duty initally
#define ONESCALED            1000000  //1 scaled for integer arithmetic 
#define SCALEFACTOR          5e8      //converts steps to constant that gets added
#define FULLSCALE            1023      //output for D/A converter to speaker


//scale factors
#define SF7   FULLSCALE * 0.4 
#define SF6   FULLSCALE * 0.45 
#define SF5   FULLSCALE * 0.5 
#define SF4   FULLSCALE * 0.55
#define SF3   FULLSCALE * 0.6  
#define SF2   FULLSCALE * 0.65
#define SF1   FULLSCALE * 0.75
#define SF0   FULLSCALE 
#define NUMSCALEFACTORS  8
