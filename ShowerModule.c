//###########################################################################
//
// FILE:    ShowerModule.c
//
// ASSUMPTIONS:
//
//    This program requires the F2802x0 header files.
//
//    This program makes use of variables stored in OTP during factory
//    test on 2802x TMS devices only.
//    These OTP locations on pre-TMS devices may not be populated.
//    Ensure that the following memory locations in TI OTP are populated
//    (not 0xFFFF) before use:
//
//    0x3D7E90 to 0x3D7EA4
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2802x Boot Mode table is shown below.
//
//    $Boot_Table
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:   EMU_KEY        EMU_BMODE
//                   (0xD00)         (0xD01)
//      ---------------------------------------
//      Wait         !=0x55AA        X
//      I/O          0x55AA          0x0000
//      SCI          0x55AA          0x0001
//      Wait         0x55AA          0x0002
//      Get_Mode     0x55AA          0x0003
//      SPI          0x55AA          0x0004
//      I2C          0x55AA          0x0005
//      OTP          0x55AA          0x0006
//      Wait         0x55AA          0x0007
//      Wait         0x55AA          0x0008
//      SARAM        0x55AA          0x000A   <-- "Boot to SARAM"
//      Flash        0x55AA          0x000B
//      Wait         0x55AA          Other
//
//   Write EMU_KEY to 0xD00 and EMU_BMODE to 0xD01 via the debugger
//   according to the Boot Mode Table above. Build/Load project,
//   Reset the device, and Run example
//
//   $End_Boot_Table

#include <stdio.h>
#include <file.h>

#include "DSP28x_Project.h"     // DSP28x Headerfile

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/sci_io.h"
#include "f2802x_common/include/wdog.h"

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)
//printf ("Leading text "BYTETOBINARYPATTERN, BYTETOBINARY(byte));
#define MOTOR_MAX 3.89 //in milliseconds
#define MOTOR_MIN 1.8
#define MOTOR_POSITIONS 10

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  23340  // Period register
#define EPWM1_MAX_CMPA     23340
#define EPWM1_MIN_CMPA      23340/2
#define EPWM1_MAX_CMPB     23340
#define EPWM1_MIN_CMPB      23340/2

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

extern void DSP28x_usDelay(Uint32 Count);

typedef struct
{
//    volatile struct EPWM_REGS *EPwmRegHandle;
    PWM_Handle myPwmHandle;
    uint16_t EPwm_CMPA_Direction;
    uint16_t EPwm_CMPB_Direction;
    uint16_t EPwmTimerIntCount;
    uint16_t EPwmMaxCMPA;
    uint16_t EPwmMinCMPA;
    uint16_t EPwmMaxCMPB;
    uint16_t EPwmMinCMPB;
}EPWM_INFO;

EPWM_INFO epwm1_info;
int16_t referenceTemp;
int16_t currentTemp;

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SCI_Handle mySci;
PWM_Handle myPwm1;

uint8_t digit[] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};
double motorIncrement = (MOTOR_MAX - MOTOR_MIN) / MOTOR_POSITIONS;

//34 is max pin available on Launchpad, 38 is max pin available on C2000 chip
//GPIO_Number_e digit1Pins[] = {GPIO_Number_3, GPIO_Number_4, GPIO_Number_5, GPIO_Number_6, GPIO_Number_7, GPIO_Number_12, GPIO_Number_16}; //Avoid 0 for PWM
GPIO_Number_e digit2Pins[] = {GPIO_Number_17, GPIO_Number_18, GPIO_Number_19, GPIO_Number_32, GPIO_Number_33, GPIO_Number_34, GPIO_Number_35}; //Avoid 28 and 29 for console output, 32 and 33 for multiplexing
GPIO_Number_e upButton = GPIO_Number_1;
GPIO_Number_e downButton = GPIO_Number_2;

// SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_init()
{

    CLK_enableSciaClock(myClk);

    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);
    
    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
    // Configured for 115.2kbps
#if (CPU_FRQ_60MHZ)
    SCI_setBaudRate(mySci, SCI_BaudRate_115_2_kBaud);    
#elif (CPU_FRQ_50MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)10);
#endif

    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_4_Words);

    SCI_setPriority(mySci, SCI_Priority_FreeRun);
    
    SCI_enable(mySci);
  
    return;
}

//Take a number 0-9 and displays it on the LCD
void display_on_LCD(int number)
{
	if (number < 0 || number > 9)
		return;

	//int tens = number / 10;
	int ones = number % 10;
	//uint8_t encodedTen = digit[tens];
	uint8_t encodedOne = digit[ones];

	int counter = 0;
	int downCounter = 6;
	uint8_t bit;

	//01000000 because encoding is 7 bits long
	for (bit = 0x40; bit; bit >>= 1)
	{
		//if ((bit & encodedTen) >> downCounter == 1)
		//	GPIO_setHigh(myGpio, digit1Pins[counter]);
		//else
			//GPIO_setLow(myGpio, digit1Pins[counter]);
		if ((bit & encodedOne) >> downCounter == 1)
			GPIO_setHigh(myGpio, digit2Pins[counter]);
		else
			GPIO_setLow(myGpio, digit2Pins[counter]);
		counter++;
		downCounter--;
	}
}

//Returns the needed period for a particular numerical position in seconds
double pwm_period_for_motor(int position)
{
	if (position < 0)
		return MOTOR_MIN / 1000;
	if (position > MOTOR_POSITIONS - 1)
		return MOTOR_MAX / 1000;

	return (MOTOR_MIN + (position * motorIncrement)) / 1000;
}

void turn_motor_to(int position)
{
	display_on_LCD(position);
	float sysclkout = 60000000;
	int highSpeedClkDiv = 10;
	int clkDiv = 1;
	printf("Period should be %d ms\n", pwm_period_for_motor(position) * 1000);
	int PWMPeriod = (pwm_period_for_motor(position) * sysclkout) / (highSpeedClkDiv * clkDiv);
	int halfPWMPeriod = PWMPeriod / 2;

    // Set Compare values
    PWM_setCmpA(myPwm1, halfPWMPeriod);    // Set compare A value
    PWM_setCmpB(myPwm1, halfPWMPeriod);    // Set compare B value
    PWM_setPeriod(myPwm1, PWMPeriod);

    epwm1_info.EPwmMaxCMPA = PWMPeriod;
    epwm1_info.EPwmMinCMPA = halfPWMPeriod;
    epwm1_info.EPwmMaxCMPB = PWMPeriod;
    epwm1_info.EPwmMinCMPB = halfPWMPeriod;
}

interrupt void epwm1_isr(void)
{
    // Update the CMPA and CMPB values
    //update_compare(&epwm1_info);

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm1);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
}

void InitEPwm1()
{

    CLK_enablePwmClock(myClk, PWM_Number_1);

    // Setup TBCLK
    PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);         // Count up
    PWM_setPeriod(myPwm1, EPWM1_TIMER_TBPRD);               // Set timer period
    PWM_disableCounterLoad(myPwm1);                         // Disable phase loading
    PWM_setPhase(myPwm1, 0x0000);                           // Phase is 0
    PWM_setCount(myPwm1, 0x0000);                           // Clear counter
    PWM_setHighSpeedClkDiv(myPwm1, PWM_HspClkDiv_by_10);     // Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm1, PWM_ClkDiv_by_1);

    // Setup shadow register load on ZERO
    PWM_setShadowMode_CmpA(myPwm1, PWM_ShadowMode_Shadow);
    PWM_setShadowMode_CmpB(myPwm1, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm1, PWM_LoadMode_Zero);
    PWM_setLoadMode_CmpB(myPwm1, PWM_LoadMode_Zero);

    // Set Compare values
    PWM_setCmpA(myPwm1, EPWM1_MIN_CMPA);    // Set compare A value
    PWM_setCmpB(myPwm1, EPWM1_MIN_CMPB);    // Set compare B value

    // Set actions
    PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Set);            // Set PWM1A on Zero
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm1, PWM_ActionQual_Clear);    // Clear PWM1A on event A, up count

    PWM_setActionQual_Zero_PwmB(myPwm1, PWM_ActionQual_Set);            // Set PWM1B on Zero
    PWM_setActionQual_CntUp_CmpB_PwmB(myPwm1, PWM_ActionQual_Clear);    // Clear PWM1B on event B, up count

    // Interrupt where we will change the Compare Values
    PWM_setIntMode(myPwm1, PWM_IntMode_CounterEqualZero);   // Select INT on Zero event
    PWM_enableInt(myPwm1);                                  // Enable INT
    PWM_setIntPeriod(myPwm1, PWM_IntPeriod_ThirdEvent);     // Generate INT on 3rd event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA & CMPB
    epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP;
    epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
    epwm1_info.myPwmHandle = myPwm1;                // Set the pointer to the ePWM module
    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}

void main()
{
    volatile int status = 0;
    volatile FILE *fid;
    
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;
    
    // Initialize all the handles needed for this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    mySci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myPwm1 = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));

    // Perform basic system initialization    
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    CLK_disableAdcClock(myClk);
    
    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
    
    // Setup the PLL for x12 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);
    
    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);
        
    // If running from flash copy RAM only functions to RAM   
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif      

    // Enable XCLOCKOUT to allow monitoring of oscillator 1
    //GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
    //CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);   

    // Initialize SCIA
    scia_init();

    // Set the flash OTP wait-states to minimum.
    FLASH_setup(myFlash);

    // Initalize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
    
    int i;
    for (i = 0; i < 7; i++)
    {
    	//GPIO_setMode(myGpio, digit1Pins[i], GPIO_0_Mode_GeneralPurpose);
    	//GPIO_setDirection(myGpio, digit1Pins[i], GPIO_Direction_Output);
    	GPIO_setMode(myGpio, digit2Pins[i], GPIO_0_Mode_GeneralPurpose);
    	GPIO_setDirection(myGpio, digit2Pins[i], GPIO_Direction_Output);
    }

	GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Disable);
	//GPIO_setPullUp(myGpio, GPIO_Number_1, GPIO_PullUp_Disable);
	GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_EPWM1A);
	//GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_EPWM1B);

    GPIO_setMode(myGpio, upButton, GPIO_0_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, upButton, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, upButton, GPIO_PullUp_Disable);

    GPIO_setMode(myGpio, downButton, GPIO_0_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, downButton, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, downButton, GPIO_PullUp_Disable);

	// Register interrupt handlers in the PIE vector table
	PIE_registerPieIntHandler(myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_1, (intVec_t)&epwm1_isr);

	CLK_disableTbClockSync(myClk);
	InitEPwm1();
	CLK_enableTbClockSync(myClk);

	// Enable CPU INT3 which is connected to EPWM1-3 INT:
	CPU_enableInt(myCpu, CPU_IntNumber_3);

	// Enable EPWM INTn in the PIE: Group 3 interrupt 1
	PIE_enablePwmInt(myPie, PWM_Number_1);

	CPU_enableGlobalInts(myCpu);
	CPU_enableDebugInt(myCpu);
    
    //Redirect STDOUT to SCI
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);

    int currentPosition = 0;
    turn_motor_to(currentPosition);
    //printf("Starting");

    //Main program loop
    for(;;)
    {
    	//printf("ahoy!");
		if (GPIO_getData(myGpio, upButton) == 1)
		{
			printf("up %i, ", currentPosition);
			if (currentPosition < 9)
				turn_motor_to(++currentPosition);
		}
		else if (GPIO_getData(myGpio, downButton) == 1)
		{
			printf("down %i, ", currentPosition);
			if (currentPosition > 0)
				turn_motor_to(--currentPosition);
		}

		DELAY_US(75000);
    }
}




