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
#include "ti_ascii.h"

#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/sci_io.h"
#include "f2802x_common/include/wdog.h"

extern void DSP28x_usDelay(Uint32 Count);

int16_t referenceTemp;
int16_t currentTemp;

ADC_Handle myAdc;
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;
SCI_Handle mySci;

uint8_t digit[] = {0x7E, 0x30, 0x60, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};

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

//Take a number 0-99 and displays it on the LCD
void display_on_LCD(int number)
{
	if (number < 0 || number > 99)
		return;

	int tens = number / 10;
	int ones = number % 10;
	uint8_t encodedTen = digit[tens];
	uint8_t encodedOne = digit[ones];

	int counter = 0;
	uint8_t bit;
	//01000000 because encoding is 7 bits long
	for (bit = 0x40; bit; bit >>= 1)
	{
		//bit mask down the line starting from high bits
		switch (counter)
		{
		case 0:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_0);
			else
				GPIO_setLow(myGpio, GPIO_Number_0);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_7);
			else
				GPIO_setLow(myGpio, GPIO_Number_7);
			break;
		case 1:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_1);
			else
				GPIO_setLow(myGpio, GPIO_Number_1);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_12);
			else
				GPIO_setLow(myGpio, GPIO_Number_12);
			break;
		case 2:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_2);
			else
				GPIO_setLow(myGpio, GPIO_Number_2);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_16);
			else
				GPIO_setLow(myGpio, GPIO_Number_16);
			break;
		case 3:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_3);
			else
				GPIO_setLow(myGpio, GPIO_Number_3);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_17);
			else
				GPIO_setLow(myGpio, GPIO_Number_17);
			break;
		case 4:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_4);
			else
				GPIO_setLow(myGpio, GPIO_Number_4);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_18);
			else
				GPIO_setLow(myGpio, GPIO_Number_18);
			break;
		case 5:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_5);
			else
				GPIO_setLow(myGpio, GPIO_Number_5);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_19);
			else
				GPIO_setLow(myGpio, GPIO_Number_19);
			break;
		case 6:
			if (bit & encodedTen == 1)
				GPIO_setHigh(myGpio, GPIO_Number_6);
			else
				GPIO_setLow(myGpio, GPIO_Number_6);
			if (bit & encodedOne == 1)
				GPIO_setHigh(myGpio, GPIO_Number_28);
			else
				GPIO_setLow(myGpio, GPIO_Number_28);
			break;

		}
		counter++;
	}
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

    // Perform basic system initialization    
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    
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

    // Initalize GPIO
    // Enable XCLOCKOUT to allow monitoring of oscillator 1
    GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
    CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);

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
    
    // Configure GPIO 0-6 as outputs, 10's digit
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_4, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_5, GPIO_0_Mode_GeneralPurpose);
    GPIO_setMode(myGpio, GPIO_Number_6, GPIO_0_Mode_GeneralPurpose);

    //7, 12, 16-19, 28 for 1's digit
    GPIO_setMode(myGpio, GPIO_Number_7, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_12, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_16, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_17, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_18, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_19, GPIO_0_Mode_GeneralPurpose);
	GPIO_setMode(myGpio, GPIO_Number_28, GPIO_0_Mode_GeneralPurpose);
    
    GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Output);
    GPIO_setDirection(myGpio, GPIO_Number_6, GPIO_Direction_Output);
    
    GPIO_setDirection(myGpio, GPIO_Number_7, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_16, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_17, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_18, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_19, GPIO_Direction_Output);
	GPIO_setDirection(myGpio, GPIO_Number_28, GPIO_Direction_Output);
/*
    GPIO_setMode(myGpio, GPIO_Number_12, GPIO_12_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, GPIO_Number_12, GPIO_PullUp_Disable);*/
    
    //Redirect STDOUT to SCI
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);

        
    //Scan the LEDs until the pushbutton is pressed
    while(GPIO_getData(myGpio, GPIO_Number_12) != 1)
    {       
        GPIO_setHigh(myGpio, GPIO_Number_0);
        GPIO_setHigh(myGpio, GPIO_Number_1);
        GPIO_setHigh(myGpio, GPIO_Number_2);
        GPIO_setLow(myGpio, GPIO_Number_3);
        DELAY_US(50000);

        GPIO_setHigh(myGpio, GPIO_Number_0);
        GPIO_setHigh(myGpio, GPIO_Number_1);
        GPIO_setLow(myGpio, GPIO_Number_2);
        GPIO_setHigh(myGpio, GPIO_Number_3);
        DELAY_US(50000);

        GPIO_setHigh(myGpio, GPIO_Number_0);
        GPIO_setLow(myGpio, GPIO_Number_1);
        GPIO_setHigh(myGpio, GPIO_Number_2);
        GPIO_setHigh(myGpio, GPIO_Number_3);
        DELAY_US(50000);

        GPIO_setLow(myGpio, GPIO_Number_0);
        GPIO_setHigh(myGpio, GPIO_Number_1);
        GPIO_setHigh(myGpio, GPIO_Number_2);
        GPIO_setHigh(myGpio, GPIO_Number_3);
        DELAY_US(500000);
    }

    //Main program loop
    for(;;) {
    	int numberToDisplay = 0;
        
        if(GPIO_getData(myGpio, GPIO_Number_12) == 1) {
        	display_on_LCD(numberToDisplay++);
            DELAY_US(500000);
        }
        
    }
}




