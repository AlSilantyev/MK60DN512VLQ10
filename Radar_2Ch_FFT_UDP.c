/**
 * @file    MK60DN512xxx10_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <stdlib.h>
#include "fsl_enet.h"
#include "fsl_phy.h"
#include "fsl_sysmpu.h"
#include "fsl_adc16.h"
#include "fsl_cmp.h"
#include "fsl_lptmr.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK60D10.h"
#include "arm_math.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "UDP_sender.h"
/* TODO: insert other include files here. */


 void Get_Vector();
 void Data_Process();
 void Jubula_Event();


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

uint16_t TimeToEvent = 50;
uint16_t ChanelA[2048];
uint16_t ChanelB[2048];
uint16_t capData[4096];
uint8_t numPack = 8;

float32_t Input[2048];
float32_t Output[4096];

uint32_t fftSize = 2048;  //1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_instance_f32 rfft_inst;
arm_cfft_radix4_instance_f32 cfft_inst;


lptmr_config_t lptmrConfig;
#define LPTMR_LED_HANDLER LPTMR0_IRQHandler
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
#define LPTMR_USEC_COUNT 200000U

cmp_config_t mCmpConfigStruct;
cmp_dac_config_t mCmpDacConfigStruct;
#define CMP_BASE CMP1
#define CMP_USER_CHANNEL 3U
#define CMP_DAC_CHANNEL 1U
#define CMP_IRQ_ID CMP1_IRQn
#define CMP_IRQ_HANDLER_FUNC CMP1_IRQHandler
volatile uint32_t g_CmpFlags = 0U;

adc16_config_t adc16_1ConfigStruct;
adc16_channel_config_t adc16_1ChannelConfigStruct;
#define ADC16_1_BASE ADC1
#define ADC16_1_CHANNEL_GROUP 0U
#define ADC16_1_USER_CHANNEL 0U

adc16_config_t adc16_0ConfigStruct;
adc16_channel_config_t adc16_0ChannelConfigStruct;
#define ADC16_0_BASE ADC0
#define ADC16_0_CHANNEL_GROUP 0U
#define ADC16_0_USER_CHANNEL 0U

/*******************************************************************************
 * Code
 ****************************************************************************/
int main(void) {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    SYSMPU_Enable(SYSMPU, false);
    ENET_Initialization();

    //GPIO_SetPinsOutput(PTB, 1u << 4);
    //GPIO_SetPinsOutput(PTB, 1u << 5);
    //GPIO_SetPinsOutput(PTB, 1u << 6);
    GPIO_SetPinsOutput(PTB, 1u << 7);

    ADC16_GetDefaultConfig(&adc16_1ConfigStruct);
    adc16_1ConfigStruct.enableHighSpeed = true;
    adc16_1ConfigStruct.enableLowPower = false;
    adc16_1ConfigStruct.clockSource = kADC16_ClockSourceAlt0;
    adc16_1ConfigStruct.enableAsynchronousClock = false;
    adc16_1ConfigStruct.clockDivider = kADC16_ClockDivider1;
    adc16_1ConfigStruct.resolution = kADC16_ResolutionDF13Bit;
    adc16_1ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
    adc16_1ConfigStruct.enableContinuousConversion = true;
    ADC16_Init(ADC16_1_BASE, &adc16_1ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC16_1_BASE, false); /* Make sure the software trigger is used. */
    ADC16_SetHardwareAverage(ADC16_1_BASE, kADC16_HardwareAverageDisabled);
    adc16_1ChannelConfigStruct.channelNumber = ADC16_1_USER_CHANNEL;
    adc16_1ChannelConfigStruct.enableDifferentialConversion = true;
    adc16_1ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

    ADC16_GetDefaultConfig(&adc16_0ConfigStruct);
    adc16_0ConfigStruct.enableHighSpeed = true;
    adc16_0ConfigStruct.enableLowPower = false;
    adc16_0ConfigStruct.clockSource = kADC16_ClockSourceAlt0;
    adc16_0ConfigStruct.enableAsynchronousClock = false;
    adc16_0ConfigStruct.clockDivider = kADC16_ClockDivider1;
    adc16_0ConfigStruct.resolution = kADC16_ResolutionDF13Bit;
    adc16_0ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
    adc16_0ConfigStruct.enableContinuousConversion = true;
    ADC16_Init(ADC16_0_BASE, &adc16_0ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC16_0_BASE, false); /* Make sure the software trigger is used. */
    ADC16_SetHardwareAverage(ADC16_0_BASE, kADC16_HardwareAverageDisabled);
    adc16_0ChannelConfigStruct.channelNumber = ADC16_0_USER_CHANNEL;
    adc16_0ChannelConfigStruct.enableDifferentialConversion = true;
    adc16_0ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

    ADC16_SetChannelConfig(ADC16_1_BASE, ADC16_1_CHANNEL_GROUP, &adc16_1ChannelConfigStruct);
    ADC16_SetChannelConfig(ADC16_0_BASE, ADC16_0_CHANNEL_GROUP, &adc16_0ChannelConfigStruct);

    /* Configure LPTMR */
     LPTMR_GetDefaultConfig(&lptmrConfig);
     LPTMR_Init(LPTMR0, &lptmrConfig);
     LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));
     LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
     EnableIRQ(LPTMR0_IRQn);

    /* Configure CMP */
     CMP_GetDefaultConfig(&mCmpConfigStruct);
     CMP_Init(CMP_BASE, &mCmpConfigStruct);
     mCmpDacConfigStruct.referenceVoltageSource = kCMP_VrefSourceVin2;
     mCmpDacConfigStruct.DACValue = 32U;
     CMP_SetDACConfig(CMP_BASE, &mCmpDacConfigStruct);
     CMP_SetInputChannels(CMP_BASE, CMP_USER_CHANNEL, CMP_DAC_CHANNEL);
     CMP_EnableInterrupts(CMP_BASE, kCMP_OutputRisingInterruptEnable | kCMP_OutputFallingInterruptEnable);
     EnableIRQ(CMP1_IRQn); //DEMO_CMP_IRQ_ID);

     // RFFT initialization
     arm_rfft_init_f32(&rfft_inst, &cfft_inst, fftSize, ifftFlag, doBitReverse);




     PRINTF("Hello World\n");

     for (uint16_t i=0;i<4096;i++){ capData[i] = i;}

     LPTMR_StartTimer(LPTMR0);

       while (1){}

    return 0 ;
}
//------------------------- Timer Interrupt -------------------------------------------------
void LPTMR_LED_HANDLER(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);

    if (TimeToEvent > 0)
    {
        TimeToEvent --;
        PRINTF("TimeToEvent %d\n", TimeToEvent);
    }
    else if(TimeToEvent == 0)
    {
        Jubula_Event();
    }

    __DSB();
    __ISB();
}
//--------------------------- CMP Interrupt --------------------------------------------------
void CMP_IRQ_HANDLER_FUNC(void)
{
    g_CmpFlags = CMP_GetStatusFlags(CMP_BASE);
    CMP_ClearStatusFlags(CMP_BASE, kCMP_OutputRisingEventFlag | kCMP_OutputFallingEventFlag);

    if (0U != (g_CmpFlags & kCMP_OutputRisingEventFlag))
    {
    	 TimeToEvent = 50;
    	 Jubula_Event();
    }
}

void Jubula_Event(void)
{
	 GPIO_SetPinsOutput(PTB, 1u << 3);
	 Get_Vector();
	 Data_Process();
	 Send_UDP(numPack, capData);
	 GPIO_ClearPinsOutput(PTB, 1u << 3);
}

void Get_Vector()
{
	 GPIO_SetPinsOutput(PTB, 1u << 4);
	 for(int i=0;i<2048;i++)
	 {
		while (0U == (kADC16_ChannelConversionDoneFlag & ADC16_GetChannelStatusFlags(ADC16_1_BASE, ADC16_1_CHANNEL_GROUP))){}
		ChanelB[i] = ADC16_GetChannelConversionValue(ADC16_1_BASE, ADC16_1_CHANNEL_GROUP);

		while (0U == (kADC16_ChannelConversionDoneFlag & ADC16_GetChannelStatusFlags(ADC16_0_BASE, ADC16_0_CHANNEL_GROUP))){}
		//ChanelA[i] = ADC16_GetChannelConversionValue(ADC16_0_BASE, ADC16_0_CHANNEL_GROUP);
		Input[i] = (float32_t)(ADC16_GetChannelConversionValue(ADC16_0_BASE, ADC16_0_CHANNEL_GROUP) - 2048);
	 }
	 GPIO_ClearPinsOutput(PTB, 1u << 4);
}

void Data_Process()
{
	GPIO_SetPinsOutput(PTB, 1u << 5);

	 arm_rfft_f32(&rfft_inst, Input, Output);
	 arm_cmplx_mag_f32(Output, Input, fftSize);

	 for (int i=0;i<1024; i++)
	 {
	    capData[i] = (uint16_t)(Input[i]);
	 }

	/*
	 for (int i=0;i<2048; i++)
	 {
		Input[i]= (float32_t)(ChanelB[i] -2048);
	 }

	 //arm_rfft_init_f32(&rfft_inst, &cfft_inst, fftSize, ifftFlag, doBitReverse);
	 arm_rfft_f32(&rfft_inst, Input, Output);
	 arm_cmplx_mag_f32(Output, Input, fftSize);


	 for (int i=0;i<1024; i++)
	 {
	    capData[i+1024] = (uint16_t)(Input[i]);
	 }
  */
	 GPIO_ClearPinsOutput(PTB, 1u << 5);
}

