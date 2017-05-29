/* Includes */
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "pdm_filter.h" //PDM - PCM conversion filter library
#include "misc.h"
#include <math.h>
//#include "arm_math.h" // fft; falta linkar a biblioteca arm_cortexM4lf_math.lib

extern "C"{
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
}

//tasks
TaskHandle_t fft_task;				// task 1
TaskHandle_t StoreData_task; 		// task 2
TaskHandle_t PDM_Filter_task;		// task 3
TaskHandle_t ButtonReader_task; 	// task 4


#define fs 16000
#define DECIMATION 64
#define fft_Size 256
static uint16_t PCM_Buffer[fft_Size];			// Buffer of PCM data before fft and storage
#define PDM_Buffer_Size DECIMATION*fft_Size
//static uint16_t PDM_Buffer[PDM_Buffer_size]; 	// Buffer of PDM data before filtering
static uint32_t PDM_Buffer_index = 0;
//static uint32_t FilterEnable = 0;				// Enable PDM to PCM filtering
static uint32_t ButRecStatus = 0;				// Recording Status, not recording by default

static uint8_t AudioRecEnable = 0; // Audio Recording Disabled until communications protocols are 									 									  initialized
static PDMFilter_InitStruct Filter; // Audio Conditioning Filter
static uint16_t DataStorage[1000];  // Maximum 4KBytes of data
static uint32_t AudioRecEnable = 0; // Audio Recording Disabled until communications protocols are 									 									  initialized
static PDMFilter_InitStruct Filter; // Audio Conditioning Filter

#ifndef PDM_Filtering
#define PDM_Filtering

struct PDM_Filtering_struct
{
	uint32_t FilterEnable = 0;				// Enable PDM to PCM filtering
	uint16_t PDM_Buffer[PDM_Buffer_size]; 	// Buffer of PDM data before filtering
};

#endif

//instantaneous data buffer adress
uint16_t* pDataBuffer_CurrentAdress;

//instantaneous data buffer store size
uint32_t CurrentDataSize = 0;


void MP45DT02_config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE); //	clock enabled before the PDM Filter
														//	function is called

	//configuring PDM Conditioning Audio Filter
	Filter.Fs = fs;
	Filter.LP_HZ = 10; //frequency bellow human audition capacities (20 Hz)
	Filter.HP_HZ = 22000; //frequency above human audition capacities (20 kHz)
	Filter.Out_MicChannels = 1; //defines the number of microphones in the output stream
	Filter.In_MicChannels = 1;
	PDM_Filter_Init(&Filter);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC , ENABLE);

	//configuring MP45DT02' clock: GPIOB Pin10
	GPIO_InitTypeDef Pins;
	Pins.GPIO_Pin = GPIO_Pin_10;
	Pins.GPIO_OType = GPIO_OType_PP;
	Pins.GPIO_Speed = GPIO_Speed_50MHz; //MP45DT02 DataSheet pg 7: max clock freq : 3.25MHz
	Pins.GPIO_Mode = GPIO_Mode_AF; //configure GPIOB to use the MP45DT02 module, Alternate 									 									 Function
	GPIO_Init(GPIOB, &Pins);

	//configuring MP45DT02' DOUT: GPIOC Pin3
	Pins.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &Pins);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2); // link SPI2 and

	//configuring Nested Vector Interrupt Channel
	//interruption for SPI2,
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannel = SPI2_IRQn; //
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);


    //Enable SPI clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    //I2S2 configuration
    I2S_InitTypeDef i2s;

    SPI_I2S_DeInit(SPI2);
    i2s.I2S_AudioFreq = fs*DECIMATION; // fs*decimation; fs = 16k, decimation = 64
    i2s.I2S_Standard = I2S_Standard_LSB; //specifies standart use of I2S communication
    i2s.I2S_DataFormat = I2S_DataFormat_16b; //interruption after 16 bit array is aquired
    i2s.I2S_CPOL = I2S_CPOL_High; //clock polarity high : samples taken for high level clock
    i2s.I2S_Mode = I2S_Mode_MasterRx;
    i2s.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    I2S_Init(SPI2, &i2s);
    I2S_Cmd(SPI2, ENABLE);

    //enable the Rx buffer for not empty interrupt ( SPI_I2S_IT_RXNE : IT-> interrupt, RXNE-> Rx 																						Not Empty
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

    AudioRecEnable = 1;
}

uint8_t AudioRecStart(uint16_t* pbuf, uint32_t size)
{
	//check for initialization of the communication protocols
	if(AudioRecEnable)
	{
		//memory location on buffer and size of data stored in the moment
		pDataBuffer_CurrentAdress = pbuf;
		CurrentDataSize = size;
		return 1;

		//enable the Rx buffer for not empty interrupt
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

		I2S_Cmd(SPI2, ENABLE);

	}
	else
	{
		//if audio recording is disabled
		return 1;
	}
}

uint32_t AudioRecStop(void)
{
	//check for initialization of the communication protocols
	if(AudioRecEnable)
	{
		//Disabling the communication protocols stops recording
		I2S_Cmd(SPI2, ENABLE);

		return 0;
	}
	else
	{
		return 1;
	}
}

void SPI2_IRQHandler(void) //SP2 Interrupt Service Routine
{
	u16 measurement;

	//Checks for SPI data flags
	if(SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{
	    measurement = SPI_I2S_ReceiveData(SPI2);
	    PDM_Buffer_index++;
	    PDM_Buffer[PDM_Buffer_index] = HTONS(measurement);

	    if(PDM_Buffer_index == PDM_Buffer_Size - 1)
	    {
	    	FilterEnable = 1;
	    }
	}
}


void taskCreator(void)
{

	xTaskCreate( 	SpectrumAnalyser,
			( const char * ) "fft",
			configMINIMAL_STACK_SIZE,
			PCM_Buffer, 								//input Buffer
			2,											//priority
			& fft_task );

	xTaskCreate( 	StoreData,
			( const char * ) "data",
			configMINIMAL_STACK_SIZE,
			PCM_Buffer,
			3,
			& StoreData_task );

	xTaskCreate( 	PDM_Filter,
			( const char * ) "pdm",
			configMINIMAL_STACK_SIZE,
			PDM_Buffer,
			configMAX_PRIORITIES - 1,
			& PDM_Filter_task );

	xTaskCreate( 	ButtonReader,
			( const char * ) "but",
			configMINIMAL_STACK_SIZE,
			AudioRecEnable ,
			tskIDLE_PRIORITY,
			& ButtonReader_task );

	vTaskStartScheduler(); //creates scheduler for tasks defined above
}

int main(void)
{
	MP45DT02_config();
	Init_LEDS();
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); //initialize Button for start and finish recording
	taskCreator();


  /* Infinite loop */

  while (1)
  {

  }
}
//task functions

void SpectrumAnalyser()
{


	vTaskDelete( NULL );
}

void StoreData()
{


	vTaskDelete( NULL );
}

void PDM_Filter()
{


	vTaskDelete( NULL );
}

void ButtonReader(void AudioRecEnable)
{

	u8 button;
	button = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
	if(button)
	{
		AudioRecEnable = ~AudioRecEnable;
	}
	if(AudioRecEnable == 0)
	{
		//clear buffers
	}
	vTaskDelete( NULL );
}

void Init_LEDS(void)
{
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
}

