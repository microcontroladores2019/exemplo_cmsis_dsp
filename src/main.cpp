/* Includes */
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "pdm_filter.h" //PDM - PCM conversion filter library
#include "misc.h"
#include <math.h>

extern "C"{
#include "arm_math.h"	// arm_math.h is a C library implemented in C++, therefore it is necessary to
#include "FreeRTOS.h"	// declared it as an extern C labrary
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
}

//tasks
TaskHandle_t fft_task;				// task 1
TaskHandle_t PDM_Filter_task;		// task 2
TaskHandle_t ButtonReader_task; 	// task 3


#define fs 16000
#define DECIMATION 64
#define fft_Size 256
static q15_t PCM_Buffer[fft_Size];				// Buffer of PCM data before fft and storage
static q15_t fft_Buffer[fft_Size/2];			// Buffer of fft results
#define PDM_Buffer_Size DECIMATION*fft_Size
static q15_t PDM_Buffer[PDM_Buffer_Size]; 		// Buffer of PDM data before filtering

static uint8_t FilterEnable = 0;				// Enable PDM to PCM filtering

static PDMFilter_InitStruct Filter; 			// Audio Conditioning Filter
static uint16_t DataStorage[1000];  			// Maximum 4KBytes of data
static uint8_t AudioRecEnable = 0;  			// Audio Recording Disabled until communications protocols are 									 									  initialized
static  q15_t * PDM_index = PDM_Buffer;			// pointer to PDM_Buffer
static  q15_t * PCM_index = PCM_Buffer ;		// pointer to PCM_Buffer
static uint16_t * Data_index = DataStorage;
static uint16_t fftSize = 256;

#ifndef PDM_Filtering
#define PDM_Filtering
#endif

//instantaneous data buffer store size
//uint32_t CurrentDataSize = 0;


//configuring FFT DSP Arquitecture
arm_status MathOpStatus;
arm_cfft_radix4_instance_q15 fftConfigStruct;


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

void fftConfig(void)
{
	arm_status MathOpStatus;
	arm_cfft_radix4_instance_q15 fftConfigStruct; // creates a structure for performing a fft to a 16bit data sample
												  // PCM data from filters come in 16bit size

	uint8_t ifftFlag = 0 ; 	 //flag for forward or backward fft operation, set to 0 selects forward fft
	uint8_t bitReversal = 1; //flag for enableling bit reversal
	//uint16_t fftSize = 256;

	MathOpStatus = ARM_MATH_SUCCESS;

	//Initializing the FFT module
	//starting the fft configuration structure
	MathOpStatus = arm_cfft_radix4_init_q15(&fftConfigStruct,  fftSize, ifftFlag, bitReversal);

	if(MathOpStatus != ARM_MATH_SUCCESS)
	{
		//if math operations fail to start, all LEDS are turned on
		AudioRecEnable = 0;
		STM_EVAL_LEDOn(LED3);
		STM_EVAL_LEDOn(LED4);
		STM_EVAL_LEDOn(LED5);
		STM_EVAL_LEDOn(LED6);
	}

}

void AudioRecStart(void)
{
	//check for initialization of the communication protocols
	if(AudioRecEnable)
	{
		//enable the Rx buffer for not empty interrupt
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

		I2S_Cmd(SPI2, ENABLE);

	}
}

void AudioRecStop(void)
{
	//check for initialization of the communication protocols
	if(AudioRecEnable)
	{
		//Disabling the communication protocols stops recording
		I2S_Cmd(SPI2, DISABLE);
	}
}

//SP2 Interrupt Service Routine
void SPI2_IRQHandler(void)
{
	u16 measurement;

	//Checks for SPI-I2S non empty Rx and calls for interruption
	if(SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{
		int index = 0;
	    measurement = SPI_I2S_ReceiveData(SPI2);
	    PDM_index++;
	    index++;
	    *PDM_index = HTONS(measurement);		//memory adress pointed by PDM_index, PDM_Buffer[&PDM_index], receives value

	    if(index == PDM_Buffer_Size - 1)
	    {
	    	FilterEnable = 1;
	    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	    	//notification of the PDM filtering task coming from SPI2 Interrupt Service Routine
	    	xTaskNotifyFromISR(PDM_Filter_task, 0 , eNoAction, &xHigherPriorityTaskWoken);
	    	//interrupt returns directily to the highest priority task, witch is set to pdTrue in the line above
	    	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	    }
	}
}

//Button Interrupt Routine
extern "C" void EXTI0_IRQHandler()
{
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(ButtonReader_task, 0, eNoAction, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


//void taskCreator(void)


void Init_LEDS(void)
{
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
}

void LEDS_OFF()
{
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);
}

//task functions

void Store_Analyse(void *p)
{
	q15_t maxAmplitude;
	uint32_t peakIndex;

	for(;;)
	{

		arm_cfft_radix4_q15(& fftConfigStruct, PDM_Buffer);     						// process data, computing fft
		arm_cmplx_mag_q15(PCM_Buffer, fft_Buffer,(uint32_t) fft_Size); 							// computes complex amplitudes
		arm_max_q15(fft_Buffer, (uint32_t) fft_Size, &maxAmplitude, &peakIndex); 		// analyses amplidete array and sends maxAplitude
																						// value and maxAmplitude index
		if(peakIndex < 32)
		{
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDOff(LED6);
			STM_EVAL_LEDOn(LED3);
		}
		else if( peakIndex < 64)
			{
				STM_EVAL_LEDOff(LED3);
				STM_EVAL_LEDOff(LED5);
				STM_EVAL_LEDOff(LED6);
				STM_EVAL_LEDOn(LED4);
			}
		else if (peakIndex < 96)
		{
			STM_EVAL_LEDOff(LED3);
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED6);
			STM_EVAL_LEDOn(LED5);
		}

		else
		{
			STM_EVAL_LEDOff(LED3);
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDOn(LED6);
		}


		while(Data_index < DataStorage + fftSize)
		{
			PCM_index = PCM_Buffer;
			*Data_index = *PCM_index;
			Data_index ++;
			PCM_index ++;
		}
/*		for(int i = 0;  i< fft_Size; i++)
		{
			DataStorage[i + CurrentDataSize] = PCM_Buffer[i];
		}
		CurrentDataSize = CurrentDataSize + fft_Size;*/
	}
}

void PDM_Filter(void *p)
{
	for(;;)
	{
		if(FilterEnable)//for security, checks if the PDM data is ready for filtering, otherwise, it could filter trash data
		{
			u16 volume = 50;

			PDM_Filter_64_LSB((uint8_t *)PDM_Buffer,  (uint16_t *)PCM_Buffer, volume , (PDMFilter_InitStruct *)&Filter);
			FilterEnable = 0;
			xTaskNotify( fft_task, 0, eNoAction);
		}
	}
}

void ButtonReader(void *p)
{
	for( ;; )
	{
		AudioRecEnable = ~AudioRecEnable;
		if(AudioRecEnable)
		{
			//Enables LEDs at the start of recording
			Init_LEDS();
			AudioRecStart();
		}
		if(AudioRecEnable == 0)
		{
			//disables recording
			AudioRecStop();
			//turns LEDs off
			LEDS_OFF();
			//does not empty buffers, but sets to overwrite any previous stored data
			PDM_index = PDM_Buffer; 	//pointer back to PDM_Buffer[0]'s adress
			PCM_index = PCM_Buffer;
			Data_index = DataStorage;

		}
	}
}

int main(void)
{
	MP45DT02_config();
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI); //initialize Button for start and finish recording
	fftConfig();
	//taskCreator();

	xTaskCreate( 	Store_Analyse,
				( const char * ) "fft",
				configMINIMAL_STACK_SIZE,
				NULL,
				2,
				& fft_task );


		xTaskCreate( 	PDM_Filter,
				( const char * ) "pdm",
				configMINIMAL_STACK_SIZE,
				NULL ,
				configMAX_PRIORITIES - 1,
				& PDM_Filter_task );

		xTaskCreate( 	ButtonReader,
				( const char * ) "but",
				configMINIMAL_STACK_SIZE,
				NULL ,
				tskIDLE_PRIORITY,
				& ButtonReader_task );

		vTaskStartScheduler(); //creates scheduler for tasks defined above


  /* Infinite loop */

  while (1)
  {

  }
}

extern "C" void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
	return;
}

extern "C" uint16_t EVAL_AUDIO_GetSampleCallBack(void){
	return -1;
}

extern "C" void vApplicationTickHook( void ){
}

extern "C" void vApplicationMallocFailedHook( void ){
	for( ;; );
}

extern "C" void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ){
	( void ) pcTaskName;
	( void ) pxTask;
	for( ;; );
}

extern "C" void vApplicationIdleHook( void ){
	volatile size_t xFreeStackSpace;
	xFreeStackSpace = xPortGetFreeHeapSize();
}

