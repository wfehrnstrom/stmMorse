#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "main.h"

#include "datalog_application.h"
#include "usbd_cdc_interface.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_COLLECTION_PERIOD_MS (20)
#define DATA_CHECK_PERIOD_MS (2)

#define F0 1
#define GAIN 1024
#define SAMPLING_FREQUENCY 1000/DATA_COLLECTION_PERIOD_MS
#define DOT 0
#define DASH 1
#define FILLER 2
#define ROTATION_ANGLE_ONE_MEASUREMENT 180;
#define RMS_GYRO_THRESHOLD 30 // degrees per second
#define RMS_GYRO_MIN_THRESHOLD 5 // degrees per second
#define ACC_THRESHOLD -950 // milliG's
#define MEASUREMENT_TIME 3000 // milliseconds
#define ASCII_SHIFT 97

//BEGIN DEFINITIONS OF CHARACTER SEQUENCES FOR MORSE CODE. The sequence of dots and dashes is transcribed onto a int, where each dash is a 1, and each dot is a 0
const unsigned int A[] = {DOT, DASH, FILLER, FILLER, FILLER};
const unsigned int B[] = {DASH, DOT, DOT, DOT, FILLER};
const unsigned int C[] = {DASH, DOT, DASH, DOT, FILLER};
const unsigned int D[] = {DASH, DOT, DOT, FILLER, FILLER};
const unsigned int E[] = {DOT, FILLER, FILLER, FILLER, FILLER};
const unsigned int F[] = {DOT, DOT, DASH, DOT, FILLER};
const unsigned int G[] = {DASH, DASH, DOT, FILLER, FILLER};
const unsigned int H[] = {DOT, DOT, DOT, DOT, FILLER};
const unsigned int I[] = {DOT, DOT, FILLER, FILLER, FILLER};
const unsigned int J[] = {DOT, DASH, DASH, DASH, FILLER};
const unsigned int K[] = {DASH, DOT, DASH, FILLER, FILLER};
const unsigned int L[] = {DOT, DASH, DOT, DOT, FILLER};
const unsigned int M[] = {DASH, DASH, FILLER, FILLER, FILLER};
const unsigned int N[] = {DASH, DOT, FILLER, FILLER, FILLER};
const unsigned int O[] = {DASH, DASH, DASH, FILLER, FILLER};
const unsigned int P[] = {DOT, DASH, DASH, DOT, FILLER};
const unsigned int Q[] = {DASH, DASH, DOT, DASH, FILLER};
const unsigned int R[] = {DOT, DASH, DOT, FILLER, FILLER};
const unsigned int S[] = {DOT, DOT, DOT, FILLER, FILLER};
const unsigned int T[] = {DASH, FILLER, FILLER, FILLER, FILLER};
const unsigned int U[] = {DOT, DOT, DASH, FILLER, FILLER};
const unsigned int V[] = {DOT, DOT, DOT, DASH, FILLER};
const unsigned int W[] = {DOT, DASH, DASH, FILLER, FILLER};
const unsigned int X[] = {DASH, DOT, DOT, DASH, FILLER};
const unsigned int Y[] = {DASH, DOT, DASH, DASH, FILLER};
const unsigned int Z[] = {DASH, DASH, DOT, DOT, FILLER};

const unsigned int letters[26][5] = {{DOT, DASH, FILLER, FILLER, FILLER}, {DASH, DOT, DOT, DOT, FILLER}, {DASH, DOT, DASH, DOT, FILLER}, {DASH, DOT, DOT, FILLER, FILLER},
    {DOT, FILLER, FILLER, FILLER, FILLER}, {DOT, DOT, DASH, DOT, FILLER}, {DASH, DASH, DOT, FILLER, FILLER}, {DOT, DOT, DOT, DOT, FILLER}, {DOT, DOT, FILLER, FILLER, FILLER},
    {DOT, DASH, DASH, DASH, FILLER}, {DASH, DOT, DASH, FILLER, FILLER}, {DOT, DASH, DOT, DOT, FILLER}, {DASH, DASH, FILLER, FILLER, FILLER}, {DASH, DOT, FILLER, FILLER, FILLER},
    {DASH, DASH, DASH, FILLER, FILLER}, {DOT, DASH, DASH, DOT, FILLER}, {DASH, DASH, DOT, DASH, FILLER}, {DOT, DASH, DOT, FILLER, FILLER}, {DOT, DOT, DOT, FILLER, FILLER},
    {DASH, FILLER, FILLER, FILLER, FILLER}, {DOT, DOT, DASH, FILLER, FILLER}, {DOT, DOT, DOT, DASH, FILLER}, {DOT, DASH, DASH, FILLER, FILLER}, {DASH, DOT, DOT, DASH, FILLER},
    {DASH, DOT, DASH, DASH, FILLER}, {DASH, DASH, DOT, DOT, FILLER}};


//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 0  --> Save sensors data on SDCard (enable with double click) */
/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;

USBD_HandleTypeDef  USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
static volatile uint8_t acquire_data_enable_request  = 1;
static volatile uint8_t acquire_data_disable_request = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;
static volatile uint8_t no_GG = 0;

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;
static void *GG_handle = NULL;

/* Private function prototypes -----------------------------------------------*/

static void Error_Handler( void );
static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void initializeAllSensors( void );


/* Private functions ---------------------------------------------------------*/
int sensorUpsideDown(int32_t acc_z){
    if(acc_z < ACC_THRESHOLD){
        return 1;
    }
    return 0;
}

uint32_t rmsLowPass(int32_t* data_millis){
    uint32_t sum = 0;
    int16_t IWon;
    int16_t c[3];
    int32_t prev_in, curr_in;
    int32_t data[50];
    IWon = (int)SAMPLING_FREQUENCY/(3.14159 * F0);
    c[0] = GAIN / (1.0f + IWon);
    c[1] = c[0];
    c[2] = c[0] * (1.0f - IWon);
    for (int i = 0; i < 50; i++) {
        if (i == 0) {
            prev_in = data_millis[0];
        } else {
            curr_in = data_millis[i];
            data_millis[i] = curr_in * c[0] + prev_in * c[1] - data_millis[i-1] * c[2];
            data_millis[i] = data_millis[i] / GAIN;
            prev_in = curr_in;
        }
        data[i] = data_millis[i] / 1000; // divide by 1000 to get number of degrees
    }
    // calculate RMS amplitude for the 50 samples
    sum = 0;
    for (int i = 0; i < 50; i++) {
        sum += data[i] * data[i];
    }
    sum = sum / 50;
    return sqrt(sum);
}

char getLetter(int* morseSequence){
    char dataOut[256];
    char c = 0;
    for(int i = 0; i < 26; i++){
        int letterMatch = 1;
        for(int j = 0; j < 5; j++){
            if(morseSequence[j] != letters[i][j]){
                letterMatch = 0;
                break;
            }
        }
        if(letterMatch){
            c = i + ASCII_SHIFT;
            break;
        }
    }
    sprintf(dataOut, "ASCII VALUE: %c", (char)c);
    CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
    return c;
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
    uint32_t msTick, msTickPrev = 0;
    
    /* STM32L4xx HAL library initialization:
     - Configure the Flash prefetch, instruction and Data caches
     - Configure the Systick to generate an interrupt each 1 msec
     - Set NVIC Group Priority to 4
     - Global MSP (MCU Support Package) initialization
     */
    
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    if(SendOverUSB)
    {
        /* Initialize LED */
        BSP_LED_Init(LED1);
        BSP_LED_On(LED1);
    }
#ifdef NOT_DEBUGGING
    else
    {
        /* Initialize LEDSWD: Cannot be used during debug because it overrides SWDCLK pin configuration */
        BSP_LED_Init(LEDSWD);
        BSP_LED_Off(LEDSWD);
    }
#endif
    
    /* Initialize RTC */
    RTC_Config();
    RTC_TimeStampConfig();
    
    /* enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();
    
    if(SendOverUSB) /* Configure the USB */
    {
        /*** USB CDC Configuration ***/
        /* Init Device Library */
        USBD_Init(&USBD_Device, &VCP_Desc, 0);
        /* Add Supported Class */
        USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
        /* Add Interface callbacks for AUDIO and CDC Class */
        USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
        /* Start Device Process */
        USBD_Start(&USBD_Device);
    }
    else /* Configure the SDCard */
    {
        DATALOG_SD_Init();
    }
    HAL_Delay(200);
    
    /* Configure and disable all the Chip Select pins */
    Sensor_IO_SPI_CS_Init_All();
    
    /* Initialize and Enable the available sensors */
    initializeAllSensors();
    enableAllSensors();
    
    // declare two arrays, one to hold raw gyro_x values, the other to hold gyro_x values in units of degrees
    int32_t gyro_x[50];
    int counter = 0; // a counter for the number of gyro samples we collect before we apply a low pass filter
    int nextCodeReady = 0; // boolean
    int sequenceIndex = 0; // stores our current index in the morse code input sequence
    char dataOut[256]; // an array to put serial characters in
    int userMorseSequence[10]; // the array into which we store our dots and dashes
    int sequenceOver = 0; //boolean checking whether our sequence is over
    
    while (1) // always run this loop forever
    {
        /* Get sysTick value and check if it's time to execute the task */
        msTick = HAL_GetTick();
        // if some time has elapsed since our last visit through here, and this time is an increment of DATA_CHECK_PERIOD_MS
        if(msTick % DATA_CHECK_PERIOD_MS == 0 && msTickPrev != msTick)
        {
            // if we are not yet ready to start sending a dot or dash respectively
            if(!nextCodeReady){
                // get the current acceleration in the z-direction
                int32_t acc_z = Accelero_Sensor_Handler(LSM6DSM_X_0_handle);
                // check if the sensor is being held upside down.  If it is, we're ready to receive a dot or dash
                nextCodeReady = sensorUpsideDown(acc_z);
                // Log that we are ready
                if(nextCodeReady){
                    sprintf(dataOut, "NEXT CODE READY\n\r");
                    CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                }
            }
            // if we are at a time that is an increment of DATA_COLLECTION_PERIOD_MS and we are not at the same time as our last stop through here
            if(msTick % (DATA_COLLECTION_PERIOD_MS) == 0 && msTickPrev != msTick){
                // set the previous tick to this tick
                msTickPrev = msTick;
                // if we are ready to receive a dot and dash and the entire dot and dash sequence is not over
                if(nextCodeReady && !sequenceOver){
                    // debugging info
                    if(counter == 0){
                        sprintf(dataOut, "Collecting Data....\n\r");
                        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                    }
                    // if we have less than 50 samples so far
                    if(counter < 50){
                        // grab a gyro value from the sensor tile
                        uint32_t gyro_meas = Gyro_Sensor_Handler(LSM6DSM_G_0_handle);
                        // set our array index at counter containing raw values to the raw gyro value just retrieved
                        gyro_x[counter] = gyro_meas;
                        // increment our counter so we correctly grab the next value
                        counter++;
                        // debugging info
                        if(counter > 45){
                            sprintf(dataOut, "COUNTER: %d\n\r", (int)counter);
                            CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                        }
                    }
                    // apply filter if 50 gyro_x data samples are is collected
                    if (counter >= 50) {
                        sprintf(dataOut, "TICK\n\r");
                        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                        // reset counter
                        counter = 0;
                        // apply low pass filter
                        uint32_t rms = rmsLowPass(gyro_x);
                        // if we have less than 5 dots and dashes
                        if(sequenceIndex < 5){
                            // print out what index we're at
                            sprintf(dataOut, "SEQUENCE INDEX: %d, ", (int)sequenceIndex);
                            CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                            // if the rms value we just got was greater than the threshold for a dash
                            if(rms > RMS_GYRO_THRESHOLD){
                                // we just detected a dash!
                                sprintf(dataOut, "DASH\n\r");
                                CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                userMorseSequence[sequenceIndex] = DASH;
                                sequenceIndex++;
                            }
                            // if the rms gyro value was greater than the minimum threshold, then we received a dot
                            else if(rms > RMS_GYRO_MIN_THRESHOLD){
                                // we just detected a dot!
                                sprintf(dataOut, "DOT\n\r");
                                CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                userMorseSequence[sequenceIndex] = DOT;
                                sequenceIndex++;
                            }
                            // if it's less than both of these, the user is holding the sensor tile steady.  This means that we want to terminate our sequence
                            else{
                                sprintf(dataOut, "SEQUENCE OVER\n\r");
                                CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                // if we have more than one dot or dash stored
                                if(sequenceIndex > 0){
                                    // set a flag indicating the sequence is over to true
                                    sequenceOver = 1;
                                    // if we have less than 5 dots and dashes
                                    if(sequenceIndex < 5){
                                        // put filler values in the rest of the user morse sequence array
                                        for(int i = sequenceIndex; i < 5; i++){
                                            userMorseSequence[i] = FILLER;
                                        }
                                        sprintf(dataOut, "\n\n\r*BEGIN SEQUENCE*\n\r");
                                        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                        // now print out our dots and dashes
                                        for(int i = 0 ; i < 5; i++){
                                            sprintf(dataOut, "%d\n\r", (int)userMorseSequence[i]);
                                            CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                        }
                                        sprintf(dataOut, "\n\n\r*END SEQUENCE*\n\r");
                                        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                                    }
                                    // get which letter we encoded with our dots and dashes
                                    getLetter(userMorseSequence);
                                    sequenceIndex = 0;
                                }
                            }
                        }
                        // display the RMS value by USB streaming
                        // fill dataOut buffer with display message
                        sprintf(dataOut, "\n\r\n\r\n\rRMS: %d\n\r", (int) rms);
                        // send message in buffer by USB streaming
                        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
                        // Set the boolean checking whether the next code is ready to false
                        nextCodeReady = 0;
                        // delay by half a second
                        HAL_Delay(500);
                    }
                }
            }
        }
        //HAL_Delay(1);
    }
    
    
    /* Go to Sleep */
    __WFI();
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{
    if (BSP_ACCELERO_Init( LSM6DSM_X_0, &LSM6DSM_X_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if (BSP_GYRO_Init( LSM6DSM_G_0, &LSM6DSM_G_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if (BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if (BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if (BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ) != COMPONENT_OK)
    {
        while(1);
    }
    
    if(BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle ) == COMPONENT_ERROR)
    {
        no_T_HTS221 = 1;
    }
    
    if(BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle ) == COMPONENT_ERROR)
    {
        no_H_HTS221 = 1;
    }
    
    /* Inialize the Gas Gauge if the battery is present */
    if(BSP_GG_Init(&GG_handle) == COMPONENT_ERROR)
    {
        no_GG=1;
    }
    
    //  if(!SendOverUSB)
    //  {
    //    /* Enable HW Double Tap detection */
    //    BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
    //    BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle, LSM6DSM_TAP_THRESHOLD_MID);
    //  }
    
    
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors( void )
{
    BSP_ACCELERO_Sensor_Enable( LSM6DSM_X_0_handle );
    BSP_GYRO_Sensor_Enable( LSM6DSM_G_0_handle );
    BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
    BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
    BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
    BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
    if(!no_T_HTS221)
    {
        BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
        BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
    }
    
}



/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
void disableAllSensors( void )
{
    BSP_ACCELERO_Sensor_Disable( LSM6DSM_X_0_handle );
    BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
    BSP_GYRO_Sensor_Disable( LSM6DSM_G_0_handle );
    BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
    BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
    BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
    BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
    BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config( void )
{
    /*##-1- Configure the RTC peripheral #######################################*/
    RtcHandle.Instance = RTC;
    
    /* Configure RTC prescaler and RTC data registers */
    /* RTC configured as follow:
     - Hour Format    = Format 12
     - Asynch Prediv  = Value according to source clock
     - Synch Prediv   = Value according to source clock
     - OutPut         = Output Disable
     - OutPutPolarity = High Polarity
     - OutPutType     = Open Drain */
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    
    if ( HAL_RTC_Init( &RtcHandle ) != HAL_OK )
    {
        
        /* Initialization Error */
        Error_Handler();
    }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig( void )
{
    
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;
    
    /*##-3- Configure the Date using BCD format ################################*/
    /* Set Date: Monday January 1st 2000 */
    sdatestructure.Year    = 0x00;
    sdatestructure.Month   = RTC_MONTH_JANUARY;
    sdatestructure.Date    = 0x01;
    sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
    
    if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
    {
        
        /* Initialization Error */
        Error_Handler();
    }
    
    /*##-4- Configure the Time using BCD format#################################*/
    /* Set Time: 00:00:00 */
    stimestructure.Hours          = 0x00;
    stimestructure.Minutes        = 0x00;
    stimestructure.Seconds        = 0x00;
    stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
    
    if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
    {   
        /* Initialization Error */
        Error_Handler();
    }
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{
    
    RTC_TimeTypeDef stimestructure;
    
    stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
    stimestructure.Hours          = hh;
    stimestructure.Minutes        = mm;
    stimestructure.Seconds        = ss;
    stimestructure.SubSeconds     = 0;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
    
    if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
    {
        /* Initialization Error */
        Error_Handler();
    }
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    MEMSInterrupt=1;
}



/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
static void Error_Handler( void )
{
    
    while (1)
    {}
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{
    
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    while (1)
    {}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
