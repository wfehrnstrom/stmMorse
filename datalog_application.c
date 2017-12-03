/**
  ******************************************************************************
  * @file    DataLog/Src/datalog_application.c
  * @author  Central Labs
  * @version V1.1.0
  * @date    27-Sept-2016
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "datalog_application.h"
#include "main.h"
#include "string.h"
#include "SensorTile.h"
#include <math.h>

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#define DATA_PERIOD_MS (100)

#define F0 1
#define GAIN 1024
#define SAMPLING_FREQUENCY 1000/DATA_PERIOD_MS
#define HIGH_PASS 1
#define LOW_PASS 2
#define DOT 0
#define DASH 1
#define ROTATION_ANGLE_ONE_MEASUREMENT =

const int morse_lengths[] = {2, 4, 4, 3, 1, 4, 3, 4, 2, 4, 3, 4, 2, 2, 3, 4, 4, 3, 3, 1, 3, 4, 3, 4, 4, 4};

//BEGIN DEFINITIONS OF CHARACTER SEQUENCES FOR MORSE CODE. The sequence of dots and dashes is transcribed onto a short, where each dash is a 1, and each dot is a 0
const unsigned short A[] = {DOT, DASH};
const unsigned short B[] = {DASH, DOT, DOT, DOT};
const unsigned short C[] = {DASH, DOT, DASH, DOT};
const unsigned short D[] = {DASH, DOT, DOT};
const unsigned short E[] = {DOT};
const unsigned short F[] = {DOT, DOT, DASH, DOT};
const unsigned short G[] = {DASH, DASH, DOT};
const unsigned short H[] = {DOT, DOT, DOT, DOT};
const unsigned short I[] = {DOT, DOT};
const unsigned short J[] = {DOT, DASH, DASH, DASH};
const unsigned short K[] = {DASH, DOT, DASH};
const unsigned short L[] = {DOT, DASH, DOT, DOT};
const unsigned short M[] = {DASH, DASH};
const unsigned short N[] = {DASH, DOT};
const unsigned short O[] = {DASH, DASH, DASH};
const unsigned short P[] = {DOT, DASH, DASH, DOT};
const unsigned short Q[] = {DASH, DASH, DOT, DASH};
const unsigned short R[] = {DOT, DASH, DOT};
const unsigned short S[] = {DOT, DOT, DOT};
const unsigned short T[] = {DASH};
const unsigned short U[] = {DOT, DOT, DASH};
const unsigned short V[] = {DOT, DOT, DOT, DASH};
const unsigned short W[] = {DOT, DASH, DASH};
const unsigned short X[] = {DASH, DOT, DOT, DASH};
const unsigned short Y[] = {DASH, DOT, DASH, DASH};
const unsigned short Z[] = {DASH, DASH, DOT, DOT};

const unsigned short LETTERS[26][5] = {A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z};

FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

volatile uint8_t SD_Log_Enabled = 0;
static uint8_t verbose = 0;  /* Verbose output to UART terminal ON/OFF. */

static char dataOut[256];
char newLine[] = "\r\n";

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  char SDPath[4];

  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  char header[] = "Timestamp\tAccX [mg]\tAccY [mg]\tAccZ [mg]\tGyroX [mdps]\tGyroY [mdps]\tGyroZ [mdps]\tMagX [mgauss]\tMagY [mgauss]\tMagZ [mgauss]\tP [mB]\tT [�C]\tH [%]\tVOL [mV]\tBAT [%]\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30] = {0};

  /* SD SPI CS Config */
  SD_IO_CS_Init();

  sprintf(file_name, "%s%.3d%s", "SensorTile_Log_N", sdcard_file_counter, ".tsv");
  sdcard_file_counter++;

  HAL_Delay(100);

  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 0;
  }

  if(f_write(&MyFile, (const void*)&header, sizeof(header)-1, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  f_close(&MyFile);

  /* SD SPI Config */
  SD_IO_CS_DeInit();
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&MyFile, (const void*)&newLine, 2, (void *)&byteswritten);
}

/**
* @brief  Handles the time+date getting/sending
* @param  None
* @retval None
*/
void RTC_Handler( RTC_HandleTypeDef *RtcHandle )
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime( RtcHandle, &stimestructure, FORMAT_BIN );
  HAL_RTC_GetDate( RtcHandle, &sdatestructureget, FORMAT_BIN );
  subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) / ( RTC_SYNCH_PREDIV + 1 )) & 0xff );

  if(SendOverUSB) /* Write data on the USB */
  {
    sprintf( dataOut, "\nTimeStamp: %02d:%02d:%02d.%02d", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, subSec );
    CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
  }
  else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
  {
    uint8_t size;
    size = sprintf( dataOut, "%02d:%02d:%02d.%02d\t", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, subSec);
    res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
  }
}



/**
* @brief  Handles the accelerometer axes data getting/sending
* @param  handle the device handle
* @retval None
*/
void Accelero_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;
  int32_t d1, d2;

  BSP_ACCELERO_Get_Instance( handle, &id );

  BSP_ACCELERO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
      acceleration.AXIS_X = 0;
      acceleration.AXIS_Y = 0;
      acceleration.AXIS_Z = 0;
    }

    if(SendOverUSB) /* Write data on the USB */
    {
      sprintf( dataOut, "\n\rACC_X: %d, ACC_Y: %d, ACC_Z: %d", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z );
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

      if ( verbose == 1 )
      {
        if ( BSP_ACCELERO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "WHO AM I address[%d]: ERROR\n", id );
        }
        else
        {
          sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\n", id, who_am_i );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_ACCELERO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "ODR[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( odr, &d1, &d2, 3 );
          sprintf( dataOut, "ODR[%d]: %d.%03d Hz\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_ACCELERO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "FS[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( fullScale, &d1, &d2, 3 );
          sprintf( dataOut, "FS[%d]: %d.%03d g\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
      }
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf(dataOut, "%d\t%d\t%d\t", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}



/**
* @brief  Handles the gyroscope axes data getting/sending
* @param  handle the device handle
* @retval None
*/
void Gyro_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t angular_velocity;
  uint8_t status;
  int32_t d1, d2;

  BSP_GYRO_Get_Instance( handle, &id );

  BSP_GYRO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }
    else{
		if(SendOverUSB) /* Write data on the USB */
		{
		  // Low pass filter initialization and assignment
		  int16_t IWon;
		  int16_t c[3];
		  IWon = (int) SAMPLING_FREQUENCY / (3.14159 * F0);
		  c[0] = GAIN / (1.0f + IWon);
		  c[1] = c[0];
		  c[2] = c[0] * (1.0f - IWon);

		  float angular_mag = sqrt(pow(angular_velocity.AXIS_X, 2) + pow(angular_velocity.AXIS_Y, 2) + pow(angular_velocity.AXIS_Z, 2));
		  sprintf( dataOut, "\n\rGYR_X: %d, GYR_Y: %d, GYR_Z: %d MAG: %d", (int)angular_velocity.AXIS_X, (int)angular_velocity.AXIS_Y, (int)angular_velocity.AXIS_Z , (int)angular_mag);
		  CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
		}
    }
  }
}



/**
* @brief  Handles the magneto axes data getting/sending
* @param  handle the device handle
* @retval None
*/
void Magneto_Sensor_Handler( void *handle )
{
  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t magnetic_field;
  uint8_t status;
  int32_t d1, d2;

  BSP_MAGNETO_Get_Instance( handle, &id );

  BSP_MAGNETO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }

    if(SendOverUSB) /* Write data on the USB */
    {
      sprintf( dataOut, "\n\rMAG_X: %d, MAG_Y: %d, MAG_Z: %d", (int)magnetic_field.AXIS_X, (int)magnetic_field.AXIS_Y, (int)magnetic_field.AXIS_Z );
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

      if ( verbose == 1 )
      {
        if ( BSP_MAGNETO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "WHO AM I address[%d]: ERROR\n", id );
        }
        else
        {
          sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\n", id, who_am_i );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_MAGNETO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "ODR[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( odr, &d1, &d2, 3 );
          sprintf( dataOut, "ODR[%d]: %d.%03d Hz\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_MAGNETO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "FS[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( fullScale, &d1, &d2, 3 );
          sprintf( dataOut, "FS[%d]: %d.%03d Gauss\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
      }
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf(dataOut, "%d\t%d\t%d\t", (int)magnetic_field.AXIS_X, (int)magnetic_field.AXIS_Y, (int)magnetic_field.AXIS_Z);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}


/**
* @brief  Handles the humidity data getting/sending
* @param  handle the device handle
* @retval None
*/
void Humidity_Sensor_Handler( void *handle )
{
  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float humidity;
  uint8_t status;

  BSP_HUMIDITY_Get_Instance( handle, &id );

  BSP_HUMIDITY_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
    {
      humidity = 0.0f;
    }

    floatToInt( humidity, &d1, &d2, 2 );

    if(SendOverUSB) /* Write data on the USB */
    {
      sprintf( dataOut, "\n\rHUM: %d.%02d", (int)d1, (int)d2 );
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

      if ( verbose == 1 )
      {
        if ( BSP_HUMIDITY_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "WHO AM I address[%d]: ERROR\n", id );
        }
        else
        {
          sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\n", id, who_am_i );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_HUMIDITY_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "ODR[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( odr, &d1, &d2, 3 );
          sprintf( dataOut, "ODR[%d]: %d.%03d Hz\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
      }
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf( dataOut, "%5.2f\t", humidity);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}


/**
* @brief  Handles the temperature data getting/sending
* @param  handle the device handle
* @retval None
*/
void Temperature_Sensor_Handler( void *handle )
{

  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float temperature;
  uint8_t status;

  BSP_TEMPERATURE_Get_Instance( handle, &id );

  BSP_TEMPERATURE_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
    {
      temperature = 0.0f;
    }

    floatToInt( temperature, &d1, &d2, 2 );

    if(SendOverUSB) /* Write data on the USB */
    {
      sprintf( dataOut, "\n\rTEMP: %d.%02d", (int)d1, (int)d2 );
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

      if ( verbose == 1 )
      {
        if ( BSP_TEMPERATURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "WHO AM I address[%d]: ERROR\n", id );
        }
        else
        {
          sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\n", id, who_am_i );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_TEMPERATURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "ODR[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( odr, &d1, &d2, 3 );
          sprintf( dataOut, "ODR[%d]: %d.%03d Hz\n", (int)id, (int)d1, (int)d2 );
        }

        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
      }
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf( dataOut, "%3.1f\t", temperature);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}



/**
* @brief  Handles the pressure sensor data getting/sending
* @param  handle the device handle
* @retval None
*/
void Pressure_Sensor_Handler( void *handle )
{
  int32_t d1, d2;
  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float pressure;
  uint8_t status;

  BSP_PRESSURE_Get_Instance( handle, &id );

  BSP_PRESSURE_IsInitialized( handle, &status );

  if( status == 1 )
  {
    if ( BSP_PRESSURE_Get_Press( handle, &pressure ) == COMPONENT_ERROR )
    {
      pressure = 0.0f;
    }

    floatToInt( pressure, &d1, &d2, 2 );

    if(SendOverUSB)
    {
      sprintf(dataOut, "\n\rPRESS: %d.%02d", (int)d1, (int)d2);
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

      if ( verbose == 1 )
      {
        if ( BSP_PRESSURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "WHO AM I address[%d]: ERROR\n", id );
        }
        else
        {
          sprintf( dataOut, "WHO AM I address[%d]: 0x%02X\n", id, who_am_i );
        }
        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));

        if ( BSP_PRESSURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
        {
          sprintf( dataOut, "ODR[%d]: ERROR\n", id );
        }
        else
        {
          floatToInt( odr, &d1, &d2, 3 );
          sprintf( dataOut, "ODR[%d]: %d.%03d Hz\n", (int)id, (int)d1, (int)d2 );
        }
        CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
      }
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf( dataOut, "%5.2f\t", pressure);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}

void Gas_Gauge_Handler( void *handle )
{
  uint32_t voltage, soc;
  uint8_t vmode, status;

  BSP_GG_IsInitialized(handle, &status );

  if ( status == 1 )
  {
    /* Update Gas Gauge Status */
    if (BSP_GG_Task(handle, &vmode)== COMPONENT_ERROR )
    {
      voltage= 0.0f;
      soc= 0.0f;
    }

    /* Read the Gas Gauge Status */
    if ( BSP_GG_GetVoltage(handle, &voltage) == COMPONENT_ERROR )
    {
      voltage = 0.0f;
    }

    if ( BSP_GG_GetSOC(handle, &soc) == COMPONENT_ERROR )
    {
      soc= 0.0f;
    }

    if(SendOverUSB) /* Write data on the USB */
    {
      sprintf( dataOut, "\n\rV: %dmV Chrg: %d%%", (uint32_t)voltage, (uint32_t)soc);
      CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
    }
    else if(SD_Log_Enabled) /* Write data to the file on the SDCard */
    {
      uint8_t size;
      size = sprintf( dataOut, "%d\t%d\t", (uint32_t)voltage, (uint32_t)soc);
      res = f_write(&MyFile, dataOut, size, (void *)&byteswritten);
    }
  }
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
