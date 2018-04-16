/**
  ******************************************************************************
  * @file    LIS3DSH.c
  * @author  Ashraf Suyyagh based on the MCD Application Team implementation of the LIS3DSH driver
	           of both the standard peripheral library drivers and the STM32F4 Cube drivers
  * @version V 1.1.0
  * @date    17-January-2016
  * @brief   This file provides a set of functions needed to manage the LIS3DSH
  *          MEMS accelerometer available on STM32F4-Discovery Kit.
  ******************************************************************************
	* NOTES:
	* The accelerometer packaged driver in STM32Cube drivers is highly abstracted and uses 
	* five files (accelero.h, stm32f4_discovery_accelerometer.c/.h and lis3dsh/lis302dl.c/.h)
	* I have modified the drivers I written in previous years to be STM32F4Cube compatible
	* with minimum modifications to the API/Definitions
	******************************************************************************
	* LIST OF CHANGES 1.1.0 (17-January-2016)
	* Added an SPI handle declaration "SpiHandle"
	* Modified  the following functions to be compatible with the STM32CUBE driver HAL
	* 1. LIS3DSH_Init 
	* 2. HAL_SPI_MspInit
	* 3. LIS3DSH_SendByte
	* 4. SPI_SendData 
	* 5. SPI_ReceiveData
	* (Experimented with porting the new STM32Cube bundeled versions of HAL_SPI_TransmitReceive
     which are more thourough and have added functionality of setting SPI handle states, but
     I ran into several issues. Reverted back to basic SPI read/write from old drivers set for 
     now)
	******************************************************************************
	* BUGS/FIXES:
	* The new GPIO struct initialisation has new definitions for inputs used as external interrupt
	* sources, that is inputs are explicitly defined as input source with triggering edge. This is 
	* used to abstract the configuration of the pin as an EXTI source. The SPI initialisation for 
	* the driver sets PE0 and PE1 as INT1/INT2 for the LIS3DSH and configures them accordingly.
	* By default, PE0 is set as EXTI_Line0 source. It conflicts with the Push Button. A work around 
	* is either 
	  1. Modify the driver to remove the initialisation of PE0/PE1 (not recommended)
    2. Simply call HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0) AFTER initialsing the Accelerometer, this
  * resets PE0		
  */

/* Includes ------------------------------------------------------------------*/
#include "lis3dsh.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */

/** @addtogroup STM32F4_DISCOVERY_LIS3DSH
  * @{
  */


/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_Defines
  * @{
  */
__IO uint32_t  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;

 SPI_HandleTypeDef    SpiHandle;

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

#define USE_DEFAULT_TIMEOUT_CALLBACK

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_FunctionPrototypes
  * @{
  */
static uint8_t LIS3DSH_SendByte(uint8_t byte);
void SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t Data);
uint8_t SPI_ReceiveData(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/** @defgroup STM32F4_DISCOVERY_LIS3DSH_Private_Functions
  * @{
  */


/**
  * @brief  Set LIS3DSH Initialization.
  * @param  LIS3DSH_Config_Struct: pointer to a LIS3DSH_Config_TypeDef structure
  *         that contains the configuration setting for the LIS3DSH.
  * @retval None
  */
void LIS3DSH_Init(LIS3DSH_InitTypeDef *LIS3DSH_InitStruct)
{
  uint8_t ctrl = 0x00;

  /* Configure the low level interface ---------------------------------------*/
	  /* SPI configuration -------------------------------------------------------*/
	__HAL_RCC_SPI1_CLK_ENABLE();
	
  HAL_SPI_DeInit(&SpiHandle);
  SpiHandle.Instance 							  = SPI1;
  SpiHandle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_4;
  SpiHandle.Init.Direction 					= SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase 					= SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity 				= SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation			= SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial 			= 7;
  SpiHandle.Init.DataSize 					= SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit 					= SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS 								= SPI_NSS_SOFT;
  SpiHandle.Init.TIMode 						= SPI_TIMODE_DISABLED;
  SpiHandle.Init.Mode 							= SPI_MODE_MASTER;
	if (HAL_SPI_Init(&SpiHandle) != HAL_OK) {printf ("ERROR: Error in initialising SPI1 \n");};
  
	__HAL_SPI_ENABLE(&SpiHandle);
  
	/* Configure MEMS: data rate, update mode and axes */
  ctrl = (uint8_t) (LIS3DSH_InitStruct->Power_Mode_Output_DataRate | \
										LIS3DSH_InitStruct->Continous_Update           | \
										LIS3DSH_InitStruct->Axes_Enable);


  /* Write value to MEMS CTRL_REG4 regsister */
  LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG4, 1);

	/* Configure MEMS: Anti-aliasing filter, full scale, self test  */
	ctrl = (uint8_t) (LIS3DSH_InitStruct->AA_Filter_BW | \
										LIS3DSH_InitStruct->Full_Scale   | \
										LIS3DSH_InitStruct->Self_Test);

	/* Write value to MEMS CTRL_REG5 regsister */
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG5, 1);
}

/**
  * @brief Set LIS3DSH Interrupt configuration
  * @param  LIS3DSH_InterruptConfig_TypeDef: pointer to a LIS3DSH_InterruptConfig_TypeDef
  *         structure that contains the configuration setting for the LIS3DSH Interrupt.
  * @retval None
  */
void LIS3DSH_DataReadyInterruptConfig(LIS3DSH_DRYInterruptConfigTypeDef *LIS3DSH_IntConfigStruct)
{
  uint8_t ctrl = 0x00;

  /* Read CLICK_CFG register */
  LIS3DSH_Read(&ctrl, LIS3DSH_CTRL_REG3, 1);

  /* Configure latch Interrupt request, click interrupts and double click interrupts */
  ctrl = (uint8_t)(LIS3DSH_IntConfigStruct->Dataready_Interrupt| \
                   LIS3DSH_IntConfigStruct->Interrupt_signal | \
                   LIS3DSH_IntConfigStruct->Interrupt_type);

  /* Write value to MEMS CLICK_CFG register */
  LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG3, 1);
}

/**
  * @brief  Change to lowpower mode for LIS3DSH
  * @retval None
  */
void LIS3DSH_LowpowerCmd(void)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  LIS3DSH_Read(&tmpreg, LIS3DSH_CTRL_REG4, 1);

  /* Set new low power mode configuration */
  tmpreg &= (uint8_t)0x0F;
  tmpreg |= LIS3DSH_PWRDWN;

  /* Write value to MEMS CTRL_REG1 regsister */
  LIS3DSH_Write(&tmpreg, LIS3DSH_CTRL_REG4, 1);
}

/**
  * @brief  Data Rate command
  * @param  DataRateValue: Data rate value
  *   This parameter can be one of the following values:
  *     @arg LIS3DSH_DATARATE_3_125	: 3.125 Hz output data rate
  *     @arg LIS3DSH_DATARATE_6_25	: 6.25 	Hz output data rate
  *     @arg LIS3DSH_DATARATE_12_5	: 12.5	Hz output data rate
  *     @arg LIS3DSH_DATARATE_25		: 25 		Hz output data rate
  *     @arg LIS3DSH_DATARATE_50		: 50 		Hz output data rate
  *     @arg LIS3DSH_DATARATE_100		: 100 	Hz output data rate
  *     @arg LIS3DSH_DATARATE_400		: 400 	Hz output data rate
  *     @arg LIS3DSH_DATARATE_800		: 800 	Hz output data rate
  *     @arg LIS3DSH_DATARATE_1600	: 1600 	Hz output data rate


  * @retval None
  */
void LIS3DSH_DataRateCmd(uint8_t DataRateValue)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  LIS3DSH_Read(&tmpreg, LIS3DSH_CTRL_REG4, 1);

  /* Set new Data rate configuration */
  tmpreg &= (uint8_t)0x0F;
  tmpreg |= DataRateValue;

  /* Write value to MEMS CTRL_REG1 regsister */
  LIS3DSH_Write(&tmpreg, LIS3DSH_CTRL_REG4, 1);
}

/**
  * @brief  Change the Full Scale of LIS3DSH
  * @param  FS_value: new full scale value.
  *   This parameter can be one of the following values:
  *     @arg LIS3DSH_FULLSCALE_2	: +-2g
  *     @arg LIS3DSH_FULLSCALE_4	: +-4g
  *     @arg LIS3DSH_FULLSCALE_6	: +-6g
  *     @arg LIS3DSH_FULLSCALE_8	: +-8g
  *     @arg LIS3DSH_FULLSCALE_16	: +-16g
  * @retval None
  */
void LIS3DSH_FullScaleCmd(uint8_t FS_value)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  LIS3DSH_Read(&tmpreg, LIS3DSH_CTRL_REG5, 1);

  /* Set new full scale configuration */
  tmpreg &= (uint8_t)0xC7;
  tmpreg |= FS_value;

  /* Write value to MEMS CTRL_REG1 regsister */
  LIS3DSH_Write(&tmpreg, LIS3DSH_CTRL_REG5, 1);
}

/**
  * @brief  Writes one byte to the LIS3DSH.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the LIS3DSH.
  * @param  WriteAddr : LIS3DSH's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void LIS3DSH_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  LIS3DSH_CS_LOW();

  /* Send the Address of the indexed register */
  LIS3DSH_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    LIS3DSH_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  LIS3DSH_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the LIS3DSH.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LIS3DSH.
  * @param  ReadAddr : LIS3DSH's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the LIS3DSH.
  * @retval None
  */
void LIS3DSH_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  LIS3DSH_CS_LOW();

  /* Send the Address of the indexed register */
  LIS3DSH_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS3DSH (Slave device) */
    *pBuffer = LIS3DSH_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  LIS3DSH_CS_HIGH();
}

/**
  * @brief  Read LIS3DSH output register, and calculate the acceleration
  *         ACC[mg]=SENSITIVITY* (out_h*256+out_l)/16 (12 bit rappresentation)
  * @param  s16 buffer to store data
  * @retval None
  */
void LIS3DSH_ReadACC(float* out)
{
  uint8_t buffer[6];
  uint8_t ctrl, i = 0x00;
	uint8_t offsetX, offsetY, offsetZ;
	int16_t aggregateResult = 0;

  LIS3DSH_Read(&offsetX, LIS3DSH_OFF_X, 1);
  LIS3DSH_Read(&offsetY, LIS3DSH_OFF_Y, 1);
  LIS3DSH_Read(&offsetZ, LIS3DSH_OFF_Z, 1);

  LIS3DSH_Read(&ctrl, LIS3DSH_CTRL_REG5, 1);
	LIS3DSH_Read(&buffer[0], LIS3DSH_OUT_X_L, 1);
	LIS3DSH_Read(&buffer[1], LIS3DSH_OUT_X_H, 1);
	LIS3DSH_Read(&buffer[2], LIS3DSH_OUT_Y_L, 1);
	LIS3DSH_Read(&buffer[3], LIS3DSH_OUT_Y_H, 1);
	LIS3DSH_Read(&buffer[4], LIS3DSH_OUT_Z_L, 1);
	LIS3DSH_Read(&buffer[5], LIS3DSH_OUT_Z_H, 1);

	ctrl = (ctrl & 0x38) >> 3;

  switch(ctrl)
    {
    /* FS bits = 000 ==> Sensitivity typical value = 0.061 milligals/digit*/
    case 0x00:
      for(i=0; i<0x06; i=i+2)
      {
				aggregateResult = (int32_t)(buffer[i] | buffer[i+1] << 8);
        *out =(float)(LIS3DSH_SENSITIVITY_2G * (float)aggregateResult);
        out++;
      }
      break;

    /* FS bit = 001 ==> Sensitivity typical value = 0.122 milligals/digit*/
    case 0x01:
      for(i=0; i<0x06; i=i+2)
      {
				aggregateResult = (int32_t)(buffer[i] | buffer[i+1] << 8);
        *out =(float)(LIS3DSH_SENSITIVITY_4G * (float)aggregateResult);
        out++;
      }
      break;

		/* FS bit = 010 ==> Sensitivity typical value = 0.183 milligals/digit*/
    case 0x02:
      for(i=0; i<0x06; i=i+2)
      {
				aggregateResult = (int32_t)(buffer[i] | buffer[i+1] << 8);
        *out =(float)(LIS3DSH_SENSITIVITY_6G * (float)aggregateResult);
        out++;
      }
      break;

		 /* FS bit = 011 ==> Sensitivity typical value = 0.244 milligals/digit*/
    case 0x03:
      for(i=0; i<0x06; i=i+2)
      {
				aggregateResult = (int32_t)(buffer[i] | buffer[i+1] << 8);
        *out =(float)(LIS3DSH_SENSITIVITY_8G * (float)aggregateResult);
        out++;
      }
      break;

		/* FS bit = 100 ==> Sensitivity typical value = 0.488 milligals/digit*/
    case 0x04:
      for(i=0; i<0x06; i=i+2)
      {
				aggregateResult = (int32_t)(buffer[i] | buffer[i+1] << 8);
        *out =(float)(LIS3DSH_SENSITIVITY_16G * (float)aggregateResult);
        out++;
      }
      break;

    default:
      break;
    }
 }

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t LIS3DSH_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_TXE) == RESET)
  {
    if((LIS3DSHTimeout--) == 0) return LIS3DSH_TIMEOUT_UserCallback();
  }

  /* Send a Byte through the SPI peripheral */
  SPI_SendData(&SpiHandle,  byte);

  /* Wait to receive a Byte */
  LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_RXNE) == RESET)
  {
    if((LIS3DSHTimeout--) == 0) {
			return LIS3DSH_TIMEOUT_UserCallback();
		}
  }

  /* Return the Byte read from the SPI bus */ 
  return SPI_ReceiveData(&SpiHandle);
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS3DSH_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
//  while (1)
 // {
  //}
	return 0;
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){
	GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the SPI periph */
  __SPI1_CLK_ENABLE();

  /* Enable SCK, MOSI and MISO GPIO clocks */
  __GPIOA_CLK_ENABLE();

  /* Enable CS, INT1, INT2  GPIO clock */
  __GPIOE_CLK_ENABLE();

  //GPIO_PinAFConfig(LIS3DSH_SPI_SCK_GPIO_PORT, LIS3DSH_SPI_SCK_SOURCE, LIS3DSH_SPI_SCK_AF);
  //GPIO_PinAFConfig(LIS3DSH_SPI_MISO_GPIO_PORT, LIS3DSH_SPI_MISO_SOURCE, LIS3DSH_SPI_MISO_AF);
  //GPIO_PinAFConfig(LIS3DSH_SPI_MOSI_GPIO_PORT, LIS3DSH_SPI_MOSI_SOURCE, LIS3DSH_SPI_MOSI_AF);

  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.Pin = LIS3DSH_SPI_SCK_PIN;
  HAL_GPIO_Init(LIS3DSH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.Pin =  LIS3DSH_SPI_MOSI_PIN;
  HAL_GPIO_Init(LIS3DSH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.Pin = LIS3DSH_SPI_MISO_PIN;
  HAL_GPIO_Init(LIS3DSH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin   = LIS3DSH_SPI_CS_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LIS3DSH_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  HAL_GPIO_WritePin(LIS3DSH_SPI_CS_GPIO_PORT, LIS3DSH_SPI_CS_PIN, GPIO_PIN_SET);

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.Pin   = LIS3DSH_SPI_INT1_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LIS3DSH_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = LIS3DSH_SPI_INT2_PIN;
  HAL_GPIO_Init(LIS3DSH_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Transmits a Data through the SPIx/I2Sx peripheral.
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t Data)
{ 
  /* Write in the DR register the data to be sent */
  hspi->Instance->DR = Data;
}

/**
  * @brief  Returns the most recent received data by the SPIx/I2Sx peripheral. 
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @retval The value of the received data.
  */
uint8_t SPI_ReceiveData(SPI_HandleTypeDef *hspi)
{
  /* Return the data in the DR register */
  return hspi->Instance->DR;
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
