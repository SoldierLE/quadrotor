/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "I2C.h"
#include "stm32f37x.h"

/* I2C TIMING Register define when I2C clock source is SYSCLK */
#define   SENSOR_I2C_TIMING  0x00200404//344us

/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define FLAG_TIMEOUT         ((uint32_t)0x1000)
#define LONG_TIMEOUT         ((uint32_t)(20 * FLAG_TIMEOUT))

volatile uint32_t  _time_out = LONG_TIMEOUT;


I2C_StatusTypeDef i2c_timeout_callback(I2C_TypeDef* I2Cx)
{
    I2Cx->CR1|=I2C_CR1_SWRST;
    return I2C_TIMEOUT;
}

/*********************************************************
函数名: void imu_i2c_init(void)
描  述: I2C初始化
输入值: 无
输出值: 无
返回值: 无
**********************************************************/
void imu_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    /*!<  Periph clock enable */
    RCC_APB1PeriphClockCmd(IMU_I2C_CLK, ENABLE);

    /*!< I2C_SCL_GPIO_CLK, I2C_SDA_GPIO_CLK Periph clock enable */
    RCC_AHBPeriphClockCmd(IMU_I2C_SCL_GPIO_CLK | IMU_I2C_SDA_GPIO_CLK , ENABLE);

    /* Connect PXx to I2C_SCL */
    GPIO_PinAFConfig(IMU_I2C_SCL_GPIO_PORT, IMU_I2C_SCL_SOURCE, IMU_I2C_SCL_AF);

    /* Connect PXx to I2C_SDA */
    GPIO_PinAFConfig(IMU_I2C_SDA_GPIO_PORT, IMU_I2C_SDA_SOURCE, IMU_I2C_SDA_AF);

    /*!< Configure SENSOR_I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = IMU_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(IMU_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure SENSOR_I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = IMU_I2C_SDA_PIN;
    GPIO_Init(IMU_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

    //config I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Timing = SENSOR_I2C_TIMING;      /*!< Specifies the I2C_TIMINGR_register value.
                                                          // This parameter must be set by referring to I2C_Timing_Config_Tool*/
    /* Apply I2C configuration after enabling it */
    I2C_Init(IMU_I2C , &I2C_InitStructure);
    /* I2C  Peripheral Enable */
    I2C_Cmd(IMU_I2C , ENABLE);
}
/****************************************************************************************
函数名: I2C_StatusTypeDef i2c_reg_write(uint8_t txData,uint8_t deviceAdd,uint8_t regAdd) 
描  述: i2c写
输入值: 
* @brief  Write one value in a register of the device through the bus.
* @param  buf: the pointer to data to be written.
* @param  deviceAdd: the device address on bus.
* @param  regAdd: the target register address to be written.
输出值: 无
返回值: 无
*****************************************************************************************/
I2C_StatusTypeDef i2c_reg_write(I2C_TypeDef* I2Cx,uint8_t txData,uint8_t deviceAdd,uint8_t regAdd)
{
    /* Test on BUSY Flag */
    _time_out = LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
    {
        if((_time_out--) == 0) {
            return i2c_timeout_callback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    _time_out = LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
    {
        if((_time_out--) == 0) {
            return i2c_timeout_callback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) regAdd);

    /* Wait until TCR flag is set */
    _time_out = LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if((_time_out--) == 0) {
            return i2c_timeout_callback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Write data to TXDR */
    I2C_SendData(I2Cx, txData);

    /* Wait until STOPF flag is set */
    _time_out = LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
    {
        if((_time_out--) == 0) {
            return i2c_timeout_callback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return I2C_OK;
}
/****************************************************************************************
函数名: I2C_StatusTypeDef i2c_reg_read(uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd) 
描  述: i2c读
输入值: 
* @brief  Read one byte value through the bus
* @param  buf: the pointer to data to be read.
* @param  deviceAdd : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
* @param  regAdd : specifies the Sensors internal address register to read from.
* @retval true or false
输出值: 无
返回值: 无
*****************************************************************************************/
I2C_StatusTypeDef i2c_reg_read(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd)
{
  /* Test on BUSY Flag */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Send Register address */
  I2C_SendData(I2C2, (uint8_t)regAdd);

  /* Wait until TC flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  /* Wait until RXNE flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Read data from RXDR */
  *buf = I2C_ReceiveData(I2Cx);

  /* Wait until STOPF flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

  /* If all operations OK */
  return I2C_OK;
}
/****************************************************************************************
函数名: I2C_StatusTypeDef i2c_reg_read(uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd) 
描  述: i2c读
输入值:
* @brief  Write a value in a register of the device through the bus.
* @param  buf: the pointer to data to be written.
* @param  deviceAdd: the device address on bus.
* @param  regAdd: the target register address to be written.
* @param  len: the size in bytes of the value to be written.
* @retval None
输出值: 无
返回值: 无
*****************************************************************************************/

I2C_StatusTypeDef i2c_write(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd,uint8_t len)
{
  uint8_t dataNum = 0;

  /* Test on BUSY Flag */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Send Register address */
  I2C_SendData(I2Cx, (uint8_t) regAdd);

  /* Wait until TCR flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, len, I2C_AutoEnd_Mode, I2C_No_StartStop);

  /* Write data to TXDR */
  while(dataNum != len)
  {
      /* Wait until TXIS flag is set */
      _time_out = LONG_TIMEOUT;
      while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
      {
          if((_time_out--) == 0) {
              return i2c_timeout_callback(I2Cx);
          }
      }

      /* Write data to TXDR */
      I2C_SendData(I2Cx, (uint8_t)(buf[dataNum]));
      /* Update number of transmitted data */
      dataNum++;
  }

  /* Wait until STOPF flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

  return I2C_OK;
}
/****************************************************************************************
函数名: I2C_StatusTypeDef i2c_read(uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd,uint8_t len)
描  述: i2c读
输入值:
* @brief  Read the value of a register of the device through the bus
* @param  buf: the pointer to data to be read.
* @param  deviceAdd : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
* @param  regAdd : specifies the Sensors internal address register to read from.
* @param  len : number of bytes to read from the Sensors.
* @retval true or false
输出值: 无
返回值: 无
*****************************************************************************************/
I2C_StatusTypeDef i2c_read(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd,uint8_t len)
{
  uint8_t dataNum = 0;

  /* Test on BUSY Flag */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Send Register address */
  I2C_SendData(I2C2, (uint8_t)regAdd);

  /* Wait until TC flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, deviceAdd, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Reset local variable */
  dataNum = 0;
  /* Wait until all data are received */
  while(dataNum != len)
  {
      /* Wait until RXNE flag is set */
      _time_out = LONG_TIMEOUT;
      while(I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET)
      {
          if((_time_out--) == 0) {
              return i2c_timeout_callback(I2Cx);
          }
      }
      /* Read data from RXDR */
      *buf = I2C_ReceiveData(I2Cx);
      /* Point to the next location where the byte read will be saved */
      buf++;
      /* Decrement the read bytes counter */
      dataNum++;
  }

  /* Wait until STOPF flag is set */
  _time_out = LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
  {
      if((_time_out--) == 0) {
          return i2c_timeout_callback(I2Cx);
      }
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

  /* If all operations OK */
  return I2C_OK;
}
