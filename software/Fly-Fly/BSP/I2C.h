/* 
 * File:   Sys.h
 * Author: Administrator
 *
 * Created on 2016?12?10?, ??9:31
 */
#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif
	
#include "stm32f37x.h"

//---------------------------IMU IIC---------------------------------
#define IMU_I2C                             I2C2
#define IMU_I2C_CLK                         RCC_APB1Periph_I2C2

//SCL GPIO CONFIG
#define IMU_I2C_SCL_GPIO_PORT               GPIOF                      /* GPIOF */
#define IMU_I2C_SCL_PIN                     GPIO_Pin_6                 /* PF.06 */
#define IMU_I2C_SCL_GPIO_CLK                RCC_AHBPeriph_GPIOF        /* GPIO CLOCK */
#define IMU_I2C_SCL_AF                      GPIO_AF_4                  /* GPIO AF4 */
#define IMU_I2C_SCL_SOURCE                  GPIO_PinSource6            /* GPIO Pinsource 6 */

//SDA GPIO CONFIG
#define IMU_I2C_SDA_GPIO_PORT               GPIOF                      /* GPIOF */
#define IMU_I2C_SDA_PIN                     GPIO_Pin_7                 /* PF.07 */
#define IMU_I2C_SDA_GPIO_CLK                RCC_AHBPeriph_GPIOF        /* GPIO CLOCK */
#define IMU_I2C_SDA_AF                      GPIO_AF_4                  /* GPIO AF4 */
#define IMU_I2C_SDA_SOURCE                  GPIO_PinSource7            /* GPIO Pinsource 7 */

	
typedef enum {
    I2C_OK = 0,
    I2C_ERROR = 1,
    I2C_TIMEOUT = 2,
    I2C_NOT_IMPLEMENTED = 3
}I2C_StatusTypeDef;




I2C_StatusTypeDef i2c_timeout_callback(I2C_TypeDef* I2Cx);

void imu_i2c_init(void);
void i2c2_init(void);

I2C_StatusTypeDef i2c_reg_write(I2C_TypeDef* I2Cx,uint8_t txData,uint8_t deviceAdd,uint8_t regAdd);
I2C_StatusTypeDef i2c_reg_read(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd);
I2C_StatusTypeDef i2c_write(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd,uint8_t len);
I2C_StatusTypeDef i2c_read(I2C_TypeDef* I2Cx,uint8_t *buf,uint8_t deviceAdd,uint8_t regAdd,uint8_t len);

#ifdef	__cplusplus
}
#endif

#endif	/* SYS_H */

