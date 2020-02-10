#ifndef _MPU6500_H_
#define _MPU6500_H_





#define Gyr_Gain 	0.06103f        // +-2000 DEG/s     2000 deg/s/32767
#define ACC_Gain 	0.0011963f      // +-4G							4g * 9.8 / 32767z

#define	SMPLRT_DIV		        0x19	
#define	CONFIG			        0x1A	
#define	GYRO_CONFIG		        0x1B	
#define	ACCEL_CONFIG	        0x1C	
#define	MPU6500_ACCEL_CONFIG2   0x1D  
#define	MPU6500_INT_PIN_CFG	    0x37
#define MPU6500_INT_ENABLE      0x38
#define	ACCEL_XOUT_H	        0x3B
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	TEMP_OUT_H		        0x41
#define	TEMP_OUT_L		        0x42
#define	GYRO_XOUT_H		        0x43
#define	GYRO_XOUT_L		        0x44	
#define	GYRO_YOUT_H		        0x45
#define	GYRO_YOUT_L		        0x46
#define	GYRO_ZOUT_H		        0x47
#define	GYRO_ZOUT_L		        0x48
#define MPU6500_SIGNAL_PATH_RESET 0x68
#define	MPU6500_USER_CTRL		  0x6A	
#define	PWR_MGMT_1		        0x6B	
#define	MPU6500_Addr          0xD0//0x68   //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改 0x68



/**********************************************************************************************************/
void mpu6500_init(void);
void READ_MPU6500(void);
void Get_GRYOffset(void);
void Cal_TsData(void);
float get_acc_mod(void);	

/**********************************************************************************************************/
#endif



