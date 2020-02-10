/****************************************************************************
*						   									张飞实战电子	
*1.技术支持QQ:3403129447  手机:15618987206
*2.微信公众号:张飞实战电子
*3.淘宝店铺:https://shop105175919.taobao.com/
*4.作者:张飞实战电子								  
*本软件仅供学习使用，对用户直接引用代码所带来的风险或后果不承担任何法律责任。
*****************************************************************************/
#include "stm32f37x.h"
#include "Sys.h"
#include "math.h"

//const uint16_t PointX[] = {0,118,236,354,472,590,708,826,950};     //0-950
//const uint16_t PointY[] = {0,100,250,380,400,420,450,500,600};     //0-600 
const uint16_t PointX[] = {0,118,236,354,472,590,708,826,950};     //0-950
const uint16_t PointY[] = {0,30,80,160,250,300,400,500,600};     //0-600 

float Curve_K[8];
float Curve_B[8];
float IMU_P = 0.8;
uint8_t flip_flag;
PID Pitch,Roll;
PID  PitchRate,RollRate,YawRate;
extern float_RPY Q_ANGLE;
extern float_RPY Q_Rad;
extern float_XYZ acce_f;
extern float_XYZ gyro_f;        //IMU 数据
extern uint8_t FlyStatus;
extern bool GRY_Stable_Flag;     // 陀螺仪数据稳定标志
extern uint8_t flip_direction;  // back
extern uint16_t flip_count;
extern float rollRateDesired;
extern float pitchRateDesired;
extern float yawRateDesired;
extern float PitchOutput;
extern float	RollOutput;
extern float YawOutput; 
extern float ADC_Value;
extern float_XYZ EXP_ANGLE;
extern PID Pitch;
extern PID Roll;
extern PID PitchRate;
extern PID RollRate;
extern PID YawRate;
uint16_t flip_time = 0;
extern uint8_t RF_Mode;
extern uint16_t AccelerateValue;
/*********************************************************
函数名: void Delay_us(unsigned int data)
描  述: 延时函数,以1us为单位
输入值: (unsigned int)data(data*1us)
输出值: 无
返回值: 无 
**********************************************************/
void Delay_us(unsigned int data)
{
    unsigned int x;
    unsigned int y;
    for(x = data;x>0;x--)
        for(y =7;y>0;y--);    
}
/*********************************************************
函数名: void Delay_ms(unsigned int data)
描  述: 延时函数,以1ms为单位
输入值: (unsigned int)data(data*1us)
输出值: 无
返回值: 无 
**********************************************************/
void Delay_ms(unsigned int data)
{
    unsigned int x;
    for(x = data;x>0;x--)
       Delay_us(1000);    
}
/*********************************************************
函数名: void RCC_InitConfig(void)
描  述: 时钟初始化
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void SysTick_Init(void)
{
	uint32_t ticks;
  ticks = SystemCoreClock / 1000;  
	SysTick_Config((ticks/8));       //1ms //9000	
	//change the SysTick Clock source to be HCLK_Div8   
	//note: the function must be call after the function SysTick_Config();
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  //systick --> 9Mhz
}

/*********************************************************
函数名: void PID_Init(void)
描  述: PID参数初始化
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void PID_Init(void)
{ 
	Pitch.kp=20.0;
	Pitch.kd=0.15f;
	Pitch.Dt=0.001f;//0.0075
	
	Roll.kp=Pitch.kp;
	Roll.kd=Pitch.kd;
	Roll.Dt=Pitch.Dt;

 
	PitchRate.kp = 2.8;     //2.0
	PitchRate.ki = 0.1;
	PitchRate.kd = 0.1;
		
	RollRate.kp = PitchRate.kp;
	RollRate.ki = PitchRate.ki;
	RollRate.kd = PitchRate.kd;

	YawRate.kp=6.0f;
	YawRate.kd=0.1f;
	YawRate.ki=30.0f;
	YawRate.Dt=0.001f;
	YawRate.Imax=300;
    
	PitchRate.Imax=200.0f;
	RollRate. Imax=200.0f;
	PitchRate.Dt  =0.001f;
	RollRate. Dt  =0.001f;
}
/*******************************************************************************
* 函数名 : curve_ctrl
* 描述   : 油门曲线拟合处理函数
* 输入   : raw_data 处理前数据
* 输出   : None
* 返回值 : 拟合后数据
*******************************************************************************/
float curve_ctrl(uint16_t raw_data)
{	
	uint8_t Curve_index;	
	Curve_index = raw_data / 118.0f;    
	if (Curve_index > 7)
  { 
    Curve_index = 7;
  }
	return ((raw_data * Curve_K[Curve_index]) + Curve_B[Curve_index]);
}

/*******************************************************************************
* 函数名 : Get_64K
* 描述   : 获取 Y = kx + b 中的比例系数k
* 输入   : 平面直角坐标系中的两个点坐标(Ax,Ay) (Bx,By)
* 输出   : None
* 返回值 : 比例系数k
*******************************************************************************/
static float Get_64K(float Ax,float Ay,float Bx,float By)
{
	float Tmp1,Tmp2;
	Tmp1 = By - Ay;
	Tmp2 = Bx - Ax;
	return (Tmp1/Tmp2);
}

/*******************************************************************************
* 函数名 : Get_64B
* 描述   : 获取 Y = kx + b 中的常数b
* 输入   : 平面直角坐标系中的两个点坐标(Ax,Ay) (Bx,By)
* 输出   : None
* 返回值 : 常数b
*******************************************************************************/
static float Get_64B(float Ax,float Ay,float Bx,float By)
{
	float Tmp1,Tmp2,TmpK;
	Tmp1 = By - Ay;
	Tmp2 = Bx - Ax;
	TmpK = (Tmp1/Tmp2);
	return ((Ay - (Ax*TmpK)));
}

/*******************************************************************************
* 函数名 : calc_curve
* 描述   : 处理曲线方程参数
* 输入   : None
* 输出   : None
* 返回值 : None
*******************************************************************************/
void calc_curve(void)
{
	uint8_t i;
	for ( i=0; i<8; i++)
	{
		Curve_K[i] = Get_64K(PointX[i],PointY[i],PointX[i+1],PointY[i+1]);
		Curve_B[i] = Get_64B(PointX[i],PointY[i],PointX[i+1],PointY[i+1]);
	}
}
/*******************************************************************************
* 函数名 : float applyDeadband(float value, float deadband)
* 描述   : 死区范围
* 输入   : 当前值,死区大小
* 输出   : None
* 返回值 : 计算死区后的值
*******************************************************************************/
float applyDeadband(float value, float deadband)
{
	if (fabs(value) <= deadband) value = 0;
	
	if( value >  deadband ) value -= deadband;
	if( value < -deadband ) value += deadband;
	
	return value;
}
/*******************************************************************************
* 函数名 : float constrain_float(float amt, float low, float high) 
* 描述   : 限定最大输出和最小输出
* 输入   : 当前值,最大值,最小值
* 输出   : None
* 返回值 : 处理后的值
*******************************************************************************/
//
float constrain_float(float amt, float low, float high) 
{
	if (isnan(amt)) {		
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

float q0=1, q1=0, q2=0, q3=0;
/*******************************************************************************
* 函数名 : static float invSqrt(float x)
* 描述   : 对输入值开根号
* 输入   : 输入值
* 输出   : None
* 返回值 : 开根号后的值
*******************************************************************************/
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*******************************************************************************
* 函数名 : void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
* 描述   : 求四元数
* 输入   : 角速度值,加速度值
* 输出   : None
* 返回值 : 无
*******************************************************************************/
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ 
	float twoKp ;
  
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;                
  
	//1度=0.0174弧度   1弧度 = 57.3度
	gx = gx * 0.0174f;         //转化为弧度
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;
  
	twoKp=IMU_P;      //互补滤波比例系数
	
  /*
	加速度计数据归一化（单位化），把加速度计的三维向量转换为单位向量， 因为是单位矢量到参考性的投影， 
  所以要把加速度计数据单位化，其实归一化改变的只是这三个向量的长度，
  也就是只改变了相同的倍数，方向并没有改变，也是为了与单位四元数对应
  */
	//归1化加速度值
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
  /*
  把四元数换算成“方向余弦矩阵”中的第三列的三个元素。根据余弦矩阵和欧拉角的定义，
  地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。所以这里的halfvx、halfvy、halfvz 
  其实就是当前的机体坐标参照系上换算出来的重力单位向量的一半。(用表示机体姿态的四元数进行换算)
  */
	halfvx = q1 * q3 - q0 * q2;  //参考系X轴与载体系 x 轴之间方向余弦向量 
	halfvy = q0 * q1 + q2 * q3;   //参考系Y轴与载体系 y 轴之间方向余弦向量 
	halfvz = q0 * q0 - 0.5f + q3 * q3; //参考系Z轴与载体系 z 轴之间方向余弦向量 
	
 /*
 这里说明一点，加速度计由于噪声比较大，而且在飞行过程中，受机体振动影响比陀螺仪明显，短时间内的可靠性不高。
 陀螺仪噪声小，但是由于积分是离散的，长时间的积分会出现漂移的情况，因此需要将用加速度计求得的姿态来矫正陀螺仪积分姿态的漂移。
 解释： 
 ax,ay,az是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
 vx,vy,vz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
 向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，ex,ey,ez就是两个重力向量的叉积。
 这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，
 而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
 由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

 计算由当前姿态的重力在三个轴上的分量与加速度计测得的重力在三个轴上的分量的差,这里采用三维空间的差积(向量积)方法求差.
 计算公式由矩阵运算推导而来,公式参见http://en.wikipedia.org/wiki/Cross_product 中的Mnemonic部分 

 向量叉积得出姿态误差，通过加速度计测得的重力坐标系下的单位向量与上一次四元数转换成的单位向量进行叉乘，
 以此得到其误差量外积在相减得到差分就是误差，把叉积等同于角速度误差
 */ 
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);        //机体系 下


  /*
  将该误差输入 PID 控制器后与本次姿态更新周期中陀螺仪测得的角速度相加，
  最终得到一个修正的角速度值,将其输入四元数微分方程，更新四元数
  */
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;



  //解四元数微分方程用到的是一阶毕卡法，当然也可以用龙格库塔法更新四元数，dt：陀螺采样的间隔 
  //如果要研究具体的毕卡算法 参考链接：https://wenku.baidu.com/view/80c788f90242a8956bece4ee.html  
	gx *= (0.5f * dt);   // pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
  
	// 四元数归一化（单位化）
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
/*******************************************************************************
* 函数名 : void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw)
* 描述   : 四元数转欧拉角
* 输入   : 
* 输出   : None
* 返回值 : 欧拉角
*四元数转欧拉角 有计算公式 公式的推算过程可以参考我在工程里附加的文档或此链接：http://www.cnblogs.com/21207-iHome/p/6894128.html
*******************************************************************************/

void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw)
{
	float gx, gy, gz;    

	gx = 2.0f * (q1*q3 - q0*q2);
	gy = 2.0f * (q0*q1 + q2*q3);
	gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//*pitch = atanf(gx / sqrt(gy*gy + gz*gz)) * 57.3f;
	*pitch = -asinf(gx) * 57.3f;
	*roll  = atanf(gy / gz) * 57.3f;
//*yaw   = atan2f(2.0f*q1*q2 - 2.0f*q0*q3, 2.0f*q0*q0 + 2.0f*q1*q1 - 1) * -57.3f;
}
/*******************************************************************************
* 函数名 : void PID_EurLoop(PID* pid,float EurDesir,float measure,float* Output)
* 描述   : 姿态环 角度环PID  
* 输入   : pid---pid指针，EurDesir---期望的欧拉角，measure---测量的欧拉角,
* 输出   : None
* 返回值 : output---输出期望的角速度   
*******************************************************************************/
void PID_EurLoop(PID* pid,float EurDesir,float measure,float* Output)
{
	float Err,Diff;
	
  Err = EurDesir-measure;      //得到误差       
	Diff = (Err-pid->PreErr)/pid->Dt;   
	pid->PreErr = Err;            //更新前次误差      

	pid->Pout = pid->kp * Err;         //比例计算,P*误差 
	pid->Dout = pid->kd * Diff;

	*Output = pid->Pout + pid->Iout + pid->Dout;
}
/*******************************************************************************
* 函数名 : void PID_GRYLoop(PID* pid,float EurDesir,float measure,float* Output)
* 描述   : 角速度环PID  
* 输入   : pid---pid指针，EurDesir---期望的欧拉角，measure---测量的欧拉角,
* 输出   : None
* 返回值 : output---输出期望的角速度   
*******************************************************************************/
void PID_GRYLoop(PID* pid,float EurDesir,float measure,float* Output)
{
	float Err,Diff;
	
  Err = EurDesir-measure;        //得到误差     
	Diff = (Err-pid->PreErr)/pid->Dt; //微分
	pid->PreErr=Err;                //更新前次误差    
	
	if(fabs(Err)>=0.5 && fabs(Err)<=800  ) pid->I_sum+=(Err*pid->Dt);//积分       
	if (pid->I_sum>  pid->Imax  )        pid->I_sum =    pid->Imax;
	if (pid->I_sum<-(pid->Imax) )        pid->I_sum =   -(pid->Imax);
	if (FlyStatus == FlyStop) pid->I_sum=0;
	
	pid->Pout = pid->kp * Err;          //比例计算,P*误差 
	pid->Iout = pid->ki * pid->I_sum;
	pid->Dout = pid->kd * Diff;

	*Output=pid->Pout+pid->Iout+pid->Dout;
}

/*********************************************************
函数名: void attitude_pid_1ms_task(void)
描  述: 内环控制 1ms执行一次
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void attitude_pid_1ms_task(void)
{
		
	if (flip_flag)
	{
        switch(flip_direction) 		//通过flip_direction判断翻跟头的放向
        {
          case 1:
							PID_GRYLoop(&PitchRate, 900*(3.8f/ADC_Value),gyro_f.Y ,  &PitchOutput);			//固定X轴前后翻
							PID_GRYLoop(&RollRate , 0,                  gyro_f.X,  &RollOutput );
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z,        gyro_f.Z,  &YawOutput  );				
							break;
                
           case 2: 
							PID_GRYLoop(&PitchRate, -900*(3.8f/ADC_Value), gyro_f.Y,  &PitchOutput);			//固定X轴前后翻
							PID_GRYLoop(&RollRate , 0,                  gyro_f.X,  &RollOutput );			
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,    gyro_f.Z,   &YawOutput  ); 
							break;
            
           case 3:
							PID_GRYLoop(&PitchRate, 0,                   gyro_f.Y,  &PitchOutput);			//固定Y轴左右翻
							PID_GRYLoop(&RollRate , 800*(3.8f/ADC_Value),gyro_f.X,  &RollOutput );	
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z     ,   gyro_f.Z,  &YawOutput  );
							break;	
           case 4:
							PID_GRYLoop(&PitchRate, 0,                   gyro_f.Y,  &PitchOutput);			//固定Y轴左右翻
							PID_GRYLoop(&RollRate , -800*(3.8f/ADC_Value),gyro_f.X,  &RollOutput );			
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z     ,    gyro_f.Z,  &YawOutput  );
							break;
           default :
							 PID_GRYLoop(&PitchRate, pitchRateDesired,  gyro_f.Y,  &PitchOutput);	//内环角速率环控制  正常飞行内环PID
							 PID_GRYLoop(&RollRate , rollRateDesired ,  gyro_f.X,  &RollOutput );
							 PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,   gyro_f.Z,  &YawOutput  );
        }
  }
  else
  {
		 PID_GRYLoop(&PitchRate, pitchRateDesired,  gyro_f.Y,  &PitchOutput);	//内环角速率环控制   正常飞行内环PID
		 PID_GRYLoop(&RollRate , rollRateDesired ,  gyro_f.X,  &RollOutput );
		 PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,  gyro_f.Z,  &YawOutput  );
  }
}
/*********************************************************
函数名: void flip_3ms_task(void)
描  述: 翻跟头控制 3ms执行一次
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void flip_3ms_task(void)
{
 	if (flip_count == 0)
	{			
		if (flip_direction != 0)
		{
			flip_time ++;
			if(flip_time >= 200)    //600ms
			{
				IMU_P = 6.6f;
				flip_flag = false;   //have flipped
				if(flip_time >= 280)
				{  
					flip_count = 1; 
					flip_time = 0; 
					PitchRate.I_sum=0.0f;
					Pitch.I_sum=0.0f;
					RollRate.I_sum=0.0f;
					Roll.I_sum=0.0f;
					YawRate.I_sum=0.0f;
					IMU_P = 0.6f; 
					RF_Mode = RFMODE_6AXIE;
				}   //1000ms
				
         AccelerateValue = 950;
			 }
			 else
			 {	
						if(flip_time <= 85)
            { 
              AccelerateValue = 950;    //200ms爬升
            }
						/*else if(flip_time>85&&flip_time<100) 
            {
              AccelerateValue=0;
            }*/
						else 
            {
              AccelerateValue = 0; 
              flip_flag = true; 
              IMU_P = 6.6f;
            }  //flipping
			  }				
		 }
	 }		
}
/*******************************************************************************
* 函数名 : void attitude_quat_5ms_task(void)
* 描述   : 四元数欧拉角的封装,5ms执行一次 
* 输入   : 无
* 输出   : None
* 返回值 : 无
*******************************************************************************/
void attitude_quat_5ms_task(void)
{
		if (FlyStatus!=FlyStart && GRY_Stable_Flag == true)       //根据飞机状态 更新互补滤波系数
		{                
			IMU_P = 10.0f;        //未起飞且IMU数据稳定时加大互补滤波P 加快 加速度数据融合比例
		} 
    else
    {                         
			  IMU_P = 0.6f;      //起飞后减小加速度数据融合比例
		}			
		sensfusion6UpdateQ(gyro_f.X, gyro_f.Y, gyro_f.Z,acce_f.X*100.0f,acce_f.Y*100.0f,acce_f.Z*100.0f, 0.005f);    //6Axie IMU更新四元数
		sensfusion6GetEulerRPY(&Q_ANGLE.Roll,&Q_ANGLE.Pitch,&Q_ANGLE.Yaw);   //四元数转欧拉交		
}

