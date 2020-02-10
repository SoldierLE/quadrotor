/****************************************************************************
*						   									�ŷ�ʵս����	
*1.����֧��QQ:3403129447  �ֻ�:15618987206
*2.΢�Ź��ں�:�ŷ�ʵս����
*3.�Ա�����:https://shop105175919.taobao.com/
*4.����:�ŷ�ʵս����								  
*���������ѧϰʹ�ã����û�ֱ�����ô����������ķ��ջ������е��κη������Ρ�
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
extern float_XYZ gyro_f;        //IMU ����
extern uint8_t FlyStatus;
extern bool GRY_Stable_Flag;     // �����������ȶ���־
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
������: void Delay_us(unsigned int data)
��  ��: ��ʱ����,��1usΪ��λ
����ֵ: (unsigned int)data(data*1us)
���ֵ: ��
����ֵ: �� 
**********************************************************/
void Delay_us(unsigned int data)
{
    unsigned int x;
    unsigned int y;
    for(x = data;x>0;x--)
        for(y =7;y>0;y--);    
}
/*********************************************************
������: void Delay_ms(unsigned int data)
��  ��: ��ʱ����,��1msΪ��λ
����ֵ: (unsigned int)data(data*1us)
���ֵ: ��
����ֵ: �� 
**********************************************************/
void Delay_ms(unsigned int data)
{
    unsigned int x;
    for(x = data;x>0;x--)
       Delay_us(1000);    
}
/*********************************************************
������: void RCC_InitConfig(void)
��  ��: ʱ�ӳ�ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
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
������: void PID_Init(void)
��  ��: PID������ʼ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
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
* ������ : curve_ctrl
* ����   : ����������ϴ�����
* ����   : raw_data ����ǰ����
* ���   : None
* ����ֵ : ��Ϻ�����
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
* ������ : Get_64K
* ����   : ��ȡ Y = kx + b �еı���ϵ��k
* ����   : ƽ��ֱ������ϵ�е�����������(Ax,Ay) (Bx,By)
* ���   : None
* ����ֵ : ����ϵ��k
*******************************************************************************/
static float Get_64K(float Ax,float Ay,float Bx,float By)
{
	float Tmp1,Tmp2;
	Tmp1 = By - Ay;
	Tmp2 = Bx - Ax;
	return (Tmp1/Tmp2);
}

/*******************************************************************************
* ������ : Get_64B
* ����   : ��ȡ Y = kx + b �еĳ���b
* ����   : ƽ��ֱ������ϵ�е�����������(Ax,Ay) (Bx,By)
* ���   : None
* ����ֵ : ����b
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
* ������ : calc_curve
* ����   : �������߷��̲���
* ����   : None
* ���   : None
* ����ֵ : None
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
* ������ : float applyDeadband(float value, float deadband)
* ����   : ������Χ
* ����   : ��ǰֵ,������С
* ���   : None
* ����ֵ : �����������ֵ
*******************************************************************************/
float applyDeadband(float value, float deadband)
{
	if (fabs(value) <= deadband) value = 0;
	
	if( value >  deadband ) value -= deadband;
	if( value < -deadband ) value += deadband;
	
	return value;
}
/*******************************************************************************
* ������ : float constrain_float(float amt, float low, float high) 
* ����   : �޶�����������С���
* ����   : ��ǰֵ,���ֵ,��Сֵ
* ���   : None
* ����ֵ : ������ֵ
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
* ������ : static float invSqrt(float x)
* ����   : ������ֵ������
* ����   : ����ֵ
* ���   : None
* ����ֵ : �����ź��ֵ
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
* ������ : void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
* ����   : ����Ԫ��
* ����   : ���ٶ�ֵ,���ٶ�ֵ
* ���   : None
* ����ֵ : ��
*******************************************************************************/
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ 
	float twoKp ;
  
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;                
  
	//1��=0.0174����   1���� = 57.3��
	gx = gx * 0.0174f;         //ת��Ϊ����
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;
  
	twoKp=IMU_P;      //�����˲�����ϵ��
	
  /*
	���ٶȼ����ݹ�һ������λ�������Ѽ��ٶȼƵ���ά����ת��Ϊ��λ������ ��Ϊ�ǵ�λʸ�����ο��Ե�ͶӰ�� 
  ����Ҫ�Ѽ��ٶȼ����ݵ�λ������ʵ��һ���ı��ֻ�������������ĳ��ȣ�
  Ҳ����ֻ�ı�����ͬ�ı���������û�иı䣬Ҳ��Ϊ���뵥λ��Ԫ����Ӧ
  */
	//��1�����ٶ�ֵ
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
  /*
  ����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء��������Ҿ����ŷ���ǵĶ��壬
  ��������ϵ������������ת����������ϵ��������������Ԫ�ء����������halfvx��halfvy��halfvz 
  ��ʵ���ǵ�ǰ�Ļ����������ϵ�ϻ��������������λ������һ�롣(�ñ�ʾ������̬����Ԫ�����л���)
  */
	halfvx = q1 * q3 - q0 * q2;  //�ο�ϵX��������ϵ x ��֮�䷽���������� 
	halfvy = q0 * q1 + q2 * q3;   //�ο�ϵY��������ϵ y ��֮�䷽���������� 
	halfvz = q0 * q0 - 0.5f + q3 * q3; //�ο�ϵZ��������ϵ z ��֮�䷽���������� 
	
 /*
 ����˵��һ�㣬���ٶȼ����������Ƚϴ󣬶����ڷ��й����У��ܻ�����Ӱ������������ԣ���ʱ���ڵĿɿ��Բ��ߡ�
 ����������С���������ڻ�������ɢ�ģ���ʱ��Ļ��ֻ����Ư�Ƶ�����������Ҫ���ü��ٶȼ���õ���̬�����������ǻ�����̬��Ư�ơ�
 ���ͣ� 
 ax,ay,az�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
 vx,vy,vz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
 ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
 ������������������������Ҳ�������������ˣ�����ʾ��ex,ey,ez�����������������Ĳ����
 �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ��
 ���Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ�
 ���������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

 �����ɵ�ǰ��̬���������������ϵķ�������ٶȼƲ�õ��������������ϵķ����Ĳ�,���������ά�ռ�Ĳ��(������)�������.
 ���㹫ʽ�ɾ��������Ƶ�����,��ʽ�μ�http://en.wikipedia.org/wiki/Cross_product �е�Mnemonic���� 

 ��������ó���̬��ͨ�����ٶȼƲ�õ���������ϵ�µĵ�λ��������һ����Ԫ��ת���ɵĵ�λ�������в�ˣ�
 �Դ˵õ�����������������õ���־������Ѳ����ͬ�ڽ��ٶ����
 */ 
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);        //����ϵ ��


  /*
  ����������� PID ���������뱾����̬���������������ǲ�õĽ��ٶ���ӣ�
  ���յõ�һ�������Ľ��ٶ�ֵ,����������Ԫ��΢�ַ��̣�������Ԫ��
  */
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;



  //����Ԫ��΢�ַ����õ�����һ�ױϿ�������ȻҲ���������������������Ԫ����dt�����ݲ����ļ�� 
  //���Ҫ�о�����ıϿ��㷨 �ο����ӣ�https://wenku.baidu.com/view/80c788f90242a8956bece4ee.html  
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
  
	// ��Ԫ����һ������λ����
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
/*******************************************************************************
* ������ : void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw)
* ����   : ��Ԫ��תŷ����
* ����   : 
* ���   : None
* ����ֵ : ŷ����
*��Ԫ��תŷ���� �м��㹫ʽ ��ʽ��������̿��Բο����ڹ����︽�ӵ��ĵ�������ӣ�http://www.cnblogs.com/21207-iHome/p/6894128.html
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
* ������ : void PID_EurLoop(PID* pid,float EurDesir,float measure,float* Output)
* ����   : ��̬�� �ǶȻ�PID  
* ����   : pid---pidָ�룬EurDesir---������ŷ���ǣ�measure---������ŷ����,
* ���   : None
* ����ֵ : output---��������Ľ��ٶ�   
*******************************************************************************/
void PID_EurLoop(PID* pid,float EurDesir,float measure,float* Output)
{
	float Err,Diff;
	
  Err = EurDesir-measure;      //�õ����       
	Diff = (Err-pid->PreErr)/pid->Dt;   
	pid->PreErr = Err;            //����ǰ�����      

	pid->Pout = pid->kp * Err;         //��������,P*��� 
	pid->Dout = pid->kd * Diff;

	*Output = pid->Pout + pid->Iout + pid->Dout;
}
/*******************************************************************************
* ������ : void PID_GRYLoop(PID* pid,float EurDesir,float measure,float* Output)
* ����   : ���ٶȻ�PID  
* ����   : pid---pidָ�룬EurDesir---������ŷ���ǣ�measure---������ŷ����,
* ���   : None
* ����ֵ : output---��������Ľ��ٶ�   
*******************************************************************************/
void PID_GRYLoop(PID* pid,float EurDesir,float measure,float* Output)
{
	float Err,Diff;
	
  Err = EurDesir-measure;        //�õ����     
	Diff = (Err-pid->PreErr)/pid->Dt; //΢��
	pid->PreErr=Err;                //����ǰ�����    
	
	if(fabs(Err)>=0.5 && fabs(Err)<=800  ) pid->I_sum+=(Err*pid->Dt);//����       
	if (pid->I_sum>  pid->Imax  )        pid->I_sum =    pid->Imax;
	if (pid->I_sum<-(pid->Imax) )        pid->I_sum =   -(pid->Imax);
	if (FlyStatus == FlyStop) pid->I_sum=0;
	
	pid->Pout = pid->kp * Err;          //��������,P*��� 
	pid->Iout = pid->ki * pid->I_sum;
	pid->Dout = pid->kd * Diff;

	*Output=pid->Pout+pid->Iout+pid->Dout;
}

/*********************************************************
������: void attitude_pid_1ms_task(void)
��  ��: �ڻ����� 1msִ��һ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void attitude_pid_1ms_task(void)
{
		
	if (flip_flag)
	{
        switch(flip_direction) 		//ͨ��flip_direction�жϷ���ͷ�ķ���
        {
          case 1:
							PID_GRYLoop(&PitchRate, 900*(3.8f/ADC_Value),gyro_f.Y ,  &PitchOutput);			//�̶�X��ǰ��
							PID_GRYLoop(&RollRate , 0,                  gyro_f.X,  &RollOutput );
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z,        gyro_f.Z,  &YawOutput  );				
							break;
                
           case 2: 
							PID_GRYLoop(&PitchRate, -900*(3.8f/ADC_Value), gyro_f.Y,  &PitchOutput);			//�̶�X��ǰ��
							PID_GRYLoop(&RollRate , 0,                  gyro_f.X,  &RollOutput );			
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,    gyro_f.Z,   &YawOutput  ); 
							break;
            
           case 3:
							PID_GRYLoop(&PitchRate, 0,                   gyro_f.Y,  &PitchOutput);			//�̶�Y�����ҷ�
							PID_GRYLoop(&RollRate , 800*(3.8f/ADC_Value),gyro_f.X,  &RollOutput );	
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z     ,   gyro_f.Z,  &YawOutput  );
							break;	
           case 4:
							PID_GRYLoop(&PitchRate, 0,                   gyro_f.Y,  &PitchOutput);			//�̶�Y�����ҷ�
							PID_GRYLoop(&RollRate , -800*(3.8f/ADC_Value),gyro_f.X,  &RollOutput );			
							PID_GRYLoop(&YawRate  , EXP_ANGLE.Z     ,    gyro_f.Z,  &YawOutput  );
							break;
           default :
							 PID_GRYLoop(&PitchRate, pitchRateDesired,  gyro_f.Y,  &PitchOutput);	//�ڻ������ʻ�����  ���������ڻ�PID
							 PID_GRYLoop(&RollRate , rollRateDesired ,  gyro_f.X,  &RollOutput );
							 PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,   gyro_f.Z,  &YawOutput  );
        }
  }
  else
  {
		 PID_GRYLoop(&PitchRate, pitchRateDesired,  gyro_f.Y,  &PitchOutput);	//�ڻ������ʻ�����   ���������ڻ�PID
		 PID_GRYLoop(&RollRate , rollRateDesired ,  gyro_f.X,  &RollOutput );
		 PID_GRYLoop(&YawRate  , EXP_ANGLE.Z    ,  gyro_f.Z,  &YawOutput  );
  }
}
/*********************************************************
������: void flip_3ms_task(void)
��  ��: ����ͷ���� 3msִ��һ��
����ֵ: ��
���ֵ: ��
����ֵ: �� 
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
              AccelerateValue = 950;    //200ms����
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
* ������ : void attitude_quat_5ms_task(void)
* ����   : ��Ԫ��ŷ���ǵķ�װ,5msִ��һ�� 
* ����   : ��
* ���   : None
* ����ֵ : ��
*******************************************************************************/
void attitude_quat_5ms_task(void)
{
		if (FlyStatus!=FlyStart && GRY_Stable_Flag == true)       //���ݷɻ�״̬ ���»����˲�ϵ��
		{                
			IMU_P = 10.0f;        //δ�����IMU�����ȶ�ʱ�Ӵ󻥲��˲�P �ӿ� ���ٶ������ںϱ���
		} 
    else
    {                         
			  IMU_P = 0.6f;      //��ɺ��С���ٶ������ںϱ���
		}			
		sensfusion6UpdateQ(gyro_f.X, gyro_f.Y, gyro_f.Z,acce_f.X*100.0f,acce_f.Y*100.0f,acce_f.Z*100.0f, 0.005f);    //6Axie IMU������Ԫ��
		sensfusion6GetEulerRPY(&Q_ANGLE.Roll,&Q_ANGLE.Pitch,&Q_ANGLE.Yaw);   //��Ԫ��תŷ����		
}

