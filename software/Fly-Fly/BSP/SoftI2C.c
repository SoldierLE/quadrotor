//===========================================================================
#define DelayTick 0
#define TWI_SCL_PIN       		GPIO_Pin_6
#define TWI_SDA_PIN       		GPIO_Pin_7
//?迆芍ao‘那y,???迆?a??3足D辰㊣?谷赤米?D∫?那?那足a,2?那那o??∩?車米?o‘那y
static __inline void TWI_SCL_0(void)  	{GPIOB->BRR = GPIO_Pin_6;}
static __inline void TWI_SCL_1(void)  	{GPIOB->BSRR = GPIO_Pin_6;}
static __inline void TWI_SDA_0(void)  	{GPIOB->BRR = GPIO_Pin_7;}
static __inline void TWI_SDA_1(void)  	{GPIOB->BSRR = GPIO_Pin_7;}

/*********************************************************
o‘那y??: void Input_pin_sda(void)
?豕  那?: 谷豕???a那?豕?
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void Input_pin_sda(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}	

/*********************************************************
o‘那y??: void out_pin_sda(void)
?豕  那?: 谷豕???a那?3?
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void out_pin_sda(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
	
/*********************************************************
o‘那y??: void out_pin_scl(void)
?豕  那?: 谷豕???a那?3?
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void out_pin_scl(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/*********************************************************
o‘那y??: void SoftI2C_delay(uint8_t i)
?豕  那?: ?車那㊣
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void SoftI2C_delay(uint8_t i)
{
    while(i--);
}
/*********************************************************
o‘那y??: void SoftI2C_stop(void)
?豕  那?: 赤㏒?1
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void SoftI2C_stop(void)
{
	TWI_SCL_1();
	SoftI2C_delay(DelayTick);
	TWI_SDA_1();
	SoftI2C_delay(DelayTick);
}
/*********************************************************
o‘那y??: void SoftI2C_stop(void)
?豕  那?: 赤㏒?1
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
void SoftI2C_start(void)
{
	out_pin_scl();
	out_pin_sda();
	TWI_SDA_1();
  SoftI2C_delay(DelayTick);
	TWI_SCL_1();
	TWI_SDA_0();
  SoftI2C_delay(DelayTick);
	TWI_SCL_0();
}

/*********************************************************
o‘那y??: uint8_t SoftI2C_write_byte(uint8_t value)
?豕  那?: D∩那y?Y
那?豕??米: ?T
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
uint8_t SoftI2C_write_byte(uint8_t value)
{
    uint8_t i;
    uint8_t result = 0;
    i = 8;
    while (i)
    {
			if (value & 0x80)
				TWI_SDA_1();
			else
				TWI_SDA_0();
			SoftI2C_delay(DelayTick);
			
			TWI_SCL_1();
			i--;
			value <<= 1;
			SoftI2C_delay(DelayTick);
			TWI_SCL_0();
			SoftI2C_delay(DelayTick);
    }
		Input_pin_sda();
    SoftI2C_delay(DelayTick);

		TWI_SCL_1();
    SoftI2C_delay(DelayTick);
		if(GPIO_ReadInputDataBit(GPIOB, TWI_SDA_PIN))
    {
        result = 1;
    }
		TWI_SCL_0();
		TWI_SDA_0();
		out_pin_sda();
		return result;
}
/*********************************************************
o‘那y??: uint8_t SoftI2C_read_byte(uint8_t ack)
?豕  那?: ?芍那y?Y
那?豕??米: ack
那?3??米: ?T
﹞米???米: ?T
**********************************************************/
uint8_t SoftI2C_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t result = 0;
    i = 8;
		Input_pin_sda();
    while (i)
    {
			result <<= 1;
			TWI_SCL_1();
			i--;
			SoftI2C_delay(DelayTick);
			if(GPIO_ReadInputDataBit(GPIOB, TWI_SDA_PIN))
			{
					result |= 0x01;
			}
			TWI_SCL_0();
			SoftI2C_delay(DelayTick);
    }
		out_pin_sda();
    if (ack)
			TWI_SDA_1();
    else
			TWI_SDA_0();
    SoftI2C_delay(DelayTick);

		TWI_SCL_1();
    SoftI2C_delay(DelayTick);
		TWI_SCL_0();
		TWI_SDA_0();
    SoftI2C_delay(3);
    return result;
}