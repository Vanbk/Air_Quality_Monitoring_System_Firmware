#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "LCD.h"
#include "font.h"

//Define the LCD Operation function
void LCD5110_LCD_write_byte(unsigned char dat,unsigned char LCD5110_MOde);
void LCD5110_LCD_delay_ms(unsigned int t);

//Define the hardware operation function
void LCD5110_GPIO_Config(void);
void LCD5110_SCK(unsigned char temp);
void LCD5110_MO(unsigned char temp);
void LCD5110_CS(unsigned char temp);
void LCD5110_RST(unsigned char temp);
void LCD5110_DC(unsigned char temp);





void LCD5110_init()
{
	LCD5110_GPIO_Config();

	LCD5110_DC(1);//LCD_DC = 1;
	LCD5110_MO(1);//SPI_MO = 1;
	LCD5110_SCK(1);//SPI_SCK = 1;
	LCD5110_CS(1);//SPI_CS = 1;
	
	LCD5110_RST(0);//LCD_RST = 0;
	LCD5110_LCD_delay_ms(10);
	LCD5110_RST(1);//LCD_RST = 1;

	LCD5110_LCD_write_byte(0x21,0);
	LCD5110_LCD_write_byte(0xc6,0);
	LCD5110_LCD_write_byte(0x06,0);
	LCD5110_LCD_write_byte(0x13,0);
	LCD5110_LCD_write_byte(0x20,0);
	LCD5110_clear();
	LCD5110_LCD_write_byte(0x0c,0);
}

void LCD5110_LCD_write_byte(unsigned char dat,unsigned char mode)
{
	unsigned char i;

	LCD5110_CS(0);//SPI_CS = 0;

	if (0 == mode)
		LCD5110_DC(0);//LCD_DC = 0;
	else
		LCD5110_DC(1);//LCD_DC = 1;

	for(i=0;i<8;i++)
	{
		LCD5110_MO(dat & 0x80);//SPI_MO = dat & 0x80;
		dat = dat<<1;
		LCD5110_SCK(0);//SPI_SCK = 0;
		LCD5110_SCK(1);//SPI_SCK = 1;
	}

	LCD5110_CS(1);//SPI_CS = 1;

}

void LCD5110_write_char(unsigned char c)
{
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for(line=0;line<6;line++)
	{
		ch = font6_8[c][line];
		LCD5110_LCD_write_byte(ch,1);
		
	}
}
void LCD5110_write_char_reg(unsigned char c)
{
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for(line=0;line<6;line++)
	{
		ch = ~font6_8[c][line];
		LCD5110_LCD_write_byte(ch,1);
		
	}
}

void LCD5110_write_string(char *s)
{
	unsigned char ch;
  	while(*s!='\0')
	{
		ch = *s;
		LCD5110_write_char(ch);
		s++;
	}
}


void LCD5110_clear()
{
	unsigned char i,j;
	for(i=0;i<6;i++)
		for(j=0;j<84;j++)
			LCD5110_LCD_write_byte(0,1);	
}

void LCD5110_set_XY(unsigned char X,unsigned char Y)
{
	unsigned char x;
	x = 6*X;

	LCD5110_LCD_write_byte(0x40|Y,0);
	LCD5110_LCD_write_byte(0x80|x,0);
}

void LCD5110_Write_Dec(unsigned int b)
{

	unsigned char datas[3];

	datas[0] = b/1000;
	b = b - datas[0]*1000;
	datas[1] = b/100;
	b = b - datas[1]*100;
	datas[2] = b/10;
	b = b - datas[2]*10;
	datas[3] = b;

	datas[0]+=48;
	datas[1]+=48;
	datas[2]+=48;
	datas[3]+=48;

	LCD5110_write_char(datas[0]);
	LCD5110_write_char(datas[1]);
	LCD5110_write_char(datas[2]);
	LCD5110_write_char(datas[3]);

	//a++;
}

void LCD5110_LCD_delay_ms(unsigned int nCount)
{
  unsigned long t;
	t = nCount * 40000;
	while(t--);
}

void LCD5110_GPIO_Config()
{
	GPIO_InitTypeDef GPIOA_Init;
	GPIO_InitTypeDef GPIOC_Init;

	GPIOA_Init.GPIO_Pin = GPIO_Pin_8;
	GPIOA_Init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIOA_Init.GPIO_Mode = GPIO_Mode_Out_PP;


	GPIOC_Init.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIOC_Init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIOC_Init.GPIO_Mode = GPIO_Mode_Out_PP;

	RCC_HSICmd(DISABLE);
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
	RCC_PLLCmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_Init(GPIOA,&GPIOA_Init);
	GPIO_Init(GPIOC,&GPIOC_Init);

}

void LCD5110_CS(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOC,GPIO_Pin_9);
	else GPIO_ResetBits(GPIOC,GPIO_Pin_9);
}

void LCD5110_RST(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOC,GPIO_Pin_8);
	else GPIO_ResetBits(GPIOC,GPIO_Pin_8);
}

void LCD5110_DC(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOC,GPIO_Pin_7);
	else GPIO_ResetBits(GPIOC,GPIO_Pin_7);
}

void LCD5110_MO(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOC,GPIO_Pin_6);
	else GPIO_ResetBits(GPIOC,GPIO_Pin_6);
}

void LCD5110_SCK(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOA,GPIO_Pin_8);
	else GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}
