#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"

typedef struct dht_sensor dht_sensor;
typedef struct sds_sensor sds_sensor;
typedef struct dusty_sensor dusty_sensor;

struct dht_sensor{
   float temperature;
   float humi;		
};

struct sds_sensor{
   uint16_t pm25_value ;
   uint16_t pm10_value;		
};

struct dusty_sensor{
	dht_sensor  p_dht_ss;
	sds_sensor p_sds_ss;
};

volatile uint32_t usTicks;
volatile uint8_t sds_timeout;

#define DHT_RCC_GPIO														RCC_APB2Periph_GPIOA
#define DHT_GPIO																GPIOA
#define DHT_GPIO_Pin														GPIO_Pin_4
#define DHT_ERROR																0
#define DHT_OKE																	1

#define SIM808_TXD_PIN 												GPIO_Pin_9
#define SIM808_RXD_PIN 												GPIO_Pin_10
#define SIM808_USART_PORT						  				GPIOA
#define SIM808_USART_DEVICE	    							USART1
#define SIM808_USART_MODULE_CLK					      RCC_APB2Periph_USART1
#define SIM808_USART_CLK											RCC_APB2Periph_GPIOA
#define SIM808_BaudRate												115200

#define SDS_TXD_PIN 													GPIO_Pin_2
#define SDS_RXD_PIN														GPIO_Pin_3
#define SDS_USART_PORT						  					GPIOA
#define SDS_USART_DEVICE	   	 								USART2
#define SDS_USART_MODULE_CLK							    RCC_APB1Periph_USART2
#define SDS_USART_CLK								          RCC_APB2Periph_GPIOA
#define SDS_BaudRate													9600


#define LCD_RST_PIN														GPIO_Pin_14	
#define LCD_CE_PIN														GPIO_Pin_15
#define LCD_DC_PIN														GPIO_Pin_7
#define LCD_DIN_PIN														GPIO_Pin_8
#define LCD_CLK_PIN														GPIO_Pin_9

#define LCD_RST_PORT													GPIOB
#define LCD_CE_PORT														GPIOB
#define LCD_DC_PORT														GPIOB
#define LCD_DIN_PORT													GPIOB
#define LCD_CLK_PORT													GPIOB
#define LCD_PORT															GPIOB

#define N5110_set_Y_addr                                                               0x40
#define N5110_set_X_addr                                                               0x80

#define N5110_set_temp                                                                 0x04
#define N5110_set_bias                                                                 0x10
#define N5110_set_VOP                                                                  0x80

#define N5110_power_down                                                               0x04
#define N5110_entry_mode                                                               0x02
#define N5110_extended_instruction                                                     0x01

#define N5110_display_blank                                                            0x00
#define N5110_display_normal                                                           0x04
#define N5110_display_all_on                                                           0x01
#define N5110_display_inverted                                                         0x05

#define N5110_function_set                                                             0x20
#define N5110_display_control                                                          0x08

#define CMD                                                                              0
#define DAT                                                                              1

#define X_max                                                                            84
#define Y_max                                                                            48
#define Rows                                                                             6

#define BLACK                                                                            0
#define WHITE                                                                            1
#define PIXEL_XOR                                                                        2

#define ON                                                                               1
#define OFF                                                                              0

#define NO                                                                               0
#define YES                                                                              1

#define buffer_size                                                                      504

const char DEC_DIGIT[]  ={'0','1','2','3','4','5','6','7','8','9'};

  
static const unsigned char font[][5] =
{
     {0x00, 0x00, 0x00, 0x00, 0x00} // 20
    ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ?
    ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};


uint8_t DHT_GetTemHumi (dht_sensor* p_sensor); 

unsigned char N5110_buffer[X_max][Rows];
 volatile uint16_t data = 0;
 volatile uint8_t sds10_data[10] = {0};
 volatile uint8_t counter_byte = 0;
 volatile uint8_t received_flag  = 0;
 dusty_sensor p_dusty_sensor;

static void delay_ms(uint32_t ms);
static void delay_us(uint32_t us);
static void integer_to_string(uint32_t x, uint8_t* s);


static void sim808_init_rcc(void);
static void	sim808_init_gpio(void);
static void	sim808_init_interrupt(void);
static void	sim808_init_usart(void);
void sim808_init(void);
void sim808_call_phone(void);
void sim808_send_sms(char s[]);
void sim808_delete_sms(void);
void sim808_send_data_to_web(uint16_t pm10_value, uint16_t pm25_value);
void sim808_send_data_to_server(uint16_t pm10_value, uint16_t pm25_value);


static void SDS_init_rcc(void);
static void	SDS_init_gpio(void);
static void	SDS_init_interrupt(void);
static void	SDS_init_usart(void);
void SDS_init(void);

static void N5110_init_gpio(void);
static void N5110_write(unsigned char type, unsigned char value);
static void N5110_reset();
static void N5110_init();
static void N5110_set_contrast(unsigned char value);
static void N5110_set_cursor(unsigned char x_pos, unsigned char y_pos);
static void N5110_print_char(unsigned char ch, unsigned char colour);
static void N5110_print_custom_char(unsigned char *map);
static void N5110_fill(unsigned char bufr);
static void N5110_clear_buffer(unsigned char colour);
static void N5110_clear_screen(unsigned char colour);
static void N5110_print_image(const unsigned char *bmp);
static void N5110_print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
static void print_char(unsigned char x_pos, unsigned char y_pos, unsigned char ch, unsigned char colour);
static void print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
static void print_chr(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char colour);
static void print_int(unsigned char x_pos, unsigned char y_pos, signed long value, unsigned char colour);
static void print_decimal(unsigned char x_pos, unsigned char y_pos, unsigned int value, unsigned char points, unsigned char colour);
static void print_float(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points, unsigned char colour);
static void Draw_Pixel(unsigned char x_pos, unsigned char y_pos, unsigned char colour);


void SysTick_Handler()
{
	if(usTicks !=0)
	{
		usTicks--;
	}
}

void delayInit()
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000000);
}

void delay_us(uint32_t us)
{
	usTicks=us;
	while(usTicks);
}

void delay_ms(uint32_t ms)
{
	while(ms--)
	{
		delay_us(1000);
	}
}


static void integer_to_string(uint32_t x, uint8_t* s) 
{
	int tem[10], i = 0;
	if (x == 0)
		*s++ = '0';
	else {
		while (x) {
			tem[i++] = x % 10;
			x /= 10;
		}
		i--;
		while (i + 1) {
			*s++ = DEC_DIGIT[tem[i--]];
		}
	}
	*s = '\0';
}



static void N5110_init_gpio(void){
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	 GPIO_InitTypeDef  GPIO_Initstructure;
	
	 GPIO_Initstructure.GPIO_Pin = LCD_RST_PIN | LCD_CE_PIN | LCD_CLK_PIN | LCD_DC_PIN | LCD_DIN_PIN;
	 GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	 GPIO_Init(LCD_PORT,&GPIO_Initstructure);
	
   GPIO_SetBits(LCD_CE_PORT,LCD_CE_PIN);
   GPIO_SetBits(LCD_DC_PORT,LCD_DC_PIN);
   GPIO_ResetBits(LCD_RST_PORT,LCD_RST_PIN);
   GPIO_ResetBits(LCD_DIN_PORT,LCD_DIN_PIN);
   GPIO_ResetBits(LCD_CLK_PORT,LCD_CLK_PIN);

    delay_ms(10);
	
	}

	static void N5110_write(unsigned char type, unsigned char value)
{
     unsigned char s = 0x08;
     
     if(type != 0)
     {
         GPIO_SetBits(LCD_DC_PORT,LCD_DC_PIN);
     }
     else
     {
        GPIO_ResetBits(LCD_DC_PORT,LCD_DC_PIN);
     }

     GPIO_ResetBits(LCD_CE_PORT,LCD_CE_PIN);

     while(s > 0)
     {
         GPIO_ResetBits(LCD_CLK_PORT,LCD_CLK_PIN);
         
         if((value & 0x80) == 0)
         {
             GPIO_ResetBits(LCD_DIN_PORT,LCD_DIN_PIN);
         }
         else
         {
             GPIO_SetBits(LCD_DIN_PORT,LCD_DIN_PIN);
         }
         
         value <<= 1;
         GPIO_SetBits(LCD_CLK_PORT,LCD_CLK_PIN);
         s--;
     };
     
     GPIO_SetBits(LCD_CE_PORT,LCD_CE_PIN);
}


static void N5110_reset()
{
     GPIO_ResetBits(LCD_RST_PORT,LCD_RST_PIN);
     delay_us(100);
     GPIO_SetBits(LCD_RST_PORT,LCD_RST_PIN);
}


static void N5110_init()
{
    N5110_init_gpio();
    N5110_reset();
    N5110_write(CMD, (N5110_extended_instruction | N5110_function_set));
    N5110_write(CMD, (N5110_set_bias | 0x02));
    N5110_set_contrast(0x39);
    N5110_write(CMD, N5110_set_temp);
    N5110_write(CMD, (N5110_display_normal | N5110_display_control));
    N5110_write(CMD, N5110_function_set);
    N5110_write(CMD, N5110_display_all_on);
    N5110_write(CMD, N5110_display_normal);
    N5110_clear_buffer(OFF);
	  N5110_clear_screen(WHITE);
}




static void N5110_set_contrast(unsigned char value) 
{
    if(value >= 0x7F)
    {
       value = 0x7F;
    }

    N5110_write(CMD, (N5110_extended_instruction | N5110_function_set));
    N5110_write(CMD, (N5110_set_VOP | value));
    N5110_write(CMD, N5110_function_set);
}


static void N5110_set_cursor(unsigned char x_pos, unsigned char y_pos)
{
    N5110_write(CMD, (N5110_set_X_addr | x_pos));
    N5110_write(CMD, (N5110_set_Y_addr | y_pos));
}


static void N5110_print_char(unsigned char ch, unsigned char colour)
{
     unsigned char s = 0;
     unsigned char chr = 0;
     
     for(s = 0; s <= 4; s++)
     {
           chr = font[(ch - 0x20)][s];
           if(colour == BLACK)
           {
               chr = ~chr;
           }
           N5110_write(DAT, chr);
     }
}


static void N5110_print_custom_char(unsigned char *map)
{
    unsigned char s = 0;

    for(s = 0; s <= 4; s++)
    {
        N5110_write(DAT, *map++);
    }
}


static void N5110_fill(unsigned char bufr)
{
    unsigned int s = 0;

    N5110_set_cursor(0, 0);

    for(s = 0; s < buffer_size; s++)
    {
        N5110_write(DAT, bufr);
    }
}


static void N5110_clear_buffer(unsigned char colour)
{
    unsigned char x_pos = 0;
    unsigned char y_pos = 0;
    
    for(x_pos; x_pos < X_max; x_pos++)
    {
        for(y_pos; y_pos < Rows; y_pos++)
        {
            N5110_buffer[x_pos][y_pos] = colour;
        }
    }
}


static void N5110_clear_screen(unsigned char colour)
{
    unsigned char x_pos = 0;
    unsigned char y_pos = 0;
    
    for(y_pos = 0; y_pos < Rows; y_pos++)
    {
        for(x_pos = 0; x_pos < X_max; x_pos++)
        {
            N5110_print_string(x_pos, y_pos, " ", colour);
        }
    }
}


static void N5110_print_image(const unsigned char *bmp)
{
    unsigned int s = 0;
    
    N5110_set_cursor(0, 0);
    
    for(s = 0; s < buffer_size; s++)
    {
        N5110_write(DAT, bmp[s]);
    }
}


static void N5110_print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour)
{
    N5110_set_cursor(x_pos, y_pos);

    do
    {
       N5110_print_char(*ch++, colour);
    }while((*ch >= 0x20) && (*ch <= 0x7F));
}


static void print_chr(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char colour)
{
    unsigned char ch = 0x00;

    if(value < 0)
    {
        N5110_set_cursor(x_pos, y_pos);
        N5110_print_char(0x2D, colour);
        value = -value;
    }
    else
    {
        N5110_set_cursor(x_pos, y_pos);
        N5110_print_char(0x20, colour);
    }

     if((value > 99) && (value <= 999))
     {
         ch = (value / 100);
         N5110_set_cursor((x_pos + 6), y_pos);
         N5110_print_char((48 + ch), colour);
         
         ch = ((value % 100) / 10);
         N5110_set_cursor((x_pos + 12), y_pos);
         N5110_print_char((48 + ch), colour);

         ch = (value % 10);
         N5110_set_cursor((x_pos + 18), y_pos);
         N5110_print_char((48 + ch), colour);
     }
     else if((value > 9) && (value <= 99))
     {
         ch = ((value % 100) / 10);
         N5110_set_cursor((x_pos + 6), y_pos);
         N5110_print_char((48 + ch), colour);
         
         ch = (value % 10);
         N5110_set_cursor((x_pos + 12), y_pos);
         N5110_print_char((48 + ch), colour);

         N5110_set_cursor((x_pos + 18), y_pos);
         N5110_print_char(0x20, colour);
     }
     else if((value >= 0) && (value <= 9))
     {
         ch = (value % 10);
         N5110_set_cursor((x_pos + 6), y_pos);
         N5110_print_char((48 + ch), colour);
         
         N5110_set_cursor((x_pos + 12), y_pos);
         N5110_print_char(0x20, colour);
         
         N5110_set_cursor((x_pos + 18), y_pos);
         N5110_print_char(0x20, colour);
     }
}


static void print_int(unsigned char x_pos, unsigned char y_pos, signed long value, unsigned char colour)
{
    unsigned char ch = 0x00;

    if(value < 0)
    {
        N5110_set_cursor(x_pos, y_pos);
        N5110_print_char(0x2D, colour);
        value = -value;
    }
    else
    {
        N5110_set_cursor(x_pos, y_pos);
        N5110_print_char(0x20, colour);
    }

    if(value > 9999)
    {
        ch = (value / 10000);
        N5110_set_cursor((x_pos + 6), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 10000)/ 1000);
        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 1000) / 100);
        N5110_set_cursor((x_pos + 18), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        N5110_set_cursor((x_pos + 24), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = (value % 10);
        N5110_set_cursor((x_pos + 30), y_pos);
        N5110_print_char((48 + ch), colour);
    }

    else if((value > 999) && (value <= 9999))
    {
        ch = ((value % 10000)/ 1000);
        N5110_set_cursor((x_pos + 6), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 1000) / 100);
        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        N5110_set_cursor((x_pos + 18), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = (value % 10);
        N5110_set_cursor((x_pos + 24), y_pos);
        N5110_print_char((48 + ch), colour);

        N5110_set_cursor((x_pos + 30), y_pos);
        N5110_print_char(0x20, colour);
    }
    else if((value > 99) && (value <= 999))
    {
        ch = ((value % 1000) / 100);
        N5110_set_cursor((x_pos + 0), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        N5110_set_cursor((x_pos + 6), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = (value % 10);
        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char((48 + ch), colour);
        
        N5110_set_cursor((x_pos + 18), y_pos);
        N5110_print_char(0x20, colour);

        N5110_set_cursor((x_pos + 24), y_pos);
        N5110_print_char(0x20, colour);
    }
    else if((value > 9) && (value <= 99))
    {
        ch = ((value % 100) / 10);
        N5110_set_cursor((x_pos + 0), y_pos);
        N5110_print_char((48 + ch), colour);

        ch = (value % 10);
        N5110_set_cursor((x_pos + 6), y_pos);
        N5110_print_char((48 + ch), colour);

        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char(0x20, colour);
        
        N5110_set_cursor((x_pos + 18), y_pos);
        N5110_print_char(0x20, colour);
        
        N5110_set_cursor((x_pos + 24), y_pos);
        N5110_print_char(0x20, colour);
    }
    else
    {
        ch = (value % 10);
        N5110_set_cursor((x_pos + 6), y_pos);
        N5110_print_char((48 + ch), colour);
        
        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char(0x20, colour);
        
        N5110_set_cursor((x_pos + 18), y_pos);
        N5110_print_char(0x20, colour);
        
        N5110_set_cursor((x_pos + 24), y_pos);
        N5110_print_char(0x20, colour);
        
        N5110_set_cursor((x_pos + 30), y_pos);
        N5110_print_char(0x20, colour);
    }
}


static void print_decimal(unsigned char x_pos, unsigned char y_pos, unsigned int value, unsigned char points, unsigned char colour)
{
    unsigned char ch = 0x00;

    N5110_set_cursor(x_pos, y_pos);
    N5110_print_char(0x2E, colour);

    ch = (value / 1000);
    N5110_set_cursor((x_pos + 6), y_pos);
    N5110_print_char((48 + ch), colour);

    if(points > 1)
    {
        ch = ((value % 1000) / 100);
        N5110_set_cursor((x_pos + 12), y_pos);
        N5110_print_char((48 + ch), colour);


        if(points > 2)
        {
            ch = ((value % 100) / 10);
            N5110_set_cursor((x_pos + 18), y_pos);
            N5110_print_char((48 + ch), colour);

            if(points > 3)
            {
                ch = (value % 10);
                N5110_set_cursor((x_pos + 24), y_pos);
                N5110_print_char((48 + ch), colour);;
            }
        }
    }
}


static void print_float(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points, unsigned char colour)
{
    signed long tmp = 0x00;

    tmp = ((signed long)value);
    print_int(x_pos, y_pos, tmp, colour);
    tmp = ((value - tmp) * 10000);

    if(tmp < 0)
    {
       tmp = -tmp;
    }

    if((value >= 9999) && (value < 99999))
    {
        print_decimal((x_pos + 36), y_pos, tmp, points, colour);
    }
    else if((value >= 999) && (value < 9999))
    {
        print_decimal((x_pos + 30), y_pos, tmp, points, colour);
    }
    else if((value >= 99) && (value < 999))
    {
        print_decimal((x_pos + 24), y_pos, tmp, points, colour);
    }
    else if((value >= 9) && (value < 99))
    {
        print_decimal((x_pos + 18), y_pos, tmp, points, colour);
    }
    else if(value < 9)
    {
        print_decimal((x_pos + 12), y_pos, tmp, points, colour);
        if((value) < 0)
        {
            N5110_set_cursor(x_pos, y_pos);
            N5110_print_char(0x2D, colour);
        }
        else
        {
            N5110_set_cursor(x_pos, y_pos);
            N5110_print_char(0x20, colour);
        }
    }
}



void DHT11_init(void)
{
	RCC_APB2PeriphClockCmd(DHT_RCC_GPIO | RCC_APB2Periph_AFIO, ENABLE);
}


uint8_t DHT_GetTemHumi(dht_sensor* p_sensor)
{
	
    GPIO_InitTypeDef GPIO_InitStructure;

    	uint8_t buffer[5]={0,0,0,0,0};
    	uint8_t ii,i,checksum;
    	int timeout=0;

    	GPIO_InitStructure.GPIO_Pin=DHT_GPIO_Pin;
    	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
    	GPIO_Init(DHT_GPIO,&GPIO_InitStructure);

    	GPIO_SetBits(DHT_GPIO,DHT_GPIO_Pin);
    	delay_us(60);
    	GPIO_ResetBits(DHT_GPIO,DHT_GPIO_Pin);
    	delay_ms(25);
    	GPIO_SetBits(DHT_GPIO,DHT_GPIO_Pin);

    	GPIO_InitStructure.GPIO_Pin=DHT_GPIO_Pin;
    	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
    	GPIO_Init(DHT_GPIO,&GPIO_InitStructure);

    	delay_us(50);////60->50
      if(GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==1) return DHT_ERROR;
    	else
    	{
    		timeout=480;
    		while((GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==0)&&((timeout--)>0));
    		if(timeout<=0) return DHT_ERROR;
    	}
    	delay_us(50);///60->50
    	if(GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==0) return DHT_ERROR;
    	else
    	{
    		timeout=480;
    		while((GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==1)&&((timeout--)>0));
    		if(timeout<=0) return DHT_ERROR;
    	}

    	for(i=0;i<5;i++)
    	{
    		for(ii=0;ii<8;ii++)
    		{
    			timeout=480;
    			while((GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==0)&&((timeout--)>0));
    			if(timeout<=0) return DHT_ERROR;
    			delay_us(50);//50->40
    			if(GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==1)
    			{
    				buffer[i]|=(1<<(7-ii));
    				timeout=480;
    				while((GPIO_ReadInputDataBit(DHT_GPIO,DHT_GPIO_Pin)==1)&&((timeout--)>0));
    				if(timeout<=0) return DHT_ERROR;
    			}
    		}
    	}

    	checksum=buffer[0]+buffer[1]+buffer[2]+buffer[3];
    	if((checksum)!=buffer[4]) return DHT_ERROR;
    	p_sensor->temperature=(float)(buffer[2]*256.0+buffer[3])/10.0;
    	p_sensor->humi=(float)(buffer[0]*256.0+buffer[1])/10.0;

    	return DHT_OKE;
}



static void sim808_init_rcc(void) {
	RCC_APB2PeriphClockCmd(SIM808_USART_CLK|SIM808_USART_MODULE_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}


static void sim808_init_gpio(void) {
		
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = SIM808_TXD_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SIM808_USART_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = SIM808_RXD_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SIM808_USART_PORT, &GPIO_InitStruct);

}


static void sim808_init_usart(void) {

	USART_InitTypeDef usart_initstruct;

	usart_initstruct.USART_BaudRate = SIM808_BaudRate;
	usart_initstruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_initstruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart_initstruct.USART_Parity = USART_Parity_No;
	usart_initstruct.USART_StopBits = USART_StopBits_1;
	usart_initstruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(SIM808_USART_DEVICE, &usart_initstruct);

	USART_ClearFlag(SIM808_USART_DEVICE, USART_FLAG_RXNE);
	USART_ITConfig(SIM808_USART_DEVICE, USART_IT_RXNE, ENABLE);
	USART_Cmd(SIM808_USART_DEVICE, ENABLE);
	
}

static void sim808_init_interrupt(void) {
	NVIC_InitTypeDef nvic_usart_initstruct; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	nvic_usart_initstruct.NVIC_IRQChannel = USART1_IRQn;
	nvic_usart_initstruct.NVIC_IRQChannelSubPriority = 1;
	nvic_usart_initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_usart_initstruct);
	
}


void sim808_usart_send_char(uint8_t Data) {
	while (USART_GetFlagStatus(SIM808_USART_DEVICE, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(SIM808_USART_DEVICE, Data);
	
}


void sim808_usart_send_string(uint8_t* str) {
	while (*str != '\0' )
		sim808_usart_send_char(*str++);
}



static void SDS_init_rcc(void) {
	RCC_APB1PeriphClockCmd(SDS_USART_MODULE_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(SDS_USART_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
}


static void SDS_init_gpio(void) {
	GPIO_InitTypeDef  GPIO_InitStruct;	
	
	GPIO_InitStruct.GPIO_Pin = SDS_TXD_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SDS_USART_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = SDS_RXD_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SDS_USART_PORT, &GPIO_InitStruct);
	
}


static void SDS_init_usart(void) {

	USART_InitTypeDef usart_initstruct;
	usart_initstruct.USART_BaudRate = SDS_BaudRate;
	usart_initstruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_initstruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart_initstruct.USART_Parity = USART_Parity_No;
	usart_initstruct.USART_StopBits = USART_StopBits_1;
	usart_initstruct.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(SDS_USART_DEVICE, &usart_initstruct);
	USART_ClearFlag(SDS_USART_DEVICE, USART_FLAG_RXNE);
	USART_ITConfig(SDS_USART_DEVICE, USART_IT_RXNE, ENABLE);
	USART_Cmd(SDS_USART_DEVICE, ENABLE);
	
}

static void SDS_init_interrupt(void) {
	NVIC_InitTypeDef nvic_usart_initstruct; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	nvic_usart_initstruct.NVIC_IRQChannel = USART2_IRQn;
	nvic_usart_initstruct.NVIC_IRQChannelSubPriority = 2;
	nvic_usart_initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_usart_initstruct);

}


void SDS_usart_send_char(uint8_t Data) {
	while (USART_GetFlagStatus(SDS_USART_DEVICE, USART_FLAG_TXE) == RESET);
	USART_SendData(SDS_USART_DEVICE, Data);
}

void SDS_usart_send_string(uint8_t* str) {
	while (*str != '\0' )
		SDS_usart_send_char(*str++);
}


void update_data(dusty_sensor* p_ss){
	
				char s1[10],s2[20],s3[10],s4[10];
	
				integer_to_string((uint32_t)p_ss->p_dht_ss.temperature,&s1[0]);
				integer_to_string((uint32_t)p_ss->p_dht_ss.humi,&s2[0]);
				integer_to_string(p_ss->p_sds_ss.pm10_value,&s3[0]);
				integer_to_string(p_ss->p_sds_ss.pm25_value,&s4[0]);
		
				sim808_usart_send_string("AT+HTTPDATA=200,10000");
				sim808_usart_send_string("\r\n");
	      delay_ms(1000);
				sim808_usart_send_string("{ \"deviceId\": 199, \"timeStamp\": 231095, \"temperature\": ");
				sim808_usart_send_string(s1);
				sim808_usart_send_string(", \"humidity\": ");
				sim808_usart_send_string(s2);
				sim808_usart_send_string(", \"pm25\": ");
				sim808_usart_send_string(s4);
				sim808_usart_send_string(", \"pm10\":");
				sim808_usart_send_string(s3);
				sim808_usart_send_string(", \"act1State\": 1, \"act2State\": 1 }");
		   	sim808_usart_send_string("\r\n");
	      delay_ms(12000);
				sim808_usart_send_string("AT+HTTPACTION=1");
				sim808_usart_send_string("\r\n");
	      delay_ms(3000);
}

void sds_update_data(sds_sensor* p_sensor)
{
	sim808_send_data_to_server(p_sensor->pm10_value,p_sensor->pm25_value);
}

void SDS_init(void)
{
	SDS_init_rcc();
	SDS_init_gpio();
	SDS_init_usart();
	SDS_init_interrupt();
}
	


void sim808_init(void)
{
	sim808_init_rcc();
	sim808_init_gpio();
	sim808_init_usart();
	sim808_init_interrupt();
	delay_ms(100);
	sim808_usart_send_string("AT\r\n");    										// check for module's status
	delay_ms(20);
	sim808_usart_send_string("ATE0\r\n");											// disable echo mode 
	delay_ms(20);
	sim808_usart_send_string("AT+CMGF=1\r\n");
	delay_ms(20);
	sim808_usart_send_string("AT+IPR=115200\r\n");	          // sim808_BaudRate = 115200 
	delay_ms(20);
	sim808_usart_send_string("AT+CNMI=2,2,0,0\r\n");
	delay_ms(20);
	sim808_usart_send_string("AT&W\r\n");                     // save data
	delay_ms(200);	
  sim808_usart_send_string("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
	sim808_usart_send_string("AT+SAPBR=3,1,\"APN\",\"CMNET\"");
	sim808_usart_send_string("\r\n");
	delay_ms(1000);
	sim808_usart_send_string("AT+SAPBR=1,1");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
  sim808_usart_send_string("AT+SAPBR=2,1");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
	sim808_usart_send_string("AT+HTTPINIT");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
	sim808_usart_send_string("AT+HTTPPARA=\"CID\",1");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
	sim808_usart_send_string("AT+HTTPPARA=\"URL\",\"http://110.76.86.170:9000/data/\"");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
	sim808_usart_send_string("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
	sim808_usart_send_string("\r\n");
	delay_ms(200);
}

void sim808_call_phone(void)
{    // call 2 times 
	sim808_usart_send_string("ATD");
	sim808_usart_send_string("+84945947698");
	sim808_usart_send_string(";\r\n");
	delay_ms(20000);
	sim808_usart_send_string("ATH\r\n");	
}

void sim808_send_sms(char s[])
{
		sim808_usart_send_string("AT+CMGF=1");
		sim808_usart_send_char(0x0D);
		delay_ms(2000);
		sim808_usart_send_string("AT+CMGS=\"0945947698\"");
		sim808_usart_send_char(0x0D);
		delay_ms(2000);
		sim808_usart_send_string((uint8_t*)s);
		sim808_usart_send_char(0x1A);
		delay_ms(1000);
}


void lcd_display(void){
		 N5110_clear_screen(WHITE);
	   N5110_print_string(12, 0,"DAI HOC BKHN", WHITE);
		 N5110_print_string(18, 1,"THIET BI 1", WHITE);
	   N5110_print_string(0, 2,"PM10 :", WHITE);
     N5110_print_string(0, 3,"PM2.5:", WHITE);
		 N5110_print_string(0, 4,"Tem  :", WHITE);
		 N5110_print_string(0, 5,"Hum  :", WHITE);
     N5110_print_string(59, 2,"ug/m3", WHITE);
     N5110_print_string(59, 3,"ug/m3", WHITE);
     N5110_print_string(63, 4,"*C", WHITE); 
		 N5110_print_string(63, 5,"%", WHITE);  	
		 print_int(32, 2,p_dusty_sensor.p_sds_ss.pm10_value, WHITE);  
		 print_int(32, 3,p_dusty_sensor.p_sds_ss.pm25_value, WHITE); 
	   print_int(32, 4,(int32_t)p_dusty_sensor.p_dht_ss.temperature, WHITE);  
		 print_int(32, 5,(int32_t)p_dusty_sensor.p_dht_ss.humi, WHITE); 
}

int main()
{  
     SystemInit();
	   delayInit();
  	 DHT11_init();
		 SDS_init();
	 	 sim808_init();
     N5110_init();
		 N5110_clear_screen(WHITE);
       

while(1)
	{
		while(DHT_GetTemHumi(&p_dusty_sensor.p_dht_ss) == DHT_ERROR );
		
		if(received_flag)
			  {
				  p_dusty_sensor.p_sds_ss.pm25_value = (uint16_t)((sds10_data[3] *256)+sds10_data[2])/10;
				  p_dusty_sensor.p_sds_ss.pm10_value = (uint16_t)((sds10_data[5] *256)+sds10_data[4])/10;
				  received_flag = 0;
				  update_data(&p_dusty_sensor);
					lcd_display();
					
				}
	}

}
void USART2_IRQHandler(void) {
		  sds10_data[counter_byte] = USART_ReceiveData(SDS_USART_DEVICE );
	    if(counter_byte == 9){
			    if((sds10_data[0] == 0xAA) && (sds10_data[1] == 0xC0) && (sds10_data[9] == 0xAB) && (sds10_data[8] ==(uint8_t)(sds10_data[2]+sds10_data[3]+sds10_data[4]+sds10_data[5]+sds10_data[6]+sds10_data[7]))){
					    received_flag = 1;	
					sds_timeout=0;
			   }
					else 
						{
							if (sds_timeout++ == 10)
{
	//
}								
							}
			    counter_byte = 0;		
			}
		else{
			counter_byte++;
			}
		USART_ClearFlag	(SDS_USART_DEVICE, USART_IT_RXNE);
}	


void USART1_IRQHandler(void) {
	data = USART_ReceiveData(SIM808_USART_DEVICE);
USART_ClearFlag(SIM808_USART_DEVICE, USART_IT_RXNE);
}	









