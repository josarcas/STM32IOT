/*
 * lcd_driver.c
 *
 *  Created on: Oct 29, 2021
 *      Author: JoseCarlos
 */

/*INCLUDES********************************************************************/
#include "lcd_driver.h"
#include "widget_map.h"
#include "math.h"

/*DEFINES*********************************************************************/
#define RD_ACTIVE  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0)
#define RD_IDLE    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1)
#define WR_ACTIVE  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0)
#define WR_IDLE    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1)
#define CD_COMMAND			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0)
#define CD_DATA				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1)
#define CS_ACTIVE  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0)
#define CS_IDLE    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1)

#define WR_STROBE		    {WR_ACTIVE;WR_IDLE;}

#define TFTWIDTH   240
#define TFTHEIGHT  320

#define ILI9341_SOFTRESET          0x01
#define ILI9341_SLEEPIN            0x10
#define ILI9341_SLEEPOUT           0x11
#define ILI9341_NORMALDISP         0x13
#define ILI9341_INVERTOFF          0x20
#define ILI9341_INVERTON           0x21
#define ILI9341_GAMMASET           0x26
#define ILI9341_DISPLAYOFF         0x28
#define ILI9341_DISPLAYON          0x29
#define ILI9341_COLADDRSET         0x2A
#define ILI9341_PAGEADDRSET        0x2B
#define ILI9341_MEMORYWRITE        0x2C
#define ILI9341_PIXELFORMAT        0x3A
#define ILI9341_FRAMECONTROL       0xB1
#define ILI9341_DISPLAYFUNC        0xB6
#define ILI9341_ENTRYMODE          0xB7
#define ILI9341_POWERCONTROL1      0xC0
#define ILI9341_POWERCONTROL2      0xC1
#define ILI9341_VCOMCONTROL1       0xC5
#define ILI9341_VCOMCONTROL2       0xC7
#define ILI9341_MEMCONTROL         0x36
#define ILI9341_MADCTL  		   0x36
#define ILI9341_MADCTL_MY  		   0x80
#define ILI9341_MADCTL_MX  		   0x40
#define ILI9341_MADCTL_MV  		   0x20
#define ILI9341_MADCTL_ML  		   0x10
#define ILI9341_MADCTL_RGB 		   0x00
#define ILI9341_MADCTL_BGR         0x08
#define ILI9341_MADCTL_MH          0x04

/*CONSTANTS*******************************************************************/
const uint16_t spf_pin[] ={
		GPIO_PIN_2,
		GPIO_PIN_15,
		GPIO_PIN_14,
		GPIO_PIN_0,
		GPIO_PIN_3,
		GPIO_PIN_4,
		GPIO_PIN_1,
		GPIO_PIN_4
};

GPIO_TypeDef * spf_port[]={
		GPIOB,
		GPIOA,
		GPIOD,
		GPIOB,
		GPIOA,
		GPIOB,
		GPIOB,
		GPIOA
};

/*PROTOTYPES******************************************************************/
void write8(uint8_t info);
void writeRegister8(uint8_t reg, uint8_t data);
void writeRegister16(uint16_t reg, uint16_t data);
void writeRegister32(uint8_t reg, uint32_t d);
void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d) ;
void set_cursor(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

//TOUCH PROTOTYPES
void gpio_pin_mode(GPIO_TypeDef *port  , uint16_t pin, uint8_t mode);

/*FUNCTIONS*******************************************************************/
void write8(uint8_t info)
{

	for(uint8_t i=0; i<8; i++)
	{
		HAL_GPIO_WritePin(spf_port[i], spf_pin[i], info>>i & 0x01);
	}
	WR_STROBE;
}

void writeRegister8(uint8_t reg, uint8_t data)
{
	CD_COMMAND;
	write8(reg);
	CD_DATA;
	write8(data);
}

void writeRegister16(uint16_t reg, uint16_t data)
{
	CD_COMMAND;
	write8(reg>>8);
	write8(reg);
	CD_DATA;
	write8(data>>8);
	write8(data);
}

void writeRegister32(uint8_t reg, uint32_t data)
{
	CS_ACTIVE;
	CD_COMMAND;
	write8(reg);
	CD_DATA;
	write8(data >> 24);
	write8(data >> 16);
	write8(data >> 8);
	write8(data);
	CS_IDLE;

}

void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d)
{
	uint8_t hi = (d) >> 8, lo = (d);
	CD_COMMAND; write8(aH); CD_DATA; write8(hi);
	CD_COMMAND; write8(aL); CD_DATA; write8(lo);
}


void set_cursor(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{

	uint32_t t;

	t = x0;
	t <<= 16;
	t |= x1;
	writeRegister32(ILI9341_COLADDRSET, t);
	t = y0;
	t <<= 16;
	t |= y1;
	writeRegister32(ILI9341_PAGEADDRSET, t);
}

void lcd_init()
{
	lcd_reset();
	HAL_Delay(20);

	CS_ACTIVE;
	writeRegister8(ILI9341_SOFTRESET, 0);
	HAL_Delay(50);
	writeRegister8(ILI9341_DISPLAYOFF, 0);

	writeRegister8(ILI9341_POWERCONTROL1, 0x23);
	writeRegister8(ILI9341_POWERCONTROL2, 0x10);
	writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
	writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
	writeRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
	writeRegister8(ILI9341_PIXELFORMAT, 0x55);
	writeRegister16(ILI9341_FRAMECONTROL, 0x001B);

	writeRegister8(ILI9341_ENTRYMODE, 0x07);


	writeRegister8(ILI9341_SLEEPOUT, 0);
	HAL_Delay(150);
	writeRegister8(ILI9341_DISPLAYON, 0);
	HAL_Delay(500);
	set_cursor(0, 0, TFTWIDTH-1, TFTHEIGHT-1);

    return;
}

void lcd_reset(void)
{
	CS_IDLE;
	WR_IDLE;
	RD_IDLE;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

	CS_ACTIVE;
	CD_COMMAND;
	write8(0x00);
	for(uint8_t i=0; i<3; i++) WR_STROBE;
	CS_IDLE;
}

void lcd_clear(uint16_t color)
{
	uint8_t hi = color>>8;
	uint8_t lo = color;

	set_cursor(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(uint16_t i=0; i<LCD_WIDTH; i++)
	{
		for(uint16_t j=0; j<LCD_HEIGHT; j++)
		{
			write8(hi);
			write8(lo);
		}
	}
	CS_IDLE;

}

void lcd_set_pixel(uint16_t x, uint16_t y, uint16_t color)
{

	set_cursor(x, y, x, y);

	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	write8(color>>8);
	write8(color);
	CS_IDLE;

}

void lcd_set_window_color(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t color)
{
	uint8_t hi = color>>8;
	uint8_t lo = color;

	set_cursor(x0, y0, x1-1, y1-1);

	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(uint16_t i=x0; i<x1; i++)
	{
		for(uint16_t j=y0; j<y1; j++)
		{
			write8(hi);
			write8(lo);
		}
	}
	CS_IDLE;

}

void lcd_draw_string(char *str, uint8_t size, uint16_t x, uint16_t y,
		uint16_t color, font_t font)
{
	uint16_t page;

	switch (font)
	{
		case font8:
		{
			for(uint8_t i=0; i<size; i++)
			{
				x+=8;
				page = (str[i]-32)*8;
				for(uint8_t j=0; j<8; j++)
				{
					for(uint8_t k=0; k<8; k++)
					{
						if((font8_table[page]>>k)& 0x01)
							lcd_set_pixel(x-k, j+y, color);
					}
					page++;
				}
			}

		}
		break;

		case font12:
		{
			for(uint8_t i=0; i<size; i++)
			{
				x+=8;
				page = (str[i]-32)*12;
				for(uint8_t j=0; j<12; j++)
				{
					for(uint8_t k=0; k<8; k++)
					{
						if((font12_table[page]>>k)& 0x01)
							lcd_set_pixel(x-k, j+y, color);
					}
					page++;
				}
			}
		}
		break;

		case font16:
		{
			for(uint8_t i=0; i<size; i++)
			{
				//x+=16;
				x+=13;
				page = (str[i]-32)*32;
				for(uint8_t j=0; j<16; j++)
				{
					page++;
					for(uint8_t k=0; k<8; k++)
					{
						if((font16_table[page]>>k)& 0x01)
							lcd_set_pixel(x-k, j+y, color);
						if((font16_table[page-1]>>k)& 0x01)
							lcd_set_pixel((x-8)-k, j+y, color);
					}
					page++;
				}
			}
		}
		break;

		case font20:
		{
			for(uint8_t i=0; i<size; i++)
			{
				x+=16;
				page = (str[i]-32)*40;
				for(uint8_t j=0; j<20; j++)
				{
					page++;
					for(uint8_t k=0; k<8; k++)
					{
						if((font20_table[page]>>k)& 0x01)
							lcd_set_pixel(x-k, j+y, color);
						if((font20_table[page-1]>>k)& 0x01)
							lcd_set_pixel((x-8)-k, j+y, color);
					}
					page++;
				}
			}
		}
		break;

		case font24:
		{
			for(uint8_t i=0; i<size; i++)
			{
				x+=24;
				page = (str[i]-32)*47;
				for(uint8_t j=0; j<24; j++)
				{
					page+=2;
					for(uint8_t k=0; k<8; k++)
					{
						if((font24_table[page]>>k)& 0x01)
							lcd_set_pixel(x-k, j+y, color);
						if((font24_table[page-1]>>k)& 0x01)
							lcd_set_pixel((x-8)-k, j+y, color);
						if((font24_table[page-2]>>k)& 0x01)
							lcd_set_pixel((x-16)-k, j+y, color);
					}
					page++;
				}
			}
		}
		break;

	}
}

void lcd_draw_image(const uint16_t *image, uint16_t x, uint16_t y, uint16_t width,
		uint16_t height)
{
	uint16_t page=0;

	set_cursor(x, y, x+width-1, y+height-1);

	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(uint16_t i=0; i< height; i++)
	{
		for(uint16_t j= 0; j<width; j++)
		{
			write8(image[page+j]>>8);
			write8(image[page+j]);
		}
		page += width;
	}
	CS_IDLE;

}

void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t color)
{
	int slope;
	int dx, incE, incNE, x, y;
	int dy, d;

	if(x0 > x1)
	{
		lcd_draw_line(x1, y1, x0, y0, color);
		return;
	}

	dx = x1 - x0;
	dy = y1 - y0;

	if(dy < 0)
	{
		slope = -1;
		dy = -dy;
	}

	else
		slope = 1;

	incE = 2 * dy;
	incNE = 2 * dy - 2 * dx;
	d = 2 * dy - dx;
	y = y0;

	for(x=x0; x<=x1; x++)
	{
		lcd_set_pixel(x, y, color);

		if(d <= 0)
			d += incE;

		else
		{
			d += incNE;
			y += slope;
		}
	}
}

void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
		uint16_t color)
{

  	uint8_t i;

  	uint8_t hi = color>>8;
  	uint8_t lo = color;

  	height--;
  	width--;

	set_cursor(x, y, x, y+height);
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(i=0; i<=height; i++)
	{
		write8(hi);
		write8(lo);
	}

	set_cursor(x+width, y, x+width, y+height);
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(i=0; i<=height; i++)
	{
		write8(hi);
		write8(lo);
	}

	set_cursor(x, y, x+width, y);
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(i=0; i<=width; i++)
	{
		write8(hi);
		write8(lo);
	}

	set_cursor(x, y+height, x+width, y+height);
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x2C);
	CD_DATA;
	for(i=0; i<=width; i++)
	{
		write8(hi);
		write8(lo);
	}

}

void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t rad, uint16_t color)
{
	int f = 1-rad;
	int ddf_x = 1;
	int ddf_y = -2*rad;
	int x_pos = 0;
	int y_pos = rad;

	lcd_set_pixel(x, y+rad, color);
	lcd_set_pixel(x, y-rad, color);
	lcd_set_pixel(x-rad, y, color);
	lcd_set_pixel(x+rad, y, color);

	while(x_pos<y_pos)
	{
		if(f >= 0)
		{
			y_pos--;
			ddf_y += 2;
			f += ddf_y;
		}

		x_pos++;
		ddf_x += 2;
		f += ddf_x;

		lcd_set_pixel(x+x_pos, y+y_pos, color);
		lcd_set_pixel(x-x_pos, y+y_pos, color);
		lcd_set_pixel(x+x_pos, y-y_pos, color);
		lcd_set_pixel(x-x_pos, y-y_pos, color);
		lcd_set_pixel(x+y_pos, y+x_pos, color);
		lcd_set_pixel(x-y_pos, y+x_pos, color);
		lcd_set_pixel(x+y_pos, y-x_pos, color);
		lcd_set_pixel(x-y_pos, y-x_pos, color);

	}
}

void lcd_draw_fill_circle(uint16_t x, uint16_t y, uint16_t rad, uint16_t color)
{
    int16_t x_pos, y_pos;
    x_pos = 0;
    y_pos = rad;

    int16_t esp = 3 - (rad << 1 );

    int16_t s_count_y;

	while(x_pos <= y_pos )
	{
		for(s_count_y = x_pos; s_count_y <= y_pos; s_count_y ++ )
		{
			lcd_set_pixel(x + x_pos, y + s_count_y, color);
			lcd_set_pixel(x - x_pos, y + s_count_y, color);
			lcd_set_pixel(x - s_count_y, y + x_pos, color);
			lcd_set_pixel(x - s_count_y, y - x_pos, color);
			lcd_set_pixel(x - x_pos, y - s_count_y, color);
			lcd_set_pixel(x + x_pos, y - s_count_y, color);
			lcd_set_pixel(x + s_count_y, y - x_pos, color);
			lcd_set_pixel(x + s_count_y, y + x_pos, color);
		}
		if(esp < 0 )
			esp += 4 * x_pos + 6;
		else
		{
			esp += 10 + 4 * (x_pos - y_pos );
			y_pos --;
		}
		x_pos ++;
	}
}

void lcd_draw_batery_widget(uint16_t x, uint16_t y, uint8_t level)
{
	uint16_t color;

	if(level<30)
		color = RED;
	else if(level< 60)
		color = YELLOW;
	else
		color = GREEN;

	lcd_draw_rect(x, y, 21, 12, WHITE);
	level = level*2/10;
	x++;
	y++;
	lcd_set_window_color(x, y, x+level, y+10, color);
	lcd_set_window_color(x+level, y, x+19 , y+10, BLACK);
	lcd_set_window_color(x+20, y+3, x+23, y+7, WHITE);

}

/*
void lcd_set_rotation(uint8_t rotation)
{
	   uint16_t buffer;

	   switch (rotation) {
	   case 2:
		   buffer = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
	     break;
	   case 3:
		   buffer = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
	     break;
	  case 0:
		  buffer = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
	    break;
	   case 1:
		   buffer = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
	     break;
	  }
	   writeRegister8(ILI9341_MADCTL, buffer);
	   set_cursor(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
}
*/

void lcd_draw_wifi_signal_widget(uint16_t x,uint16_t y, int8_t level)
{
	uint16_t color;
	//uint8_t signal_lines;

	int f;
	int ddf_x;
	int ddf_y;
	int x_pos;
	int y_pos;

	lcd_set_pixel(x, y, WHITE);

	int rad = 4;//primer radio

	while(rad<=12)//radio maximo
	{
		f = 1-rad;
		ddf_x = 1;
		ddf_y = -2*rad;
		x_pos = 0;
		y_pos = rad;

		if((level+=40)<0)
			color = WHITE;
		else
			color = 0x8410;

		lcd_set_pixel(x, y-rad, color);
		lcd_set_pixel(x-rad, y, color);

		while(x_pos<y_pos)
		{
			if(f >= 0)
			{
				y_pos--;
				ddf_y += 2;
				f += ddf_y;
			}

			x_pos++;
			ddf_x += 2;
			f += ddf_x;

			lcd_set_pixel(x-x_pos, y-y_pos, color);
			lcd_set_pixel(x-y_pos, y-x_pos, color);
		}

		rad+=4;//incremento del radio

	}
}


void lcd_draw_circular_load_widget(uint16_t x, uint16_t y, uint16_t rad,
		uint8_t percent, uint16_t color, uint16_t background_color)
{
    float theta = 0;
    int x_pos = rad;
    int y_pos = 0;
    float xd;
    float yd;
    float increment = (float)3*M_PI/(float)180;

    for(uint8_t i=0; i<120; i++)
    {
    	if(i<percent)
    		lcd_set_pixel(x + x_pos, y + y_pos, color);
    	else
    		lcd_set_pixel(x + x_pos, y + y_pos, background_color);

        theta = theta + increment;
        xd = rad * cos(theta);
        x_pos = round(xd);
        yd = rad * sin(theta);
        y_pos = yd;
    }
}

void lcd_draw_rect_load_widget(uint16_t x, uint16_t y, uint8_t percent,
		uint16_t color, uint16_t left_color, uint16_t right_color)
{
	lcd_draw_rect(x, y, 102, 21, color);

	x++;
	y++;

	char buffer[5];
	lcd_set_window_color(x, y, x+percent, y+19, left_color);
	lcd_set_window_color(x+percent, y, x+100, y+19, right_color);

	sprintf(buffer, "%d%%", percent);
	lcd_draw_string(buffer, strlen(buffer), (x+50)-((strlen(buffer)*7)/2), y+4, color, font12);

}

/*
float u(float j, float x, float k, float y)
{
	return sqrt(pow(j-x,2)+pow(k-y,2));
}

float MAX(float a, float b)
{
	return a>b? a:b;
}


void lcd_draw_arc(int j, int k, int l, int m, int n, int o, uint16_t color)
{
	float error = 0;
	int x = j;
	int y = k;

	float c0 = (k-m)*pow(u(j,n,k,o), 2) - (k-o)*pow(u(j,l,k,m),2);
	float c1 = (j-l)*pow(u(j,n,k,o),2) - (j-n)*pow(u(j,l,k,m),2);
	float tau = (float)(MAX(fabs(c0), fabs(c1)))/2*(float)(fabs(j-l)+fabs(k-m));
	float h = j- (float)c0/(float)tau;
	float i = k+(float)c1/(float)tau;
	float alpha = pow(u(j,n,k,o),2);
	float beta = pow(u(h,n,i,o),2)-pow(u(h,j,i,k),2);
	float diff_err_x = alpha*(l+h-j)-beta;
	float diff_err_y = alpha*(l-i+k)-beta;
	float test_error;

	do
	{
		lcd_set_pixel(x, y, color);
		y++;
		error+=diff_err_y;
		diff_err_y += 2*(alpha-beta);
		test_error = error+diff_err_x;

		if(fabs(test_error)<fabs(error))
		{
			x--;
			error = test_error;
			diff_err_x += 2*(alpha-beta);

			if(-diff_err_y>diff_err_x)
				return;
		}

	}while(x != n || y!= n);
}
*/



uint16_t convert_from_rgb_8(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t r_5 = r>>3;
	uint8_t g_6 = g>>2;
	uint8_t b_5 = b>>3;

	return (r_5<<11) | (g_6<<5) | (b_5);
}

void lcd_draw_message_widget(uint16_t x, uint16_t y, uint8_t n_messages)
{
	lcd_set_window_color(x, y, x+16, y+10, WHITE);

	lcd_draw_line(x, y, x+(16/2), y+(10/2), BLACK);
	lcd_draw_line(x+15, y, x+(16/2), y+(10/2), BLACK);

	lcd_draw_line(x, y+9, x+(16/2)-2, y+(10/2)-2, BLACK);
	lcd_draw_line(x+15, y+9, x+(16/2)+2, y+(10/2)-2, BLACK);

	if(n_messages)
		lcd_draw_fill_circle(x+15, y+2, 2, RED);

}



//TOUCH PROTOTYPES
void gpio_pin_mode(GPIO_TypeDef *port  , uint16_t pin, uint8_t mode)
{
	HAL_GPIO_DeInit(port, pin);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	if(mode)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	}

	else
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}

	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}
