#ifndef _OLED_H
#define _OLED_H

//#include "common.h"

//sbit OLED_DC=P0^0;
//sbit OLED_RST=P0^1;   
//sbit OLED_SCL=P0^3;   
//sbit OLED_SDA=P0^2;   
#include "fsl_common.h"
#include "fsl_snvs_hp.h"

#define OLED_SCL_0    GPIO5->DR  &= ~(1 << 2)
#define OLED_SCL_1    GPIO5->DR  |=  (1 << 2)
#define OLED_SDA_0    GPIO3->DR  &= ~(1 << 4)
#define OLED_SDA_1    GPIO3->DR  |= (1 << 4)
#define OLED_RST_0    GPIO3->DR  &= ~(1 << 2)
#define OLED_RST_1    GPIO3->DR  |= (1 << 2)
#define OLED_DC_0     GPIO2->DR  &= ~(1 << 30)
#define OLED_DC_1     GPIO2->DR  |= (1 << 30)
#define OLED_CS_0     GPIO1->DR  &= ~(1 << 20)
#define OLED_CS_1     GPIO1->DR  |= (1 << 20)

void  msDelay(uint32_t  ulDly);


// this sentence can be regard as BIT(x);  1<<(x)


 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_Fill(uint8_t dat);
 void OLED_PutPixel(uint8_t x,uint8_t y);
 void OLED_DrawBMP(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1);
 /* ������ʾ */
void OLED_P14x16Ch(uint8_t x,uint8_t y,uint8_t ch[]);
void OLED_Print(uint8_t x, uint8_t y, uint8_t ch[]);
void OLED_P16x16Str(uint8_t x,uint8_t y,const uint8_t ch[][32],uint8_t len);
 /* �ַ���ʾ */
 void OLED_P6x8Str(uint8_t x,uint8_t y,uint8_t ch[],uint8_t yn);
 void OLED_P8x16Char(uint8_t x,uint8_t y,uint8_t wan);
 void OLED_P8x16Str(uint8_t x,uint8_t y,uint8_t ch[],uint8_t yn);
 /* ��ֵ��ʾ */
 void OLED_P8x16Three(uint8_t x,uint8_t y,int16_t m);  // ��ʾ3λ����  ��  
 void OLED_P8x16Four(uint8_t x,uint8_t y,int m);  //  ��ʾ4λ����  �� 
 void OLED_P8x16Num3(uint8_t x,uint8_t y,uint16_t m); //  ��ʾ3λ������
 void OLED_P8x16Num4(uint8_t x,uint8_t y,uint16_t m); //  ��ʾ4λ������
 void OLED_P8x16Num5(uint8_t x,uint8_t y,uint16_t m); //  ��ʾ5λ������
 /* ������ֵ��ʾ */
 void OLED_P16x32Num(uint8_t p,int num,uint8_t unit);// �ش�����
 void OLED_P8x16Dot(uint8_t x,uint8_t y,float m, uint8_t ucFracNum, uint8_t ucUnit);   // ��ʾС��
 void OLED_P6x8Four(uint8_t x, uint8_t y, int  num);// ��ʾ4λ����  С 
 void OLED_HexDisp(uint8_t x,uint8_t y,uint8_t *dat,uint8_t N);
 void OLED_P16x32Time(uint8_t p,  snvs_hp_rtc_datetime_t *ptTime);//������ʱ����ʾ
 
#endif

