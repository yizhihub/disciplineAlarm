#ifndef __VS10XX_H__
#define __VS10XX_H__
#include "fsl_common.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//VS10XX ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/25
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved													    								  
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//���ⲿ�Ľӿ�

#define VS_DQ         ((GPIO1->DR & (1 << 14)) >> 14)		//DREQ 
#define VS_RST_0       GPIO1->DR &= ~(1 << 15)		                            //RST
#define VS_RST_1       GPIO1->DR |= (1 << 15)
//#define VS_XCS      PAout(8)  		//XCS
//#define VS_XDCS     PAout(4)  		//XDCS 
//////////////////////////////////////////////////////////////

__packed typedef struct 
{							  
	INT8U mvol;		//������,��Χ:0~254
	INT8U bflimit;		//����Ƶ���޶�,��Χ:2~15(��λ:10Hz)
	INT8U bass;		//����,��Χ:0~15.0��ʾ�ر�.(��λ:1dB)
	INT8U tflimit;		//����Ƶ���޶�,��Χ:1~15(��λ:Khz)
	INT8U treble;		//����,��Χ:0~15(��λ:1.5dB)(ԭ����Χ��:-8~7,ͨ�������޸���);
	INT8U effect;		//�ռ�Ч������.0,�ر�;1,��С;2,�е�;3,���.

	INT8U saveflag; 	//�����־,0X0A,�������;����,����δ����	   
}_vs10xx_obj;


extern _vs10xx_obj vsset;		//VS10XX����

#define VS_WRITE_COMMAND 	0x02
#define VS_READ_COMMAND 	0x03
//VS10XX�Ĵ�������
#define SPI_MODE        	0x00   
#define SPI_STATUS      	0x01   
#define SPI_BASS        	0x02   
#define SPI_CLOCKF      	0x03   
#define SPI_DECODE_TIME 	0x04   
#define SPI_AUDATA      	0x05   
#define SPI_WRAM        	0x06   
#define SPI_WRAMADDR    	0x07   
#define SPI_HDAT0       	0x08   
#define SPI_HDAT1       	0x09 
  
#define SPI_AIADDR      	0x0a   
#define SPI_VOL         	0x0b   
#define SPI_AICTRL0     	0x0c   
#define SPI_AICTRL1     	0x0d   
#define SPI_AICTRL2     	0x0e   
#define SPI_AICTRL3     	0x0f   
#define SM_DIFF         	0x01   
#define SM_JUMP         	0x02   
#define SM_RESET        	0x04   
#define SM_OUTOFWAV     	0x08   
#define SM_PDOWN        	0x10   
#define SM_TESTS        	0x20   
#define SM_STREAM       	0x40   
#define SM_PLUSV        	0x80   
#define SM_DACT         	0x100   
#define SM_SDIORD       	0x200   
#define SM_SDISHARE     	0x400   
#define SM_SDINEW       	0x800   
#define SM_ADPCM        	0x1000   
#define SM_ADPCM_HP     	0x2000 		 



INT16U  VS_RD_Reg(INT8U address);				//���Ĵ���
INT16U  VS_WRAM_Read(INT16U addr);	    	//��RAM
void VS_WR_Data(INT8U data);				//д����
void VS_WR_Cmd(INT8U address,INT16U data);	//д����
INT8U   VS_HD_Reset(void);			    	//Ӳ��λ
void VS_Soft_Reset(void);           	//��λ
INT16U VS_Ram_Test(void);             		//RAM����	    
void VS_Sine_Test(void);            	//���Ҳ���
													 
INT8U 	 VS_SPI_ReadWriteByte(INT8U data);
void VS_SPI_SpeedLow(void);
void VS_SPI_SpeedHigh(void);
void VS_Init(void);						//��ʼ��VS10XX	 
void VS_Set_Speed(INT8U t);				//���ò����ٶ�
INT16U  VS_Get_HeadInfo(void);     		//�õ�������
INT32U VS_Get_ByteRate(void);				//�õ��ֽ�����
INT16U VS_Get_EndFillByte(void);			//�õ�����ֽ�
INT8U 	 VS_Send_MusicData(const INT8U* buf);		//��VS10XX����32�ֽ�
void VS_Restart_Play(void);				//���¿�ʼ��һ�׸貥��	  
void VS_Reset_DecodeTime(void);			//�������ʱ��
INT16U  VS_Get_DecodeTime(void);   		//�õ�����ʱ��

void VS_Load_Patch(INT16U *patch,INT16U len);	//�����û�patch
INT8U 	 VS_Get_Spec(INT16U *p);       		//�õ���������	   
void VS_Set_Bands(INT16U *buf,INT8U bands);	//��������Ƶ��
void VS_Set_Vol(INT8U volx);				//����������   
void VS_Set_Bass(INT8U bfreq,INT8U bass,INT8U tfreq,INT8U treble);//���øߵ���
void VS_Set_Effect(INT8U eft);				//������Ч
void VS_Set_All(void);

void vs10xx_read_para(_vs10xx_obj * vs10xxdev);
void vs10xx_save_para(_vs10xx_obj * vs10xxdev);

#endif

















