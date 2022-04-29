#ifndef __VS10XX_H__
#define __VS10XX_H__
#include "fsl_common.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//VS10XX 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/25
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved													    								  
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//与外部的接口

#define VS_DQ         ((GPIO1->DR & (1 << 14)) >> 14)		//DREQ 
#define VS_RST_0       GPIO1->DR &= ~(1 << 15)		                            //RST
#define VS_RST_1       GPIO1->DR |= (1 << 15)
//#define VS_XCS      PAout(8)  		//XCS
//#define VS_XDCS     PAout(4)  		//XDCS 
//////////////////////////////////////////////////////////////

__packed typedef struct 
{							  
	INT8U mvol;		//主音量,范围:0~254
	INT8U bflimit;		//低音频率限定,范围:2~15(单位:10Hz)
	INT8U bass;		//低音,范围:0~15.0表示关闭.(单位:1dB)
	INT8U tflimit;		//高音频率限定,范围:1~15(单位:Khz)
	INT8U treble;		//高音,范围:0~15(单位:1.5dB)(原本范围是:-8~7,通过函数修改了);
	INT8U effect;		//空间效果设置.0,关闭;1,最小;2,中等;3,最大.

	INT8U saveflag; 	//保存标志,0X0A,保存过了;其他,还从未保存	   
}_vs10xx_obj;


extern _vs10xx_obj vsset;		//VS10XX设置

#define VS_WRITE_COMMAND 	0x02
#define VS_READ_COMMAND 	0x03
//VS10XX寄存器定义
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



INT16U  VS_RD_Reg(INT8U address);				//读寄存器
INT16U  VS_WRAM_Read(INT16U addr);	    	//读RAM
void VS_WR_Data(INT8U data);				//写数据
void VS_WR_Cmd(INT8U address,INT16U data);	//写命令
INT8U   VS_HD_Reset(void);			    	//硬复位
void VS_Soft_Reset(void);           	//软复位
INT16U VS_Ram_Test(void);             		//RAM测试	    
void VS_Sine_Test(void);            	//正弦测试
													 
INT8U 	 VS_SPI_ReadWriteByte(INT8U data);
void VS_SPI_SpeedLow(void);
void VS_SPI_SpeedHigh(void);
void VS_Init(void);						//初始化VS10XX	 
void VS_Set_Speed(INT8U t);				//设置播放速度
INT16U  VS_Get_HeadInfo(void);     		//得到比特率
INT32U VS_Get_ByteRate(void);				//得到字节速率
INT16U VS_Get_EndFillByte(void);			//得到填充字节
INT8U 	 VS_Send_MusicData(const INT8U* buf);		//向VS10XX发送32字节
void VS_Restart_Play(void);				//重新开始下一首歌播放	  
void VS_Reset_DecodeTime(void);			//重设解码时间
INT16U  VS_Get_DecodeTime(void);   		//得到解码时间

void VS_Load_Patch(INT16U *patch,INT16U len);	//加载用户patch
INT8U 	 VS_Get_Spec(INT16U *p);       		//得到分析数据	   
void VS_Set_Bands(INT16U *buf,INT8U bands);	//设置中心频率
void VS_Set_Vol(INT8U volx);				//设置主音量   
void VS_Set_Bass(INT8U bfreq,INT8U bass,INT8U tfreq,INT8U treble);//设置高低音
void VS_Set_Effect(INT8U eft);				//设置音效
void VS_Set_All(void);

void vs10xx_read_para(_vs10xx_obj * vs10xxdev);
void vs10xx_save_para(_vs10xx_obj * vs10xxdev);

#endif

















