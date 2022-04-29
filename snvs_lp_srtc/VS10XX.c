#include "fsl_common.h"
#include "vs10XX.h"	
#include "fsl_lpspi.h"
#include "imu_spi.h"
#include "fsl_debug_console.h"

  	    
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


#define LPSPI_MASTER_BASEADDR   (LPSPI3)
#define LPSPI_MASTER_IRQN       (LPSPI3_IRQn)
#define LPSPI_MASTER_IRQHandler (LPSPI3_IRQHandler)

#define VS_LPSPI_MASTER_PCS_FOR_INIT_XDCS   (kLPSPI_Pcs0)                               /* OnBoard Gyro.  */
#define VS_LPSPI_MASTER_PCS_FOR_INIT_XCS    (kLPSPI_Pcs2)                               /* imu board Gyro.*/    

#define VS_PCS_XDCS                        (kLPSPI_MasterPcs0)  
#define VS_PCS_XCS                         (kLPSPI_MasterPcs2)

//VS10XX默认设置参数
_vs10xx_obj vsset=
{
	200,	//音量:210
	6,		//低音上线 60Hz 
	7,		//低音提升 15dB	 15 -> 7
	10,		//高音下限 10Khz	
	15,		//高音提升 10.5dB
	0,		//空间效果	
};

////////////////////////////////////////////////////////////////////////////////
//移植时候的接口
//data:要写入的数据
//返回值:读到的数据
INT8U VS_SPI_ReadWriteByte(INT8U data)
{			  	 
	//return SPI1_ReadWriteByte(data);	
    return 0;    
}
//SD卡初始化的时候,需要低速
void VS_SPI_SpeedLow(void)
{
    ;    
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_64);//设置到低速模式 
}
//SD卡正常工作的时候,可以高速了
void VS_SPI_SpeedHigh(void)
{
    ;    
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_16);//设置到高速模式		 
}
//初始化VS10XX的IO口	 
void VS_Init(void)
{	 
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟
//	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PA12
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //输入
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);

// 
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_11;//PF6,PF7
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	
//	
//	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12);
//	
//	SPI1_Init();	 
    
//    RCC->APB2ENR|=1<<2;  	//PORTA时钟使能 
//	GPIOA->CRL&=0XFFF0FFFF;	//PA4 XDCS
//	GPIOA->CRL|=0X00030000; 
//	GPIOA->CRH&=0XFFF00FF0;	//PA8 XCS;PA11 RST;PA12 DQ
//	GPIOA->CRH|=0X00083003; 	 
//	GPIOA->ODR|=(1<<4)|(1<<8)|(1<<11)|(1<<12);	//PA4,8,11,12上拉   
//	SPI1_Init();								//初始化SPI 

 lpspi_master_config_t masterConfig;
    
    /*
     * Clock setting for LPI2C
     */
    CLOCK_SetMux(kCLOCK_LpspiMux, 1);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 7);
    
    masterConfig.baudRate = 2000000;                      // 2MHz (actuall 1.992MHz use oscilloscope)
    masterConfig.bitsPerFrame = 8;
    masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction = kLPSPI_MsbFirst;

    masterConfig.pcsToSckDelayInNanoSec = 10;          //20(actually 34ns use oscilloscope)    100ns(actually 112ns use Oscil) 10ns(actuall 14ns)
    masterConfig.lastSckToPcsDelayInNanoSec = 100;     //200(actuall 200ns exactly use Oscill) tXCSH min = 1 CLKI cycle 
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;          //600->200 otherwise it will stuck in 1038A (search this) or add some delay after 1038B func.
    masterConfig.betweenTransferDelayInNanoSec = 100;  //600(actuall 600ns exactly use Oscill) 

    masterConfig.whichPcs = VS_LPSPI_MASTER_PCS_FOR_INIT_XDCS;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    masterConfig.pinCfg = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig = kLpspiDataOutRetained;
    LPSPI_MasterInit(LPSPI_MASTER_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk)/8);
    
    masterConfig.whichPcs = VS_LPSPI_MASTER_PCS_FOR_INIT_XCS;
    LPSPI_MasterInit(LPSPI_MASTER_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk)/8);
    
    LPSPI_Enable(LPSPI_MASTER_BASEADDR, true);
    
    /* 
     * Set the Int Priority and enable Int
     */
    NVIC_SetPriority(LPSPI_MASTER_IRQN, 2);
    EnableIRQ(LPSPI_MASTER_IRQN);   
    
    /* 
     *   rxWatermark  设置 14字节，读一次MPU共读出15个字节 
     */
    LPSPI_SetFifoWatermarks(LPSPI_MASTER_BASEADDR, 1, 14);

    LPSPI_Enable(LPSPI_MASTER_BASEADDR, false);
    LPSPI_MASTER_BASEADDR->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable(LPSPI_MASTER_BASEADDR, true);
    
    /*
     * Flush FIFO , clear status , disable all the inerrupts.
     */
    LPSPI_FlushFifo(LPSPI_MASTER_BASEADDR, true, true);
    LPSPI_ClearStatusFlags(LPSPI_MASTER_BASEADDR, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(LPSPI_MASTER_BASEADDR, kLPSPI_AllInterruptEnable); 
    
    spiReadWriteByte(VS_PCS_XCS, 0xFF, 0xFF);
}	  
////////////////////////////////////////////////////////////////////////////////	 	  
//软复位VS10XX
void VS_Soft_Reset(void)
{	 
	INT8U retry=0;  				   
	while(VS_DQ==0); //等待软件复位结束	   
//	VS_SPI_ReadWriteByte(0Xff);//启动传输  no way  no such api in RT1050's SDK 
	retry=0;
	while(VS_RD_Reg(SPI_MODE)!=0x0800)// 软件复位,新模式  
	{
		VS_WR_Cmd(SPI_MODE,0x0804);// 软件复位,新模式	    
		msDelay(2);//等待至少1.35ms 
		if(retry++>100)break; 	    
	}	 		 
	while(VS_DQ==0);//等待软件复位结束	 
	retry=0;
	while(VS_RD_Reg(SPI_CLOCKF)!=0X9800)//设置VS10XX的时钟,3倍频 ,1.5xADD 
	{
		VS_WR_Cmd(SPI_CLOCKF,0X9800);//设置VS10XX的时钟,3倍频 ,1.5xADD
		if(retry++>100)break; 	    
	}		    										    
	msDelay(20);
} 
//硬复位MP3
//返回1:复位失败;0:复位成功	   
INT8U VS_HD_Reset(void)
{
	INT8U ucRetryCnt=0;
	VS_RST_0;
	msDelay(20);
//	VS_XDCS=1;//取消数据传输   no way 
//	VS_XCS=1;//取消数据传输    no way 
	VS_RST_1; 
	while (VS_DQ==0 && ucRetryCnt<200)//等待DREQ为高
	{
		ucRetryCnt++;
		usDelay(50);
	};
	msDelay(20);	
	if (ucRetryCnt >= 200) return 1;
	else                   return 0;	    		 
}
//正弦测试 
void VS_Sine_Test(void)
{											    
// 	VS_HD_Reset();	 
//	VS_WR_Cmd(0x0b,0X2020);	  //设置音量	 
// 	VS_WR_Cmd(SPI_MODE,0x0820);//进入VS10XX的测试模式     
//	while(VS_DQ==0);     //等待DREQ为高
//	printf("mode sin:%x\n",VS_RD_Reg(SPI_MODE));
// 	//向VS10XX发送正弦测试命令：0x53 0xef 0x6e n 0x00 0x00 0x00 0x00
// 	//其中n = 0x24, 设定VS10XX所产生的正弦波的频率值，具体计算方法见VS10XX的datasheet
//  	VS_SPI_SpeedLow();//低速 
//	VS_XDCS=0;//选中数据传输
//	VS_SPI_ReadWriteByte(0x53);
//	VS_SPI_ReadWriteByte(0xef);
//	VS_SPI_ReadWriteByte(0x6e);
//	VS_SPI_ReadWriteByte(0x24);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	msDealy(100);
//	VS_XDCS=1; 
//    //退出正弦测试
//    VS_XDCS=0;//选中数据传输
//	VS_SPI_ReadWriteByte(0x45);
//	VS_SPI_ReadWriteByte(0x78);
//	VS_SPI_ReadWriteByte(0x69);
//	VS_SPI_ReadWriteByte(0x74);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	msDealy(100);
//	VS_XDCS=1;	

////    msDealy(500);

//    //再次进入正弦测试并设置n值为0x44，即将正弦波的频率设置为另外的值
//    VS_XDCS=0;//选中数据传输      
//	VS_SPI_ReadWriteByte(0x53);
//	VS_SPI_ReadWriteByte(0xef);
//	VS_SPI_ReadWriteByte(0x6e);
//	VS_SPI_ReadWriteByte(0x44);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	msDealy(100);
// 	VS_XDCS=1;
//    //退出正弦测试
//    VS_XDCS=0;//选中数据传输
//	VS_SPI_ReadWriteByte(0x45);
//	VS_SPI_ReadWriteByte(0x78);
//	VS_SPI_ReadWriteByte(0x69);
//	VS_SPI_ReadWriteByte(0x74);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	msDealy(100);
//	VS_XDCS=1;	

    INT8U  ucTxBuf[8] = {0};
    INT8U  ucRxBuf[8] = {0};
    status_t  kStatus;
    lpspi_transfer_t  xTrans;  
    
    VS_HD_Reset();
 	VS_WR_Cmd(0x0b,0X3030);	  //设置音量	 
 	VS_WR_Cmd(SPI_MODE,0x0820);//进入VS10XX的测试模式 
    PRINTF("mode sin:%x\n",VS_RD_Reg(SPI_MODE));
    
    ucTxBuf[0] = 0x53;
    ucTxBuf[1] = 0xef;
    ucTxBuf[2] = 0x6e; 
    ucTxBuf[3] = 0x24; 
    ucTxBuf[4] = 0x00; 
    ucTxBuf[5] = 0x00; 
    ucTxBuf[6] = 0x00; 
    ucTxBuf[7] = 0x00;  
    
    xTrans.dataSize    = 8;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;               // VS_XDCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    msDelay(100);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
    
    ucTxBuf[0] = 0x45;
    ucTxBuf[1] = 0x78;
    ucTxBuf[2] = 0x69; 
    ucTxBuf[3] = 0x74; 
    ucTxBuf[4] = 0x00; 
    ucTxBuf[5] = 0x00; 
    ucTxBuf[6] = 0x00; 
    ucTxBuf[7] = 0x00;  
    
    xTrans.dataSize    = 8;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;               // VS_XDCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    msDelay(100);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
    
    ucTxBuf[0] = 0x53;
    ucTxBuf[1] = 0xef;
    ucTxBuf[2] = 0x6e; 
    ucTxBuf[3] = 0x44; 
    ucTxBuf[4] = 0x00; 
    ucTxBuf[5] = 0x00; 
    ucTxBuf[6] = 0x00; 
    ucTxBuf[7] = 0x00;  
    
    xTrans.dataSize    = 8;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;               // VS_XDCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    msDelay(100);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
    
    ucTxBuf[0] = 0x45;
    ucTxBuf[1] = 0x78;
    ucTxBuf[2] = 0x69; 
    ucTxBuf[3] = 0x74; 
    ucTxBuf[4] = 0x00; 
    ucTxBuf[5] = 0x00; 
    ucTxBuf[6] = 0x00; 
    ucTxBuf[7] = 0x00;  
    
    xTrans.dataSize    = 8;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;               // VS_XDCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    msDelay(100);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
}	 
//ram 测试 
//返回值:RAM测试结果
// VS1003如果得到的值为0x807F，则表明完好;VS1053为0X83FF.																				 
INT16U VS_Ram_Test(void)
{ 
//	VS_HD_Reset();     
// 	VS_WR_Cmd(SPI_MODE,0x0820);// 进入VS10XX的测试模式
//	while (VS_DQ==0); // 等待DREQ为高			   
// 	VS_SPI_SpeedLow();//低速 
//	VS_XDCS=0;	       		    // xDCS = 1，选择VS10XX的数据接口
//	VS_SPI_ReadWriteByte(0x4d);
//	VS_SPI_ReadWriteByte(0xea);
//	VS_SPI_ReadWriteByte(0x6d);
//	VS_SPI_ReadWriteByte(0x54);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_SPI_ReadWriteByte(0x00);
//	VS_XDCS=1;
//	msDealy(150);  

//	return VS_RD_Reg(SPI_HDAT0);// VS1003如果得到的值为0x807F，则表明完好;VS1053为0X83FF.;       
    
    INT8U  ucTxBuf[8] = {0x4d, 0xea, 0x6d, 0x54, 0x00, 0x00, 0x00, 0x00};
    INT8U  ucRxBuf[8] = {0};
    status_t  kStatus;
    lpspi_transfer_t  xTrans;  
    
    VS_HD_Reset();
    VS_WR_Cmd(SPI_MODE,0x0820);                                              // 1038B                                    
    
    xTrans.dataSize    = 8;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;               // VS_XDCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    
    if (kStatus != kStatus_Success) {
        while(1);                                                             // 1038A 
    }
    msDelay(150);  
    return VS_RD_Reg(SPI_HDAT0);// VS1003如果得到的值为0x807F，则表明完好;VS1053为0X83FF.;      
}     					   
//向VS10XX写命令
//address:命令地址
//data:命令数据
void VS_WR_Cmd(INT8U address,INT16U data)
{  
//	while(VS_DQ==0);//等待空闲		  
//	VS_SPI_SpeedLow();//低速 	   
//	VS_XDCS=1; 	 
//	VS_XCS=0; 	 
//	VS_SPI_ReadWriteByte(VS_WRITE_COMMAND);//发送VS10XX的写命令
//	VS_SPI_ReadWriteByte(address); //地址
//	VS_SPI_ReadWriteByte(data>>8); //发送高八位
//	VS_SPI_ReadWriteByte(data);	 //第八位
//	VS_XCS=1;           
//	VS_SPI_SpeedHigh();//高速	

    INT8U  ucTxBuf[4];
    INT8U  ucRxBuf[4];
    status_t  kStatus;
    lpspi_transfer_t  xTrans;  

    ucTxBuf[0] = VS_WRITE_COMMAND;
    ucTxBuf[1] = address;
    ucTxBuf[2] = data >> 8;
    ucTxBuf[3] = data;
    
    xTrans.dataSize    = 4;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XCS | kLPSPI_MasterPcsContinuous;               // VS_XCS
    
    while(VS_DQ==0);//等待空闲 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    
    if (kStatus != kStatus_Success) {
        //while(1);
        while(1);
    }      
} 
//向VS10XX写数据
//data:要写入的数据
void VS_WR_Data(INT8U data)
{
//	VS_SPI_SpeedHigh();//高速,对VS1003B,最大值不能超过36.864/4Mhz，这里设置为9M 
//	VS_XDCS=0;   
//	VS_SPI_ReadWriteByte(data);
//	VS_XDCS=1;      
}         
//读VS10XX的寄存器           
//address：寄存器地址
//返回值：读到的值
//注意不要用倍速读取,会出错
INT16U VS_RD_Reg(INT8U address)
{     
//	while(VS_DQ==0);	//等待空闲	   
//	VS_SPI_SpeedLow();//低速 
//	VS_XDCS=1;       
//	VS_XCS=0; 
//	VS_SPI_ReadWriteByte(VS_READ_COMMAND);	//发送VS10XX的读命令
//	VS_SPI_ReadWriteByte(address);       	//地址
//	temp=VS_SPI_ReadWriteByte(0xff); 		//读取高字节
//	temp=temp<<8;
//	temp+=VS_SPI_ReadWriteByte(0xff); 		//读取低字节
//	VS_XCS=1;   

   	INT16U temp=0;   
    INT8U  ucTxBuf[4];
    INT8U  ucRxBuf[4];
    status_t  kStatus;
    lpspi_transfer_t  xTrans; 

    ucTxBuf[0] = VS_READ_COMMAND;
    ucTxBuf[1] = address;
 
    xTrans.dataSize    = 4;
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XCS | kLPSPI_MasterPcsContinuous;               // VS_XCS
    
    while(VS_DQ==0);	//等待空闲	  
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    usDelay(50);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
    
    temp = (ucRxBuf[2] << 8) | ucRxBuf[3];
  
//	VS_SPI_SpeedHigh();//高速	  
   return temp; 
}  
//读取VS10xx的RAM
//addr：RAM地址
//返回值：读到的值
INT16U VS_WRAM_Read(INT16U addr) 
{ 
	INT16U res;			   	  
 	VS_WR_Cmd(SPI_WRAMADDR, addr); 
	res=VS_RD_Reg(SPI_WRAM);  
 	return res;
} 
//设置播放速度（仅VS1053有效） 
//t:0,1,正常速度;2,2倍速度;3,3倍速度;4,4倍速;以此类推
void VS_Set_Speed(INT8U t)
{
	VS_WR_Cmd(SPI_WRAMADDR,0X1E04);	//速度控制地址  
	while(VS_DQ==0); 				//等待空闲	   
	VS_WR_Cmd(SPI_WRAM,t); 			//写入播放速度
}
//FOR WAV HEAD0 :0X7761 HEAD1:0X7665    
//FOR MIDI HEAD0 :other info HEAD1:0X4D54
//FOR WMA HEAD0 :data speed HEAD1:0X574D
//FOR MP3 HEAD0 :data speed HEAD1:ID
//比特率预定值,阶层III
const INT16U bitrate[2][16]=
{ 
{0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0}, 
{0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0}
};
//返回Kbps的大小
//返回值：得到的码率
INT16U VS_Get_HeadInfo(void)
{
	unsigned int HEAD0;
	unsigned int HEAD1;  
 	HEAD0=VS_RD_Reg(SPI_HDAT0); 
    HEAD1=VS_RD_Reg(SPI_HDAT1);
  	//printf("(H0,H1):%x,%x\n",HEAD0,HEAD1);
    switch(HEAD1)
    {        
        case 0x7665://WAV格式
        case 0X4D54://MIDI格式 
		case 0X4154://AAC_ADTS
		case 0X4144://AAC_ADIF
		case 0X4D34://AAC_MP4/M4A
		case 0X4F67://OGG
        case 0X574D://WMA格式
		case 0X664C://FLAC格式
        {
			////printf("HEAD0:%d\n",HEAD0);
            HEAD1=HEAD0*2/25;//相当于*8/100
            if((HEAD1%10)>5)return HEAD1/10+1;//对小数点第一位四舍五入
            else return HEAD1/10;
        }
        default://MP3格式,仅做了阶层III的表
        {
            HEAD1>>=3;
            HEAD1=HEAD1&0x03; 
            if(HEAD1==3)HEAD1=1;
            else HEAD1=0;
            return bitrate[HEAD1][HEAD0>>12];
        }
    }  
}
//得到平均字节数
//返回值：平均字节数速度
INT32U VS_Get_ByteRate(void)
{
	return VS_WRAM_Read(0X1E05);//平均位速
}
//得到需要填充的数字
//返回值:需要填充的数字
INT16U VS_Get_EndFillByte(void)
{
	return VS_WRAM_Read(0X1E06);//填充字节
}  
//发送一次音频数据
//固定为32字节
//返回值:0,发送成功
//		 1,VS10xx不缺数据,本次数据未成功发送    
INT8U VS_Send_MusicData(const INT8U* buf)
{
//	INT8U n;
//	if(VS_DQ!=0)  //送数据给VS10XX
//	{			   	 
//		VS_XDCS=0;  
//        for(n=0;n<32;n++)
//		{
//			VS_SPI_ReadWriteByte(buf[n]);	 			
//		}
//		VS_XDCS=1;     				   
//	}else return 1;
//	return 0;//成功发送了
    
    INT8U ucRxBuf[32];
    status_t  kStatus;
    lpspi_transfer_t  xTrans;

    xTrans.dataSize    = 32;
    xTrans.txData      = (INT8U *)buf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;
   
    if (VS_DQ!=0) {  //送数据给VS10XX
        kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    

        if (kStatus != kStatus_Success) {
            while(1);
        } 
    } else {
        return 1;
    }
    return 0;        
}
//切歌
//通过此函数切歌，不会出现切换“噪声”				
void VS_Restart_Play(void)
{
	INT16U temp;
	INT16U i;
	INT8U n;	  
	INT8U vsbuf[32];
	for(n=0;n<32;n++)vsbuf[n]=0;//清零
	temp=VS_RD_Reg(SPI_MODE);	//读取SPI_MODE的内容
	temp|=1<<3;					//设置SM_CANCEL位
	temp|=1<<2;					//设置SM_LAYER12位,允许播放MP1,MP2
	VS_WR_Cmd(SPI_MODE,temp);	//设置取消当前解码指令
	for(i=0;i<2048;)			//发送2048个0,期间读取SM_CANCEL位.如果为0,则表示已经取消了当前解码
	{
		if(VS_Send_MusicData(vsbuf)==0)//每发送32个字节后检测一次
		{
			i+=32;						//发送了32个字节
   			temp=VS_RD_Reg(SPI_MODE);	//读取SPI_MODE的内容
 			if((temp&(1<<3))==0)break;	//成功取消了
		}	
	}
	if(i<2048)//SM_CANCEL正常
	{
		temp=VS_Get_EndFillByte()&0xff;//读取填充字节
		for(n=0;n<32;n++)vsbuf[n]=temp;//填充字节放入数组
		for(i=0;i<2052;)
		{
			if(VS_Send_MusicData(vsbuf)==0)i+=32;//填充	  
		}   	
	}else VS_Soft_Reset();  	//SM_CANCEL不成功,坏情况,需要软复位 	  
	temp=VS_RD_Reg(SPI_HDAT0); 
    temp+=VS_RD_Reg(SPI_HDAT1);
	if(temp)					//软复位,还是没有成功取消,放杀手锏,硬复位
	{
		VS_HD_Reset();		   	//硬复位
		VS_Soft_Reset();  		//软复位 
	} 
}
//重设解码时间                          
void VS_Reset_DecodeTime(void)
{
	VS_WR_Cmd(SPI_DECODE_TIME,0x0000);
	VS_WR_Cmd(SPI_DECODE_TIME,0x0000);//操作两次
}
//得到mp3的播放时间n sec
//返回值：解码时长
INT16U VS_Get_DecodeTime(void)
{ 		
	INT16U dt=0;	 
	dt=VS_RD_Reg(SPI_DECODE_TIME);      
 	return dt;
} 	    					  
//vs10xx装载patch.
//patch：patch首地址
//len：patch长度
void VS_Load_Patch(INT16U *patch,INT16U len) 
{
	INT16U i; 
	INT16U addr, n, val; 	  			   
	for(i=0;i<len;) 
	{ 
		addr = patch[i++]; 
		n    = patch[i++]; 
		if(n & 0x8000U) //RLE run, replicate n samples 
		{ 
			n  &= 0x7FFF; 
			val = patch[i++]; 
			while(n--)VS_WR_Cmd(addr, val);  
		}else //copy run, copy n sample 
		{ 
			while(n--) 
			{ 
				val = patch[i++]; 
				VS_WR_Cmd(addr, val); 
			} 
		} 
	} 	
} 		  	  
//设定VS10XX播放的音量和高低音
//volx:音量大小(0~254)
void VS_Set_Vol(INT8U volx)
{
    INT16U volt=0; 			//暂存音量值
    volt=254-volx;			//取反一下,得到最大值,表示最大的表示 
	volt<<=8;
    volt+=254-volx;			//得到音量设置后大小
    VS_WR_Cmd(SPI_VOL,volt);//设音量 
}
//设定高低音控制
//bfreq:低频上限频率	2~15(单位:10Hz)
//bass:低频增益			0~15(单位:1dB)
//tfreq:高频下限频率 	1~15(单位:Khz)
//treble:高频增益  	 	0~15(单位:1.5dB,小于9的时候为负数)
void VS_Set_Bass(INT8U bfreq,INT8U bass,INT8U tfreq,INT8U treble)
{
    INT16U bass_set=0; //暂存音调寄存器值
    signed char temp=0;   	 
	if(treble==0)temp=0;	   		//变换
	else if(treble>8)temp=treble-8;
 	else temp=treble-9;  
	bass_set=temp&0X0F;				//高音设定
	bass_set<<=4;
	bass_set+=tfreq&0xf;			//高音下限频率
	bass_set<<=4;
	bass_set+=bass&0xf;				//低音设定
	bass_set<<=4;
	bass_set+=bfreq&0xf;			//低音上限    
	VS_WR_Cmd(SPI_BASS,bass_set);	//BASS 
}
//设定音效
//eft:0,关闭;1,最小;2,中等;3,最大.
void VS_Set_Effect(INT8U eft)
{
	INT16U temp;	 
	temp=VS_RD_Reg(SPI_MODE);	//读取SPI_MODE的内容
	if(eft&0X01)temp|=1<<4;		//设定LO
	else temp&=~(1<<5);			//取消LO
	if(eft&0X02)temp|=1<<7;		//设定HO
	else temp&=~(1<<7);			//取消HO						   
	VS_WR_Cmd(SPI_MODE,temp);	//设定模式    
}	  
///////////////////////////////////////////////////////////////////////////////
//设置音量,音效等.
void VS_Set_All(void) 				
{			 
	VS_Set_Vol(vsset.mvol);
	VS_Set_Bass(vsset.bflimit,vsset.bass,vsset.tflimit,vsset.treble);  
	VS_Set_Effect(vsset.effect);
}































