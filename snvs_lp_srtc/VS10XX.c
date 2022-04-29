#include "fsl_common.h"
#include "vs10XX.h"	
#include "fsl_lpspi.h"
#include "imu_spi.h"
#include "fsl_debug_console.h"

  	    
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


#define LPSPI_MASTER_BASEADDR   (LPSPI3)
#define LPSPI_MASTER_IRQN       (LPSPI3_IRQn)
#define LPSPI_MASTER_IRQHandler (LPSPI3_IRQHandler)

#define VS_LPSPI_MASTER_PCS_FOR_INIT_XDCS   (kLPSPI_Pcs0)                               /* OnBoard Gyro.  */
#define VS_LPSPI_MASTER_PCS_FOR_INIT_XCS    (kLPSPI_Pcs2)                               /* imu board Gyro.*/    

#define VS_PCS_XDCS                        (kLPSPI_MasterPcs0)  
#define VS_PCS_XCS                         (kLPSPI_MasterPcs2)

//VS10XXĬ�����ò���
_vs10xx_obj vsset=
{
	200,	//����:210
	6,		//�������� 60Hz 
	7,		//�������� 15dB	 15 -> 7
	10,		//�������� 10Khz	
	15,		//�������� 10.5dB
	0,		//�ռ�Ч��	
};

////////////////////////////////////////////////////////////////////////////////
//��ֲʱ��Ľӿ�
//data:Ҫд�������
//����ֵ:����������
INT8U VS_SPI_ReadWriteByte(INT8U data)
{			  	 
	//return SPI1_ReadWriteByte(data);	
    return 0;    
}
//SD����ʼ����ʱ��,��Ҫ����
void VS_SPI_SpeedLow(void)
{
    ;    
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_64);//���õ�����ģʽ 
}
//SD������������ʱ��,���Ը�����
void VS_SPI_SpeedHigh(void)
{
    ;    
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_16);//���õ�����ģʽ		 
}
//��ʼ��VS10XX��IO��	 
void VS_Init(void)
{	 
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��
//	
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PA12
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //����
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);

// 
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_11;//PF6,PF7
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	
//	
//	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12);
//	
//	SPI1_Init();	 
    
//    RCC->APB2ENR|=1<<2;  	//PORTAʱ��ʹ�� 
//	GPIOA->CRL&=0XFFF0FFFF;	//PA4 XDCS
//	GPIOA->CRL|=0X00030000; 
//	GPIOA->CRH&=0XFFF00FF0;	//PA8 XCS;PA11 RST;PA12 DQ
//	GPIOA->CRH|=0X00083003; 	 
//	GPIOA->ODR|=(1<<4)|(1<<8)|(1<<11)|(1<<12);	//PA4,8,11,12����   
//	SPI1_Init();								//��ʼ��SPI 

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
     *   rxWatermark  ���� 14�ֽڣ���һ��MPU������15���ֽ� 
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
//��λVS10XX
void VS_Soft_Reset(void)
{	 
	INT8U retry=0;  				   
	while(VS_DQ==0); //�ȴ������λ����	   
//	VS_SPI_ReadWriteByte(0Xff);//��������  no way  no such api in RT1050's SDK 
	retry=0;
	while(VS_RD_Reg(SPI_MODE)!=0x0800)// �����λ,��ģʽ  
	{
		VS_WR_Cmd(SPI_MODE,0x0804);// �����λ,��ģʽ	    
		msDelay(2);//�ȴ�����1.35ms 
		if(retry++>100)break; 	    
	}	 		 
	while(VS_DQ==0);//�ȴ������λ����	 
	retry=0;
	while(VS_RD_Reg(SPI_CLOCKF)!=0X9800)//����VS10XX��ʱ��,3��Ƶ ,1.5xADD 
	{
		VS_WR_Cmd(SPI_CLOCKF,0X9800);//����VS10XX��ʱ��,3��Ƶ ,1.5xADD
		if(retry++>100)break; 	    
	}		    										    
	msDelay(20);
} 
//Ӳ��λMP3
//����1:��λʧ��;0:��λ�ɹ�	   
INT8U VS_HD_Reset(void)
{
	INT8U ucRetryCnt=0;
	VS_RST_0;
	msDelay(20);
//	VS_XDCS=1;//ȡ�����ݴ���   no way 
//	VS_XCS=1;//ȡ�����ݴ���    no way 
	VS_RST_1; 
	while (VS_DQ==0 && ucRetryCnt<200)//�ȴ�DREQΪ��
	{
		ucRetryCnt++;
		usDelay(50);
	};
	msDelay(20);	
	if (ucRetryCnt >= 200) return 1;
	else                   return 0;	    		 
}
//���Ҳ��� 
void VS_Sine_Test(void)
{											    
// 	VS_HD_Reset();	 
//	VS_WR_Cmd(0x0b,0X2020);	  //��������	 
// 	VS_WR_Cmd(SPI_MODE,0x0820);//����VS10XX�Ĳ���ģʽ     
//	while(VS_DQ==0);     //�ȴ�DREQΪ��
//	printf("mode sin:%x\n",VS_RD_Reg(SPI_MODE));
// 	//��VS10XX�������Ҳ������0x53 0xef 0x6e n 0x00 0x00 0x00 0x00
// 	//����n = 0x24, �趨VS10XX�����������Ҳ���Ƶ��ֵ��������㷽����VS10XX��datasheet
//  	VS_SPI_SpeedLow();//���� 
//	VS_XDCS=0;//ѡ�����ݴ���
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
//    //�˳����Ҳ���
//    VS_XDCS=0;//ѡ�����ݴ���
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

//    //�ٴν������Ҳ��Բ�����nֵΪ0x44���������Ҳ���Ƶ������Ϊ�����ֵ
//    VS_XDCS=0;//ѡ�����ݴ���      
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
//    //�˳����Ҳ���
//    VS_XDCS=0;//ѡ�����ݴ���
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
 	VS_WR_Cmd(0x0b,0X3030);	  //��������	 
 	VS_WR_Cmd(SPI_MODE,0x0820);//����VS10XX�Ĳ���ģʽ 
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
    
    while(VS_DQ==0);//�ȴ����� 
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
    
    while(VS_DQ==0);//�ȴ����� 
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
    
    while(VS_DQ==0);//�ȴ����� 
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
    
    while(VS_DQ==0);//�ȴ����� 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    msDelay(100);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
}	 
//ram ���� 
//����ֵ:RAM���Խ��
// VS1003����õ���ֵΪ0x807F����������;VS1053Ϊ0X83FF.																				 
INT16U VS_Ram_Test(void)
{ 
//	VS_HD_Reset();     
// 	VS_WR_Cmd(SPI_MODE,0x0820);// ����VS10XX�Ĳ���ģʽ
//	while (VS_DQ==0); // �ȴ�DREQΪ��			   
// 	VS_SPI_SpeedLow();//���� 
//	VS_XDCS=0;	       		    // xDCS = 1��ѡ��VS10XX�����ݽӿ�
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

//	return VS_RD_Reg(SPI_HDAT0);// VS1003����õ���ֵΪ0x807F����������;VS1053Ϊ0X83FF.;       
    
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
    
    while(VS_DQ==0);//�ȴ����� 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    
    if (kStatus != kStatus_Success) {
        while(1);                                                             // 1038A 
    }
    msDelay(150);  
    return VS_RD_Reg(SPI_HDAT0);// VS1003����õ���ֵΪ0x807F����������;VS1053Ϊ0X83FF.;      
}     					   
//��VS10XXд����
//address:�����ַ
//data:��������
void VS_WR_Cmd(INT8U address,INT16U data)
{  
//	while(VS_DQ==0);//�ȴ�����		  
//	VS_SPI_SpeedLow();//���� 	   
//	VS_XDCS=1; 	 
//	VS_XCS=0; 	 
//	VS_SPI_ReadWriteByte(VS_WRITE_COMMAND);//����VS10XX��д����
//	VS_SPI_ReadWriteByte(address); //��ַ
//	VS_SPI_ReadWriteByte(data>>8); //���͸߰�λ
//	VS_SPI_ReadWriteByte(data);	 //�ڰ�λ
//	VS_XCS=1;           
//	VS_SPI_SpeedHigh();//����	

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
    
    while(VS_DQ==0);//�ȴ����� 
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    
    if (kStatus != kStatus_Success) {
        //while(1);
        while(1);
    }      
} 
//��VS10XXд����
//data:Ҫд�������
void VS_WR_Data(INT8U data)
{
//	VS_SPI_SpeedHigh();//����,��VS1003B,���ֵ���ܳ���36.864/4Mhz����������Ϊ9M 
//	VS_XDCS=0;   
//	VS_SPI_ReadWriteByte(data);
//	VS_XDCS=1;      
}         
//��VS10XX�ļĴ���           
//address���Ĵ�����ַ
//����ֵ��������ֵ
//ע�ⲻҪ�ñ��ٶ�ȡ,�����
INT16U VS_RD_Reg(INT8U address)
{     
//	while(VS_DQ==0);	//�ȴ�����	   
//	VS_SPI_SpeedLow();//���� 
//	VS_XDCS=1;       
//	VS_XCS=0; 
//	VS_SPI_ReadWriteByte(VS_READ_COMMAND);	//����VS10XX�Ķ�����
//	VS_SPI_ReadWriteByte(address);       	//��ַ
//	temp=VS_SPI_ReadWriteByte(0xff); 		//��ȡ���ֽ�
//	temp=temp<<8;
//	temp+=VS_SPI_ReadWriteByte(0xff); 		//��ȡ���ֽ�
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
    
    while(VS_DQ==0);	//�ȴ�����	  
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    usDelay(50);
    
    if (kStatus != kStatus_Success) {
        while(1);
    }
    
    temp = (ucRxBuf[2] << 8) | ucRxBuf[3];
  
//	VS_SPI_SpeedHigh();//����	  
   return temp; 
}  
//��ȡVS10xx��RAM
//addr��RAM��ַ
//����ֵ��������ֵ
INT16U VS_WRAM_Read(INT16U addr) 
{ 
	INT16U res;			   	  
 	VS_WR_Cmd(SPI_WRAMADDR, addr); 
	res=VS_RD_Reg(SPI_WRAM);  
 	return res;
} 
//���ò����ٶȣ���VS1053��Ч�� 
//t:0,1,�����ٶ�;2,2���ٶ�;3,3���ٶ�;4,4����;�Դ�����
void VS_Set_Speed(INT8U t)
{
	VS_WR_Cmd(SPI_WRAMADDR,0X1E04);	//�ٶȿ��Ƶ�ַ  
	while(VS_DQ==0); 				//�ȴ�����	   
	VS_WR_Cmd(SPI_WRAM,t); 			//д�벥���ٶ�
}
//FOR WAV HEAD0 :0X7761 HEAD1:0X7665    
//FOR MIDI HEAD0 :other info HEAD1:0X4D54
//FOR WMA HEAD0 :data speed HEAD1:0X574D
//FOR MP3 HEAD0 :data speed HEAD1:ID
//������Ԥ��ֵ,�ײ�III
const INT16U bitrate[2][16]=
{ 
{0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0}, 
{0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0}
};
//����Kbps�Ĵ�С
//����ֵ���õ�������
INT16U VS_Get_HeadInfo(void)
{
	unsigned int HEAD0;
	unsigned int HEAD1;  
 	HEAD0=VS_RD_Reg(SPI_HDAT0); 
    HEAD1=VS_RD_Reg(SPI_HDAT1);
  	//printf("(H0,H1):%x,%x\n",HEAD0,HEAD1);
    switch(HEAD1)
    {        
        case 0x7665://WAV��ʽ
        case 0X4D54://MIDI��ʽ 
		case 0X4154://AAC_ADTS
		case 0X4144://AAC_ADIF
		case 0X4D34://AAC_MP4/M4A
		case 0X4F67://OGG
        case 0X574D://WMA��ʽ
		case 0X664C://FLAC��ʽ
        {
			////printf("HEAD0:%d\n",HEAD0);
            HEAD1=HEAD0*2/25;//�൱��*8/100
            if((HEAD1%10)>5)return HEAD1/10+1;//��С�����һλ��������
            else return HEAD1/10;
        }
        default://MP3��ʽ,�����˽ײ�III�ı�
        {
            HEAD1>>=3;
            HEAD1=HEAD1&0x03; 
            if(HEAD1==3)HEAD1=1;
            else HEAD1=0;
            return bitrate[HEAD1][HEAD0>>12];
        }
    }  
}
//�õ�ƽ���ֽ���
//����ֵ��ƽ���ֽ����ٶ�
INT32U VS_Get_ByteRate(void)
{
	return VS_WRAM_Read(0X1E05);//ƽ��λ��
}
//�õ���Ҫ��������
//����ֵ:��Ҫ��������
INT16U VS_Get_EndFillByte(void)
{
	return VS_WRAM_Read(0X1E06);//����ֽ�
}  
//����һ����Ƶ����
//�̶�Ϊ32�ֽ�
//����ֵ:0,���ͳɹ�
//		 1,VS10xx��ȱ����,��������δ�ɹ�����    
INT8U VS_Send_MusicData(const INT8U* buf)
{
//	INT8U n;
//	if(VS_DQ!=0)  //�����ݸ�VS10XX
//	{			   	 
//		VS_XDCS=0;  
//        for(n=0;n<32;n++)
//		{
//			VS_SPI_ReadWriteByte(buf[n]);	 			
//		}
//		VS_XDCS=1;     				   
//	}else return 1;
//	return 0;//�ɹ�������
    
    INT8U ucRxBuf[32];
    status_t  kStatus;
    lpspi_transfer_t  xTrans;

    xTrans.dataSize    = 32;
    xTrans.txData      = (INT8U *)buf;
    xTrans.rxData      = ucRxBuf;
    xTrans.configFlags = VS_PCS_XDCS | kLPSPI_MasterPcsContinuous;
   
    if (VS_DQ!=0) {  //�����ݸ�VS10XX
        kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    

        if (kStatus != kStatus_Success) {
            while(1);
        } 
    } else {
        return 1;
    }
    return 0;        
}
//�и�
//ͨ���˺����и裬��������л���������				
void VS_Restart_Play(void)
{
	INT16U temp;
	INT16U i;
	INT8U n;	  
	INT8U vsbuf[32];
	for(n=0;n<32;n++)vsbuf[n]=0;//����
	temp=VS_RD_Reg(SPI_MODE);	//��ȡSPI_MODE������
	temp|=1<<3;					//����SM_CANCELλ
	temp|=1<<2;					//����SM_LAYER12λ,������MP1,MP2
	VS_WR_Cmd(SPI_MODE,temp);	//����ȡ����ǰ����ָ��
	for(i=0;i<2048;)			//����2048��0,�ڼ��ȡSM_CANCELλ.���Ϊ0,���ʾ�Ѿ�ȡ���˵�ǰ����
	{
		if(VS_Send_MusicData(vsbuf)==0)//ÿ����32���ֽں���һ��
		{
			i+=32;						//������32���ֽ�
   			temp=VS_RD_Reg(SPI_MODE);	//��ȡSPI_MODE������
 			if((temp&(1<<3))==0)break;	//�ɹ�ȡ����
		}	
	}
	if(i<2048)//SM_CANCEL����
	{
		temp=VS_Get_EndFillByte()&0xff;//��ȡ����ֽ�
		for(n=0;n<32;n++)vsbuf[n]=temp;//����ֽڷ�������
		for(i=0;i<2052;)
		{
			if(VS_Send_MusicData(vsbuf)==0)i+=32;//���	  
		}   	
	}else VS_Soft_Reset();  	//SM_CANCEL���ɹ�,�����,��Ҫ��λ 	  
	temp=VS_RD_Reg(SPI_HDAT0); 
    temp+=VS_RD_Reg(SPI_HDAT1);
	if(temp)					//��λ,����û�гɹ�ȡ��,��ɱ���,Ӳ��λ
	{
		VS_HD_Reset();		   	//Ӳ��λ
		VS_Soft_Reset();  		//��λ 
	} 
}
//�������ʱ��                          
void VS_Reset_DecodeTime(void)
{
	VS_WR_Cmd(SPI_DECODE_TIME,0x0000);
	VS_WR_Cmd(SPI_DECODE_TIME,0x0000);//��������
}
//�õ�mp3�Ĳ���ʱ��n sec
//����ֵ������ʱ��
INT16U VS_Get_DecodeTime(void)
{ 		
	INT16U dt=0;	 
	dt=VS_RD_Reg(SPI_DECODE_TIME);      
 	return dt;
} 	    					  
//vs10xxװ��patch.
//patch��patch�׵�ַ
//len��patch����
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
//�趨VS10XX���ŵ������͸ߵ���
//volx:������С(0~254)
void VS_Set_Vol(INT8U volx)
{
    INT16U volt=0; 			//�ݴ�����ֵ
    volt=254-volx;			//ȡ��һ��,�õ����ֵ,��ʾ���ı�ʾ 
	volt<<=8;
    volt+=254-volx;			//�õ��������ú��С
    VS_WR_Cmd(SPI_VOL,volt);//������ 
}
//�趨�ߵ�������
//bfreq:��Ƶ����Ƶ��	2~15(��λ:10Hz)
//bass:��Ƶ����			0~15(��λ:1dB)
//tfreq:��Ƶ����Ƶ�� 	1~15(��λ:Khz)
//treble:��Ƶ����  	 	0~15(��λ:1.5dB,С��9��ʱ��Ϊ����)
void VS_Set_Bass(INT8U bfreq,INT8U bass,INT8U tfreq,INT8U treble)
{
    INT16U bass_set=0; //�ݴ������Ĵ���ֵ
    signed char temp=0;   	 
	if(treble==0)temp=0;	   		//�任
	else if(treble>8)temp=treble-8;
 	else temp=treble-9;  
	bass_set=temp&0X0F;				//�����趨
	bass_set<<=4;
	bass_set+=tfreq&0xf;			//��������Ƶ��
	bass_set<<=4;
	bass_set+=bass&0xf;				//�����趨
	bass_set<<=4;
	bass_set+=bfreq&0xf;			//��������    
	VS_WR_Cmd(SPI_BASS,bass_set);	//BASS 
}
//�趨��Ч
//eft:0,�ر�;1,��С;2,�е�;3,���.
void VS_Set_Effect(INT8U eft)
{
	INT16U temp;	 
	temp=VS_RD_Reg(SPI_MODE);	//��ȡSPI_MODE������
	if(eft&0X01)temp|=1<<4;		//�趨LO
	else temp&=~(1<<5);			//ȡ��LO
	if(eft&0X02)temp|=1<<7;		//�趨HO
	else temp&=~(1<<7);			//ȡ��HO						   
	VS_WR_Cmd(SPI_MODE,temp);	//�趨ģʽ    
}	  
///////////////////////////////////////////////////////////////////////////////
//��������,��Ч��.
void VS_Set_All(void) 				
{			 
	VS_Set_Vol(vsset.mvol);
	VS_Set_Bass(vsset.bflimit,vsset.bass,vsset.tflimit,vsset.treble);  
	VS_Set_Effect(vsset.effect);
}































