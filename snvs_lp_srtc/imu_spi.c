/*********************************************Copyright (c)***********************************************
**                                Guangzhou ZLG MCU Technology Co., Ltd.
**
**                                        http://www.zlgmcu.com
**
**      ������������Ƭ���Ƽ����޹�˾���ṩ�����з�������ּ��Э���ͻ����ٲ�Ʒ���з����ȣ��ڷ�����������ṩ
**  ���κγ����ĵ������Խ����������֧�ֵ����Ϻ���Ϣ���������ο����ͻ���Ȩ��ʹ�û����вο��޸ģ�����˾��
**  �ṩ�κε������ԡ��ɿ��Եȱ�֤�����ڿͻ�ʹ�ù��������κ�ԭ����ɵ��ر�ġ�żȻ�Ļ��ӵ���ʧ������˾��
**  �е��κ����Ρ�
**                                                                        �D�D������������Ƭ���Ƽ����޹�˾
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               imu_spi.c
** Last modified Date:      2018-10-19
** Last Version:            1.0
** Description:             imu's spi driver files 
**
**--------------------------------------------------------------------------------------------------------
** Created By:              liguangdao
** Created date:            2018-04-10
** Version:                 1.0
** Descriptions:            The original version ��ʼ�汾
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             lgd(liguangdao)
** Modified date:           2018-10-19
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "board.h" 
#include "fsl_lpspi.h"



/*********************************************************************************************************
 MACRO 
*********************************************************************************************************/                          
#define LPSPI_MASTER_BASEADDR   (LPSPI3)
#define LPSPI_MASTER_IRQN       (LPSPI3_IRQn)
#define LPSPI_MASTER_IRQHandler (LPSPI3_IRQHandler)

//#define MPU_LPSPI_MASTER_PCS_FOR_INIT_1   (kLPSPI_Pcs0)                               /* OnBoard Gyro.  */
//#define MPU_LPSPI_MASTER_PCS_FOR_INIT_2   (kLPSPI_Pcs1)                               /* imu board Gyro.*/    
//#define AMS_LPSPI_MASTER_PCS_FOR_INIT     (kLPSPI_Pcs2)
//#define NRF_LPSPI_MASTER_PCS_FOR_INIT     (kLPSPI_Pcs3)

//#define MPU_PCS_1                         (kLPSPI_MasterPcs0)  
//#define MPU_PCS_2                         (kLPSPI_MasterPcs1)
//#define AMS_PCS                           (kLPSPI_MasterPcs2) 
//#define NRF_PCS                           (kLPSPI_MasterPcs3)  

/*********************************************************************************************************
 Struct Sensor 
*********************************************************************************************************/ 
struct sensor_data {
    
    INT16S sGyroX;
    INT16S sGyroY;
    INT16S sGyroZ;
    INT16S sGyroFiltX;
    INT16S sGyroFiltY;
    INT16S sGyroFiltZ;
    INT16S sAcceX;
    INT16S sAcceY;
    INT16S sAcceZ;
    INT16S sAcceFiltX;
    INT16S sAcceFiltY;
    INT16S sAcceFiltZ;
    INT16S sGyroT;
    INT16S sGyroOffsetX;
    INT16S sGyroOffsetY;
    INT16S sGyroOffsetZ;
    
    float  fGyroX; 
    float  fGyroY;
    float  fGyroZ;
    float  fGyroT;
    float  fAcceX;
    float  fAcceY;
    float  fAcceZ;  
    float  fGyroFiltX; 
    float  fGyroFiltY;
    float  fGyroFiltZ;
    float  fAcceFiltX;
    float  fAcceFiltY;
    float  fAcceFiltZ;  
    float  fGyroIntX;                                                /* X�� ���ֵýǶ�                  */
    float  fGyroIntY;                                                /* Y�� ���ֵýǶ�                  */
    float  fGyroIntZ;                                                /* Z�� ���ֵýǶ�                  */
    float  fAcceRol;                                                 /* ���ٶȼƲ����roll              */
    float  fAccePit;                                                 /* ���ٶȼƲ����pitch             */
    float  fRol;                                                     /* Roll -> ��X�᷽����ת           */
    float  fPit;                                                     /* Pitch-> ��Y�᷽����ת           */
    float  fYaw;                                                     /* Yaw  -> ��Z�᷽����ת           */ 
    float  fRolOffset;                                               /* ����ˮƽУ׼                    */
    float  fPitOffset;                                               /* ����ˮƽУ׼                    */  
    INT32S lBaroT;                                                   /* ��ѹ���е��¶�ֵ��0.01��        */
    INT32S lPressure;                                                /* ԭʼ��ѹֵ        0.01mbar      */ 
    INT32S lPressureFilt;                                            /* �˲������ѹ��ֵ  0.01mbar      */  
    float  fAltitudeCm;                                              /* ���θ߶�          1cm           */
    float  fAltitudeStartCm;                                         /* ���ʱ����߶�                  */ 
    float  fHeightCm;                                                /* ���и߶ȣ��������ɵ���        */
    float  fAcceZDeG;                                                /* ȥ���������ٶȺ��Z����ٶ� cm/s^2 */     
    float  fAcceZDeGFilt;     
    float  fAcceZDeGDead;          
    float  fEstVelZ;                                                 /* ����õ��Ĵ�ֱ�����ٶ�          */                         
    float  fEstLocZ;                                                 /* ����õ��ķɻ��߶�              */
volatile   INT8U  ucIsPressureOk;                                           /* ѹ����ѹ�и���                  */
volatile   INT8U  ucIsRstHeight;                                            /* ��λ���θ߶�                    */
volatile   INT8U  ucIsGyroBiasOK;                                           /* �Ƿ�������У׼��                */
   
    /*
     *  ���ٶȼ�У׼���ӱ���
     */
    float  fAcceOffsetX;
    float  fAcceOffsetY;
    float  fAcceOffsetZ;
    float  fAcceSlopeX;
    float  fAcceSlopeY;
    float  fAcceSlopeZ;
    INT8U  ucAcceStartCorrect;
    INT8U  ucAcceFinishCorrect;
    INT8U  ucIsAcceXPosOK;
    INT8U  ucIsAcceXNegOK;
    INT8U  ucIsAcceYPosOK;
    INT8U  ucIsAcceYNegOK;
    INT8U  ucIsAcceZPosOK;
    INT8U  ucIsAcceZNegOK; 
    INT16U usAcceSampleNum;

};
typedef  struct sensor_data SENSOR_DATA ; 


/*********************************************************************************************************
 Global Variables 
*********************************************************************************************************/

/*********************************************************************************************************
 Local Variables
*********************************************************************************************************/

/*
 * IMU SPI related   
 */ 
volatile  bool __GbIsMasterTransferCompleted = true;                    /* signal flag for SPI done    */
INT8U  __GucRxBuf[32] = {1, 1, 1, 1, 1, 1, 1};
INT8U  __GucTxBuf[32] = {0x80 + 0x3B};                                   /* 0x3B is the Read Base        */
INT8U  __GucRxCount   = 0;
INT8U  __GucTxCount   = 0;
INT32U __GulImuPcs    = 0;  
INT8U  __GucDeviceId  = 0;
SENSOR_DATA  GtImu; 

/* 
 *  MPU6500 �Ĵ�����ַ���� 
 */
#define  PWR_MGMT_1        0x6B
#define  SIGNAL_PATH_RESET 0x68   
#define  USER_CTRL         0x6A  

#define  PWR_MGMT_2        0x6C 
#define  CONFIG            0x1A 
#define  SMPLRT_DIV        0x19 
#define  GYRO_CONFIG       0x1B
#define  ACCEL_CONFIG      0x1C 
#define  ACCEL_CONFIG2     0x1D 

/*********************************************************************************************************
** Function name:           LPSPI_MASTER_IRQHandler 
** Descriptions:            RT1050��SPI�жϷ������������־λ�ᵼ�¸ý�����жϡ�  
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            
** Descriptions:            
**                          
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**                          
*********************************************************************************************************/
void LPSPI_MASTER_IRQHandler (void)
{ 
    LPSPI_DisableInterrupts(LPSPI_MASTER_BASEADDR, kLPSPI_RxInterruptEnable);  /*  close the interrupt and read data  */
    while (LPSPI_GetRxFifoCount(LPSPI_MASTER_BASEADDR)) {
        
        __GucRxBuf[__GucRxCount++] = LPSPI_ReadData(LPSPI_MASTER_BASEADDR);
    }
    
    if (__GucRxCount < 15) {
        LPSPI_EnableInterrupts(LPSPI_MASTER_BASEADDR, kLPSPI_RxInterruptEnable);
    } else {
        LPSPI_DisableInterrupts(LPSPI_MASTER_BASEADDR, kLPSPI_RxInterruptEnable); 
        __GbIsMasterTransferCompleted = true;
    }
    
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*********************************************************************************************************
** Function name:           imuSpiInit 
** Descriptions:            RT1050��SPI ��ʼ��, ����SDK����ʼ�� 
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            
** Descriptions:            
**                          
**--------------------------------------------------------------------------------------------------------
** Modified by:            lgd 
** Modified date:          MPU_LPSPI_MASTER_PCS_FOR_INIT_2 ���һ��Ƭѡ��ʼ����
**                          
*********************************************************************************************************/
void  imuSpiInit(void)
{
    lpspi_master_config_t masterConfig;
    
    /*
     * Clock setting for LPI2C
     */
    CLOCK_SetMux(kCLOCK_LpspiMux, 1);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 7);
    
    masterConfig.baudRate = 2000000;
    masterConfig.bitsPerFrame = 8;
    masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction = kLPSPI_MsbFirst;

    masterConfig.pcsToSckDelayInNanoSec = 20;
    masterConfig.lastSckToPcsDelayInNanoSec = 600; 
    masterConfig.betweenTransferDelayInNanoSec = 600;

    masterConfig.whichPcs = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    masterConfig.pinCfg = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig = kLpspiDataOutRetained;
    LPSPI_MasterInit(LPSPI_MASTER_BASEADDR, &masterConfig, CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk)/8);
    
    masterConfig.whichPcs = kLPSPI_Pcs2;
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

}
/*********************************************************************************************************
** Function name:           spiReadWriteByte
** Descriptions:            ��ָ���Ĵ�������ַ��д��һ��8λ���ݣ�ͬʱ����һ��8λ���ݡ�
                            
** input parameters:        ulPcs :  Ƭѡ�ź�  ����SDK���ĺ궨�� 
**                          ucReg :  Ҫ��д�ļĴ�����ַ
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018��8��10��
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
INT8U spiReadWriteByte(INT32U ulPcs, INT8U ucReg, INT8U ucData)
{
    status_t  kStatus;
    lpspi_transfer_t  xTrans;
    INT8U  ucTxBuf[2];
    INT8U  ucRxBuf[2];
    volatile INT8U i;
    
    ucTxBuf[0] = ucReg;
    ucTxBuf[1] = ucData;
  
    xTrans.dataSize    = 2;  
    xTrans.txData      = ucTxBuf;
    xTrans.rxData      = ucRxBuf; 
    xTrans.configFlags = ulPcs | kLPSPI_MasterPcsContinuous;
    
    kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR, &xTrans);  
 
    for(i = 0; i < 250; i++) {
        
    }
    
    for(i = 0; i < 250; i++) {
        
    }  
    
   if (kStatus != kStatus_Success) {
        assert(0);
   }
    return  ucRxBuf[1];
}
/*********************************************************************************************************
** Function name:           spiReadWriteBuf
** Descriptions:            д��N��8λ���ݣ�ͬʱ����N��8λ���ݡ�    ��δʵ�� ��ָ���Ĵ�������ַ) ��д ���SDK
                            
** input parameters:        ulPcs :  Ƭѡ�ź�  ����SDK���ĺ궨�� 
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018��8��10��
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
void spiReadWriteBuf(INT32U ulPcs, INT8U *pucTxBuf, INT8U *pucRxBuf, INT16U usLen)
{
    status_t  kStatus;
    
    lpspi_transfer_t  xTrans;

    xTrans.dataSize    = usLen;
    xTrans.txData      = pucTxBuf;
    xTrans.rxData      = pucRxBuf;
    xTrans.configFlags = ulPcs | kLPSPI_MasterPcsContinuous;
    
   kStatus = LPSPI_MasterTransferBlocking(LPSPI_MASTER_BASEADDR,  &xTrans);    
    
   if (kStatus != kStatus_Success) {
        assert(0);
   }
} 

/*********************************************************************************************************
** Function name:           mpu6500RegInit
** Descriptions:            дMPU6500�ڲ��Ĵ������г�ʼ��
                            
** input parameters:        
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018��8��10��
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/ 
void mpu6500RegInit (void)
{
    
    __GulImuPcs = kLPSPI_MasterPcs0;                                            /* Detect Onboard Gyro firstly  */      
    
    __GucDeviceId = spiReadWriteByte(__GulImuPcs, 0x80 + 0x75, 0xFF);  
//    __GucDeviceId = 0xFF;
   
    if(__GucDeviceId == 0xFF || __GucDeviceId == 0x00) {                /* There is no Onboard Gyro     */ 
         
        __GulImuPcs   = kLPSPI_MasterPcs2;                                      /* Dectect Outboard Gyro Secondly */
        __GucDeviceId = spiReadWriteByte(__GulImuPcs, 0x80 + 0x75, 0xFF);  
    }
    msDelay(5);   
    
    /* 
     *  The follow Device ID: 
     *  MPU6000: 0x68 
     *  MPU6500: 0x70 
     *  MPU9250: 0x71
     */
    if (__GucDeviceId == 0x68 || __GucDeviceId == 0x70 || __GucDeviceId == 0x71) {
        
        PRINTF("MPU Detected : 0x%02x \r\n", __GucDeviceId);
    } else {
        
        PRINTF("MPU Failed   : 0x%02x \r\n", __GucDeviceId);
        assert(0);                                                      /* assert fail                  */ 
    }
    
    spiReadWriteByte(__GulImuPcs, PWR_MGMT_1,       0x80);              /* write 1 to Reset the internal register and restores the default settings , the bit will auto clear */
    msDelay(100);
    spiReadWriteByte(__GulImuPcs, SIGNAL_PATH_RESET,0x07);              /* reset Gyro  Accel Temp        */
    msDelay(100);
    spiReadWriteByte(__GulImuPcs, USER_CTRL,        0x11);              /* disable DMP,  use SPI Bus Only*/
    
    /* 
     * ѡ��ʱ��Դ 
     */
     spiReadWriteByte(__GulImuPcs, PWR_MGMT_1, 0x01);                   /* auto select the best available clock,PLL or Internal oscillator */ 
    
     /* 
      * Register address : 0x19
      * Sample Rate Divider = 0 , No Divider ,namely 1KHz 
      */ 
     spiReadWriteByte(__GulImuPcs, SMPLRT_DIV,   0x00); 
    
    /* 
     *  Register address 0x1A 
     *  FIFO_MODE = 0 replacing the oldest data.  
     *  EXT_SYNC_SET = 0 function disable.
     *  DLPF_CFG  = 1 BandWidth = 92Hz  Delay-3.9ms Fs-1KHz   
     */
     spiReadWriteByte(__GulImuPcs, CONFIG,     0x02);

     /* 
      * Register address 0x1B
      * don't bypass the Gyro DLPF  no self-test   Gyro Measure Scale is +-2000dps 
      */ 
     spiReadWriteByte(__GulImuPcs, GYRO_CONFIG,  0x18); 
     
     /* 
      * Register address 0x1C
      * no self-test  Acce Measure Scale is +-4g 
      */
     spiReadWriteByte(__GulImuPcs, ACCEL_CONFIG, 0x08); 
     /*
      * Register address 0x1D
      * don't bypass the DLPF   BandWidth = 41Hz 
      */
     spiReadWriteByte(__GulImuPcs, ACCEL_CONFIG2, 0x03);
     
     msDelay(10);   
     
}

/*********************************************************************************************************
** Function name:           mpu6500Read
** Descriptions:            ��ȡIMU���� 
                            
** input parameters:        
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018��8��10��
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
void mpu6500Read (SENSOR_DATA *ptMPU)
{  
    INT8U  uci; 

    /* 
     *******************Wait for master and slave transfer completed.***************** 
     *  works well without Int   
     *  while (!__GbIsMasterTransferCompleted);   
     */
    while (!__GbIsMasterTransferCompleted); 
    
    while (LPSPI_GetRxFifoCount(LPSPI_MASTER_BASEADDR)) {
        
        __GucRxBuf[__GucRxCount++] = LPSPI_ReadData(LPSPI_MASTER_BASEADDR); 
    }
    
    if (__GucDeviceId == 0x68) {
        
        ptMPU->sAcceX =  (__GucRxBuf[3]<<8 | __GucRxBuf[4]);
        ptMPU->sAcceY = -(__GucRxBuf[1]<<8 | __GucRxBuf[2]);
        ptMPU->sAcceZ =  (__GucRxBuf[5]<<8 | __GucRxBuf[6]);
        ptMPU->sGyroT =  (__GucRxBuf[7]<<8 | __GucRxBuf[8]); 

        ptMPU->sGyroX =  (__GucRxBuf[11] << 8 | __GucRxBuf[12]);
        ptMPU->sGyroY = -(__GucRxBuf[9]  << 8 | __GucRxBuf[10]);
        ptMPU->sGyroZ =  (__GucRxBuf[13] << 8 | __GucRxBuf[14]); 

        ptMPU->sGyroX -= ptMPU->sGyroOffsetX;
        ptMPU->sGyroY -= ptMPU->sGyroOffsetY;
        ptMPU->sGyroZ -= ptMPU->sGyroOffsetZ;    
    } else {
        ptMPU->sAcceX = __GucRxBuf[1]<<8 | __GucRxBuf[2];
        ptMPU->sAcceY = __GucRxBuf[3]<<8 | __GucRxBuf[4];
        ptMPU->sAcceZ = __GucRxBuf[5]<<8 | __GucRxBuf[6];
        ptMPU->sGyroT = __GucRxBuf[7]<<8 | __GucRxBuf[8]; 

        ptMPU->sGyroX = __GucRxBuf[9]  << 8 | __GucRxBuf[10];
        ptMPU->sGyroY = __GucRxBuf[11] << 8 | __GucRxBuf[12];
        ptMPU->sGyroZ = __GucRxBuf[13] << 8 | __GucRxBuf[14]; 

        ptMPU->sGyroX -= ptMPU->sGyroOffsetX;
        ptMPU->sGyroY -= ptMPU->sGyroOffsetY;
        ptMPU->sGyroZ -= ptMPU->sGyroOffsetZ;    
    } 
    
    ptMPU->fGyroT =  36.53f + (float)ptMPU->sGyroT / 340.0f;
    
    /* 
     * Trigger Next Sample  
     */
    __GbIsMasterTransferCompleted = false; 

    __GucTxCount = 0;
    __GucRxCount = 0;

    /* 
    * Config the TCR  
    */
    LPSPI_MASTER_BASEADDR->TCR =
    (LPSPI_MASTER_BASEADDR->TCR &
     ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
    LPSPI_TCR_CONT(1) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(__GulImuPcs >> LPSPI_MASTER_PCS_SHIFT);

    /* 
     * TCR is also shared the FIFO , so wait for TCR written.
     */
    while (LPSPI_GetTxFifoCount(LPSPI_MASTER_BASEADDR) != 0); 

    for (uci = 0; uci < 15; uci++)   {
        LPSPI_WriteData(LPSPI_MASTER_BASEADDR, __GucTxBuf[uci]);      
    }


    LPSPI_MASTER_BASEADDR->TCR = (LPSPI_MASTER_BASEADDR->TCR & ~(LPSPI_TCR_CONTC_MASK));
    
    /*
     *  works well without Int   
     *  
     */
    LPSPI_EnableInterrupts(LPSPI_MASTER_BASEADDR, kLPSPI_RxInterruptEnable);   
}
/*********************************************************************************************************
 THE END 
*********************************************************************************************************/
