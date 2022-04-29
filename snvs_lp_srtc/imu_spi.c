/*********************************************Copyright (c)***********************************************
**                                Guangzhou ZLG MCU Technology Co., Ltd.
**
**                                        http://www.zlgmcu.com
**
**      广州周立功单片机科技有限公司所提供的所有服务内容旨在协助客户加速产品的研发进度，在服务过程中所提供
**  的任何程序、文档、测试结果、方案、支持等资料和信息，都仅供参考，客户有权不使用或自行参考修改，本公司不
**  提供任何的完整性、可靠性等保证，若在客户使用过程中因任何原因造成的特别的、偶然的或间接的损失，本公司不
**  承担任何责任。
**                                                                        ――广州周立功单片机科技有限公司
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
** Descriptions:            The original version 初始版本
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
    float  fGyroIntX;                                                /* X轴 积分得角度                  */
    float  fGyroIntY;                                                /* Y轴 积分得角度                  */
    float  fGyroIntZ;                                                /* Z轴 积分得角度                  */
    float  fAcceRol;                                                 /* 加速度计测出的roll              */
    float  fAccePit;                                                 /* 加速度计测出的pitch             */
    float  fRol;                                                     /* Roll -> 沿X轴方向旋转           */
    float  fPit;                                                     /* Pitch-> 沿Y轴方向旋转           */
    float  fYaw;                                                     /* Yaw  -> 沿Z轴方向旋转           */ 
    float  fRolOffset;                                               /* 机体水平校准                    */
    float  fPitOffset;                                               /* 机体水平校准                    */  
    INT32S lBaroT;                                                   /* 气压计中的温度值，0.01℃        */
    INT32S lPressure;                                                /* 原始气压值        0.01mbar      */ 
    INT32S lPressureFilt;                                            /* 滤波后的气压计值  0.01mbar      */  
    float  fAltitudeCm;                                              /* 海拔高度          1cm           */
    float  fAltitudeStartCm;                                         /* 起飞时地面高度                  */ 
    float  fHeightCm;                                                /* 飞行高度，相对于起飞地面        */
    float  fAcceZDeG;                                                /* 去除重力加速度后的Z轴加速度 cm/s^2 */     
    float  fAcceZDeGFilt;     
    float  fAcceZDeGDead;          
    float  fEstVelZ;                                                 /* 估算得到的垂直升降速度          */                         
    float  fEstLocZ;                                                 /* 估算得到的飞机高度              */
volatile   INT8U  ucIsPressureOk;                                           /* 压力气压有更新                  */
volatile   INT8U  ucIsRstHeight;                                            /* 复位海拔高度                    */
volatile   INT8U  ucIsGyroBiasOK;                                           /* 是否陀螺仪校准过                */
   
    /*
     *  加速度计校准增加变量
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
 *  MPU6500 寄存器地址定义 
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
** Descriptions:            RT1050的SPI中断服务函数，多个标志位会导致该进入该中断。  
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
** Descriptions:            RT1050的SPI 初始化, 调用SDK包初始化 
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            
** Descriptions:            
**                          
**--------------------------------------------------------------------------------------------------------
** Modified by:            lgd 
** Modified date:          MPU_LPSPI_MASTER_PCS_FOR_INIT_2 添加一个片选初始化。
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

}
/*********************************************************************************************************
** Function name:           spiReadWriteByte
** Descriptions:            向指定寄存器（地址）写入一个8位数据，同时读出一个8位数据。
                            
** input parameters:        ulPcs :  片选信号  参照SDK包的宏定义 
**                          ucReg :  要读写的寄存器地址
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年8月10日
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
** Descriptions:            写入N个8位数据，同时读出N个8位数据。    暂未实现 向指定寄存器（地址) 读写 需改SDK
                            
** input parameters:        ulPcs :  片选信号  参照SDK包的宏定义 
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年8月10日
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
** Descriptions:            写MPU6500内部寄存器进行初始化
                            
** input parameters:        
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年8月10日
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
     * 选择时钟源 
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
** Descriptions:            读取IMU数据 
                            
** input parameters:        
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年8月10日
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
