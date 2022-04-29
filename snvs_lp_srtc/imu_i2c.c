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
** File Name:               imu_i2c.c
** Last modified Date:      2018-10-19
** Last Version:            1.0
** Description:             imu's i2c driver files 
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
#include "includes.h" 


/*********************************************************************************************************
 MACRO 
*********************************************************************************************************/
#define LPI2C_MASTER_BASEADDR   (LPI2C4)
#define LPI2C_MASTER_IRQN       (LPI2C4_IRQn)
#define LPI2C_MASTER_IRQHandler (LPI2C4_IRQHandler)    

#define MS5611_DEVICE           0xEE                               


/*********************************************************************************************************
 Global Variables 
*********************************************************************************************************/
extern SENSOR_DATA  GtImu;   

/*********************************************************************************************************
 Local Variables
*********************************************************************************************************/

/*
 * IMU I2C related 
 */
lpi2c_master_handle_t   __GtI2cMasterHandler;                           /* 非阻塞I2C 句柄               */
lpi2c_master_transfer_t __GtTransfer;                                   /* I2C传输结构体                */ 
volatile bool __GbMasterCompletionFlag = false;      
volatile bool __GbI2cNakFlg            = false;            
INT8U __GucI2cBuf[14] = {1, 1, 1, 1};
INT32U  GulPromCaliData[7];
/*********************************************************************************************************
** Function name:           lpi2c_master_callback 
** Descriptions:            RT1050的I2c中断回调函数，多个标志位会导致该进入该中断。  
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
void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
   if (status == kStatus_Success) {
        __GbMasterCompletionFlag  = true;
    }
    if (status == kStatus_LPI2C_Nak) {
        __GbI2cNakFlg             = true;
    }
}

/*********************************************************************************************************
** Function name:           i2cReadByte()
** Descriptions:            
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018-10-27
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/ 
uint8_t  i2cReadByte(uint8_t ucDeviceAddress, uint8_t ucReg)
{
    lpi2c_master_transfer_t tTransfer;                                  /* 传输结构体                   */  
    
    uint8_t  ucData[1];
    
    tTransfer.slaveAddress   = ucDeviceAddress >> 1;                    /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Read;                             /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)ucReg;                         /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &ucData[0];                              /* 数据缓冲区                   */
    tTransfer.dataSize       = 1;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
        assert(0);
    }
    return ucData[0];
}
/*********************************************************************************************************
** Function name:           i2cWrtieByte()
** Descriptions:            
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018-10-27
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/ 
void  i2cWriteByte(uint8_t ucDeviceAddress, uint8_t ucReg, uint8_t ucData)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */ 
    
    uint8_t  ucTmp = ucData;
    
    tTransfer.slaveAddress   = ucDeviceAddress >> 1;                    /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Write;                            /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)ucReg;                         /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &ucTmp;                                  /* 数据缓冲区                   */
    tTransfer.dataSize       = 1;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
        assert(0);
    }
}

/*********************************************************************************************************
** Function name:           i2cReadBuf()
** Descriptions:            
** input parameters:        none
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018-10-27
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/ 
void i2cReadBuf(uint8_t ucDeviceAddress, uint8_t ucReg, uint8_t *pucData, uint8_t ucLen)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */
    
    tTransfer.slaveAddress   = ucDeviceAddress >> 1;                    /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Read;                             /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)ucReg;                         /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = pucData;                                 /* 数据缓冲区                   */
    tTransfer.dataSize       = ucLen;                                   /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
        assert(0);
    }
}

/*********************************************************************************************************
** Function name:           imuI2cInit 
** Descriptions:            RT1050的I2C 初始化, 调用SDK包初始化 ， 暂未实现 
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
void  imuI2cInit (void)
{
    lpi2c_master_config_t masterConfig;
    
    /*
     * Clock setting for LPI2C
     */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0u);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 5u);
    
    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* 
     * Change the default baudrate configuration
     */
    masterConfig.baudRate_Hz = 400000u;

    /* 
     * Initialize the LPI2C master peripheral
     */
    LPI2C_MasterInit(LPI2C4, &masterConfig, (CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (6));  
    
    /*
     * Create handle to NO BLOCKING Transfer .  
     */
    LPI2C_MasterTransferCreateHandle(LPI2C4,&__GtI2cMasterHandler,lpi2c_master_callback, NULL);
    NVIC_SetPriority(LPI2C_MASTER_IRQN, 2);
}

/*********************************************************************************************************
** Function name:           ms5611RegInit
** Descriptions:            写IMU内部寄存器进行初始化
                            
** input parameters:        none
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年10月10日
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/ 

void ms5611RegInit (void)
{      
    INT8U     uci;  
//    INT32U    ulD1,ulD2,ulTemp;
//    INT32S    lDt, lBaroT, lPressure; 
//    INT64S    llOff, llSen;  
//    INT32U    ulT2, ulOff2, ulSen2;     
    

    PRINTF("start I2c Test \r\n");
    ms5611Reset();     

    /* 
     * populates its internal registers from the PROM need some time(at least 2ms), which is critical.
     *   
     */ 
    msDelay(10);
   
    for (uci = 0; uci <= 6; uci++) {
         
       GulPromCaliData[uci] = ms5611PromRead(0xA0 + 2 * uci);
       PRINTF("Prom Data%1d: %05d \r\n", uci, GulPromCaliData[uci]); 
    }
    
    /* 
     *  Just test the MS5611 sensor via Blocking transfer or Nonblocking too.   
     */
//    while(1) {
//        
//        ms561ConvStart(0x58);
//        msDelay(10);  
//        if(__GbMasterCompletionFlag == false) {
//            assert(0);
//        }     
//        ms5611ConvRead();
//        msDelay(1);
//        if(__GbMasterCompletionFlag == false) {
//            assert(0);
//        }    
//        ulD2 = __GucI2cBuf[0] << 16 | __GucI2cBuf[1] << 8 | __GucI2cBuf[2];  
//        
//        ms561ConvStart(0x48); 
//        msDelay(10); 
//        if(__GbMasterCompletionFlag == false) {
//            assert(0);
//        }    
//        ms5611ConvRead(); 
//        msDelay(1);
//        if(__GbMasterCompletionFlag == false) {
//            assert(0);
//        }  
//        ulD1 = __GucI2cBuf[0] << 16 | __GucI2cBuf[1] << 8 | __GucI2cBuf[2];
//   
//        /*
//         * Data Calculate , reference mannel Page7, Page8 
//         */
//        lDt    = ulD2 - (GulPromCaliData[5] << 8);
//        llOff  = ((INT64S)GulPromCaliData[2] << 16) + (INT64S)GulPromCaliData[4] * lDt / 128;
//        llSen  = ((INT64S)GulPromCaliData[1] << 15) + (INT64S)GulPromCaliData[3] * lDt / 256;
//  
//        lBaroT = 2000 + (long long)lDt * GulPromCaliData[6] / 8388608;  
//        
//        if (lBaroT < 2000) {
//            
//            ulT2   =  ((long long)lDt * lDt) / (1 << 31);  
//            ulTemp =  (lBaroT - 2000) * (lBaroT - 2000);
//            ulOff2 = ulTemp * 5 / 2;
//            ulSen2 = ulTemp * 5 / 4;
//            if (lBaroT < -1500) {
//                
//                ulTemp = (lBaroT + 1500) * (lBaroT * 1500);
//                ulOff2 = ulOff2 + 7 * ulTemp; 
//                ulSen2 = ulSen2 + 11 * ulTemp / 2; 
//            }
//            lBaroT = lBaroT - ulT2;
//            llOff        = llOff - ulOff2; 
//            llSen        = llSen - ulSen2;            
//        }
//        lPressure    = ((INT64S)ulD1 * llSen / 2097152 - llOff) / 32768;         
//        
////        fTemp += (lPressure - fTemp) * 0.05912 ; 
////        
////        lPressuerFilt = (INT32S)fTemp;
////        
////        fAltitudeCm = ((pow((101570.0f / lPressuerFilt), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f * 100.0f;         
//        
//        PRINTF("T: %2d.%02d C P: %4d.%02d \r\n", lBaroT/100, lBaroT%100, lPressure/100, lPressure%100);
//        msDelay(1);
//    }
}
/*********************************************************************************************************
** Function name:           ms5611Read
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
INT32U  GulMsReadErrCnt = 0;
void ms5611Read (SENSOR_DATA *ptMPU)
{  
    static INT8U  ucMsState = 0;  
    static INT16U usMsCnt   = 0; 
    static INT32U ulD1,ulD2;
           INT32S lDt,lBaroT, lPressure;
           INT64S llOff, llSen;  
           INT32U ulT2, ulOff2, ulSen2, ulTemp;      
    
    usMsCnt++;   
    
    switch (ucMsState) {
        
    case 0:                                                             /* initiate pressure convert    */

        ms561ConvStart(0x48); 
        usMsCnt = 0;  
        ucMsState = 1; 
        
        break;
    
    case 1:                                                            /* Start the pressure read  */
        
       
        if (usMsCnt > 9) {
            if(__GbMasterCompletionFlag == true) {
                ms5611ConvRead();  
                usMsCnt = 0;
                ucMsState = 2; 
            } else {
                ucMsState = 0;
                GulMsReadErrCnt++; 
            }
        }

        break;
        
    case 2:                                                           /* Get the pressure result and initiate temperature convert */
        
        if (__GbMasterCompletionFlag == true) {
            ulD1 = __GucI2cBuf[0] << 16 | __GucI2cBuf[1] << 8 | __GucI2cBuf[2];
            ms561ConvStart(0x58); 
            usMsCnt = 0;
            ucMsState = 3;
        } else if (usMsCnt > 2) {
            ucMsState = 0;
            GulMsReadErrCnt++;
        }
        
        break;
    
    case 3:                                                           /* start the temperature  read    */                                                    
        
        if (usMsCnt > 9) {
           if(__GbMasterCompletionFlag == true) {
                ms5611ConvRead();  
                usMsCnt = 0;
                ucMsState = 4; 
            } else {
                ucMsState = 0; 
                GulMsReadErrCnt++;
            }
        }
        
        break;
        
    case 4:                                                         /* Get the temperature result       */
        
        if (__GbMasterCompletionFlag == true) {
            ulD2 = __GucI2cBuf[0] << 16 | __GucI2cBuf[1] << 8 | __GucI2cBuf[2]; 
            usMsCnt = 0;
            ucMsState = 0;  
            
            /*
             * Data Calculate , reference mannel Page7, Page8 
             */
            lDt    = ulD2 - (GulPromCaliData[5] << 8);
            llOff  = ((INT64S)GulPromCaliData[2] << 16) + (INT64S)GulPromCaliData[4] * lDt / 128;
            llSen  = ((INT64S)GulPromCaliData[1] << 15) + (INT64S)GulPromCaliData[3] * lDt / 256;
      
            lBaroT = 2000 + (long long)lDt * GulPromCaliData[6] / 8388608;  
            
            if (lBaroT < 2000) {
                
                ulT2   =  (long long)lDt * (long long)lDt / 2147483648U; //  (1 << 31);  
                ulTemp =  (lBaroT - 2000) * (lBaroT - 2000);
                ulOff2 = ulTemp * 5 / 2;
                ulSen2 = ulTemp * 5 / 4;
                if (lBaroT < -1500) {
                    
                    ulTemp = (lBaroT + 1500) * (lBaroT * 1500);
                    ulOff2 = ulOff2 + 7 * ulTemp; 
                    ulSen2 = ulSen2 + 11 * ulTemp / 2; 
                }
                lBaroT = lBaroT - ulT2;  
                llOff        = llOff - ulOff2; 
                llSen        = llSen - ulSen2;            
            }
            lPressure    = ((INT64S)ulD1 * llSen / 2097152 - llOff) / 32768;     
            
            if (lPressure > 0) {
                ptMPU->lPressure    = lPressure; 
                ptMPU->lBaroT       = lBaroT;
                ptMPU->ucIsPressureOk = 1;
            }
        } else if (usMsCnt > 2) {
            ucMsState = 0; 
            GulMsReadErrCnt++;
            
        }
        
        break;
    default: 
        
        assert(0);
        
        break;        
    }
}

/*********************************************************************************************************
** Function name:           ms5611Reset
** Descriptions:            复位ms5611Reset  reference user mannel I2C INTERFACE RESET SEQUENCE  
**                          
** input parameters:        noee
**                          
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年10月01日
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
void ms5611Reset(void)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */ 
    
    uint8_t  ucTmp = 0;
    
    tTransfer.slaveAddress   = MS5611_DEVICE >> 1;                               /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Write;                            /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)0x1E;                          /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &ucTmp;                                  /* 数据缓冲区                   */
    tTransfer.dataSize       = 0;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

//    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
//        assert(0);
//    }
    if (LPI2C_MasterTransferNonBlocking(LPI2C4, &__GtI2cMasterHandler, &tTransfer) != kStatus_Success) {         
        assert(0); 
    } 
    __GbMasterCompletionFlag = false;
}

/*********************************************************************************************************
** Function name:           ms5611PromRead
** Descriptions:            Read factory  calibarate coefficient  reference user mannel I2C PROM READ SEQUENCE  
**                          
** input parameters:        ucAddr is repersent six coefficients, 0xA0 ~ 0xAE.
**                           
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年10月01日
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
INT16U  ms5611PromRead(INT8U ucAddr)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */ 
    
    uint8_t  ucTmp[2];
    
    tTransfer.slaveAddress   = MS5611_DEVICE >> 1;                               /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Write;                             /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)ucAddr;                        /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &ucTmp;                                  /* 数据缓冲区                   */
    tTransfer.dataSize       = 0;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
        assert(0);
    } 
    
    
    
    tTransfer.slaveAddress   = MS5611_DEVICE >> 1;                               /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Read;                             /* 读数据                       */ 
    tTransfer.subaddress     = NULL;                                    /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 0;                                       /* 子地址长度                   */
    tTransfer.data           = &ucTmp;                                  /* 数据缓冲区                   */
    tTransfer.dataSize       = 2;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
        assert(0);
    }
    
    return ucTmp[0] << 8 | ucTmp[1]; 
}

/*********************************************************************************************************
** Function name:           ms561ConvStart
** Descriptions:            command to initiate a pressure or temperature conversion   
**                          
** input parameters:        ucConvCmd will be follow value and meaning, D1 type is pressure ,D2 is temper
**                          Convert D1(OSR = 4096)             0x48 
**                          Convert D2(OSR = 4096)             0x58
** output parameters:       none
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年10月01日
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
void  ms561ConvStart(INT8U ucConvCmd)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */ 
    
    uint8_t  ucTmp;
    
    tTransfer.slaveAddress   = MS5611_DEVICE >> 1;                               /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Write;                            /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)ucConvCmd;                     /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &ucTmp;                                  /* 数据缓冲区                   */
    tTransfer.dataSize       = 0;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

//    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
//        assert(0);
//    }
    if (LPI2C_MasterTransferNonBlocking(LPI2C4, &__GtI2cMasterHandler, &tTransfer) != kStatus_Success) {         
        assert(0);
    } 
    __GbMasterCompletionFlag = false;
}

/*********************************************************************************************************
** Function name:           ms5611ConvRead
** Descriptions:            Read conversion result  
**                          
** input parameters:        none
**                           
** output parameters:       24bit result;
** Returned value:          none
** Created by:              lgd
** Created Date:            2018年10月01日
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
**
*********************************************************************************************************/
INT32U ms5611ConvRead(void)
{
    lpi2c_master_transfer_t tTransfer;                                   /* 传输结构体                  */ 
    
    tTransfer.slaveAddress   = MS5611_DEVICE >> 1;                               /* 设备地址                     */
    tTransfer.direction      = kLPI2C_Read;                             /* 读数据                       */ 
    tTransfer.subaddress     = (uint32_t)0x00;                          /* 要读取的寄存器地址           */
    tTransfer.subaddressSize = 1;                                       /* 子地址长度                   */
    tTransfer.data           = &__GucI2cBuf;                            /* 数据缓冲区                   */
    tTransfer.dataSize       = 3;                                       /* 要读取的数据长度             */
    tTransfer.flags=kLPI2C_TransferDefaultFlag;

//    if (LPI2C_MasterTransferBlocking(LPI2C4,&tTransfer)==kStatus_Fail) {
//        assert(0);
//    } 
    if (LPI2C_MasterTransferNonBlocking(LPI2C4, &__GtI2cMasterHandler, &tTransfer) != kStatus_Success) {         
        assert(0);
    } 
    __GbMasterCompletionFlag = false;
    
    return 0;
}
/*********************************************************************************************************
 THE END 
*********************************************************************************************************/
