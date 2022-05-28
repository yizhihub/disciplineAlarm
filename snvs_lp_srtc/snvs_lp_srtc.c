/*
 * The Clear BSD License
 * All rights reserved.
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_snvs_hp.h"
#include "fsl_snvs_lp.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "oled.h"
#include "DS18B20.h"
#include "imu_spi.h"
#include "vs10xx.h"
#include "MusicDataMP3.h"
#include "虫儿飞-少儿歌曲.h"                                                   // 虫儿飞 GBK乱码测试
#include "小字一组.h"
//#include "阳光彩虹小白马.h"
//#include "说了再见.h"
//#include "香水百合-张韶涵.h"
#include "fsl_adc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define kCLOCK_SnvsHp0 kCLOCK_SnvsHp
#define EXAMPLE_SNVS_IRQn SNVS_HP_WRAPPER_IRQn
#define EXAMPLE_SNVS_IRQHandler SNVS_HP_WRAPPER_IRQHandler 

#define BAT_ADC_BASE            ADC1
#define BAT_ADC_CHANNEL         2U
#define BAT_ADC_CHANNEL_GROUP   0U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile bool busyWait;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
 
void MusicPlay1(void)
{
    INT32S lCount,i;   
    lCount = 0x1ADD20;
    i      = 0;
    
    VS_Init();
//    VS_HD_Reset();
    VS_Soft_Reset();
    msDelay(50);
    VS_Restart_Play();                                                  // 重启播放 
    VS_Set_All();                                                       // 设置音量等信息 
    VS_Reset_DecodeTime();                                              // 复位解码时间  
    OLED_P8x16Str(16, 4, (uint8_t *)"PLay#1......", 1);    
    while (lCount > 0) {

        if (VS_Send_MusicData(&Chongerfei[i]) == 0) {
             i += 32;
             lCount -= 32;
        } else {
            GPIO1->DR ^= (1 << 19);  
        }
    }
    OLED_P8x16Str(16, 4, (uint8_t *)"            ", 1); 
    
}  

void MusicPlay2(uint8_t week)
{
    INT32S lCount,i,j,lDatalen, lDatalenSevenDivide;
    lDatalen            =  0x9DEBC;
    lDatalenSevenDivide = (lDatalen + 7u) / 7u;
    
    i      = 0;

    VS_Init();
//    VS_HD_Reset();
    VS_Soft_Reset();
    msDelay(50);
    VS_Restart_Play();                                                  //重启播放
    VS_Set_All();                                                       //设置音量等信息
    VS_Reset_DecodeTime();                                              //复位解码时间
    OLED_P8x16Str(16, 4, (uint8_t *)"PLay#2......", 1); 
   
    /*
     * 无论星期几，播放时间都控制在1min左右，18x32s
     */
    for (j = 0; j < ((8 - week) << 5); j++) {
        if (week < 7)
            lCount = lDatalenSevenDivide * week;                        // week = 1~7
        else 
            lCount = lDatalen;

        i = 0;
        VS_Restart_Play();                                              //重启播放
        VS_Set_All();                                                   //设置音量等信息
        VS_Reset_DecodeTime();                                          //复位解码时间
        while (lCount > 0) {

            if (VS_Send_MusicData(&Xiaoziyizu[i]) == 0) {
                 i += 32;
                 lCount -= 32;
            } else {
                GPIO1->DR ^= (1 << 19);  
            }
        }
    }
    
    OLED_P8x16Str(16, 4, (uint8_t *)"            ", 1); 
} 
 
 

void EXAMPLE_SNVS_IRQHandler(void)
{
    if (SNVS_HP_RTC_GetStatusFlags(SNVS) & kSNVS_RTC_AlarmInterruptFlag)
    {
        busyWait = false;

        /* Clear alarm flag */
        SNVS_HP_RTC_ClearStatusFlags(SNVS, kSNVS_RTC_AlarmInterruptFlag);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}


void beepDi(uint16_t ucStatus)
{
    uint16_t i;
    if (ucStatus != 0) {
        
        for (i = 0; i < ucStatus; i++) {
             GPIO1->DR ^= 1;
             usDelay(463);
        }
    } else {
        GPIO1->DR_CLEAR  = 1;
    }
}

void beepEi(uint16_t ucStatus)
{
    uint16_t i;
    if (ucStatus != 0) {
        
        for (i = 0; i < ucStatus; i++) {
             GPIO1->DR ^= 1;
             usDelay(500);
        }
    } else {
        GPIO1->DR_CLEAR  = 1;
    }
}

/*
* 基姆拉尔森计算公式 
* W= (d+2*m+3*(m+1)/5+y+y/4-y/100+y/400) mod 7
* https://www.cnblogs.com/lfri/p/11509669.html  
* https://blog.csdn.net/yf210yf/article/details/9744259 
* https://blog.csdn.net/x1131230123/article/details/103067453
* 
*/
int weekCalc(int y, int m, int d)
{
    if (m < 3) {
        m += 12;
        y--;
    }

    int w = (d + 2*m + 3*(m + 1)/5 + y + y/4 - y/100 + y/400 + 1) % 7;
    return w;
}

int main(void)
{
    uint8_t  ucSecBak;
    int16_t  sTemperature[2];
    int32_t  ulNetForbiddenCnt = 0;
    uint16_t usBatAdcRaw = 0;
    uint8_t  ucWeekDay;
    
    snvs_hp_rtc_datetime_t rtcDate;
    snvs_hp_rtc_config_t snvsRtcConfig;
//    snvs_lp_srtc_datetime_t srtcDate;
    snvs_lp_srtc_config_t snvsSrtcConfig;
    gpio_pin_config_t led_config ={kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
    
    adc_config_t adcConfigStrcut;
    adc_channel_config_t adcChannelConfigStruct;

    /* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    
    
    /*
     * LED and Relay pin and key
     */
    GPIO_PinInit(GPIO1, 19, &led_config);                               // led pin
    GPIO_PinInit(GPIO1, 4, &led_config);                                // relay       AD_B0_04 
    led_config.direction = kGPIO_DigitalInput;                         
    GPIO_PinInit(GPIO5, 0, &led_config);                                // key
    led_config.direction = kGPIO_DigitalOutput; 
    
    /* 
     * OLED pin  初始化  
     */
    GPIO_PinInit(GPIO5, 2, &led_config);
    GPIO_PinInit(GPIO3, 4, &led_config);
    GPIO_PinInit(GPIO3, 2, &led_config);
    GPIO_PinInit(GPIO2, 30,&led_config);
    GPIO_PinInit(GPIO1, 20,&led_config); 
    
    /*
     * Beep pin
     */ 
    GPIO_PinInit(GPIO1, 0, &led_config); 
    
    
    /*
     *  AS1053 pin 
     */
    led_config.outputLogic = 1;
    GPIO_PinInit(GPIO1, 15, &led_config);
    led_config.direction = kGPIO_DigitalInput; 
    GPIO_PinInit(GPIO1, 14, &led_config);
  
  
    /*
     *  DS18B20  pin   GPIO_PinInit(GPIO1, 13, &led_config) change to ADC channel 
     */
     led_config.direction = kGPIO_DigitalInput;
     GPIO_PinInit(GPIO1, 12, &led_config);                              // DS18B20  UART1 TX     
     
    
    /* Init SNVS_HP */
    /*
     * config->rtcCalEnable = false;
     * config->rtcCalValue = 0U;
     * config->periodicInterruptFreq = 0U;
     */
    SNVS_HP_RTC_GetDefaultConfig(&snvsRtcConfig);
    SNVS_HP_RTC_Init(SNVS, &snvsRtcConfig);

    /* Init SNVS_LP */
    /*
     * config->srtcCalEnable = false;
     * config->srtcCalValue = 0U;
     */
    SNVS_LP_SRTC_GetDefaultConfig(&snvsSrtcConfig);
    SNVS_LP_SRTC_Init(SNVS, &snvsSrtcConfig);
    
    /* Set a start date time and start RT */
//    srtcDate.year = 2022U;
//    srtcDate.month = 1U;
//    srtcDate.day = 11U;
//    srtcDate.hour = 22U;
//    srtcDate.minute = 28;
//    srtcDate.second = 30;

//    SNVS_LP_SRTC_SetDatetime(SNVS, &srtcDate);
//    SNVS_LP_SRTC_StartTimer(SNVS); 

    /* Synchronize RTC time and date with SRTC and start RTC */
    SNVS_HP_RTC_TimeSynchronize(SNVS);
    SNVS_HP_RTC_StartTimer(SNVS);

    /* Enable SNVS alarm interrupt */
    SNVS_HP_RTC_EnableInterrupts(SNVS, kSNVS_RTC_AlarmInterrupt);
    EnableIRQ(EXAMPLE_SNVS_IRQn); 
    
    
     ADC_GetDefaultConfig(&adcConfigStrcut);
     ADC_Init(BAT_ADC_BASE, &adcConfigStrcut);
#if !(defined(FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE) && FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE)
    ADC_EnableHardwareTrigger(BAT_ADC_BASE, false);
#endif
    if (kStatus_Success == ADC_DoAutoCalibration(ADC1))                        /* Do auto hardware calibration. */
    {
        PRINTF("ADC_DoAntoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
    }
    adcChannelConfigStruct.channelNumber = BAT_ADC_CHANNEL;
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;

    OLED_Init();
    OLED_Fill(0x00);
    OLED_P8x16Str(16, 4, (uint8_t *)"First Reset", 1); 
        
     /*
      * DS18B20 Init   //    Init_DS18B21(); 
      */
//  while(Init_DS18B20() & Init_DS18B21())
//  {
//       OLED_P8x16Str(0,0,"DS18B20 is't connected!!",1);
//       msDelay(100);
//   }
    Init_DS18B20();

    
     /*
      * VS1053 Init  VS_Sine_Test(), PRINTF("ram:%x\r\n", VS_Ram_Test()); 去掉这些Test，避免意外系统复位产生过多噪音。
      */
    VS_Init();
    VS_HD_Reset();
    VS_Soft_Reset();
   
    vsset.mvol=190;
    VS_Set_All(); 
    
//    while(1) {
//        
//        PRINTF("ram:%x\r\n", VS_Ram_Test()); 
//        VS_Sine_Test();
//         msDelay(300); 
//        ADC_SetChannelConfig(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
//        while (0U == ADC_GetChannelStatusFlags(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP));
//        PRINTF("ADC Value: %d\r\n", ADC_GetChannelConversionValue(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP));
//        OLED_P8x16Four(64, 6, ADC_GetChannelConversionValue(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP));
//    }

    while (1)
    {
        busyWait = true;

        /* Get date time */
        SNVS_HP_RTC_GetDatetime(SNVS, &rtcDate);  
        
        if (ucSecBak != rtcDate.second) {
            ucSecBak = rtcDate.second;
            if (ulNetForbiddenCnt > 0) ulNetForbiddenCnt--;
            
            /*
             * 按键音乐测试 
             */ 
            GPIO1->DR ^= (1 << 19);                               /* LED Tog indicator          */ 
            if ((GPIO5->DR & 0x01) == 0) {
                while((GPIO5->DR & 0x01) == 0);
                SNVS_HP_RTC_GetDatetime(SNVS, &rtcDate);
                
                if (rtcDate.second - ucSecBak <= 1) {
                    MusicPlay1();
                    ulNetForbiddenCnt = 15 * 60;
                } else {
                    
                    /*
                     * 根据星期几播放的不同的音乐
                     */ 
                    ucWeekDay = weekCalc(rtcDate.year, rtcDate.month, rtcDate.day);        /*  0 ~ 6 */ 
                    if (ucWeekDay == 0) ucWeekDay = 7; 
                    MusicPlay2(ucWeekDay);  
                    ulNetForbiddenCnt = 30 * 60;
                }
                OLED_P8x16Str(16, 4, (uint8_t *)"key Press !!", 1);
                
            }
            
            /* 
             *  温度时间电量显示
             */
            ADC_SetChannelConfig(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
            while (0U == ADC_GetChannelStatusFlags(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP));
            usBatAdcRaw     = ADC_GetChannelConversionValue(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP);
            sTemperature[0] = ReadTemperature();                        /* sTemperature[1] = ReadTemperature1();   */
      
            OLED_P16x32Time(0, &rtcDate);                               /* OLED Time Dispaly          */
            
//            OLED_P8x16Four(0, 2, weekCalc(rtcDate.year, rtcDate.month, rtcDate.day));
            OLED_P8x16Dot(0, 6, sTemperature[0] / 10.0f, 1, 0);         /*   OLED_P16x32Num(1, sTemperature[1], 1); */
            OLED_P8x16Dot(64, 6, usBatAdcRaw * 3.35f / 4095.0f * 1.639f - 1.0195f , 2, 1);  /* it cost a lot to get thsi formala  -/\-*/

            PRINTF("Current datetime: %04hd-%02hd-%02hd %02hd:%02hd:%02hd\r\n", rtcDate.year, rtcDate.month, rtcDate.day,  /* print default time */
                   rtcDate.hour, rtcDate.minute, rtcDate.second);  
            PRINTF("Temp: %4d %4d \r\n", sTemperature[0], sTemperature[1]);

            
            /*
             * 断网断电前警报提醒及继电器动作
             */ 
            if (rtcDate.hour == 22u && rtcDate.minute == 9u) { 
                if (rtcDate.second < 58)
                    beepDi(500);
                else 
                   MusicPlay1();                                        // Play ready to sleep music beepDi(5000); // 持续发声 小字三组C，大概持续2.4s。                    
            } else { 
                beepDi(0);
            }

            if ((rtcDate.hour == 22u && rtcDate.minute >= 10u) ||
                (rtcDate.hour == 23u) ||
                (rtcDate.hour == 0u)  || 
                (rtcDate.hour == 1u && rtcDate.minute <= 10u) ||
                ulNetForbiddenCnt > 0) {                                /* add a temporary net-forbidden time counter  */
                GPIO1->DR  |= (1 << 4); 
                if (ulNetForbiddenCnt > 0) {
                    OLED_P8x16Str(16, 4, (uint8_t *)"Remain:", 0);
                    OLED_P8x16Four(16 + 56, 4, ulNetForbiddenCnt);
                } else {                    
                    OLED_P8x16Str(16, 4, (uint8_t *)"NO LMML !!!!", 1);
                }
            } else { 
                GPIO1->DR  &= ~(1 << 4); 
                OLED_P8x16Str(16, 4, (uint8_t *)"            ", 1);
            }
            
            /*
             * 闹钟铃声 
             */
           if (rtcDate.hour == 06 && rtcDate.minute == 00 && rtcDate.second < 15) {
               
               OLED_P8x16Str(16, 4, (uint8_t *)"Get UPPP!!!!", 1);
               
                if (rtcDate.second < 10) {
                    beepEi(500);
                } else {
                    
                    /*
                     * 根据星期几播放的不同的音乐
                     */ 
                    ucWeekDay = weekCalc(rtcDate.year, rtcDate.month, rtcDate.day);        /*  0 ~ 6 */ 
                    if (ucWeekDay == 0) ucWeekDay = 7; 
                    MusicPlay2(ucWeekDay);                              // Play GetUP Music, beepEi(10000); // 持续发声，大概持续5s。
                }
            } else { 
                beepDi(0);
            }  
        }
//        if (rtcDate.second % 2 == 0) {
//            OLED_SCL_0;
//            OLED_SDA_0;
//            OLED_RST_0;
//            OLED_DC_0;
//            OLED_CS_0;
//        } else {
//            OLED_SCL_1;
//            OLED_SDA_1;
//            OLED_RST_1;
//            OLED_DC_1;
//            OLED_CS_1;
//        }
        
        /* Get alarm time from user */
//        PRINTF("Please input the number of second to wait for alarm and press enter \r\n");
//        PRINTF("The second must be positive value\r\n");

//        while (index != 0x0D)
//        {
//            index = GETCHAR();
//            if ((index >= '0') && (index <= '9'))
//            {
//                PUTCHAR(index);
//                sec = sec * 10 + (index - 0x30U);
//            }
//        }
//        PRINTF("\r\n");

//        SNVS_HP_RTC_GetDatetime(SNVS, &rtcDate);
//        if ((rtcDate.second + sec) < 60)
//        {
//            rtcDate.second += sec;
//        }
//        else
//        {
//            rtcDate.minute += (rtcDate.second + sec) / 60U;
//            rtcDate.second = (rtcDate.second + sec) % 60U;
//        }

//        SNVS_HP_RTC_SetAlarm(SNVS, &rtcDate);

//        /* Get alarm time */
//        SNVS_HP_RTC_GetAlarm(SNVS, &rtcDate);

//        /* Print alarm time */
//        PRINTF("Alarm will occur at: %04hd-%02hd-%02hd %02hd:%02hd:%02hd\r\n", rtcDate.year, rtcDate.month, rtcDate.day,
//               rtcDate.hour, rtcDate.minute, rtcDate.second);

//        /* Wait until alarm occurs */
//        while (busyWait)
//        {
//        }

//        PRINTF("\r\n Alarm occurs !!!! ");
    }
}
