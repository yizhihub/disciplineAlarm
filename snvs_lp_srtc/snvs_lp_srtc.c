/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "�����-�ٶ�����.h"
#include "����ʺ�С����.h"
//#include "˵���ټ�.h"
//#include "��ˮ�ٺ�-���غ�.h"
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
    lCount = 0x1ADD20; //0x450DDD; //sizeof(MusicData);
    i      = 0;
    
    VS_Init();
//    VS_HD_Reset();
    VS_Soft_Reset();
    msDelay(50);
    VS_Restart_Play();  					//�������� 
    VS_Set_All();        					//������������Ϣ 			 
    VS_Reset_DecodeTime();					//��λ����ʱ�� 	 
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

void MusicPlay2(void)
{
    INT32S lCount,i;   
    lCount = 0x37412D;//0x450DDE; //0x450DDD; //sizeof(MusicData);
    i      = 0;

    VS_Init();
//    VS_HD_Reset();
    VS_Soft_Reset();
    msDelay(50);
    VS_Restart_Play();  					//�������� 
    VS_Set_All();        					//������������Ϣ 			 
    VS_Reset_DecodeTime();					//��λ����ʱ�� 
    OLED_P8x16Str(16, 4, (uint8_t *)"PLay#2......", 1);
    while (lCount > 0) {

        if (VS_Send_MusicData(&Yangguangcaihongxiaobaima[i]) == 0) {
             i += 32;
             lCount -= 32;
        } else {
            GPIO1->DR ^= (1 << 19);  
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

int main(void)
{
    uint32_t sec;
    uint8_t  index, ucSecBak, ucBeepStatus;
    int16_t  sTemperature[2];
    int32_t  ulNetForbiddenCnt = 0;
    uint16_t usBatAdcRaw = 0;
    
    INT16U usi;
    
    snvs_hp_rtc_datetime_t rtcDate;
    snvs_lp_srtc_datetime_t srtcDate;
    snvs_hp_rtc_config_t snvsRtcConfig;
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
    GPIO_PinInit(GPIO1, 19, &led_config);              // led pin
    GPIO_PinInit(GPIO1, 4, &led_config);               // relay       AD_B0_04 
    led_config.direction = kGPIO_DigitalInput; 
    GPIO_PinInit(GPIO5, 0, &led_config);               // key
    led_config.direction = kGPIO_DigitalOutput; 
    
    /* 
     * OLED pin  ��ʼ��  
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
     GPIO_PinInit(GPIO1, 12, &led_config);             // DS18B20  UART1 TX     
     
    
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
      * VS1053 Init  VS_Sine_Test(), PRINTF("ram:%x\r\n", VS_Ram_Test()); ȥ����ЩTest����������ϵͳ��λ��������������
      */
    VS_Init();
    VS_HD_Reset();
    VS_Soft_Reset();
   
    vsset.mvol=187; // 195
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
        index = 0;
        sec = 0;

        /* Get date time */
        SNVS_HP_RTC_GetDatetime(SNVS, &rtcDate);  
        
        if (ucSecBak != rtcDate.second) {
            ucSecBak = rtcDate.second;
            if (ulNetForbiddenCnt > 0) ulNetForbiddenCnt--;
            
            GPIO1->DR ^= (1 << 19);                               /* LED Tog indicator          */ 
            if ((GPIO5->DR & 0x01) == 0) {
                while((GPIO5->DR & 0x01) == 0);
                SNVS_HP_RTC_GetDatetime(SNVS, &rtcDate);
                
                if (rtcDate.second - ucSecBak <= 1) {
                    MusicPlay1();
                    ulNetForbiddenCnt = 15 * 60;
                } else {
                    MusicPlay2();
                    ulNetForbiddenCnt = 30 * 60;
                }
                OLED_P8x16Str(16, 4, (uint8_t *)"key Press !!", 1);
                
            }
            
            /* 
             *  �¶�ʱ�������ʾ
             */
            ADC_SetChannelConfig(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
            while (0U == ADC_GetChannelStatusFlags(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP));
            usBatAdcRaw     = ADC_GetChannelConversionValue(BAT_ADC_BASE, BAT_ADC_CHANNEL_GROUP);
            sTemperature[0] = ReadTemperature();                        /* sTemperature[1] = ReadTemperature1();   */
      
            OLED_P16x32Time(0, &rtcDate);                               /* OLED Time Dispaly          */
            
            OLED_P8x16Dot(0, 6, sTemperature[0] / 10.0f, 1, 0);         /*   OLED_P16x32Num(1, sTemperature[1], 1); */
            OLED_P8x16Dot(64, 6, usBatAdcRaw * 3.35f / 4095.0f * 1.639 - 1.0195 , 2, 1);  /* it cost a lot to get thsi formala  -/\-*/

            PRINTF("Current datetime: %04hd-%02hd-%02hd %02hd:%02hd:%02hd\r\n", rtcDate.year, rtcDate.month, rtcDate.day,  /* print default time */
                   rtcDate.hour, rtcDate.minute, rtcDate.second);  
            PRINTF("Temp: %4d %4d \r\n", sTemperature[0], sTemperature[1]);

            
            /*
             * �����ϵ�ǰ�������ѡ�
             */ 
            if (rtcDate.hour == 21u && rtcDate.minute == 49) { 
                if (rtcDate.second < 58)
                    beepDi(500);
                else 
                   MusicPlay1();                // Play ready to sleep music beepDi(5000); // �������� С������C����ų���2.4s��                    
            } else { 
                beepDi(0);
            }
            
            /*
             * �������� 
             */
           if (rtcDate.hour == 06 && rtcDate.minute == 50 && rtcDate.second < 15) {
               
               OLED_P8x16Str(16, 4, (uint8_t *)"Get UPPP!!!!", 1);
               
                if (rtcDate.second < 10)
                    beepEi(500);
                else 
                    MusicPlay2();                 // Play GetUP Music, beepEi(10000); // ������������ų���5s��                    
            } else { 
                beepDi(0);
            }  
                
            
            /*
             * �̵�������
             */
            if ((rtcDate.hour == 21u && rtcDate.minute >= 50u) ||
                (rtcDate.hour == 22u) ||
                (rtcDate.hour == 23u) ||
                (rtcDate.hour == 0u)  || 
                (rtcDate.hour == 1u && rtcDate.minute <= 10u) ||
                ulNetForbiddenCnt > 0) {                                     /* add a temporary net-forbidden time counter  */
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
