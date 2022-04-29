#ifndef _ds18B20_h_
#define _ds18B20_h_

#include "fsl_common.h"
#include "fsl_snvs_hp.h"



#define DQ_SET_IN     GPIO1->GDIR &= ~(1U << 12)
#define DQ_SET_OUT    GPIO1->GDIR |= (1u << 12)
#define DQ_1          GPIO1->DR |= (1 << 12)
#define DQ_0          GPIO1->DR &= ~(1 << 12)
#define DQ            (GPIO1->DR & (1 << 12)) >> 12 


#define DQ1_SET_IN     GPIO1->GDIR &= ~(1U << 13)
#define DQ1_SET_OUT    GPIO1->GDIR |= (1u << 13)
#define DQ1_1          GPIO1->DR |= (1 << 13)
#define DQ1_0          GPIO1->DR &= ~(1 << 13)
#define DQ1            (GPIO1->DR & (1 << 13)) >> 13

uint8_t Init_DS18B20(void);
uint8_t Init_DS18B21(void);
int16_t ReadTemperature(void);
int16_t ReadTemperature1(void);

#endif 

