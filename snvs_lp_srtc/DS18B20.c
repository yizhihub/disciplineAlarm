/*  参加第9届全国信息技术大赛  实操考试  by yizhi  ****/
#include "ds18b20.h"


//void delay(unsigned int i)//延时函数
//{
// while(i--);
//}


//18b20初始化函数
uint8_t Init_DS18B20(void)
{
    unsigned char x=0;
    DQ_SET_OUT;
    DQ_1;    //DQ复位
    usDelay(40);  //稍做延时
    DQ_0;    //单片机将DQ拉低
    usDelay(500); //精确延时 大于 480us
    DQ_1;    //拉高总线
    DQ_SET_IN;
    usDelay(60); // 等待15~60us后开始采样响应信号
    x=DQ;      //稍做延时后 如果x=0则初始化成功 x=1则初始化失败
    usDelay(150);  // 这里改为50us就不行了（移植到RT1052上发现的现象）
    return x;
}
uint8_t Init_DS18B21(void)
{
    unsigned char x=0;
    DQ1_SET_OUT;
    DQ1_1;    //DQ复位
    usDelay(40);  //稍做延时
    DQ1_0;    //单片机将DQ拉低
    usDelay(500); //精确延时 大于 480us
    DQ1_1;    //拉高总线
    DQ1_SET_IN;
    usDelay(60); // 等待15~60us后开始采样响应信号
    x=DQ1;      //稍做延时后 如果x=0则初始化成功 x=1则初始化失败
    usDelay(150);  // 这里改为50us就不行了（移植到RT1052上发现的现象）
    return x;
}


//读一个字节
unsigned char ReadOneChar(void)
{
    unsigned char i=0;
    unsigned char dat = 0;
    for (i=8;i>0;i--)
    {
        DQ_SET_OUT;
        DQ_0; // 给脉冲信号
        usDelay(2);
        dat >>= 1;  // 低电平装数据， 高电平读写  先读到的是最低位置
        DQ_1; // 给脉冲信号
        DQ_SET_IN;
        usDelay(10); // 10us过后，主机开始采样；
        if(DQ)
        dat|=0x80;
        usDelay(60);
    }
    return(dat);
}
unsigned char ReadOneChar1(void)
{
    unsigned char i=0;
    unsigned char dat = 0;
    for (i=8;i>0;i--)
    {
        DQ1_SET_OUT;
        DQ1_0; // 给脉冲信号
        usDelay(2);
        dat >>= 1;  // 低电平装数据， 高电平读写  先读到的是最低位置
        DQ1_1; // 给脉冲信号
        DQ1_SET_IN;
        usDelay(10); // 10us过后，主机开始采样；
        if(DQ1)
        dat|=0x80;
        usDelay(60);
    }
    return(dat);
}

//写一个字节
void WriteOneChar(unsigned char dat)
{
    unsigned char i=0;
    for (i=8; i>0; i--) 
    {
        DQ_SET_OUT;
        DQ_0;      //给脉冲信号
        usDelay(1);
        if (dat&0x01) // 先写低位
            DQ_1;
        else
            DQ_0;
        usDelay(30); //18B20 在给脉冲信号后,15us 到 60us 之间对DQ进行采样
        DQ_1; 
        usDelay(1);   
        dat>>=1;
    }
    usDelay(50);
}
void WriteOneChar1(unsigned char dat)
{
    unsigned char i=0;
    for (i=8; i>0; i--) 
    {
        DQ1_SET_OUT;
        DQ1_0;      //给脉冲信号
        usDelay(1);
        if (dat&0x01) // 先写低位
            DQ1_1;
        else
            DQ1_0;
        usDelay(30); //18B20 在给脉冲信号后,15us 到 60us 之间对DQ进行采样
        DQ1_1; 
        usDelay(1);   
        dat>>=1;
    }
    usDelay(50);
}


//读取温度
int16_t ReadTemperature(void)
{
	unsigned char a=0;
	unsigned char b=0;
	int16_t  temp=0;
	float tt=0;
	
	Init_DS18B20();
	WriteOneChar(0xCC); // 跳过读序号列号的操作 skip
	WriteOneChar(0x44); // 启动温度转换
	msDelay(100);
	Init_DS18B20();   // 每次操作都要发送复位脉冲,然后从机发送存在脉冲
	WriteOneChar(0xCC); //跳过读序号列号的操作 
	WriteOneChar(0xBE); //读取温度寄存器等（共可读9个寄存器） 前两个就是温度 高分辨率
	a=ReadOneChar(); // LSB
	b=ReadOneChar(); // MSB


	temp=	b<<8|a;
	tt=temp*0.0625f;// 原始浮点型温度值 
	temp=(long)(tt*10+0.5f);//扩大10倍后  四舍五入转化为整形
 
//   temp=b<<4;
//	 temp|=(a&0xf0)>>4;
 //  temp1_value=a&0x0f;
	return temp;
}
//读取温度
int16_t ReadTemperature1(void)
{
	unsigned char a=0;
	unsigned char b=0;
	int16_t  temp=0;
	float tt=0;
	
	Init_DS18B21();
	WriteOneChar1(0xCC); // 跳过读序号列号的操作 skip
	WriteOneChar1(0x44); // 启动温度转换
	msDelay(100);
	Init_DS18B21();   // 每次操作都要发送复位脉冲,然后从机发送存在脉冲
	WriteOneChar1(0xCC); //跳过读序号列号的操作 
	WriteOneChar1(0xBE); //读取温度寄存器等（共可读9个寄存器） 前两个就是温度 高分辨率
	a=ReadOneChar1(); // LSB
	b=ReadOneChar1(); // MSB


	temp=	b<<8|a;
	tt=temp*0.0625f;// 原始浮点型温度值 
	temp=(long)(tt*10+0.5f);//扩大10倍后  四舍五入转化为整形。
 
//   temp=b<<4;
//	 temp|=(a&0xf0)>>4;
 //  temp1_value=a&0x0f;
	return temp;
}
