/*  �μӵ�9��ȫ����Ϣ��������  ʵ�ٿ���  by yizhi  ****/
#include "ds18b20.h"


//void delay(unsigned int i)//��ʱ����
//{
// while(i--);
//}


//18b20��ʼ������
uint8_t Init_DS18B20(void)
{
    unsigned char x=0;
    DQ_SET_OUT;
    DQ_1;    //DQ��λ
    usDelay(40);  //������ʱ
    DQ_0;    //��Ƭ����DQ����
    usDelay(500); //��ȷ��ʱ ���� 480us
    DQ_1;    //��������
    DQ_SET_IN;
    usDelay(60); // �ȴ�15~60us��ʼ������Ӧ�ź�
    x=DQ;      //������ʱ�� ���x=0���ʼ���ɹ� x=1���ʼ��ʧ��
    usDelay(150);  // �����Ϊ50us�Ͳ����ˣ���ֲ��RT1052�Ϸ��ֵ�����
    return x;
}
uint8_t Init_DS18B21(void)
{
    unsigned char x=0;
    DQ1_SET_OUT;
    DQ1_1;    //DQ��λ
    usDelay(40);  //������ʱ
    DQ1_0;    //��Ƭ����DQ����
    usDelay(500); //��ȷ��ʱ ���� 480us
    DQ1_1;    //��������
    DQ1_SET_IN;
    usDelay(60); // �ȴ�15~60us��ʼ������Ӧ�ź�
    x=DQ1;      //������ʱ�� ���x=0���ʼ���ɹ� x=1���ʼ��ʧ��
    usDelay(150);  // �����Ϊ50us�Ͳ����ˣ���ֲ��RT1052�Ϸ��ֵ�����
    return x;
}


//��һ���ֽ�
unsigned char ReadOneChar(void)
{
    unsigned char i=0;
    unsigned char dat = 0;
    for (i=8;i>0;i--)
    {
        DQ_SET_OUT;
        DQ_0; // �������ź�
        usDelay(2);
        dat >>= 1;  // �͵�ƽװ���ݣ� �ߵ�ƽ��д  �ȶ����������λ��
        DQ_1; // �������ź�
        DQ_SET_IN;
        usDelay(10); // 10us����������ʼ������
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
        DQ1_0; // �������ź�
        usDelay(2);
        dat >>= 1;  // �͵�ƽװ���ݣ� �ߵ�ƽ��д  �ȶ����������λ��
        DQ1_1; // �������ź�
        DQ1_SET_IN;
        usDelay(10); // 10us����������ʼ������
        if(DQ1)
        dat|=0x80;
        usDelay(60);
    }
    return(dat);
}

//дһ���ֽ�
void WriteOneChar(unsigned char dat)
{
    unsigned char i=0;
    for (i=8; i>0; i--) 
    {
        DQ_SET_OUT;
        DQ_0;      //�������ź�
        usDelay(1);
        if (dat&0x01) // ��д��λ
            DQ_1;
        else
            DQ_0;
        usDelay(30); //18B20 �ڸ������źź�,15us �� 60us ֮���DQ���в���
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
        DQ1_0;      //�������ź�
        usDelay(1);
        if (dat&0x01) // ��д��λ
            DQ1_1;
        else
            DQ1_0;
        usDelay(30); //18B20 �ڸ������źź�,15us �� 60us ֮���DQ���в���
        DQ1_1; 
        usDelay(1);   
        dat>>=1;
    }
    usDelay(50);
}


//��ȡ�¶�
int16_t ReadTemperature(void)
{
	unsigned char a=0;
	unsigned char b=0;
	int16_t  temp=0;
	float tt=0;
	
	Init_DS18B20();
	WriteOneChar(0xCC); // ����������кŵĲ��� skip
	WriteOneChar(0x44); // �����¶�ת��
	msDelay(100);
	Init_DS18B20();   // ÿ�β�����Ҫ���͸�λ����,Ȼ��ӻ����ʹ�������
	WriteOneChar(0xCC); //����������кŵĲ��� 
	WriteOneChar(0xBE); //��ȡ�¶ȼĴ����ȣ����ɶ�9���Ĵ����� ǰ���������¶� �߷ֱ���
	a=ReadOneChar(); // LSB
	b=ReadOneChar(); // MSB


	temp=	b<<8|a;
	tt=temp*0.0625f;// ԭʼ�������¶�ֵ 
	temp=(long)(tt*10+0.5f);//����10����  ��������ת��Ϊ����
 
//   temp=b<<4;
//	 temp|=(a&0xf0)>>4;
 //  temp1_value=a&0x0f;
	return temp;
}
//��ȡ�¶�
int16_t ReadTemperature1(void)
{
	unsigned char a=0;
	unsigned char b=0;
	int16_t  temp=0;
	float tt=0;
	
	Init_DS18B21();
	WriteOneChar1(0xCC); // ����������кŵĲ��� skip
	WriteOneChar1(0x44); // �����¶�ת��
	msDelay(100);
	Init_DS18B21();   // ÿ�β�����Ҫ���͸�λ����,Ȼ��ӻ����ʹ�������
	WriteOneChar1(0xCC); //����������кŵĲ��� 
	WriteOneChar1(0xBE); //��ȡ�¶ȼĴ����ȣ����ɶ�9���Ĵ����� ǰ���������¶� �߷ֱ���
	a=ReadOneChar1(); // LSB
	b=ReadOneChar1(); // MSB


	temp=	b<<8|a;
	tt=temp*0.0625f;// ԭʼ�������¶�ֵ 
	temp=(long)(tt*10+0.5f);//����10����  ��������ת��Ϊ���Ρ�
 
//   temp=b<<4;
//	 temp|=(a&0xf0)>>4;
 //  temp1_value=a&0x0f;
	return temp;
}
