/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��ZYF/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��10��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
��Target �� TC264DA/TC264D
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
����iLLD_1_0_1_11_0�ײ����,

ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
����CIFΪTC264DA�����⣬�����Ĵ������TC264D
����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
________________________________________________________________
����ADC������һ���ü���MIC�Ϳ�������ADC���ɡ�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_ADC.h>
#include <LQ_GPIO_LED.h>
#include <LQ_OLED096.h>
#include <LQ_STM.h>
#include <LQ_UART.h>
#include <stdio.h>
#include <LQ_TFT18.h>
/*************************************************************************
*  �������ƣ�void TestADC(void)
*  ����˵����ADC�ɼ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��   ADC���ղ�Ϊ�㣬�����
*************************************************************************/
void Test_ADC(void)
{
	unsigned long i;
    char txt[16];
    uint16 vbat;
    ADC_InitConfig(ADC0, 80000); //��ʼ��
    ADC_InitConfig(ADC1, 80000);
    ADC_InitConfig(ADC2, 80000);
    ADC_InitConfig(ADC3, 80000);
    ADC_InitConfig(ADC4, 80000);
    ADC_InitConfig(ADC5, 80000);
    ADC_InitConfig(ADC6, 80000);
    ADC_InitConfig(ADC7, 80000);

    TFTSPI_Init(0);        //LCD��ʼ��  0:����  1������
        TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
        TFTSPI_Show_Logo(0,37);//��ʾ����LOGO
        TFTSPI_P16x16Str(0,0,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);        //�ַ�����ʾ
        TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);       //�ַ�����ʾ
        delayms(1000);              //��ʱ�ȴ�
        TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
        TFTSPI_CLS(u16BLACK);   //��ɫ��Ļ
        TFTSPI_P8X16Str(0,1,"ADC:  ",u16WHITE,u16BLACK);        //�ַ�����ʾ

    while(1)
    {


    	unsigned long nowTime = STM_GetNowUs(STM0);//����ʱ�俪ʼ
    	for( i = 0; i < 10000; i++)
    	{
    	    vbat=ADC_Read(ADC7);
    	    vbat = vbat * 11 / 25;  // x/4095*3.3*100*5.7
    	    sprintf(txt, "Bat:%d.%02dV ", vbat / 100, vbat % 100);  // *3.3/4095*3
    	}
    	int time = STM_GetNowUs(STM0) - nowTime;   //��������
    	sprintf(txt, "time : %d", time);           //ת��Ϊ�ַ���
    	OLED_P6x8Str(0,3,txt);                      //��ʾ����ʱ��

    }

}
/*************************************************************************
*  �������ƣ�void Test_Beacon7mic(void)
*  ����˵�����ɼ�����6+1��˷�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��    ADC���ղ�Ϊ��
*************************************************************************/
void Test_ADC_7mic(void)
{
    char txt[200];
    unsigned int MICVmin[8];

    ADC_InitConfig(ADC0, 80000); //��ʼ��
    ADC_InitConfig(ADC1, 80000); //��ʼ��
    ADC_InitConfig(ADC2, 80000); //��ʼ��
    ADC_InitConfig(ADC3, 80000); //��ʼ��
    ADC_InitConfig(ADC4, 80000); //��ʼ��
    ADC_InitConfig(ADC5, 80000); //��ʼ��
    ADC_InitConfig(ADC6, 80000); //��ʼ��
    ADC_InitConfig(ADC7, 80000); //��ʼ��   ���ʹ������ĸ��  ����ѹ��ĵ�ص�ѹ��������Կ�ĸ��ԭ��ͼ
    delayms(500);                //��ʱ�ȴ�
	UART_PutStr(UART0,"\n-------------------------------------------------\n");//�ֽ���
	while(1)
	{
		MICVmin[0]=ADC_Read(ADC0);
		MICVmin[1]=ADC_Read(ADC1);
		MICVmin[2]=ADC_Read(ADC2);
		MICVmin[3]=ADC_Read(ADC3);
		MICVmin[4]=ADC_Read(ADC4);
		MICVmin[5]=ADC_Read(ADC5);
		MICVmin[6]=ADC_Read(ADC6);
		MICVmin[7]=ADC_Read(ADC7);

		//ת�����Ϊ�ַ��������ڷ��ͻ�����Ļ��ʾ
		sprintf(txt,"[0]:%05d  [1]:%05d  [2]:%05d  [3]:%05d  [4]:%05d  [5]:%05d  [6]:%05d  [7]:%05d  \r\n",MICVmin[0],MICVmin[1],MICVmin[2],MICVmin[3],MICVmin[4],MICVmin[5],MICVmin[6],MICVmin[7]);
		//���ڷ��͵���λ��
		UART_PutStr(UART0,txt);//AN0--AN7ʮ���������������λ��


    	LED_Ctrl(LED0,RVS);        //��ƽ��ת,LED��˸
		delayms(500);              //��ʱ�ȴ�
	}
}
//
