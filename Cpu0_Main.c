/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��chiusir
��E-mail��chiusir@163.com
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
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
=================================================================
����ͷ�ӿ�                  �������ۻ���OV7725ģ��
�� ���ݶ˿ڣ�P02.0-P02.7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� ʱ�����أ��ⲿ�жϵ�0�飺P00_4��
�� ���źţ��ⲿ�жϵ�3�飺P15_1��
-----------------------------------------------------------------
�Ƽ�GPT12ģ�飬������ʵ��5·�����������������������ݴ�������������źŲɼ�������ѡ����·���ɣ�
P33_7, P33_6   ����TCĸ�������1
P02_8, P33_5   ����TCĸ�������2
P10_3, P10_1   ����TCĸ�������3
P20_3, P20_0   ����TCĸ�������4
-----------------------------------------------------------------
��е�ѹ�ɼ�ģ�������˷�ģ��
�Ƽ�ʹ��AN0-7������·ADC����������chirp�����źż���ų���е�ѹ�ɼ���
AN0-3          ����TC���ĸ���˷�ģ����ߵ��
-----------------------------------------------------------------
Ĭ�ϵ���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM0��0-7ͨ����
��һ��˫·ȫ������
P23_1         ����TCĸ��MOTOR1_P
P32_4         ����TCĸ��MOTOR1_N
P21_2         ����TCĸ��MOTOR2_P
P22_3         ����TCĸ��MOTOR2_N
�ڶ���˫·ȫ������
P21_4         ����TCĸ��MOTOR3_P
P21_3         ����TCĸ��MOTOR3_N
P20_8         ����TCĸ��MOTOR4_P
P21_5         ����TCĸ��MOTOR4_N
-----------------------------------------------------------------
Ĭ�϶���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM2��
P33_10        ����TCĸ����1
P33_13        ����TCĸ����2
-----------------------------------------------------------------
 Ĭ����Ļ��ʾ�ӿڣ��û�����ʹ��TFT����OLEDģ��
TFTSPI_CS     P20_14     ����TCĸ�� CS�ܽ� Ĭ�����ͣ����Բ���
TFTSPI_SCK    P20_11     ����TCĸ�� SPI SCK�ܽ�
TFTSPI_SDI    P20_10     ����TCĸ�� SPI MOSI�ܽ�
TFTSPI_DC     P20_12     ����TCĸ�� D/C�ܽ�
TFTSPI_RST    P20_13     ����TCĸ�� RESET�ܽ�
-----------------------------------------------------------------
OLED_CK       P20_14     ����TCĸ��OLED CK�ܽ�
OLED_DI       P20_11     ����TCĸ��OLED DI�ܽ�
OLED_RST      P20_10     ����TCĸ��OLED RST�ܽ�
OLED_DC       P20_12     ����TCĸ��OLED DC�ܽ�
OLED_CS       P20_13     ����TCĸ��OLED CS�ܽ� Ĭ�����ͣ����Բ���
----------------------------------------------------------------
Ĭ�ϰ����ӿ�
KEY0p         P22_0      ����TCĸ���ϰ���0
KEY1p         P22_1      ����TCĸ���ϰ���1
KEY2p         P22_2      ����TCĸ���ϰ���2
DSW0p         P33_9      ����TCĸ���ϲ��뿪��0
DSW1p         P33_11     ����TCĸ���ϲ��뿪��1
-----------------------------------------------------------------
Ĭ��LED�ӿ�
LED0p         P10_6      ����TCĸ����İ���LED0 ����
LED1p         P10_5      ����TCĸ����İ���LED1 ����
LED2p         P20_6      ����TCĸ����LED0
LED3p         P20_7      ����TCĸ����LED1
-----------------------------------------------------------------
Ĭ�Ϸ������ӿ�
BEEPp         P33_8      ����TCĸ���Ϸ������ӿ�
-----------------------------------------------------------------
��ʱ��
������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
�Ƽ�ʹ��CCU6ģ�飬STM����ϵͳʱ�ӻ�����ʱ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h>//����ģ���ͷ�ļ�
#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <LQ_CAMERA.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_MotorServo.h>
#include <LQ_SOFTI2C.h>
#include <LQ_TFT18.h>
#include <stdint.h>
#include <LQ_UART.h>
#include <LQ_Inductor.h>
#include <Main.h>
#include "LQ_ImageProcess.h"
#include <LQ_PID.h>
#include<ANO_DT.h>


App_Cpu0 g_AppCpu0; // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 ��ʼ����ɱ�־λ

volatile char mutexCpu0TFTIsOk=0;         // CPU1 0ռ��/1�ͷ� TFT
pid_param_t pids,pidspeed;  //pid����
//#define Servo_Delta 170
#define Servo_Center_Mid 1270      //���ֱ����ֵ��
#define Servo_Right_Max   (Servo_Center_Mid-240)      //�����ת����ֵ
#define Servo_Left_Min (Servo_Center_Mid+230)      //�����ת����ֵ����ֵ��������÷�ʽ�йأ���ʽ
//short duty1 = 2500;
//signed short duty2 = 1410;
float Error,Re_servo,Error_h,Error_s,Errorspeed=0,Error_hw;
float Expectspeed=1.5;//m/s
float nowspeed=-8000;
extern float  KP,KI,KD,KP2,KI2,KD2;
int Servo_duty,servoTurnDuty;
//unsigned char flag1=0xFF;
unsigned char *data;
int flag=0;       //��������
//extern float kp,ki,kd;
//extern int error_e,error_d,temp_duty;
//sint16 A=3,B=2,L=1.5;
//extern float Lleft1 , Lleft2 , Lright2 , Lright1 ;  // ���ƫ����
//sint16 Lleft1 = 0, Lleft2 = 0, Lright2 = 0, Lright1 = 0;  // ���ƫ����

/*************************************************************************
*  �������ƣ�int core0_main (void)
*  ����˵����CPU0������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��
*************************************************************************/
int core0_main (void)
{
    //char txt[16];
   // char txt[32];
//    int flag1=0;
//    char txt1[32];
//    char txt2[32];

    // �ر�CPU���ж�
    IfxCpu_disableInterrupts();

    // �رտ��Ź�����������ÿ��Ź�ι����Ҫ�ر�
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    // ��ȡ����Ƶ��
    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

             // ����
    //TFTSPI_P16x16Str(0,0,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);// �ַ�����ʾ

    // ������ʼ��
    GPIO_KEY_Init();
    // LED������P10.6��P10.5��ʼ��
    GPIO_LED_Init();

    // ����P14.0�ܽ����,P14.1���룬������115200
    UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);

    // ����CPU���ж�
    IfxCpu_enableInterrupts();

    // ֪ͨCPU1��CPU0��ʼ�����
    IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=0;         // CPU1�� 0ռ��/1�ͷ� TFT

    // ����������Ƶ��ַ��https://space.bilibili.com/95313236
    // ���²��Ժ������ڽ���ѭ�����û��ɵ�������ģ��ĳ�ʼ������д������ʵ���Լ�������
    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 //  Test_ADC();            //PASS,����ADC����ʱ��  OLED����ʾ ADC����10K��ʱ��
//    Test_ADC_7mic();       //PASS,����ADC\UART0��STM��ʱ�����ƣ�ͨ��UART0��ӡ AN0--AN7��8��ͨ��ADCת������
//    LQ_Atom_Motor_8chPWM();//PASS,����GTM_ATOM���ɲ�ͬƵ���µ�8·PWM
//    LQ_ATom_Servo_2chPWM();//PASS,����GTM_ATOM��STM��ʱ�����ƣ�P33.10��P33.13��ΪATOM����ڿ��ƶ��
//    LQ_Tom_Servo_2chPWM(); //PASS,����GTM_TOM��STM��ʱ�����ƣ�P33.10��P33.13��ΪTOM����ڿ��ƶ��
//    Test_GPIO_Extern_Int();//PASS,�����ⲿ��1���ж�P15.8��P10.6��P10.5����
//    Test_GPIO_LED();       //PASS,����GPIO��P10.6��P10.5����
//    Test_GPIO_KEY();       //PASS,�����ⲿ�������룬P22.0--2   ���°��� LED��
//    Test_ComKEY_Tft();     //PASS,�����ⲿ��ϰ������벢TFT1.8��ʾ��P22.0--2
//    LQ_GPT_4mini512();     //PASS,���Ա�������������,OLED��UART���
//    LQ_GPT_4mini512TFT();  //PASS,���Ա�������������,TFT1.8��UART���
//    Test_OLED();           //PASS,����OLED0.96��ʹ��P20.14--10����ʾ�ַ�������̬����
//    LQ_STM_Timer();        //PASS,���Զ�ʱ�жϡ�����
  // Test_TFT18();          //PASS,����TFT1.8����ʹ��P20.14--10����ʾ�ַ�������̬����
//    LQ_TIM_InputCature();  //PASS,����GTM_TOM_TIM��P20_9��ΪPWM����ڣ�P15_0��ΪTIM����ڣ����߶̽Ӻ󣬴���P14.0���͵���λ��
//    Test_Bluetooth();      //PASS,����UART0(P14.0RX/P14.1TX)��
//    Test_EEPROM();         //PASS,�����ڲ�EEPROM��д����  OLED��ʾ�Ƿ�д��ɹ�
//    Test_Vl53();           //PASS,����VL53  IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_9AX();            //PASS,����������� IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_MPU6050();        //PASS,����MPU6050����ICM20602 IIC����   P13_1��SCL  P13_2��SDA OLED��ʾԭʼ����
//    Test_ICM20602();       //PASS,����ICM20602 SPI����   P15_8��SCL  P15_5��SDA  P15_7��SA  P15_2��CS OLED��ʾԭʼ����
//    Test_CAMERA();         //PASS,����������������ͷ������Ļ����ʾ  LQ_CAMERA.h ��ѡ����Ļ
//    Test_SoftFft();        //PASS,����ILLD������FFT����
//    Test_FFT();            //PASS,����Ӳ��FFT  ע����ҪоƬ��׺��DA�Ĳ���Ӳ��FFT����
//    Test_RDA5807();        //PASS,����RDA5807������������SCL �� P00_1   SDA �� P00_2

    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ��м���ص�ѹ ADC�ɼ���ʼ��
    InductorInit();
    //ADC_InitConfig(ADC7, 8000);   // ��ʼ��   ���ʹ������ĸ��  ����ѹ��ĵ�ص�ѹ��������Կ�ĸ��ԭ��ͼ

    /* ������������������ʼ�� */
    MotorInit();    // ���
    ServoInit();    // ���
    EncInit();      //������
    PidInit(&pids,KP,KI,KD);
    PidInit(&pidspeed,KP2,KI2,KD2);

    // ������
   // TFTSPI_P16x16Str(0,3,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);// �ַ�����ʾ
   // CCU6_InitConfig(CCU60, CCU6_Channel0, 1000);//����ж�
   // STM_InitConfig(STM0, STM_Channel_0,10000);//����ж�
    STM_InitConfig(STM0, STM_Channel_1,5000);//����ж�STM_InitConfig(STM0, STM_Channel_0,100);
    STM_InitConfig(STM1, STM_Channel_0,2100);//����ж�
    //TFTSPI_P16x16Str(0,5,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);// �ַ�����ʾ
    // �����������Ժ���Ϊ��ѭ�����궨���������ͱ������õģ���������治�����У�
    // TestServo();  // ���Լ��궨�����TFT1.8���
    // TestMotor();  // ���Լ��궨�����TFT1.8���
    // TestEncoder();  // ���Ա�������������,TFT1.8��UART���
    //ServoCtrl(Servo_Center_Mid);
   // TFTSPI_P16x16Str(0,4,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);// �ַ�����ʾ
    while (1)   //��ѭ��
    {
        flag=KEY_Read(KEY0);
        if(flag==0)
        {

            while (1)
                {
                //float vbat;
                    //MotorCtrl(duty1);
                    LED_Ctrl(LED2,RVS);
                    yuansuchuli();
                   // uartsong();
                 }


        }
    }
}
