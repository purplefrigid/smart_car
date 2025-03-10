/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年10月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.2.2及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
=================================================================
摄像头接口                  龙邱神眼或者OV7725模块
● 数据端口：P02.0-P02.7口，共8位，接摄像头的数据端口；
● 时钟像素：外部中断第0组：P00_4；
● 场信号：外部中断第3组：P15_1；
-----------------------------------------------------------------
推荐GPT12模块，共可以实现5路正交解码增量编码器（兼容带方向编码器）信号采集，任意选择四路即可；
P33_7, P33_6   龙邱TC母板编码器1
P02_8, P33_5   龙邱TC母板编码器2
P10_3, P10_1   龙邱TC母板编码器3
P20_3, P20_0   龙邱TC母板编码器4
-----------------------------------------------------------------
电感电压采集模块或者麦克风模块
推荐使用AN0-7，共八路ADC，可以满足chirp声音信号及电磁车电感电压采集；
AN0-3          龙邱TC接四个麦克风模块或者电感
-----------------------------------------------------------------
默认电机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM0的0-7通道；
第一组双路全桥驱动
P23_1         龙邱TC母板MOTOR1_P
P32_4         龙邱TC母板MOTOR1_N
P21_2         龙邱TC母板MOTOR2_P
P22_3         龙邱TC母板MOTOR2_N
第二组双路全桥驱动
P21_4         龙邱TC母板MOTOR3_P
P21_3         龙邱TC母板MOTOR3_N
P20_8         龙邱TC母板MOTOR4_P
P21_5         龙邱TC母板MOTOR4_N
-----------------------------------------------------------------
默认舵机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM2；
P33_10        龙邱TC母板舵机1
P33_13        龙邱TC母板舵机2
-----------------------------------------------------------------
 默认屏幕显示接口，用户可以使用TFT或者OLED模块
TFTSPI_CS     P20_14     龙邱TC母板 CS管脚 默认拉低，可以不用
TFTSPI_SCK    P20_11     龙邱TC母板 SPI SCK管脚
TFTSPI_SDI    P20_10     龙邱TC母板 SPI MOSI管脚
TFTSPI_DC     P20_12     龙邱TC母板 D/C管脚
TFTSPI_RST    P20_13     龙邱TC母板 RESET管脚
-----------------------------------------------------------------
OLED_CK       P20_14     龙邱TC母板OLED CK管脚
OLED_DI       P20_11     龙邱TC母板OLED DI管脚
OLED_RST      P20_10     龙邱TC母板OLED RST管脚
OLED_DC       P20_12     龙邱TC母板OLED DC管脚
OLED_CS       P20_13     龙邱TC母板OLED CS管脚 默认拉低，可以不用
----------------------------------------------------------------
默认按键接口
KEY0p         P22_0      龙邱TC母板上按键0
KEY1p         P22_1      龙邱TC母板上按键1
KEY2p         P22_2      龙邱TC母板上按键2
DSW0p         P33_9      龙邱TC母板上拨码开关0
DSW1p         P33_11     龙邱TC母板上拨码开关1
-----------------------------------------------------------------
默认LED接口
LED0p         P10_6      龙邱TC母板核心板上LED0 翠绿
LED1p         P10_5      龙邱TC母板核心板上LED1 蓝灯
LED2p         P20_6      龙邱TC母板上LED0
LED3p         P20_7      龙邱TC母板上LED1
-----------------------------------------------------------------
默认蜂鸣器接口
BEEPp         P33_8      龙邱TC母板上蜂鸣器接口
-----------------------------------------------------------------
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h>//各个模块的头文件
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
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 初始化完成标志位

volatile char mutexCpu0TFTIsOk=0;         // CPU1 0占用/1释放 TFT
pid_param_t pids,pidspeed;  //pid定义
//#define Servo_Delta 170
#define Servo_Center_Mid 1270      //舵机直行中值，
#define Servo_Right_Max   (Servo_Center_Mid-240)      //舵机右转极限值
#define Servo_Left_Min (Servo_Center_Mid+230)      //舵机左转极限值，此值跟舵机放置方式有关，立式
//short duty1 = 2500;
//signed short duty2 = 1410;
float Error,Re_servo,Error_h,Error_s,Errorspeed=0,Error_hw;
float Expectspeed=1.5;//m/s
float nowspeed=-8000;
extern float  KP,KI,KD,KP2,KI2,KD2;
int Servo_duty,servoTurnDuty;
//unsigned char flag1=0xFF;
unsigned char *data;
int flag=0;       //发车按键
//extern float kp,ki,kd;
//extern int error_e,error_d,temp_duty;
//sint16 A=3,B=2,L=1.5;
//extern float Lleft1 , Lleft2 , Lright2 , Lright1 ;  // 电感偏移量
//sint16 Lleft1 = 0, Lleft2 = 0, Lright2 = 0, Lright1 = 0;  // 电感偏移量

/*************************************************************************
*  函数名称：int core0_main (void)
*  功能说明：CPU0主函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
int core0_main (void)
{
    //char txt[16];
   // char txt[32];
//    int flag1=0;
//    char txt1[32];
//    char txt2[32];

    // 关闭CPU总中断
    IfxCpu_disableInterrupts();

    // 关闭看门狗，如果不设置看门狗喂狗需要关闭
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    // 读取总线频率
    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

             // 清屏
    //TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);// 字符串显示

    // 按键初始化
    GPIO_KEY_Init();
    // LED灯所用P10.6和P10.5初始化
    GPIO_LED_Init();

    // 串口P14.0管脚输出,P14.1输入，波特率115200
    UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);

    // 开启CPU总中断
    IfxCpu_enableInterrupts();

    // 通知CPU1，CPU0初始化完成
    IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk=0;         // CPU1： 0占用/1释放 TFT

    // 程序配套视频地址：https://space.bilibili.com/95313236
    // 以下测试函数，内建死循环，用户可调用所用模块的初始化及读写函数来实现自己的任务
    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 //  Test_ADC();            //PASS,测试ADC采样时间  OLED上显示 ADC采样10K次时间
//    Test_ADC_7mic();       //PASS,测试ADC\UART0、STM延时和闪灯，通过UART0打印 AN0--AN7共8个通道ADC转换数据
//    LQ_Atom_Motor_8chPWM();//PASS,测试GTM_ATOM生成不同频率下的8路PWM
//    LQ_ATom_Servo_2chPWM();//PASS,测试GTM_ATOM、STM延时和闪灯，P33.10和P33.13作为ATOM输出口控制舵机
//    LQ_Tom_Servo_2chPWM(); //PASS,测试GTM_TOM、STM延时和闪灯，P33.10和P33.13作为TOM输出口控制舵机
//    Test_GPIO_Extern_Int();//PASS,测试外部第1组中断P15.8，P10.6和P10.5闪灯
//    Test_GPIO_LED();       //PASS,测试GPIO，P10.6和P10.5闪灯
//    Test_GPIO_KEY();       //PASS,测试外部按键输入，P22.0--2   按下按键 LED亮
//    Test_ComKEY_Tft();     //PASS,测试外部组合按键输入并TFT1.8显示，P22.0--2
//    LQ_GPT_4mini512();     //PASS,测试编码器正交解码,OLED和UART输出
//    LQ_GPT_4mini512TFT();  //PASS,测试编码器正交解码,TFT1.8和UART输出
//    Test_OLED();           //PASS,测试OLED0.96屏使用P20.14--10，显示字符串及动态数据
//    LQ_STM_Timer();        //PASS,测试定时中断、闪灯
  // Test_TFT18();          //PASS,测试TFT1.8彩屏使用P20.14--10，显示字符串及动态数据
//    LQ_TIM_InputCature();  //PASS,测试GTM_TOM_TIM、P20_9作为PWM输出口，P15_0作为TIM输入口，两者短接后，串口P14.0发送到上位机
//    Test_Bluetooth();      //PASS,测试UART0(P14.0RX/P14.1TX)，
//    Test_EEPROM();         //PASS,测试内部EEPROM擦写功能  OLED提示是否写入成功
//    Test_Vl53();           //PASS,测试VL53  IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_9AX();            //PASS,测试龙邱九轴 IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_MPU6050();        //PASS,测试MPU6050或者ICM20602 IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
//    Test_ICM20602();       //PASS,测试ICM20602 SPI接线   P15_8接SCL  P15_5接SDA  P15_7接SA  P15_2接CS OLED显示原始数据
//    Test_CAMERA();         //PASS,测试龙邱神眼摄像头并在屏幕上显示  LQ_CAMERA.h 中选择屏幕
//    Test_SoftFft();        //PASS,测试ILLD库的软件FFT函数
//    Test_FFT();            //PASS,测试硬件FFT  注意需要芯片后缀带DA的才有硬件FFT功能
//    Test_RDA5807();        //PASS,测试RDA5807立体声收音机SCL 接 P00_1   SDA 接 P00_2

    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 电感及电池电压 ADC采集初始化
    InductorInit();
    //ADC_InitConfig(ADC7, 8000);   // 初始化   如果使用龙邱母板  则测分压后的电池电压，具体可以看母板原理图

    /* 电机、舵机，编码器初始化 */
    MotorInit();    // 电机
    ServoInit();    // 舵机
    EncInit();      //编码器
    PidInit(&pids,KP,KI,KD);
    PidInit(&pidspeed,KP2,KI2,KD2);

    // 编码器
   // TFTSPI_P16x16Str(0,3,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);// 字符串显示
   // CCU6_InitConfig(CCU60, CCU6_Channel0, 1000);//电机中断
   // STM_InitConfig(STM0, STM_Channel_0,10000);//电机中断
    STM_InitConfig(STM0, STM_Channel_1,5000);//舵机中断STM_InitConfig(STM0, STM_Channel_0,100);
    STM_InitConfig(STM1, STM_Channel_0,2100);//电磁中断
    //TFTSPI_P16x16Str(0,5,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);// 字符串显示
    // 以下三个测试函数为死循环，标定舵机、电机和编码器用的，开启后后面不会运行！
    // TestServo();  // 测试及标定舵机，TFT1.8输出
    // TestMotor();  // 测试及标定电机，TFT1.8输出
    // TestEncoder();  // 测试编码器正交解码,TFT1.8和UART输出
    //ServoCtrl(Servo_Center_Mid);
   // TFTSPI_P16x16Str(0,4,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);// 字符串显示
    while (1)   //主循环
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
