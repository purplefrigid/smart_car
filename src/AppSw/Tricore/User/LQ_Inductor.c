/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技TC264DA核心板
 【编    写】chiusir
 【E-mail】chiusir@163.com
 【软件版本】V1.1 版权所有，单位使用请先联系授权
 【最后更新】2020年10月28日
 【相关信息参考下列地址】
 【网    站】http:// www.lqist.cn
 【淘宝店铺】http:// longqiu.taobao.com
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
 程序配套视频地址：https:// space.bilibili.com/95313236
 =================================================================
 使用说明：
 本教学演示程序适用于电磁四轮或者三轮车：
 整车资源为：
 模块：龙邱TC264DA核心板、配套母板、双路全桥电机驱动、双编码器512带方向、TFT1.8屏幕、单舵机、四路电感模块；
 车模：三轮或者四轮均可；
 电感分布：
 ||----------左------------------------------------右--------------||
 ||---------侧--------------------------------------侧-------------||
 ||--------第----------------------------------------第------------||
 ||-------1----左侧第2个电感 -------------右侧第2个电感 -----1-----------||
 ||------个--------------------------------------------个----------||
 ||-----电----------------------------------------------电---------||
 ||----感------------------------------------------------感--------||
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <IfxCpu.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <LQ_UART.h>
#include <stdio.h>
#include <stdlib.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_MotorServo.h>
#include <LQ_GPIO_LED.h>
#include <LQ_Inductor.h>
#include <LQ_GPT12_ENC.h>
#include<ANO_DT.h>
#include <LQ_PID.h>
#include <LQ_GPT12_ENC.h>
extern int flag1;
 extern signed short cont1;

#define fork 1     //三岔路先打角的方向    0：先往右打      1：先往左打
extern pid_param_t pids;  //pid定义
sint16 TempAngle = 0;        // 根据电感偏移量计算出的临时打角值
sint16 LastAngle = 0;        // 记忆冲出赛道前的有效偏移方向
int circle_judge,circle_point,circle_out,circle_r,circle_l;//大圆环标志位
int circle_flagpwm;
int circle_judge_s=0,circle_point_s=0,circle_out_s=0,circle_r_s=0,circle_l_s=0,circle_k;//小圆环标志位
int circlenumber;   //环岛数量检测
int circle_flag=1;   //环岛先后标志  0:先入小环岛   1：先入大环岛
int fork1,fork2,angle1,fork_r_first,fork_l_first,fork_flag;
int lostline;   //丢线处理标志
int cross;      //十字路口标志
//int san_speed;   //三岔路速度
int t_flag;    //tft显示标志
extern volatile  float encValue1 ;
extern volatile float lowspeed;
//signed short value2;
float LnowADC[8],ADC_G[6];           // 电感当前ADC数值
float Lleft1 = 0, Lleft2 = 0,Lleft3 = 0,Lleft4 = 0;
float Lright1 = 0,Lright2 = 0,Lright3 = 0,Lright4 = 0,w1=0.2,w2=8;  // 电感偏移量
float ad_max[8] = {3800, 3800, 3800, 3800, 3800, 3800, 3800, 3800}; // 新板子放到赛道标定最大值,会被程序刷新
float ad_min[8] = {0, 0, 0, 0, 0, 0,0,0}; // 新板子据需要标定最小值,会被程序刷新

extern float nowspeed,Expectspeed,Errorspeed;
uint8 CircleNumber = 1;    // 入环次数，0结束；默认1次 ;环的个数一般在比赛前测试赛道时就知道了
uint8 TangentPoint = 1;    // 切点判断   0切点结束；默认1可以入环，读取脉冲为入环准备
uint8 EnterCircle = 0;     // 允许进环  默认 0不可进环；1可以进环
uint8 OutCircle = 0;       // 允许出环   默认0不可出环；1可以出环
uint8 LeftRightCircle = 2; // 左侧环还是右侧环 默认0原始；1左环；2右环
sint16 A=1.0f,B=1.0f,L=1.0f;
int error_e,error_d,temp_duty=0;
extern float KP,KI,KD,KP2,KI2,KD2;
extern pid_param_t pids,pidspeed;

sint32 TangentPointpulse = 0; // 脉冲记忆临时变量1
sint32 EnterCirclePulse = 0;  // 脉冲记忆临时变量2
sint32 OutCirclePulse =0;    // 脉冲记忆临时变量3
sint32 EnterCircleOKPulse = 0;// 脉冲记忆临时变量4
sint16 LowSpeed = 0;          // 圆环/十字口减速

uint16 MagneticField = 0;     // 磁场强度    magnetic field intensity,判断圆环是否出现

sint16 OffsetDelta = 0;

/*************************************************************************
 *  函数名称：void InductorInit (void)
 *  功能说明：四个电感ADC初始化函数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void InductorInit (void)
{
    ADC_InitConfig(ADC0, 250000); // 初始化,采样率100kHz
    //ADC_InitConfig(ADC1, 200000); // 初始化
    ADC_InitConfig(ADC2, 250000); // 初始化
    ADC_InitConfig(ADC3, 250000); // 初始化
    ADC_InitConfig(ADC4, 250000); // 初始化,采样率100kHz
    ADC_InitConfig(ADC5, 250000); // 初始化
    //ADC_InitConfig(ADC6, 200000); // 初始化
    ADC_InitConfig(ADC8, 250000); // 初始化
}
/*************************************************************************
 *  函数名称：void InductorNormal(void)
 *  功能说明：采集电感电压并归一化；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：   注意要先标定运放的放大倍数，尽量四个一致或者接近
 *************************************************************************/
void InductorNormal (void)
{
    int i=0,j;
    LnowADC[0] =(float) (ADC_ReadAverage(ADC0,5));  // 右侧第1个电感，竖直垂直赛道，采集各个电感的AD值Ad_Value(ADC0,7)
    //LnowADC[1] =(float) (ADC_ReadAverage(ADC1,5));  // 右侧第2个电感，与赛道夹角约30度，kalman_filter(ADC1)
    LnowADC[1] =(float) (ADC_ReadAverage(ADC2,5));  // 右侧第3个电感，与赛道水平，(ADC_ReadAverage(ADC2,10)-104)
    LnowADC[2] =(float) (ADC_ReadAverage(ADC3,5));  // 右侧第4个电感，垂直赛道
    LnowADC[3] =(float) (ADC_ReadAverage(ADC4,5));  // 左侧第4个电感，垂直赛道Ad_Value(ADC0,3)
    LnowADC[4] =(float) (ADC_ReadAverage(ADC5,5));  // 左侧第3个电感，与赛道水平，kalman_filter(ADC1)
    //LnowADC[6] =(float) (ADC_ReadAverage(ADC6,5));  // 左侧第2个电感，与赛道夹角约30度，(ADC_ReadAverage(ADC2,5)-104)
    LnowADC[5] =(float) (ADC_ReadAverage(ADC8,5));
    for(i=0;i<6;i++)
    {
        if (LnowADC[i] < ad_min[i])
               ad_min[i] = LnowADC[i];     // 刷新最小值
        else if (LnowADC[i] > ad_max[i])
               ad_max[i] = LnowADC[i];     // 刷新最大值
    }
    for(j=0;j<6;j++)// 各偏移量归一化到0--1000以内
    {
        ADC_G[j]=(LnowADC[j] - ad_min[j]) * 1000 / (ad_max[j] - ad_min[j]);
        if(ADC_G[j]<3)
            ADC_G[j]=3;
    }
    Lright1 = ADC_G[0];
    Lright3 = ADC_G[1];
    Lright4 = ADC_G[2];
    Lleft4 = ADC_G[3];
    Lleft3 = ADC_G[4];
    Lleft1 = ADC_G[5];
//    if(Lright3>200)
//        Lright3=200;   //限幅
//    if(Lleft3>200)
//        Lleft3=200;    //限幅

//    MagneticField = Lleft1 + Lleft2 + Lright2 + Lright1;// 磁场整体强度
//
//    if(Lleft2 + Lright2<36)      // 两个值都小于20，电磁杆远离赛道，进入直角弯和急弯
//    {
//        if (Lleft1 + Lleft2 > Lright2 + Lright1)
//        {
//            LastAngle=TempAngle = Servo_Left_Min;
//        }
//        else
//        {
//            LastAngle =TempAngle =Servo_Right_Max ;
//        }
//        LastAngle =TempAngle;// 记忆有效参数，记忆偏移方向
//    }
//    else if ((Lleft2 > 18) && (Lright2 > 18))   // 小车行走于赛道上中间
//    {
//        TempAngle = Servo_Center_Mid - (Lleft2 - Lright2) *3; //  根据偏移量差值小幅度打角，防止直道摇摆
//    }
//    else                                        //  小车行走于赛道上弯道区域，一大一小，需要较大程度控制转向
//    {
//        TempAngle = Servo_Center_Mid - (Lleft2 - Lright2) * 9; //  一般弯道转向控制，数值越大，转向越早
//    }
}
void lostline_judge(void)
{
    float i,j;
    int lostline_flag;
    i=Lright4+Lright3+Lright1;         //右边三个电感值之和
    j=Lleft4+Lleft3+Lleft1;            //左边三个电感值之和
    if((i<40)||(j<40))    lostline_flag=1;
    else         lostline_flag=0;
    if(lostline_flag==1)
    {
        if(Lleft1<5&&Lright1<5&&Lleft3<5&&Lright3<5&&Lleft4<5&&Lright4<5)     //不丢线
            lostline=0;       //不丢线,停车
        else if(i>j)    //右边电感值大于左边电感值，舵机向右边打
            lostline=1;       //左丢线
        else if(i<j)    //左边电感值大于右边电感值，舵机向左边打
            lostline=2;       //右丢线
    }
    else if(lostline_flag==0)
    {
        lostline=0;    //不丢线
    }
}
void lostline_deal()
{
    lostline_judge();     //丢线判断
    if(lostline!=0)   //丢线
          {
              if(lostline==1)  //右边电感值大于左边电感值，舵机向右边打，左丢线
              {
                  Error=-40;     //左丢线处理
              }
              else if(lostline==2)  //左边电感值大于右边电感值，舵机向左边打，右丢线
              {
                  Error=40;     //右丢线处理
              }
          }
    else if(lostline==0)     //冲出赛道判断
         {
              if(Lleft1<5&&Lright1<5&&Lleft3<5&&Lright3<5&&Lleft4<5&&Lright4<5)   //冲出赛道停车保护函数
              {
                  Expectspeed=0;
                  lowspeed=-8000;
                  fork_flag=0;
              }
              else     //非特殊元素处理
              {
                  if(fork_flag==1)        //三岔路口减速
                  {
                      Expectspeed=0.7;
                      lowspeed=-8000;
                  }
                  else
                  {
                      Expectspeed=1.5;
                      lowspeed=-8000;
                  }
              }
         }

}
void circle_big(void)
{
    float i;
    i=Lleft4+Lright4;
    if((Lleft4>200&&Lright4>200)&&((Lright3>30)&&(Lleft3>30)&&(80>Lright3)&&(80>Lleft3)))  //环岛减速标志位
       {
          circle_judge=1;
          //circle_flagpwm=1;

       }
    if((Lleft4>500||Lright4>500)||(Lleft4>380&&Lright4>380))                      //环岛交界处标志处理
        {

                if(circle_point==0)
                   {
                    circle_point=1;
                   }
                 if(circle_point==1&&circle_r==1)
                {
                    circle_point=2;
                    circle_r=2;
                }
                if(circle_point==1&&circle_l==1)
                {
                    circle_point=2;
                    circle_l=2;
                    circlenumber=2;
                }
          }
     if((Lleft4>210&&Lright4>210)&&(Lright3<150&&Lright3>90&&Lleft3<12))//右入环岛打角位
           {

               if(circle_point==1&&circle_l==0)
               {
                   circle_r=1;
               //circle_flagpwm=0;
               }
           }
     if((Lleft4>210&&Lright4>210)&&(Lleft3<120&&Lleft3>90&&Lright3<6))//左入环岛打角位
           {
               if(circle_point==1&&circle_r==0)
                   circle_l=1;
           }

}
void circle_small(void)
{
    float i;
    int j;
    i=Lleft4+Lright4;
    j=abs(Lleft1-Lright1);
        if(i>400)    //环岛减速标志位
        {
              circle_judge_s=1;
              if(circle_k==0)
                EnterCirclePulse=RAllPulse;
              circle_k=1;

        }
        if((i>440)||j>270)                      //环岛交界处标志处理
            {
                 if(circle_r_s==1||circle_l_s==1)
                     circle_point_s=1;
            }
        if((RAllPulse>(EnterCirclePulse+6000))&&LeftRightCircle==2&&circle_judge_s==1)  //右入环岛打角位
           {
                  if(circle_r_s==0)
                    circle_r_s=1;
                  //flag1=0;
           }
        if(RAllPulse>(EnterCirclePulse+5500)&&LeftRightCircle==1&&circle_judge_s==1)  //左入环岛打角位
           {
                  if(circle_l_s==0)
                    circle_l_s=1;
           }
//        if(i>260&&i<380&&(Lright3<160&&Lright3>80)&&Lleft3<15)//右入环岛打角位
//                      {
//                          if(circle_point_s==1&&circle_r_s==0)
//                              circle_r_s=1;
//                      }
//        if(i>280&&i<400&&(Lleft3>80&&Lleft3<130)&&Lright3<10)//左入环岛打角位
//                     {
//                          if(circle_point_s==1&&circle_r_s==0)
//                             circle_l_s=1;
//                     }

}
void sancha(void)
{
    float j,k;
    int i;
    i=abs(Lright1-Lleft1);         //两个外横电感值之差
    j=Lright3+Lleft3;         //两个竖电感值之和
    k=Lright4+Lleft4;         //两个横电感值之和
    if((Lleft4>90&&Lright4>90)&&(Lleft4<140&&Lright4<140))   //三岔路口检测减速位
        fork_flag=1;
    if((Lleft4>20&&Lright4>20)&&(Lleft4<100&&Lright4<100)&&(Lleft3>7&&Lright3>7&&Lleft3<30&&Lright3<30))
    {
        if(j>20&&angle1==0)
                {
                    fork2=1;    //三岔路恒定转向打开标志
                    angle1=1;   //进入三岔路标志
                }
        if(angle1==1)
        {
            switch(fork)          //打角方向判断
                    {
                        case 0:         //先右转，再左转
                        {
                            if(fork_r_first==0)   //第一次进三岔路
                                fork_r_first=1;
                            else if(fork1==1)          //第二次进三岔路
                                fork_r_first=2;
                        }
                        break;
                        case 1:         //先左转，再右转
                        {
                            if(fork_l_first==0)  //第一次进三岔路
                                fork_l_first=1;
                            else if(fork1==1)         //第二次进三岔路
                                fork_l_first=2;
                        }
                        break;
                    }
        }

    }
   if(angle1==1&&k>50)
          fork2=0;    //三岔路恒定转向关闭标志
   if(j>80&&j<120&&angle1==1)
        {
            angle1=2;    //出三岔路标志
        }

}
void yuansuchuli(void)
{
    sancha();
    //circle_big();
    switch(circle_flag)
    {
        case 0:   //先进小环岛
        {
            circle_small();
            if( circlenumber==1)
               circle_big();
        }
        break;
        case 1:   //先进大环岛
        {
            circle_big();
            if( circlenumber==2)
                circle_small();
        }
        break;
    }
    //circle_small();         //调用环岛处理函数
    //circle_big();
    Cross();          //十字路口处理

}
void circle_bigdeal(void)
{
//    if(circle_flagpwm==1)
//    {
//        Error=0;
//    }
    if((circle_r==1)||(circle_l==1))      //入环岛参数处理
           {
//               if(((Lleft4>150&&Lleft4<200)||(Lright4<200&&Lright4>150))&&(Lright3>30||Lleft3>30))      //进入环岛后循迹误差
//                   Error=Error_h;//+0.4f*Error_s;
//               else
                   Error=(sqrt(Lleft3)-sqrt(Lright3))/(Lleft3+Lright3)*300.0f;    //打角入环岛循迹误差
           }
     else if(circle_point==2)     //出环岛标志
        {
            Error=Error_h;
            if(circle_out==1)
            {
                circle_point=0;
                circle_out=0;
                circle_r=0;
                circle_l=0;
                circle_judge=0;
            }

        }
     else
         Error=Error_h;//+0.4f*Error_s;
}
void circle_smalldeal(void)
{
    if(((circle_r_s==1)||(circle_l_s==1))&&(circle_point_s!=1))      //入环岛参数处理
           {
               if(circle_r_s==1)      //进入环岛后循迹误差
                   Error=-40;
               else if(circle_l_s==1)
                   Error=40;
               else
                   Error=(sqrt(Lleft3)-sqrt(Lright3))/(Lleft3+Lright3)*400.0f;    //打角入环岛循迹误差
           }
    else if((circle_judge_s==1)&&(circle_r_s!=1)&&LeftRightCircle==2)   //右出环
        Error=0;
    else if((circle_judge_s==1)&&(circle_l_s!=1)&&LeftRightCircle==1)   //右出环
        Error=0;
    else if(circle_point_s==1)     //出环岛标志
        {
            Error=Error_h;
            if(circle_out_s==1)
            {
                circle_point_s=0;
                circle_out_s=0;
                circle_r_s=0;
                circle_l_s=0;
                circle_judge_s=0;
            }

        }
     else
         Error=Error_h+0.3f*Error_s;
}
void poserror()
{
    //int hd;       //进入环岛标志
    Error_h=(sqrt(Lleft4)-sqrt(Lright4))/(Lleft4+Lright4)*1000.0f;    //两个内横电感的差比和值
    Error_s=(sqrt(Lleft3)-sqrt(Lright3))/((Lleft3)+(Lright3))*200.0f;     //两个竖电感的差比和值
    Error_hw=(sqrt(Lleft1)-sqrt(Lright1))/(Lleft1+Lright1)*1000.0f;   //两个外横电感的差比和值
    if(circle_judge_s==1)
    {
        circle_smalldeal();     //小圆环处理
        LED_Ctrl(LED2,RVS);
    }
    else if(circle_judge==1)   //大圆环处理
    {
        circle_bigdeal();
    }
    else if((fork_r_first!=0||fork_l_first!=0)&&((fork2==1)||(angle1==2)))   //三岔路循迹处理
    {
        switch(fork)          //打角方向判断
               {
                   case 0:         //先右转，再左转
                   {
                       if(fork_r_first==1)
                           Error=-40;
                       else if(fork_r_first==2)
                           Error=40;
                   }
                   break;
                   case 1:         //先左转，再右转
                   {
                       if(fork_l_first==1)
                           Error=40;
                       else if(fork_l_first==2)
                           Error=-40;
                   }
                   break;
               }
    }
    else if(cross==1)   //十字路口处理
        Error=Error_h;
    else
        Error=Error_h+0.5f*Error_s;    //直道和弯道处理
    if(fork2==0)
       lostline_deal();    //调用丢线函数
    PIDS(Error);      //误差处理
}
void Cross(void)
{
    if(Lleft3>190&&Lright3>190)   //十字路口处理
         cross=1;
}
void PIDS(float error)
{
    //float w_flag;
//   int i;
//    i=abs(Lright3-Lleft3);
//    if(i>90)
//        w_flag=1;
    float i;
    i=Lleft4+Lright4+Lright3+Lleft3;
    if((i<300&&i>150)&&Lright3<8&&Lleft3<8)                      //直道处理
    {
            KP=4;
            KI=0;
            KD=0.1;
            if(circle_judge==1)  circle_out=1;
            if(circle_judge_s==1)  circle_out_s=1;
            cross=0;
            fork_flag=0;
            if((fork_r_first==1||fork_l_first==1)&&angle1==2)
            {
                fork1=1;           //从三岔路到直道标志
                angle1=0;          //出三岔路标志清零
            }
            else if((fork_r_first==2||fork_l_first==2)&&angle1==2)
            {
                fork_r_first=0;
                fork_l_first=0;
                fork1=0;           //从三岔路到直道标志
                angle1=0;          //出三岔路标志清零
            }
    }
//    else if(fork4==1)      //三岔路处理
//    {
//
//    }
    else if((Lright3>8||Lleft3>8)&&(circle_r!=1||circle_l!=1))       //弯道处理
    {
        if((Lright3>8||Lleft3>8)&&((Lright3<70&&Lleft3<70)))
            //小弯道处理
        {
            KP=(float)(4+w1*abs(error));
            //Expectspeed=1.7;
            KD=0.03;
        }
        else      //大弯道处理
        {
            KP=(float)(15+w2*abs(error));
            //Expectspeed=1;
            KI=0;
            KD=0.1;
        }
    }
    else if(circle_r==1||circle_l==1)      //环岛处理
    {
        KP=6;
        KD=0.1;
    }
    if(KP>30)
        KP=30.0f;
    PidInit(&pids,KP,KI,KD);
}

void uartsong(void)
{
    int a,b,c,d,e,f,g;
         a=(int)(nowspeed*1000);
         b=1000;
         c=(int)circle_out;
         d=(int)LnowADC[5];
         e=(int)Error;
         f=(int)Servo_duty;
         g=(int)(Lleft4+Lright4);
         ANO_DT_send_int16(0,a,b,c,d,e,f,0);
}
void tftxian(void)
{

    char txt[16];
    //TFTSPI_CLS(u16BLUE);
    if(circle_judge==1)
        t_flag=1;
    else if(angle1==1)
        t_flag=2;
    else if(circle_judge_s==1)
        t_flag=3;
    switch(t_flag)
    {
        case 0:   //普通电磁显示
        {
            //TFTSPI_CLS(u16BLUE);
            sprintf(txt, "Lright3: %f;",Lright3);
               TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);       //字符串显示

               sprintf(txt, "Lleft1: %f;",Lleft1);
               TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);       //字符串显示
                                           //UART_PutStr(UART0, txt);
               error_d=(int)Re_servo;
               sprintf(txt, "Lleft3: %f;",Lleft3);
               TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

               sprintf(txt, "Lleft4: %f;",Lleft4);
               TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);

               temp_duty= Servo_duty*100/1900;
               sprintf(txt, "lost: %d;",lostline);
               TFTSPI_P8X16Str(0, 9, txt, u16WHITE, u16BLACK);

               sprintf(txt, "error_d: %d;",error_d );        //encValue1
               TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);
//               sprintf(txt, "error_d: %d;",error_d );        //encValue1
//               TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);
               error_e =(int)Error;

               sprintf(txt, "error_e: %05d;", error_e );
               TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);


               sprintf(txt, "Lright4: %f;",  Lright4);
               TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);

               sprintf(txt, "Lright1: %f;",Lright1);
               TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);
        }
        break;
        case 1:   //环岛元素显示
        {
            //TFTSPI_CLS(u16BLUE);
            sprintf(txt, "Lright3: %f;",Lright3);
               TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);       //字符串显示

               sprintf(txt, "circle_r: %d;",circle_r);
               TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);       //字符串显示
                                           //UART_PutStr(UART0, txt);
               error_d=(int)Re_servo;
               sprintf(txt, "Lleft3: %f;",Lleft3);
               TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

               sprintf(txt, "Lleft4: %f;",Lleft4);
               TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);

               temp_duty= Servo_duty*100/1900;
               sprintf(txt, "circle_out: %d;",circle_out);
               TFTSPI_P8X16Str(0, 9, txt, u16WHITE, u16BLACK);

               sprintf(txt, "error_d: %d;",error_d );        //encValue1
               TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);
               error_e =(int)Error;

               sprintf(txt, "error_e: %05d;", error_e );
               TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);


               sprintf(txt, "Lright4: %f;",  Lright4);
               TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);

               sprintf(txt, "circle_judge: %d;",circle_judge);
               TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);

        }
        break;
        case 2:
        {
           // TFTSPI_CLS(u16BLUE);
            sprintf(txt, "Lright3: %f;",Lright3);
            TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);       //字符串显示

            sprintf(txt, "angle1: %d;",angle1);
            TFTSPI_P8X16Str(0, 9, txt, u16WHITE, u16BLACK);       //字符串显示
                                                       //UART_PutStr(UART0, txt);
            error_d=(int)Re_servo;
            sprintf(txt, "Lleft3: %f;",Lleft3);
            TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

           sprintf(txt, "Lleft4: %f;",Lleft4);
           TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);

           temp_duty= Servo_duty*100/1900;
           sprintf(txt, "fork2: %d;",fork2);
           TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);

           sprintf(txt, "Lleft1: %f;",Lleft1 );        //encValue1
           TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);
           error_e =(int)Error;

           sprintf(txt, "Lright1: %f;", Lright1);
           TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);


           sprintf(txt, "Lright4: %f;",  Lright4);
           TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);

           sprintf(txt, "fork1: %d;",fork1);
           TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);
        }
        break;
        case 3:   //环岛元素显示
                {
                   // TFTSPI_CLS(u16BLUE);
                    sprintf(txt, "Lright3: %f;",Lright3);
                       TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);       //字符串显示

                       sprintf(txt, "circle_r_s: %d;",circle_r_s);
                       TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);       //字符串显示
                                                   //UART_PutStr(UART0, txt);
                       error_d=(int)Re_servo;
                       sprintf(txt, "Lleft3: %f;",Lleft3);
                       TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

                       sprintf(txt, "Lleft4: %f;",Lleft4);
                       TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);

                       temp_duty= Servo_duty*100/1900;
                       sprintf(txt, "RAllPulse: %d;",RAllPulse);
                       TFTSPI_P8X16Str(0, 9, txt, u16WHITE, u16BLACK);

                       sprintf(txt, "circle_r_s: %d;",circle_l_s );        //encValue1
                       TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);
                       error_e =(int)Error;

                       sprintf(txt, "circle_p_s: %d;", circle_point_s );
                       TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);


                       sprintf(txt, "Lright4: %f;",  Lright4);
                       TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);

                       sprintf(txt, "circle_ju_s: %d;",circle_judge_s);
                       TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);

                }
                break;
    }

}
void ADCxian(void)
{
    char txt[16];
    //TFTSPI_CLS(u16BLUE);
    sprintf(txt, "KP: %f;",KP);
    TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "KD: %f;",KD);
    TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "KI: %f;",KI);
    TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "w1: %f;",w1);
    TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "w2: %f;",w2);
    TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "ADC5: %f;",LnowADC[5]);
    TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "ADC6: %f;",LnowADC[6]);
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);       //字符串显示

    sprintf(txt, "ADC8: %f;",LnowADC[7]);                  //(int)Lleft1,
    TFTSPI_P8X16Str(0, 8, txt, u16WHITE, u16BLACK);       //字符串显示
}
/*************************************************************************
 *  函数名称：void CircleDetect void
 *  功能说明：识别并进入圆环的个数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void CircleDetect (void)
{
    if (CircleNumber) // 入环次数，0结束；默认2次
    {
        // 进环切点区域判断
        if (MagneticField > 200) // 直道进入，此值不宜太大，容易丢失切点
        {
            if (TangentPoint)
            {
                TangentPointpulse = RAllPulse; // 读取当前脉冲数值
                TangentPoint = 0;             // 禁止再次读取当前脉冲数值
            }
            else if (LeftRightCircle == 0)
            {                                 // 如果前面设置为220，此处应该大点儿，距离拉长一些，太大就走过切点了也不行！
                if (RAllPulse > TangentPointpulse + 2000)   // 进入切点后再前进3000脉冲，大约50cm，
                {
                    EnterCircle = 1;      // 通过切点区域，可以入环
                }
            }
        }

        // 。约1.2米外进环无效，则需要重新识别切点
        if ((LeftRightCircle == 0) && (RAllPulse > TangentPointpulse + 8000)) // 约1.2米外进环无效
        {
            EnterCircle = 0;   // 约1.2米外进环无效
            TangentPoint = 1;  // 重新识别切点
        }
        if ((EnterCircle) && (MagneticField > 240)) // 约1.2米内再次发现强磁场，入环
        {
            LowSpeed = 500;    // 减速
            // 。左侧入环处理
            if (Lleft1 + Lleft2 > Lright2 + Lright1)     // 左边入环，存在误判！！！
            {
                LeftRightCircle = 1;  // 左侧环为1
                EnterCircle = 0;      // 入环后禁止再次入环
                EnterCirclePulse = RAllPulse;
                ServoCtrl(Servo_Left_Min);     // 舵机PWM输出，转向舵机打角控制
                while (RAllPulse < EnterCirclePulse + 2800)
                {
                    delayms(1);       // 半打角度前进1200脉冲，约20cm，龙邱512带方向编码器1米5790个脉冲
                }
            }
            else // 右侧入环处理
            {
                LeftRightCircle = 2;  // 右侧环为2
                EnterCircle = 0;  // 入环后禁止再次入环
                EnterCirclePulse = RAllPulse;
                ServoCtrl(Servo_Right_Max);        // 舵机PWM输出，转向舵机打角控制
                while (RAllPulse < EnterCirclePulse + 1800) // 用的是右侧的编码器，实际走的距离近一点儿
                {
                    delayms(1);   // 半打角度前进1200脉冲，约20cm，龙邱512带方向编码器1米5790个脉冲
                }
            }
            EnterCircleOKPulse = RAllPulse; // 出环用
        }
        // 出环处理
        if ((LeftRightCircle > 0) && (RAllPulse > EnterCircleOKPulse + 3000))
        {
            EnterCircleOKPulse = 10000000; //禁止再次出环使能
            OutCircle = 1;  // 进环后可以出环
        }
        if ((OutCircle) && (MagneticField > 220)) // 入环标志为真才能入环
        {
            LowSpeed = 500;        // 减速
            // 左侧出环处理
            if (LeftRightCircle == 1)   //左边入环
            {
                //OutCircle = 0;    // 入环后禁止再次入环
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Left_Min);     // 舵机PWM输出，转向舵机打角控制
                while (RAllPulse < OutCirclePulse + 2800)
                {
                    delayms(1); // 半打角度前进1200脉冲，约20cm，龙邱512带方向编码器1米5790个脉冲
                }
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid-Servo_Delta*2/3);     // 舵机PWM输出，反向打角
                while (RAllPulse < OutCirclePulse + 700)
                {
                    delayms(1);         // 半打角度前进600脉冲，约10cm，龙邱512带方向编码器1米5790个脉冲
                }
                CircleNumber--;         // 环计数
                TangentPoint = 1;       // 切点判断   0切点结束；默认1可以入环，读取脉冲为入环准备
                EnterCircle = 0;        // 允许进环  默认 0不可进环；1可以进环
                OutCircle = 0;          // 允许出环   默认0不可出环；1可以出环
                LeftRightCircle = 0;    // 左侧环还是右侧环 默认0原始；1左环；2右环
                LowSpeed = 0;           // 恢复速度
                //Reed_Init();            // 干簧管GPIO及中断初始化函数,为停车入库做准备
            }
            // 右侧出环处理
            else if (LeftRightCircle == 2)  //右边入环
            {
                //OutCircle = 0;     // 入环后禁止再次入环
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Right_Max);     // 舵机PWM输出，转向舵机打角控制
                while (RAllPulse < OutCirclePulse + 2500)
                {
                    delayms(1);   // 半打角度前进1200脉冲，约20cm，龙邱512带方向编码器1米5790个脉冲
                }
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Center_Mid+Servo_Delta*2/3);     // 舵机PWM输出，反向打角
                while (RAllPulse < OutCirclePulse + 1400)
                {
                    delayms(1);         // 半打角度前进600脉冲，约10cm，龙邱512带方向编码器1米5790个脉冲
                }
                CircleNumber--;         // 环计数
                TangentPoint = 1;       // 切点判断   0切点结束；默认1可以入环，读取脉冲为入环准备
                EnterCircle = 0;        // 允许进环  默认 0不可进环；1可以进环
                OutCircle = 0;          // 允许出环   默认0不可出环；1可以出环
                LeftRightCircle = 0;    // 左侧环还是右侧环 默认0原始；1左环；2右环
                LowSpeed = 0;           // 恢复速度
                Reed_Init();            // 干簧管GPIO及中断初始化函数,为停车入库做准备
            }
        }
    }
}

/*************************************************************************
 *  函数名称：void TFT_Show_EleMag_Info(void)
 *  功能说明：显示各种所需信息
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void TFT_Show_EleMag_Info(void)
{
    char txt[16] = "X:";

    sint16 mps = 0, dmm = 0;    // 速度：m/s,毫米数值
    sint16 pulse100 = 0;
    uint16 bat=0;

    dmm = (sint16) (RAllPulse * 100 / 579);           // 龙邱512带方向编码器1米5790个脉冲，数值太大，除以100
    pulse100 = (sint16) (RAllPulse / 100);
    sprintf(txt, "AP:%05d00", pulse100);              //
    TFTSPI_P8X16Str(3, 1, txt, u16RED, u16BLACK);     // 显示赛道偏差参数

    NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
    mps = (sint16) (dmm / (NowTime / 1000));          // 计算速度mm/s
    // 调试信息
    sprintf(txt, "%04d %04d %04d ", TempAngle, ECPULSE1, ECPULSE2);   // 显示舵机角度数值，电机占空比数值，编码器数值
    TFTSPI_P8X16Str(1, 0, txt, u16WHITE, u16BLACK);      // 字符串显示
    //显示各电感归一化后的偏移量  当前各电感电压值 各电感开机后历史最小值 各电感开机后历史最大值
//    sprintf(txt, "0:%04d %04d %04d ", Lleft1, LnowADC[0], ad_max[0]);
//    TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);
//    sprintf(txt, "1:%04d %04d %04d ", Lleft2, LnowADC[1], ad_max[1]);
//    TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);
//    sprintf(txt, "2:%04d %04d %04d ", Lright2, LnowADC[4], ad_max[4]);
//    TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);
//    sprintf(txt, "3:%04d %04d %04d ", Lright1, LnowADC[5], ad_max[5]);
//    TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

    sprintf(txt, "Ring num: %d ", CircleNumber);
    TFTSPI_P8X16Str(2, 6, txt, u16GREEN, u16BLACK);
    sprintf(txt, "M:%03d Q:%d J:%d ", MagneticField, TangentPoint, EnterCircle);
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);
    if (LeftRightCircle == 1)
        TFTSPI_P8X16Str(0, 8, "Left Ring ", u16WHITE, u16BLACK);
    else if (LeftRightCircle == 2)
        TFTSPI_P8X16Str(0, 8, "Right Ring", u16WHITE, u16BLACK);
    else
        TFTSPI_P8X16Str(0, 8, "No Ring   ", u16WHITE, u16BLACK);

    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, mps / 1000, (mps / 10) % 100);  // *3.3/4095*3
    TFTSPI_P8X16Str(0, 9, txt, u16PURPLE, u16BLACK);   // 字符串显示
}
/*************************************************************************
 *  函数名称：void ElectroMagneticCar(void)
 *  功能说明：电磁车双电机差速控制
 -->1.入门算法：简单的分段比例控制算法，教学演示控制算法；
 ---2.进阶算法：PID典型应用控制算法，教学演示控制算法；
 ---3.高端算法：暂定改进粒子群协同控制算法；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：驱动2个电机
 *************************************************************************/
void ElectroMagneticCar (void)
{
    sint16 bat=0;

    CircleNumber = 1;   // 入环次数，0结束；默认1次
    TangentPoint = 1;   // 切点判断   0切点结束；默认1可以入环，读取脉冲为入环准备
    EnterCircle = 0;    // 允许进环  默认 0不可进环；1可以进环
    OutCircle = 0;      // 允许出环   默认0不可出环；1可以出环
    LeftRightCircle = 0;// 左侧环还是右侧环 默认0原始；1左环；2右环
    LowSpeed = 0;       // 速度差

    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk=0;             // CPU1： 0占用/1释放 TFT
    CircleNumber = SetCircleNum();  // 设置需要进入圆环的个数；

    // 。根据需要设置出入库，出库是固定执行，
    // 。入库需要干簧管和外部中断配合实现
    // 。本例程中，干簧管在通过圆环后开启，不会出现起跑触发的可能性
   // OutInGarage(OUT_GARAGE,ReadOutInGarageMode()); // 测试出库，拨码在上左侧出入库，反之右侧出入库
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // 测试入库

    TFTSPI_CLS(u16BLACK);            // 清屏
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk=1;             // CPU1： 0占用/1释放 TFT

    RAllPulse = 0;                  // 全局变量，脉冲计数总数
    NowTime = STM_GetNowUs(STM0);   // 获取STM0 当前时间

    while (1)
    {
        InductorNormal();           // 采集电感电压并并归一化；
        if (MagneticField > 220)    // 直道进入，此值不宜太大，容易丢失切点
        {
            LowSpeed = 500;         // 减速
        }
        else if (MagneticField < 200)
        {
            LowSpeed = 0; // 恢复速度
        }
        CircleDetect();             // 识别并进入圆环的个数；

        ServoDuty = TempAngle;
        ServoCtrl(ServoDuty);       // 舵机PWM输出，转向舵机打角控制
        //
        OffsetDelta = (Lleft2 - Lright2);  // 直道偏差
        bat=(BatVolt * 11 / 25-750)*Kbat;
        MotorDuty1 = MtTargetDuty-bat - ECPULSE1 * Kencoder  - LowSpeed;  // 有差速控制，右转偏差为负，左侧加速
       // MotorDuty2 = MtTargetDuty-bat - ECPULSE2 * Kencoder + OffsetDelta * Koffset - LowSpeed;  // 有差速控制，左转偏差为正，右侧加速

        if (MagneticField < 30)     // 判断是否冲出赛道
        {
            MotorCtrl(0);        // 冲出赛道停车
            delayms(200);
        }
        else
        {
            MotorCtrl(MotorDuty1);   // 四轮双电机驱动
            // MotorCtrl(MtTargetDuty-TempAngle*8/5, MtTargetDuty+TempAngle*8/5);// 三轮车，无舵机
        }

       /* if(Game_Over)
        {
            OutInGarage(IN_GARAGE, ReadOutInGarageMode());
        }*/
    } // WHILE(1)
} // MAIN()

