/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】ZYF/chiusir
【E-mail  】chiusir@163.com
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
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LQ_GPT12_H_
#define _LQ_GPT12_H_

#include "stdint.h"
#include <IfxGpt12.h>
#include <IfxGpt12_reg.h>
#include <Platform_Types.h>

/**
* @brief ENC 脉冲管脚
*/
typedef enum
{
  ENC2_InPut_P00_7 = 0x0200, ENC2_InPut_P33_7 = 0x0201,//00.7摄像头像素占用
  ENC3_InPut_P02_6 = 0x0300,                           //02.6摄像头数据口占用
  ENC4_InPut_P02_8 = 0x0400,                           //
  ENC5_InPut_P10_3 = 0x0501,                           //
  ENC6_InPut_P20_3 = 0x0600, ENC6_InPut_P10_2 = 0x0601 //
}ENC_InputPin_t;

/**
* @brief ENC 方向管脚
*/
typedef enum
{
  ENC2_Dir_P00_8 = 0x0200, ENC2_Dir_P33_6 = 0x0201,   //00.8摄像头占用
  ENC3_Dir_P02_7 = 0x0300,                            //02.7摄像头数据口占用
  ENC4_Dir_P00_9 = 0x0400, ENC4_Dir_P33_5 = 0x0401,   //00.9摄像头占用
  ENC5_Dir_P10_1 = 0x0501,
  ENC6_Dir_P20_0 = 0x0600
}ENC_DirPin_t;


#define ENC_GetTimer(pin)    (unsigned char)(pin >> 8)
void ENC_InitConfig(ENC_InputPin_t InputPin, ENC_DirPin_t DirPin);
signed short ENC_GetCounter(ENC_InputPin_t InputPin);


#endif /* 0_APPSW_TRICORE_APP_LQ_GPT12_H_ */
