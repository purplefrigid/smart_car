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

#ifndef _LQ_STM_H_
#define _LQ_STM_H_

#include "Cpu/Std/Platform_Types.h"
#include "Stm/Std/IfxStm.h"
#include "stdint.h"
#include <Bsp.h>
#include <CompilerTasking.h>
#include <Ifx_Types.h>
#include <IfxCpu.h>
#include <IfxCpu_IntrinsicsTasking.h>
#include <IfxCpu_Irq.h>
#include <IfxStm_cfg.h>
#include <IfxStm_reg.h>
#include <IfxStm_regdef.h>

extern float Lleft1 , Lleft2 , Lright2 , Lright1 ;  // 电感偏移量
extern float Error,Re_servo,Error_h,Error_s;
/**
 * 	STM模块枚举
 */
typedef enum
{
	STM0 = 0,
	STM1
}STM_t;

/**
 * 	STM通道枚举
 */
typedef enum
{
	STM_Channel_0 = 0,
	STM_Channel_1
}STM_Channel_t;

/** STM定时器中断 STM0  channel0 中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  STM0_CH0_PRIORITY    110

/** STM定时器中断 STM0  channel1 中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  STM0_CH1_PRIORITY    111

/** STM定时器中断 STM0  中断归哪个内核管理？ 范围：0：CPU0   1：CPU1   3：DMA*/
#define  STM0_VECTABNUM       0


/** STM定时器中断 STM1  channel0 中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  STM1_CH0_PRIORITY    112

/** STM定时器中断 STM1  channel1 中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  STM1_CH1_PRIORITY    113

/** STM定时器中断 STM1   中断归哪个内核管理？ 范围：0：CPU0   1：CPU1   3：DMA*/
#define  STM1_VECTABNUM       0
//#define Servo_Center_Mid 1900      //舵机直行中值，

extern IfxStm_CompareConfig g_StmCompareConfig[4];
void STM_InitConfig(STM_t STM, STM_Channel_t channel, unsigned long us);
void STM_DelayUs(STM_t stm, unsigned long us);
unsigned long STM_GetNowUs(STM_t stm);
void STM_DisableInterrupt(STM_t stm, STM_Channel_t channel);
void STM_EnableInterrupt(STM_t stm, STM_Channel_t channel);
void delayms(unsigned short stmms);
void delayus(unsigned short stmms);
#endif /* 0_APPSW_TRICORE_APP_LQ_STM_H_ */
