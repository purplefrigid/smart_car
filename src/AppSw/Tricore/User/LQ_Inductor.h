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
#ifndef SRC_APPSW_TRICORE_MAIN_LQ_SMARTCAR_H_
#define SRC_APPSW_TRICORE_MAIN_LQ_SMARTCAR_H_

extern volatile char mutexCpu0TFTIsOk;         // CPU1 0占用/1释放 TFT
extern volatile uint8 Game_Over; // 小车完成全部任务，停车
extern float Error,Error_h,Error_s,Re_servo,Error_hw;
extern int Servo_duty, servoTurnDuty,duty,duty1;

/*************************************************************************
 *  函数名称：void InductorInit (void)
 *  功能说明：四个电感ADC初始化函数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void InductorInit(void);

/*************************************************************************
 *  函数名称：void InductorNormal(void)
 *  功能说明：采集电感电压并并归一化；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void InductorNormal(void);
void tftxian(void);
void ADCxian(void);
void uartsong(void);
void poserror(void);
void lostline_deal(void);
void lostline_judge(void);
void yuansuchuli(void);
void Cross(void);
void sancha(void);
void circle_big(void);
void circle_small(void);
void circle_bigdeal(void);
void circle_smalldeal(void);
void PIDS(float error);
/*************************************************************************
 *  函数名称：void CircleDetect void
 *  功能说明：识别并进入圆环的个数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void CircleDetect(void);

/*************************************************************************
 *  函数名称：uint8 SetCircleNum (void)
 *  功能说明：设置需要进入圆环的个数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
uint8 SetCircleNum (void);

/*************************************************************************
 *  函数名称：void OutInGarage(uint8 inout, uint8 lr)
 *  功能说明：出入库
 *  参数说明：uint8 inout:0出库，1入库；
 *          uint8 lr：0左出入库；1右出入库
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：   OutInGarage(1,0); // 右侧出库
 *************************************************************************/
void OutInGarage (uint8 inout, uint8 lr);
void OutInGarageTft (uint8 inout, uint8 lr);// 有屏幕显示，不要冲突
/*************************************************************************
 *  函数名称：uint8 ReadOutInGarageMode(void)
 *  功能说明：读取拨码开关设置出入库模式
 *  参数说明：无
 *  函数返回：出入库模式,0左出入库；默认1右出入库
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
uint8 ReadOutInGarageMode(void);

/*************************************************************************
 *  函数名称：void ElectroMagneticCar(void)
 *  功能说明：电磁车双电机差速控制
 -->1.入门算法：简单的分段比例控制算法，教学演示控制算法；
 2.进阶算法：PID典型应用控制算法，教学演示控制算法；
 3.高端算法：改进粒子群协同控制算法；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：驱动2个电机
 *************************************************************************/
void ElectroMagneticCar (void);
/*************************************************************************
 *  函数名称：void TFT_Show_EleMag_Info(void)
 *  功能说明：显示各种所需信息
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
void TFT_Show_EleMag_Info(void);
#endif /* SRC_APPSW_TRICORE_MAIN_LQ_SMARTCAR_H_ */
