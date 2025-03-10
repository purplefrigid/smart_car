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
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ

理论有8个转换器，可同步转换；每个转换8个通道，精度为8/10/12位。默认为最高精度12位。
ADC端口AN可以作为输入口，不可作为输出口，与ARM单片机不同！
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_ADC.h>

/**
 * ADC寄存器
 */
IfxVadc_Adc vadc;

/**
 * ADC转换结构体
 */
Ifx_VADC_RES conversionResult;

/**
 * ADC转换组
 */
IfxVadc_Adc_Group g_AdcGroup[3];

/**
 * ADC转换通道配置结构体
 */
IfxVadc_Adc_Channel g_AdcChannel[28];


/*************************************************************************
*  函数名称：void ADC_init(void)
*  功能说明：ADC初始化函数
*  参数说明：channel  ADC通道
*  参数说明：Freq_Hz  ADC通道采样频率     注意 ADC有三个模块 通道0-13  16-25  35-49分别属于这三个通道  这里使用扫描采样方式，因此如果一个ADC模块同时初始化了N个通道，则ADC实际采用率 = Freq_Hz / N
*  函数返回：无
*  修改时间：2020年5月10日
*  备    注：     注意 adc采样率最高1000000
*************************************************************************/
void ADC_InitConfig(ADC_Channel_t channel, unsigned long Freq_Hz)
{
	//新建配置
	IfxVadc_Adc_Config adcConfig;
	IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

	adcConfig.supplyVoltage = IfxVadc_LowSupplyVoltageSelect_3V;

	//初始化模块
	IfxVadc_Adc_initModule(&vadc, &adcConfig);

	//新建ADC组配置
	IfxVadc_Adc_GroupConfig adcGroupConfig;
	IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &vadc);

	if(Freq_Hz > 1000000)
	{
		Freq_Hz = 1000000;
	}

	/* 设置采样时间 */
	adcGroupConfig.inputClass[0].sampleTime = 1.0f/Freq_Hz;
	adcGroupConfig.inputClass[1].sampleTime = 1.0f/Freq_Hz;

	/* 设置分辨率 12位 */
	adcGroupConfig.inputClass[0].resolution = IfxVadc_ChannelResolution_12bit;
	adcGroupConfig.inputClass[1].resolution = IfxVadc_ChannelResolution_12bit;

#pragma warning 547         // 屏蔽警告
	//修改组
	adcGroupConfig.groupId = ADC_GetGroup(channel);
#pragma warning default     // 打开警告

	//重要说明:通常用同一个组作为master!
	adcGroupConfig.master = adcGroupConfig.groupId;

	//启动全部仲裁请求源，每个请求源可以顺序触发1，4或者8路转换
	adcGroupConfig.arbiter.requestSlotQueueEnabled          = FALSE;

	//启用序列模式，必须启用3种请求源中的一种
	adcGroupConfig.arbiter.requestSlotScanEnabled           = TRUE;

	//启用扫描模式
	adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = FALSE;

	//启用背景扫描
	//使能所有的门为 "always" 模式 (无边沿检测)
	adcGroupConfig.queueRequest.triggerConfig.gatingMode          = IfxVadc_GatingMode_disabled;
	adcGroupConfig.scanRequest.triggerConfig.gatingMode           = IfxVadc_GatingMode_always;
	adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_disabled;
	//启动自动扫描模式
	adcGroupConfig.scanRequest.autoscanEnabled = TRUE;

	//初始化组
	IfxVadc_Adc_initGroup(&g_AdcGroup[ADC_GetGroup(channel)], &adcGroupConfig);

	//新建通道配置
	IfxVadc_Adc_ChannelConfig adcChannelConfig;

	IfxVadc_Adc_initChannelConfig(&adcChannelConfig, &g_AdcGroup[ADC_GetGroup(channel)]);//初始化
	adcChannelConfig.channelId = (IfxVadc_ChannelId)(ADC_GetChannel(channel));           //通道ID选择
	adcChannelConfig.resultRegister = (IfxVadc_ChannelResult)(ADC_GetChannel(channel));  //对应通道转换结果结存器选择

	//使用专用结果寄存器初始化通道
	IfxVadc_Adc_initChannel(&g_AdcChannel[ADC_GetIndex(channel)], &adcChannelConfig);
	//加入自动扫描
	unsigned channels = (1 << adcChannelConfig.channelId);
	unsigned mask = channels;
	IfxVadc_Adc_setScan(&g_AdcGroup[ADC_GetGroup(channel)], channels, mask);

	//开始自动扫描
	IfxVadc_Adc_startScan(&g_AdcGroup[ADC_GetGroup(channel)]);

}

/*************************************************************************
*  函数名称：unsigned short ADC_Read(ADC_Channel_t  chn)
*  功能说明：ADC采集
*  参数说明：chn    : ADC通道
*  函数返回：ADC数值
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
unsigned short ADC_Read(ADC_Channel_t  chn)
{
	//等待转换完成
	do{
		conversionResult = IfxVadc_Adc_getResult(&g_AdcChannel[ADC_GetIndex(chn)]);
	} while( !conversionResult.B.VF );

	return 	conversionResult.B.RESULT;//返回转换结果
}

/*************************************************************************
*  函数名称：unsigned short ADC_Read(ADC_Channel_t  chn)
*  功能说明：ADC多次采集取平均值
*  参数说明：chn    : ADC通道
*  参数说明：count  : 采集次数
*  函数返回：ADC数值
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
unsigned short ADC_ReadAverage(ADC_Channel_t  chn, unsigned char count)
{
	unsigned short i = 0;
	unsigned long sum = 0;
	for(i = 0; i < count; i++)
	{
		sum += adc_mid(chn);
	}
	return 	(unsigned short)(sum/count);//返回转换结果
}
/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////完/////////////////////////////////////////////////
/*
 * 卡尔曼滤波
*/
unsigned long kalman_filter(ADC_Channel_t  chn)
{
    float LastData;
    float NowData;
    float kalman_adc;
    static float kalman_adc_old = 0;
    static float P1;
    static float Q = 0.0005;
    static float R = 2;
    static float Kg = 0;
    static float P = 1;
    NowData =(float)ADC_Read(chn);
    LastData =kalman_adc_old;
    P = P1 + Q;
    Kg = P / ( P + R );
    kalman_adc = LastData + Kg * ( NowData - kalman_adc_old );
    P1 = ( 1 - Kg ) * P;
    P = P1;
    kalman_adc_old = kalman_adc;
    return ( unsigned long )( kalman_adc );
}
/*
 * 算术平均值滤波
 */
unsigned short Ad_Value(ADC_Channel_t  chn,unsigned char count)
{
   float  LeftADC[count],t;
   int i,j,k,l;
   unsigned long adc=0;
   for(l=0;l<count;l++)
    {
        LeftADC[l]= (float)ADC_Read(chn);
    }

    for(i=0;i<count;i++)
    {
        for(j=0;j<count-i;j++)
        {
             if(LeftADC[j]>LeftADC[j+1])
            {
                t=LeftADC[j+1];
                LeftADC[j+1]=LeftADC[j];
                LeftADC[j]=t;
            }
        }
    }
    for(k = 1; k < count-1; k++)
        {
            adc += LeftADC[k];
        }
    return (unsigned short)(adc/(count-2));

}
//  @param      adcn_ch         选择ADC通道
//  @param      bit             选择分辨率ADC_8bit、ADC_10bit、ADC_12bit、ADC_16bit
//  @return     tmp
//  @since      v1.0
//  Sample usage:               adc_mid(ADC1_SE8,ADC_12bit);//通道编号为ADC1_SE8的引脚，进行3次ADC转换，选取中间值返回
//-------------------------------------------------------------------------------------------------------------------
unsigned short adc_mid(ADC_Channel_t  chn )//中值滤波
{
    unsigned short i, j, k, tmp;
    //1.取3次A/D转换结果
    i = ADC_Read(chn);
    j = ADC_Read(chn);
    k = ADC_Read(chn);
   if(i<=15) i=0;
   if(j<=15) j=0;
   if(k<=15) k=0;
    //2.取中值
    if (i > j)
    {
        tmp = i; i = j; j = tmp;
    }
    if (k > j)
        tmp = j;
    else if(k > i)
        tmp = k;
    else
        tmp = i;

    return tmp;
}
