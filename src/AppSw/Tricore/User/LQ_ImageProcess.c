/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】zyf/chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Hightec4.9.3/Tasking6.3及以上版本
【Target 】 TC264DA
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
基于iLLD_1_0_1_11_0底层程序
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <LQ_CAMERA.h>
#include <LQ_DMA.h>
#include <LQ_GPIO_LED.h>
#include <LQ_TFT18.h>
#include <LQ_MotorServo.h>
#include <IfxCpu.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_MotorServo.h>
#include <LQ_GPIO_LED.h>
#include <LQ_Inductor.h>
#include <LQ_GPT12_ENC.h>
#include "LQ_MT9V034.h"
#include "LQ_ImageProcess.h"

/*************************************************************************
 *  函数名称：void TFT_Show_Camera_Info(void)
 *  功能说明：显示各种所需信息
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
                                    //定义列和行数
                                  //定义列和行数
unsigned int left_Max = 0, right_Max = 93;                  //左、右边界识别阈值（可调整）
unsigned int bin_threshold;                                  //二值化阈值
unsigned  int flag_single_0;                                  //用于第一行的单边界判断
unsigned  int left_search,            right_search;           //无边界时寻找
unsigned int left_before,            right_before;           //无边界时寻找下一行的参考值
unsigned  int left_none_start,        right_none_start;
unsigned int flag_none_left_start,   flag_none_right_start;
unsigned int flag_single_left = 0,   flag_single_right = 0;  //左、右侧单边界判断
unsigned int flag_try_left = 0,      flag_try_right = 0;     //用于检测某行是否还存在道路边界
unsigned int left[60], right[60], middle[60], valid[60]; //左、右道路边界，中线
int i;
int j;
extern int t=0;
int flag_starting_line;
int flag_line;
unsigned int row, line;
int leftfindflag[60];
int rightfindflag[60];//右线检测到标志位
int left_up[2] ;
int left_down[2] ;
int state;//状态机
int width[120];
int  break_hangshu=0 ;//中线断线行数
int flag_of_rightbreak = 0;
int flag_of_leftbreak = 0;
int lostleft_times=0;
int lostright_times=0;
int l_start=0;//左线起始点，需要用58-
int r_start=0;//右线起始点，需要用58-
int encValue=0;
//int curvity_point1 = ((118-r_start + break_hangshu) / 2);     //中点
int curvity_point2 = 0;
int flag_poding=0;
int flag_ruku,flag_chuku;
int flag_zuoguaidian=0;
double curvity_right =0;
//int x1=right[58-r_start];
//int y1=58-r_start;
//int x2=right[curvity_point1];
//int y2=curvity_point1;
//int x3=right[curvity_point2];
//int y3=curvity_point2;
//int S_of_ABC =((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
double K_right;//斜率
double K_left;//斜率
double rou_of_right;
double K;//过渡参数
double parameterB;// ：斜率
double parameterA;// :截距
float image_error;
extern float Expectspeed;
typedef struct Contours
{
    int height;
    int width;
    int stateFlag;
    int direction;
} contour;
contour* ptr_Contour = NULL;//轮廓点数组首地址

/*void TFT_Show_Camera_Info (void)
{
    char txt[16] = "X:";
    TFTSPI_Init(0);               //TFT1.8初始化0:横屏  1：竖屏
    TFTSPI_CLS(u16BLUE);          //清屏
    sint16 mps = 0, dmm = 0;    // 速度：m/s,毫米数值
    sint16 pulse100 = 0;
    uint16 bat = 0;


    dmm = (sint16) (RAllPulse * 100 / 579);         // 龙邱512带方向编码器1米5790个脉冲，数值太大，除以100
    pulse100 = (sint16) (RAllPulse / 100);
    sprintf(txt, "AP:%05d00", pulse100);           //
    TFTSPI_P8X16Str(3, 4, txt, u16RED, u16BLUE);   // 显示赛道偏差参数

    NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
    mps = (sint16) (dmm / (NowTime / 1000));          // 计算速度mm/s
    TFTSPI_Road(18, 0, LCDH, LCDW, (unsigned char *)Image_Use); // TFT1.8动态显示摄像头灰度图像
    //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
    sprintf(txt, "%04d,%04d,%04d", OFFSET0, OFFSET1, OFFSET2);
    TFTSPI_P8X16Str(0, 5, txt, u16RED, u16BLUE);       // 显示赛道偏差参数
    BatVolt       = ADC_Read(ADC7);  // 刷新电池电压
    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, mps / 1000, (mps / 10) % 100);  // *3.3/4095*3
    TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLUE);   // 字符串显示
    // 电机和舵机参数显示
    sprintf(txt, "Sv:%04d Rno:%d", ServoDuty, CircleNumber);
    TFTSPI_P8X16Str(1, 7, txt, u16RED, u16BLUE);     // 显示舵机，电机1，编码器1数值
    sprintf(txt, "M1:%04d, M2:%04d ", MotorDuty1, MotorDuty2);
    TFTSPI_P8X16Str(0, 8, txt, u16RED, u16BLUE);     // 电机1-2数值
    sprintf(txt, "E1:%04d, E2:%04d ", ECPULSE1, ECPULSE2);
    TFTSPI_P8X16Str(0, 9, txt, u16RED, u16BLUE);     // 编码器1-2数值
    sprintf(txt, "Ring num: %d ", CircleNumber);
    TFTSPI_P8X16Str(2, 6, txt, u16GREEN, u16BLACK);
    sprintf(txt, "M:%03d Q:%d J:%d ", MagneticField, TangentPoint, EnterCircle);
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);

}*/
/*************************************************************************
 *  函数名称：void CameraCar(void)
 *  功能说明：电磁车双电机差速控制
 -->1.入门算法：简单的分段比例控制算法，教学演示控制算法；
 2.进阶算法：PID典型应用控制算法，教学演示控制算法；
 3.高端算法：改进粒子群协同控制算法；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：驱动2个电机
 *************************************************************************/
/*void CameraCar (void)
{
    // 摄像头初始化
    CAMERA_Init(50);
    TFTSPI_P8X16Str(2, 3, "LQ 9V034 Car", u16RED, u16GREEN);
    TFTSPI_P8X16Str(1, 5, "K2 Show Video", u16RED, u16GREEN);
    delayms(500);

    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk = 0;             // CPU1： 0占用/1释放 TFT
    //CircleNumber = 2;    // 设置需要进入圆环的个数；

    // 。根据需要设置出入库，出库是固定执行，
    // 。入库需要干簧管和外部中断配合实现
    // 。本例程中，干簧管在通过圆环后开启，不会出现起跑触发的可能性
   // OutInGarage(OUT_GARAGE, ReadOutInGarageMode()); // 测试出库，拨码在上左侧出入库，反之右侧出入库
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // 测试入库

    TFTSPI_CLS(u16BLUE);            // 清屏
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk = 1;             // CPU1： 0占用/1释放 TFT

    RAllPulse = 0;                  // 全局变量，脉冲计数总数
    NowTime = STM_GetNowUs(STM0);   // 获取STM0 当前时间
    while (1)
    {
        LED_Ctrl(LED1, RVS);     // LED闪烁 指示程序运行状态
        if (Camera_Flag == 2)
        {
            Camera_Flag = 0;     // 清除摄像头采集完成标志位  如果不清除，则不会再次采集数据
            Get_Use_Image();     // 取出赛道及显示所需图像数据
            Get_Bin_Image(2);    // 转换为01格式数据，0、1原图；2、3边沿提取
            Bin_Image_Filter();  // 滤波，三面被围的数据将被修改为同一数值
            Seek_Road();         // 通过黑白区域面积差计算赛道偏差值

            // 计算赛道偏差值，系数越大打角越早，数值跟舵机的范围有关，此处为±160左右，默认为7，
            ServoDuty = Servo_Center_Mid - (OFFSET1 + OFFSET2 + OFFSET2) * 1/7;

            // 圆环处理，如果面积为负数，数值越大说明越偏左边；
            if((OFFSET2 < -300)||(OFFSET2 > 300))
                ServoDuty = Servo_Center_Mid - OFFSET2 / 7;

            ServoCtrl(ServoDuty);     // 舵机PWM输出，转向

            // SPEED正负标识方向，负数为正向
            MotorDuty1 = MtTargetDuty + ECPULSE1 * 4 - (OFFSET1 + OFFSET2 + OFFSET2) / 10;        // 电机PWM
            MotorDuty2 = MtTargetDuty - ECPULSE2 * 4 + (OFFSET1 + OFFSET2 + OFFSET2) / 10;        // 双电机差分，需要去掉abs

            MotorCtrl(MotorDuty1, MotorDuty2);        // 四轮电机驱动
            // MotorCtrl(2500, 2500); // 电机PWM固定功率输出
        }
        if(RAllPulse>10000)
            Reed_Init();            // 干簧管GPIO及中断初始化函数,为停车入库做准备
        if (Game_Over)
        {
            OutInGarage(IN_GARAGE, ReadOutInGarageMode());
        }

    }
}*/
void check_starting_line(void)
        {
            //int[] black_nums_stack = new int[20];
            int times = 0;
            for (int y = 0; y < 60; y++)
            {
                int black_blocks = 0;
                int cursor = 0;    //指向栈顶的游标
                for (int x = 0; x< 94; x++)
                {
                    if (Bin_Image[y][x]== BLACK0)
                    {
                        if (cursor >= 25)
                        {
                             break;
                        }
                        else
                        {
                            cursor++;
                        }
                    }
                    else
                    {
                        if (cursor >= 3 && cursor <= 9)
                        {
                            black_blocks++;
                            cursor = 0;
                        }
                        else
                        {
                            cursor = 0;
                        }
                    }
                }
                if (black_blocks >= 3 && black_blocks <= 10) times++;
            }
            if (times >= 3 && times <= 40)
            {
                flag_starting_line = 1;

            }
            else
            {
                flag_starting_line = 0;
            }
        }
void check_line(void)
        {
            //int[] black_nums_stack = new int[20];
            int times = 0;
            for (int x = 0; x < 94; x++)
            {
                int black_blocks = 0;
                int cursor = 0;    //指向栈顶的游标
                for (int y = 0; y< 60; y++)
                {
                    if (Bin_Image[y][x]== BLACK0)
                    {
                        if (cursor >= 25)
                        {
                             break;
                        }
                        else
                        {
                            cursor++;
                        }
                    }
                    else
                    {
                        if (cursor >= 3 && cursor <= 9)
                        {
                            black_blocks++;
                            cursor = 0;
                        }
                        else
                        {
                            cursor = 0;
                        }
                    }
                }
                if (black_blocks >= 3 && black_blocks <= 10) times++;
            }
            if (times >= 3 && times <= 40)
            {
                flag_line = 1;

            }
            else
            {
                flag_line = 0;
            }
        }
void gate(void)
{
    int y1 = 0;
        int x1 = 0;
        int x = 0;
        int y = 0;
        int left_garage_left_turn_up[2] ;
        int left_garage_right_turn_down[2] ;
        // 1、   如果pixels[0, 0]为白，往上找(不可超过5)，直到pixels[y1, 0]为黑（防止姿态的原因导致车偏向道路右边）
        if (Bin_Image[59][ 85] != 0)   //不为黑,即为白
        {

            for (y = 59; y > 0; y--)
            {
                if (Bin_Image[y][99] == 0)
                {
                    y1 = y;
                    break;
                }
            }
        }
        else
        {
            y1 = 59;
        }
        //2、    从上面确定的第y1行开始往左边扫，扫到的第一个不为黑的点视为右下拐点。

        for (x = 99; x >0; x--)
        {
            if (Bin_Image[y1][ x] != 0)
            {
                x1 = x;
                break;
            }
        }
        //赋值操作
        left_garage_right_turn_down[0] = y1;
        left_garage_right_turn_down[1] = x1;
    //img[y1-5][x1]=3;
        //然后找左上拐点
        //如果前5行为黑，则第二次为黑的行数

        //否则从第10行往上找
        //1、    从第10行往上找，直到pixels[y1,185]为黑

        if (Bin_Image[1][ 0] == 0 && Bin_Image[1][ 0] == 0 && Bin_Image[2][ 0] == 15 && Bin_Image[3][ 0] == 0 && Bin_Image[4][ 0] == 0)
        {
            for (y = 5; y < 70; y++)
            {
                if (Bin_Image[y][0] != 0)//为白
                {
                   if(Bin_Image[y+1][0]==1&&Bin_Image[y+2][0]==1&&Bin_Image[y+3][0]==1&&Bin_Image[y+4][0]==1&&Bin_Image[y+5][0]==1){
                    y1 = y;
                    break;}
                }

            }
            //for (y = y1; y < 100; y++)
            //{
            //    if (img[y][ 0] == 15)
            //    {
            //        y1 = y;
            //        break;
            //    }

            //}
        }
        else
        {
            for (y = 10; y < 60; y++)
            {
                if (Bin_Image[y][ 0] == 0)
                {
                    y1 = y;
                    break;
                }
            }
        }
       // SetText("左上行："+y1);
        //2、    从上面确定的第y1行开始往右边扫，记录下扫到的第一个为白的点的列坐标
    //y1=80;
        for (x = 0; x < 93; x++)
        {
            if (Bin_Image[y1-2][ x] != 0)

            {
    if(Bin_Image[y1-2][x+1]==1&&Bin_Image[y1-2][x+2]==1&&Bin_Image[y1-2][x+3]==1&&Bin_Image[y1][x+4]==1)
                {x1 = x;
                break;}
            }
        }
        //SetText("y1是" + y1);
        //3、    遍历[y2, y2 + 2],选出3行中列坐标最大（最靠左）的列坐标以及那时的行坐标，作为左上拐点
        //int y_zhengque = 0;
        //for (y = (int)(y1 + 1); y <= (int)(y1 + 3); y++)
        //{
        //    int X = 0;
        //    for (x = 185; x > 1; x--)
        //    {
        //        if (img[y1][ x] != 15)
        //        {
        //            X = x;
        //            break;
        //        }
         //   }
         //   if (X <= x1)
         //   {
         //       x1 = X;
         //       y_zhengque = y;
          //  }
        //}
        //y1 = y_zhengque;
        //SetText("y1是"+y1);
        //赋值操作
        left_garage_left_turn_up[0] = y1;
        left_garage_left_turn_up[1] = x1;
        //img[y1][x1]=1;
        //展示
        //SetText("车库左上角" + left_garage_left_turn_up[0] + "  " + left_garage_left_turn_up[1]);
        //SetText("车库右下角" + left_garage_right_turn_down[0] + "  " + left_garage_right_turn_down[1]);
        //补线程序
        int j = 0;
        //int delta = left_garage_left_turn_up[0] - left_garage_right_turn_down[0];
        //if (delta == 0) delta = 1;
        //float k = (left_garage_left_turn_up[1] - left_garage_right_turn_down[1]) * 1.0f / delta;
        //float b = left_garage_right_turn_down[1]-k*left_garage_right_turn_down[0];
        float b=(left_garage_left_turn_up[0]-left_garage_right_turn_down[0])*(left_garage_left_turn_up[0]-left_garage_right_turn_down[0]);
    float k=(left_garage_left_turn_up[1]-left_garage_right_turn_down[1])/b;
        for (j = (int)left_garage_left_turn_up[0]; j <= (int)left_garage_right_turn_down[0]; j++)
        {
            int jicun = ((int)(k * (j -left_garage_right_turn_down[0])* (j -left_garage_right_turn_down[0])+ left_garage_right_turn_down[1]));
            if (jicun >= 93) jicun = 93;
            else if (jicun <= 0) jicun = 0;
            right[j] = (int)jicun;
        }
}
float  curvity(int x1,int y1,int x2,int y2,int x3,int y3)//求曲率
{
    float K;
    int S_of_ABC =((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    int q1 = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double AB = sqrt(q1);
        q1 = ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    double BC = sqrt(q1);
        q1 = ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    double AC = sqrt(q1);
        if (AB * BC * AC == 0)
        {
            K = 0;
        }
        else
            K = 4 * S_of_ABC / (AB * BC * AC);
        return K;
}
void edgeTracing( const unsigned char* _Binary, contour** _TracingPtr, const int _Width, const int _Height )
{
    unsigned char* _Mark = NULL;//标记矩阵
    int direction[8][2] = { 0,-1, -1,-1, -1,0, -1,1, 0,1, 1,1, 1,0, 1,-1 };//方向数组
    int i = 0, j = 0;//行、列计数
    int start_I = 0, start_J = 0;//轮廓的起始点
    int rec_I = 0, rec_J = 0;//待检测轮廓点坐标
    int mov_I = 0, mov_J = 0;//待检测轮廓点周围8领域逆时针旋转点
    int tmp_I = 0, tmp_J = 0;//临时点保存
    int point_Count = 0;//轮廓点计数
    int direc_Count = 0;//方向转次数计数
    int rot_Direct = 0;//方向数组计数
    int stepWidth = (_Width + 3) & ~3;//灰度图行步长
    int contour_Succe_Flag = 0;//轮廓寻找成功标志位
    int contour_Point_Now = 0;//现已开辟轮廓空间点数量
    int contour_Point_Add = 0;//需要增加的轮廓空间点数量
    contour* ptr_Contour_First = NULL;//轮廓点数组首地址


    contour_Point_Now = 4 * (_Width+_Height);
    contour_Point_Add = _Width + _Height;
    *_TracingPtr = (  contour *)malloc( contour_Point_Now * sizeof( contour) );//为储存轮廓点创建内存块
    ptr_Contour_First = *_TracingPtr;
    _Mark = ( unsigned char*)malloc( stepWidth*_Height );//轮廓标记矩阵
    memset(_Mark, 0, stepWidth*_Height);//轮廓标记矩阵清零
    for ( i = 0; i < _Height; i ++ )
    {
        for ( j = 1; j < _Width; j ++ )
        {
            if ( 1 == _Binary[ i*stepWidth + j - 1] && 0 == _Binary[ i*stepWidth + j] && 0 == _Mark[i*stepWidth + j] )
            {
                direc_Count = 0;//方向计数置零
                rot_Direct = 0;//旋转方向计数置零
                start_I = i;//保存每个轮廓的起始点
                start_J = j;
                rec_I = i;//存储第一个b点的行坐标
                rec_J = j;//存储第一个b点的列坐标
                mov_I = i;//存储第一个c点的行坐标
                mov_J = j-1;//存储第一个c点的列坐标
                while ( direc_Count < 8 && mov_I > 0 && mov_I < _Height && mov_J > 0 && mov_J < _Width )
                {
                    direc_Count ++;
                    rot_Direct ++;//顺时针方向转45°
                    if ( 8 == rot_Direct) rot_Direct = 0;//使方向循环且不溢出
                    mov_I = rec_I + direction[rot_Direct][0];//转完之后c点的行坐标
                    mov_J = rec_J + direction[rot_Direct][1];//转完之后c点的列坐标
                    if ( 0 == _Binary[ mov_I*stepWidth + mov_J]
                        && mov_I > 0 && mov_I < _Height
                        && mov_J > 0 && mov_J < _Width
                        && ( start_I != mov_I || start_J != mov_J ) )
                    {
                        if ( 0 == rot_Direct ) rot_Direct = 7;//方向回到上一个角度
                        else rot_Direct --;

                        tmp_I = rec_I;//记录变换前b点行坐标
                        tmp_J = rec_J;//记录变换前b点列坐标
                        rec_I = mov_I;//新的c点坐标赋值给b点
                        rec_J = mov_J;
                        mov_I = tmp_I + direction[rot_Direct][0];//将上一个c点坐标赋值给新c点坐标
                        mov_J = tmp_J + direction[rot_Direct][1];

                        direc_Count = 0;//方向次数计数清零

                        if ( rec_I > mov_I )//计算现在c点相对b点的方位
                            rot_Direct = 2;
                        else if ( rec_I < mov_I )
                            rot_Direct = 6;
                        else if ( rec_J > mov_J )
                            rot_Direct = 0;
                        else if ( rec_J < mov_J )
                            rot_Direct = 4;
                    }
                    else if ( start_I == mov_I && start_J == mov_J )
                    {
                        //如果等于起始点，则重新遍历该轮廓，标记轮廓并储存轮廓点
                        if ( point_Count > contour_Point_Now-1 )
                        {
                            //若轮廓空间点超出范围，则加入额外轮廓空间点
                            ptr_Contour_First = (  contour *)malloc( contour_Point_Now * sizeof( contour) );
                            memcpy( ptr_Contour_First, *_TracingPtr, contour_Point_Now * sizeof( contour) );
                            *_TracingPtr = (  contour *)malloc( (contour_Point_Now + contour_Point_Add) * sizeof( contour) );
                            memcpy( *_TracingPtr, ptr_Contour_First, contour_Point_Now * sizeof( contour) );
                            contour_Point_Now += contour_Point_Add;
                            free( ptr_Contour_First);
                            ptr_Contour_First = NULL;
                            ptr_Contour_First = *_TracingPtr;
                        }

                        ptr_Contour_First[ point_Count].height = start_I;//储存起始点
                        ptr_Contour_First[ point_Count].width = start_J;
                        ptr_Contour_First[ point_Count].stateFlag = 1;//状态标志位stateFlag：
                                                                      //1为起始点，0为普通边缘点，2为结束点
                        ptr_Contour_First[ point_Count].direction = rot_Direct;
                        _Mark[ start_I*stepWidth + start_J] = 1;//标记为已搜索过的点

                        direc_Count = 0;//方向计数置零
                        rot_Direct = 0;//旋转方向计数置零
                        rec_I = start_I;//存储b点的行坐标
                        rec_J = start_J;//存储b点的列坐标
                        mov_I = start_I;//存储 c点的行坐标
                        mov_J = start_J-1;//存储c点的列坐标
                        while ( direc_Count < 8 && mov_I > 0 && mov_I < _Height && mov_J > 0 && mov_J < _Width )
                        {
                            direc_Count ++;
                            rot_Direct ++;//顺时针方向转45°
                            if ( 8 == rot_Direct) rot_Direct = 0;//使方向循环且不溢出
                            mov_I = rec_I + direction[rot_Direct][0];//转完之后c点的行坐标
                            mov_J = rec_J + direction[rot_Direct][1];//转完之后c点的列坐标
                            if ( 0 == _Binary[ mov_I*stepWidth + mov_J]
                                && mov_I > 0 && mov_I < _Height
                                && mov_J > 0 && mov_J < _Width
                                && ( start_I != mov_I || start_J != mov_J ) )
                            {
                                if ( 0 == rot_Direct ) rot_Direct = 7;//方向回到上一个角度
                                else rot_Direct --;

                                tmp_I = rec_I;//记录变换前b点行坐标
                                tmp_J = rec_J;//记录变换前b点列坐标
                                rec_I = mov_I;//新的c点坐标赋值给b点
                                rec_J = mov_J;
                                mov_I = tmp_I + direction[rot_Direct][0];//将上一个c点坐标赋值给新c点坐标
                                mov_J = tmp_J + direction[rot_Direct][1];

                                direc_Count = 0;//方向次数计数清零

                                if ( rec_I > mov_I )//计算现在c点相对b点的方位
                                    rot_Direct = 2;
                                else if ( rec_I < mov_I )
                                    rot_Direct = 6;
                                else if ( rec_J > mov_J )
                                    rot_Direct = 0;
                                else if ( rec_J < mov_J )
                                    rot_Direct = 4;

                                point_Count ++;//压入新点之前，轮廓计数自加

                                if ( point_Count > contour_Point_Now-1 )
                                {
                                    //若轮廓空间点超出范围，则加入额外轮廓空间点
                                    ptr_Contour_First = (  contour *)malloc( contour_Point_Now * sizeof( contour) );
                                    memcpy( ptr_Contour_First, *_TracingPtr, contour_Point_Now * sizeof( contour) );
                                    *_TracingPtr = (  contour *)malloc( (contour_Point_Now + contour_Point_Add) * sizeof( contour) );
                                    memcpy( *_TracingPtr, ptr_Contour_First, contour_Point_Now * sizeof( contour) );
                                    contour_Point_Now += contour_Point_Add;
                                    free( ptr_Contour_First);
                                    ptr_Contour_First = NULL;
                                    ptr_Contour_First = *_TracingPtr;
                                }

                                ptr_Contour_First[ point_Count].height = rec_I;//储存新轮廓点
                                ptr_Contour_First[ point_Count].width = rec_J;
                                ptr_Contour_First[ point_Count].stateFlag = 0;//状态标志位stateFlag：
                                                                              //1为起始点，0为普通边缘点，2为结束点
                                ptr_Contour_First[ point_Count].direction = rot_Direct;
                                _Mark[ rec_I*stepWidth + rec_J] = 1;//标记为已搜索过的点
                            }
                            else if ( start_I == mov_I && start_J == mov_J )
                            {
                                //如果等于起始点，则把之前最后一个点改为终点，跳出开始新的搜索
                                ptr_Contour_First[ point_Count].stateFlag = 2;//状态标志位stateFlag：
                                                                              //1为起始点，0为普通边缘点，2为结束点
                                contour_Succe_Flag = 1;//轮廓完整，标志位置一
                                point_Count ++;//为下一个轮廓开辟一个新点
                                break;
                            }
                        }
                    }

                    if ( contour_Succe_Flag )//如果一个轮廓搜索并储存成功，则开始找新的起始点
                    {
                        contour_Succe_Flag = 0;//轮廓搜索成功标志位置零
                        break;
                    }
                }
            }
        }
    }
    ptr_Contour_First[0].stateFlag = point_Count;//将轮廓点数量保存在第一个空间点的状态位
}

void myregression(int type, int startline, int endline)
{
    int i = 0;
    int sumlines = abs(endline - startline);
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)      //拟合中线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x的平均值
            averageY = sumY / sumlines;     //y的平均值
        }
        else
        {
            averageX = 0;     //x的平均值
            averageY = 0;     //y的平均值
        }
        for (i = startline; i < endline; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 1)//拟合左线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            //SetText("lefetline"+i+" " +lefetline[i] + " averageY" +" "+ averageY);
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;//截距
    }
    else if (type == 2)//拟合右线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
}

void Image_Handle(void)//图像处理总函数
{
    // 摄像头初始化

    //char txt[16];
        CAMERA_Init(50);



        //TFTSPI_P8X16Str(2, 3, "LQ 9V034 Car", u16RED, u16GREEN);
        //TFTSPI_P8X16Str(1, 5, "K2 Show Video", u16RED, u16GREEN);
        delayms(500);

        // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
        mutexCpu0TFTIsOk = 0;             // CPU1： 0占用/1释放 TFT
        //CircleNumber = 2;    // 设置需要进入圆环的个数；

        // 。根据需要设置出入库，出库是固定执行，
        // 。入库需要干簧管和外部中断配合实现
        // 。本例程中，干簧管在通过圆环后开启，不会出现起跑触发的可能性
       // OutInGarage(OUT_GARAGE, ReadOutInGarageMode()); // 测试出库，拨码在上左侧出入库，反之右侧出入库
        //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // 测试入库

        //TFTSPI_CLS(u16BLUE);            // 清屏
        // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
        mutexCpu0TFTIsOk = 0;             // CPU1： 0占用/1释放 TFT

        //RAllPulse = 0;                  // 全局变量，脉冲计数总数
        NowTime = STM_GetNowUs(STM0);   // 获取STM0 当前时间
        while (1)
        {
            tftxian();
            LED_Ctrl(LED1, RVS);     // LED闪烁 指示程序运行状态
            if (Camera_Flag == 2)
            {
                Camera_Flag = 0;     // 清除摄像头采集完成标志位  如果不清除，则不会再次采集数据
                Get_Use_Image();     // 取出赛道及显示所需图像数据
                Get_Bin_Image(0);
               // Show_Use_Image_zhongxian();
                //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) zhongxian);// 转换为01格式数据，0、1原图；2、3边沿提取
                Bin_Image_Filter();  // 滤波，三面被围的数据将被修改为同一数值
                //First_Line_Find();
                //check_starting_line();
                //check_line();
                /*for(line=58;line>0;line--)//每循环一次得到一个Left[line]和一个Right[line]循环结束则边界赋值结束
                {
                    Find_Next_Right();
                    Find_Next_Left();
                    if((right[line]==0||right[line]==93)&&(left[line]==0||left[line]==93))
                    {
                        valid[line]=1;
                        continue;
                    }

                    middle[line]=(right[line]+left[line])/2;
                }*/
                for (line = 59; line > 0; line--)
                {
                leftfindflag[line]=0;
                    for ( row = 0; row <93; row++)
                    {
                //width[line]++;
                        if (Bin_Image[line][ row] != 1 && Bin_Image[line][row + 1] == 1)
                        {
                leftfindflag[line]=1;
                            //img[i][j]=1;
                            left[line] = row;

                            break;
                        }
                        else left[line]=0;
                    }
                }

                for (line = 58; line > 0; line--)
                {
                rightfindflag [line]=0;
                    for (row =left[line] ; row < 93; row++)
                    {
                width[line]++;

                        if (Bin_Image[line][ row - 1] == 1 &&Bin_Image[line][row] != 1)
                        {
                rightfindflag [line]=1;
                            //img[i][j]=2;
                            right[line] = row;

                            break;
                        }
                        else right[line] = 93;
                    }
                }


                for (line = 58; line > 2; line--)
                {
                    if(rightfindflag[line]==1&&(abs(right[line]-right[line+1])>3||abs(right[line]-right[line+2])>3))
                    {
                        rightfindflag[line]=0;
                    }
                    if(right[line]>85)
                    {
                        rightfindflag[line]=0;

                    }
                    if(rightfindflag[line]==0)
                    {
                        //right[line]=187;
                    }
                }

                for (line = 58; line > 2; line--)
                {
                    int l;
                    int m=0;
                    if(leftfindflag[line]==0)
                    {
                        l=line;
                        while(leftfindflag[l]==0&&l>0)
                        {
                            l--;
                            m++;
                        }
                        while(leftfindflag[l]==0&&l<58)
                        {
                            l++;
                            m++;
                        }

                        if(m<20)
                        {
                            left_down [0]=line+1;
                            left_down [1]=left[line+1];
                            l=line;
                            while(leftfindflag[l]==0&&l>0)
                            {
                                l--;
                            }
                            left_up [0]=l;
                            left_up [1]=left[l];
                            int delta = left_up[0] - left_down[0];
                            if (delta == 0) delta = 1;
                            double k = (left_up[1] - left_down[1]) * 1.0 / delta;
                            double b = left_down[1]-k*left_down[0];
                            left[line]=(int)(k*line+b);
                            if(left[line]>93)left[line]=93;
                            if(left[line]<0)left[line]=0;
                        }
                    }
               }
                for (line = 58; line > 2; line--)
                {
                    int l;
                    int m=0;

                    if(rightfindflag[line]==0)
                    {
                        l=line;
                        while(rightfindflag[l]==0&&l>0)
                        {
                            l--;
                            m++;
                        }
                        l=line;
                        while(rightfindflag[l]==0&&l<58)
                        {
                            l++;
                            m++;
                        }
                        if(m<20)
                        {
                            left_down [0]=line+1;
                            left_down [1]=right[line+1];
                            l=line;
                            while(rightfindflag[l]==0&&l>0)
                            {
                                l--;
                            }
                            left_up [0]=l;
                            left_up [1]=right[l];
                            int delta = left_up[0] - left_down[0];
                            if (delta == 0) delta = 1;
                            double k = (left_up[1] - left_down[1]) * 1.0 / delta;
                            double b = left_down[1]-k*left_down[0];
                            right[line]=(int)(k*line+b);
                            if(right[line]>93)right[line]=93;
                            if(right[line]<0)right[line]=0;
                        }
                    }
                }

                for (line = 58; line > 0; line--)
                {
                    //if(leftfindflag[line]==0){left[line] =0;}
                    //if(rightfindflag[line]==0){right[line] =93;}
                    middle[line] = (int)((left[line] + right[line]) * 0.5);
                }

              /*  for(row = 0; row <93; row++)
                {
                    if(Bin_Image[58][row]==0)
                    {
                        j++;
                    }
                    if(Bin_Image[57][row]==0)
                    {
                        j++;
                    }
                    if(Bin_Image[56][row]==0)
                    {
                        j++;
                    }
                }
                if(j>=85*3)
                {
                    while(1)
                    {
                        STM_DisableInterrupt(STM1, STM_Channel_0);
                        CCU6_DisableInterrupt(CCU60, CCU6_Channel0);
                        IfxCpu_disableInterrupts();
                        MotorCtrl(0);
                        //Expectspeed=0;
                        TFTSPI_CLS(u16BLACK);         // 清屏
                        //TFTSPI_P16x16Str(0,0,(unsigned char*)"stop!",u16RED,u16BLUE);
                    }
                }*/




                for (line = 58; line >0; line--)
                {
                    //左线操作
                    if (leftfindflag[line] == 0)       //未扫到线
                    {
                        lostleft_times++;
                        if (flag_of_leftbreak == 0)     //如果在这一行之前没有遭遇断线，则计数
                        {
                            l_start++;
                        }
                    }
                    else  if(leftfindflag[line-1] == 1)     //扫到线
                    {
                        //lostleft_times不继续增加
                        flag_of_leftbreak = 1;  //break标志成立
                    }
                    //右线操作
                    if (rightfindflag[line] == 0)       //未扫到线
                    {
                        lostright_times++;
                        if (flag_of_rightbreak == 0)     //如果在这一行之前没有遭遇断线，则计数
                        {
                            r_start++;
                        }
                    }
                    else    //扫到线
                    {
                        //lostright_times不继续增加
                        flag_of_rightbreak = 1;  //break标志成立
                    }
                }
                for (line =(l_start<20)? (59-l_start):59; line > 0; line--)
                {
                 if(Bin_Image[line-1][middle[line]]==0)
                 {
                     break_hangshu=line;
                     flag_poding=1;
                     if (break_hangshu <=38)    //防止在一开始就break
                     {
                         break;
                     }
                 }
                 else if (abs(middle[line]-middle[line-1])>3)
                 {
                     break_hangshu = line;
                     //last_break_hangshu = break_hangshu;
                     //也就是说二十行一下是不会break的
                     if (break_hangshu <=38)    //防止在一开始就break
                     {
                         break;
                     }
                 }
                }
                for(line=break_hangshu;line<59;line++)
                {
                    if(abs(left[line]-left[line+1])>3&&abs(left[line]-left[line+2])>3&&abs(left[line]-left[line+3])>3&&abs(left[line]-left[line+4])>3&&abs(left[line]-left[line+5])>3)
                    {
                        break_hangshu++;
                    }
                }


                int curvity_point1 = ((60-r_start + break_hangshu) / 2);     //中点
                if (break_hangshu >= 5)
                {
                    curvity_point2 = (break_hangshu - 3);
                }
                else
                {
                    curvity_point2 = (break_hangshu);
                }

                curvity_right= curvity(right[58-r_start],(58-r_start),right[curvity_point1],curvity_point1,right[curvity_point2],curvity_point2);
                //edgeTracing(Bin_Image,&ptr_Contour,94,60);
                //曲率接近0说明线挺直的，为了体现直线方差小的特点，从第start行开始计算
                 if (curvity_right > -0.4 && curvity_right < 0.1 && r_start <= 32 && (break_hangshu - r_start) >= 7)
                 {
                     myregression(2, break_hangshu, 60-r_start);    //拟合右线
                     K_right = parameterB;


                     for (line = 60-r_start; line > break_hangshu - 5; line--)
                     {
                         if(rightfindflag[line]==1)
                         {
                             K += (K_right*line+parameterA - right[line]) * (K_right*line+parameterA - right[line]);
                         }
                     }
                     rou_of_right=K/(abs((break_hangshu - 5)-(60-r_start)));
                     K=0;
                 }
                //否则说明边界是曲线，此时为了凸显出曲线的方差大的特点，从第0行开始计算
                 else
                 {
                     myregression(2, break_hangshu, 60);    //拟合右线
                     K_right = parameterB;
                     for (line = 60; line > break_hangshu - 5; line--)
                     {
                         if(rightfindflag[line]==1)
                         {
                             K += (K_right*line+parameterA - right[line]) * (K_right*line+parameterA - right[line]);
                         }
                     }
                     rou_of_right=K/(abs((break_hangshu - 5)-60));
                     K=0;
                 }
                 for(line=break_hangshu;line<59;line++)
                 {
                     if(abs(left[line]-left[line+1])>3&&abs(left[line]-left[line+2])>3&&abs(left[line]-left[line+3])>3&&abs(left[line]-left[line+4])>3&&abs(left[line]-left[line+5])>3)
                     {
                         break_hangshu++;
                     }

                 }
                 double brea=(double)break_hangshu;

                 for(line=59-l_start;line>break_hangshu;line--)
                 {
                     if((left[break_hangshu]-left[line])*(line+l_start-119)!=0)
                     {
                         if((left[line]-left[119-l_start])*(brea-line)/(left[break_hangshu]-left[line])/(line+l_start-119)>1)
                         {
                             flag_zuoguaidian=1;//进入坡道
                             //state=1;
                             break;
                         }
                     }
                 }
                /* switch(state)
                 {

                     case 2:
                     {
                         if(flag_zuoguaidian)
                         {
                             state=3;
                         }break;
                     }
                     case 3:
                     {
                         flag_zuoguaidian=0;
                         state=0;
                         break;
                     }
                     default:
                     {
                         if(flag_zuoguaidian)
                         {
                             state=1;

                             if(flag_poding)
                             {
                                 state=2;
                                 flag_zuoguaidian=0;
                             }
                         }break;
                     }
                 }*/
                 if(t==0)
                 {
                     if(flag_line==1)

                     {
                         //gate();
                         flag_chuku++;
                         if(flag_chuku>=3)
                         {
                             //MotorInit(); // 电机
                             t=1;

                             //OutInGarage(0,1);

                            // Expectspeed=1;
                            // STM_InitConfig(STM0, STM_Channel_1,5000);//舵机中断STM_InitConfig(STM0, STM_Channel_0,100);
                         }

                     }
                     else
                     {
                         flag_chuku=0;
                     }
                 }
               if(flag_starting_line==1&&t!=0)
                {
                    //gate();
                    flag_ruku++;
                    if(flag_ruku>=3)
                    {
                        while(1)
                        {
                        // STM_DisableInterrupt(STM0, STM_Channel_1);
                       //  CCU6_DisableInterrupt(CCU60, CCU6_Channel0);
                        // OutInGarage(1,1);
                        }
                    }
                }
                else
                {
                    flag_ruku=0;

                  image_error = ((float)middle[50]+(float)middle[40]+(float)middle[45])/3-47.0f-(float)(5*regression(20,40) );
                }
            }
           // encValue = ENC_GetCounter(ENC4_InPut_P02_8);;
            //UART_PutStr(UART0,"\n");
//            sprintf(txt, "t: %05d; ", t);
//            //UART_PutStr(UART0,txt);
//            TFTSPI_P8X16Str(0, 7, txt,u16WHITE,u16BLACK);
//            sprintf(txt, "flag:%d",flag_line);
//            TFTSPI_P8X16Str(2, 6, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
            flag_of_leftbreak=0;r_start=0;
            flag_of_rightbreak=0;l_start=0;
            flag_zuoguaidian=0;flag_poding=0;j=0;
        }
}

