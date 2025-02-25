/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��zyf/chiusir
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Hightec4.9.3/Tasking6.3�����ϰ汾
��Target �� TC264DA
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
����iLLD_1_0_1_11_0�ײ����
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
 *  �������ƣ�void TFT_Show_Camera_Info(void)
 *  ����˵������ʾ����������Ϣ
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
                                    //�����к�����
                                  //�����к�����
unsigned int left_Max = 0, right_Max = 93;                  //���ұ߽�ʶ����ֵ���ɵ�����
unsigned int bin_threshold;                                  //��ֵ����ֵ
unsigned  int flag_single_0;                                  //���ڵ�һ�еĵ��߽��ж�
unsigned  int left_search,            right_search;           //�ޱ߽�ʱѰ��
unsigned int left_before,            right_before;           //�ޱ߽�ʱѰ����һ�еĲο�ֵ
unsigned  int left_none_start,        right_none_start;
unsigned int flag_none_left_start,   flag_none_right_start;
unsigned int flag_single_left = 0,   flag_single_right = 0;  //���Ҳ൥�߽��ж�
unsigned int flag_try_left = 0,      flag_try_right = 0;     //���ڼ��ĳ���Ƿ񻹴��ڵ�·�߽�
unsigned int left[60], right[60], middle[60], valid[60]; //���ҵ�·�߽磬����
int i;
int j;
extern int t=0;
int flag_starting_line;
int flag_line;
unsigned int row, line;
int leftfindflag[60];
int rightfindflag[60];//���߼�⵽��־λ
int left_up[2] ;
int left_down[2] ;
int state;//״̬��
int width[120];
int  break_hangshu=0 ;//���߶�������
int flag_of_rightbreak = 0;
int flag_of_leftbreak = 0;
int lostleft_times=0;
int lostright_times=0;
int l_start=0;//������ʼ�㣬��Ҫ��58-
int r_start=0;//������ʼ�㣬��Ҫ��58-
int encValue=0;
//int curvity_point1 = ((118-r_start + break_hangshu) / 2);     //�е�
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
double K_right;//б��
double K_left;//б��
double rou_of_right;
double K;//���ɲ���
double parameterB;// ��б��
double parameterA;// :�ؾ�
float image_error;
extern float Expectspeed;
typedef struct Contours
{
    int height;
    int width;
    int stateFlag;
    int direction;
} contour;
contour* ptr_Contour = NULL;//�����������׵�ַ

/*void TFT_Show_Camera_Info (void)
{
    char txt[16] = "X:";
    TFTSPI_Init(0);               //TFT1.8��ʼ��0:����  1������
    TFTSPI_CLS(u16BLUE);          //����
    sint16 mps = 0, dmm = 0;    // �ٶȣ�m/s,������ֵ
    sint16 pulse100 = 0;
    uint16 bat = 0;


    dmm = (sint16) (RAllPulse * 100 / 579);         // ����512�����������1��5790�����壬��ֵ̫�󣬳���100
    pulse100 = (sint16) (RAllPulse / 100);
    sprintf(txt, "AP:%05d00", pulse100);           //
    TFTSPI_P8X16Str(3, 4, txt, u16RED, u16BLUE);   // ��ʾ����ƫ�����

    NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // ��ȡSTM0 ��ǰʱ�䣬�õ�����
    mps = (sint16) (dmm / (NowTime / 1000));          // �����ٶ�mm/s
    TFTSPI_Road(18, 0, LCDH, LCDW, (unsigned char *)Image_Use); // TFT1.8��̬��ʾ����ͷ�Ҷ�ͼ��
    //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
    sprintf(txt, "%04d,%04d,%04d", OFFSET0, OFFSET1, OFFSET2);
    TFTSPI_P8X16Str(0, 5, txt, u16RED, u16BLUE);       // ��ʾ����ƫ�����
    BatVolt       = ADC_Read(ADC7);  // ˢ�µ�ص�ѹ
    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, mps / 1000, (mps / 10) % 100);  // *3.3/4095*3
    TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLUE);   // �ַ�����ʾ
    // ����Ͷ��������ʾ
    sprintf(txt, "Sv:%04d Rno:%d", ServoDuty, CircleNumber);
    TFTSPI_P8X16Str(1, 7, txt, u16RED, u16BLUE);     // ��ʾ��������1��������1��ֵ
    sprintf(txt, "M1:%04d, M2:%04d ", MotorDuty1, MotorDuty2);
    TFTSPI_P8X16Str(0, 8, txt, u16RED, u16BLUE);     // ���1-2��ֵ
    sprintf(txt, "E1:%04d, E2:%04d ", ECPULSE1, ECPULSE2);
    TFTSPI_P8X16Str(0, 9, txt, u16RED, u16BLUE);     // ������1-2��ֵ
    sprintf(txt, "Ring num: %d ", CircleNumber);
    TFTSPI_P8X16Str(2, 6, txt, u16GREEN, u16BLACK);
    sprintf(txt, "M:%03d Q:%d J:%d ", MagneticField, TangentPoint, EnterCircle);
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);

}*/
/*************************************************************************
 *  �������ƣ�void CameraCar(void)
 *  ����˵������ų�˫������ٿ���
 -->1.�����㷨���򵥵ķֶα��������㷨����ѧ��ʾ�����㷨��
 2.�����㷨��PID����Ӧ�ÿ����㷨����ѧ��ʾ�����㷨��
 3.�߶��㷨���Ľ�����ȺЭͬ�����㷨��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע������2�����
 *************************************************************************/
/*void CameraCar (void)
{
    // ����ͷ��ʼ��
    CAMERA_Init(50);
    TFTSPI_P8X16Str(2, 3, "LQ 9V034 Car", u16RED, u16GREEN);
    TFTSPI_P8X16Str(1, 5, "K2 Show Video", u16RED, u16GREEN);
    delayms(500);

    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk = 0;             // CPU1�� 0ռ��/1�ͷ� TFT
    //CircleNumber = 2;    // ������Ҫ����Բ���ĸ�����

    // ��������Ҫ���ó���⣬�����ǹ̶�ִ�У�
    // �������Ҫ�ɻɹܺ��ⲿ�ж����ʵ��
    // ���������У��ɻɹ���ͨ��Բ������������������ܴ����Ŀ�����
   // OutInGarage(OUT_GARAGE, ReadOutInGarageMode()); // ���Գ��⣬��������������⣬��֮�Ҳ�����
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // �������

    TFTSPI_CLS(u16BLUE);            // ����
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk = 1;             // CPU1�� 0ռ��/1�ͷ� TFT

    RAllPulse = 0;                  // ȫ�ֱ����������������
    NowTime = STM_GetNowUs(STM0);   // ��ȡSTM0 ��ǰʱ��
    while (1)
    {
        LED_Ctrl(LED1, RVS);     // LED��˸ ָʾ��������״̬
        if (Camera_Flag == 2)
        {
            Camera_Flag = 0;     // �������ͷ�ɼ���ɱ�־λ  �����������򲻻��ٴβɼ�����
            Get_Use_Image();     // ȡ����������ʾ����ͼ������
            Get_Bin_Image(2);    // ת��Ϊ01��ʽ���ݣ�0��1ԭͼ��2��3������ȡ
            Bin_Image_Filter();  // �˲������汻Χ�����ݽ����޸�Ϊͬһ��ֵ
            Seek_Road();         // ͨ���ڰ�����������������ƫ��ֵ

            // ��������ƫ��ֵ��ϵ��Խ����Խ�磬��ֵ������ķ�Χ�йأ��˴�Ϊ��160���ң�Ĭ��Ϊ7��
            ServoDuty = Servo_Center_Mid - (OFFSET1 + OFFSET2 + OFFSET2) * 1/7;

            // Բ������������Ϊ��������ֵԽ��˵��Խƫ��ߣ�
            if((OFFSET2 < -300)||(OFFSET2 > 300))
                ServoDuty = Servo_Center_Mid - OFFSET2 / 7;

            ServoCtrl(ServoDuty);     // ���PWM�����ת��

            // SPEED������ʶ���򣬸���Ϊ����
            MotorDuty1 = MtTargetDuty + ECPULSE1 * 4 - (OFFSET1 + OFFSET2 + OFFSET2) / 10;        // ���PWM
            MotorDuty2 = MtTargetDuty - ECPULSE2 * 4 + (OFFSET1 + OFFSET2 + OFFSET2) / 10;        // ˫�����֣���Ҫȥ��abs

            MotorCtrl(MotorDuty1, MotorDuty2);        // ���ֵ������
            // MotorCtrl(2500, 2500); // ���PWM�̶��������
        }
        if(RAllPulse>10000)
            Reed_Init();            // �ɻɹ�GPIO���жϳ�ʼ������,Ϊͣ�������׼��
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
                int cursor = 0;    //ָ��ջ�����α�
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
                int cursor = 0;    //ָ��ջ�����α�
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
        // 1��   ���pixels[0, 0]Ϊ�ף�������(���ɳ���5)��ֱ��pixels[y1, 0]Ϊ�ڣ���ֹ��̬��ԭ���³�ƫ���·�ұߣ�
        if (Bin_Image[59][ 85] != 0)   //��Ϊ��,��Ϊ��
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
        //2��    ������ȷ���ĵ�y1�п�ʼ�����ɨ��ɨ���ĵ�һ����Ϊ�ڵĵ���Ϊ���¹յ㡣

        for (x = 99; x >0; x--)
        {
            if (Bin_Image[y1][ x] != 0)
            {
                x1 = x;
                break;
            }
        }
        //��ֵ����
        left_garage_right_turn_down[0] = y1;
        left_garage_right_turn_down[1] = x1;
    //img[y1-5][x1]=3;
        //Ȼ�������Ϲյ�
        //���ǰ5��Ϊ�ڣ���ڶ���Ϊ�ڵ�����

        //����ӵ�10��������
        //1��    �ӵ�10�������ң�ֱ��pixels[y1,185]Ϊ��

        if (Bin_Image[1][ 0] == 0 && Bin_Image[1][ 0] == 0 && Bin_Image[2][ 0] == 15 && Bin_Image[3][ 0] == 0 && Bin_Image[4][ 0] == 0)
        {
            for (y = 5; y < 70; y++)
            {
                if (Bin_Image[y][0] != 0)//Ϊ��
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
       // SetText("�����У�"+y1);
        //2��    ������ȷ���ĵ�y1�п�ʼ���ұ�ɨ����¼��ɨ���ĵ�һ��Ϊ�׵ĵ��������
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
        //SetText("y1��" + y1);
        //3��    ����[y2, y2 + 2],ѡ��3���������������󣩵��������Լ���ʱ�������꣬��Ϊ���Ϲյ�
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
        //SetText("y1��"+y1);
        //��ֵ����
        left_garage_left_turn_up[0] = y1;
        left_garage_left_turn_up[1] = x1;
        //img[y1][x1]=1;
        //չʾ
        //SetText("�������Ͻ�" + left_garage_left_turn_up[0] + "  " + left_garage_left_turn_up[1]);
        //SetText("�������½�" + left_garage_right_turn_down[0] + "  " + left_garage_right_turn_down[1]);
        //���߳���
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
float  curvity(int x1,int y1,int x2,int y2,int x3,int y3)//������
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
    unsigned char* _Mark = NULL;//��Ǿ���
    int direction[8][2] = { 0,-1, -1,-1, -1,0, -1,1, 0,1, 1,1, 1,0, 1,-1 };//��������
    int i = 0, j = 0;//�С��м���
    int start_I = 0, start_J = 0;//��������ʼ��
    int rec_I = 0, rec_J = 0;//���������������
    int mov_I = 0, mov_J = 0;//�������������Χ8������ʱ����ת��
    int tmp_I = 0, tmp_J = 0;//��ʱ�㱣��
    int point_Count = 0;//���������
    int direc_Count = 0;//����ת��������
    int rot_Direct = 0;//�����������
    int stepWidth = (_Width + 3) & ~3;//�Ҷ�ͼ�в���
    int contour_Succe_Flag = 0;//����Ѱ�ҳɹ���־λ
    int contour_Point_Now = 0;//���ѿ��������ռ������
    int contour_Point_Add = 0;//��Ҫ���ӵ������ռ������
    contour* ptr_Contour_First = NULL;//�����������׵�ַ


    contour_Point_Now = 4 * (_Width+_Height);
    contour_Point_Add = _Width + _Height;
    *_TracingPtr = (  contour *)malloc( contour_Point_Now * sizeof( contour) );//Ϊ���������㴴���ڴ��
    ptr_Contour_First = *_TracingPtr;
    _Mark = ( unsigned char*)malloc( stepWidth*_Height );//������Ǿ���
    memset(_Mark, 0, stepWidth*_Height);//������Ǿ�������
    for ( i = 0; i < _Height; i ++ )
    {
        for ( j = 1; j < _Width; j ++ )
        {
            if ( 1 == _Binary[ i*stepWidth + j - 1] && 0 == _Binary[ i*stepWidth + j] && 0 == _Mark[i*stepWidth + j] )
            {
                direc_Count = 0;//�����������
                rot_Direct = 0;//��ת�����������
                start_I = i;//����ÿ����������ʼ��
                start_J = j;
                rec_I = i;//�洢��һ��b���������
                rec_J = j;//�洢��һ��b���������
                mov_I = i;//�洢��һ��c���������
                mov_J = j-1;//�洢��һ��c���������
                while ( direc_Count < 8 && mov_I > 0 && mov_I < _Height && mov_J > 0 && mov_J < _Width )
                {
                    direc_Count ++;
                    rot_Direct ++;//˳ʱ�뷽��ת45��
                    if ( 8 == rot_Direct) rot_Direct = 0;//ʹ����ѭ���Ҳ����
                    mov_I = rec_I + direction[rot_Direct][0];//ת��֮��c���������
                    mov_J = rec_J + direction[rot_Direct][1];//ת��֮��c���������
                    if ( 0 == _Binary[ mov_I*stepWidth + mov_J]
                        && mov_I > 0 && mov_I < _Height
                        && mov_J > 0 && mov_J < _Width
                        && ( start_I != mov_I || start_J != mov_J ) )
                    {
                        if ( 0 == rot_Direct ) rot_Direct = 7;//����ص���һ���Ƕ�
                        else rot_Direct --;

                        tmp_I = rec_I;//��¼�任ǰb��������
                        tmp_J = rec_J;//��¼�任ǰb��������
                        rec_I = mov_I;//�µ�c�����긳ֵ��b��
                        rec_J = mov_J;
                        mov_I = tmp_I + direction[rot_Direct][0];//����һ��c�����긳ֵ����c������
                        mov_J = tmp_J + direction[rot_Direct][1];

                        direc_Count = 0;//���������������

                        if ( rec_I > mov_I )//��������c�����b��ķ�λ
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
                        //���������ʼ�㣬�����±������������������������������
                        if ( point_Count > contour_Point_Now-1 )
                        {
                            //�������ռ�㳬����Χ���������������ռ��
                            ptr_Contour_First = (  contour *)malloc( contour_Point_Now * sizeof( contour) );
                            memcpy( ptr_Contour_First, *_TracingPtr, contour_Point_Now * sizeof( contour) );
                            *_TracingPtr = (  contour *)malloc( (contour_Point_Now + contour_Point_Add) * sizeof( contour) );
                            memcpy( *_TracingPtr, ptr_Contour_First, contour_Point_Now * sizeof( contour) );
                            contour_Point_Now += contour_Point_Add;
                            free( ptr_Contour_First);
                            ptr_Contour_First = NULL;
                            ptr_Contour_First = *_TracingPtr;
                        }

                        ptr_Contour_First[ point_Count].height = start_I;//������ʼ��
                        ptr_Contour_First[ point_Count].width = start_J;
                        ptr_Contour_First[ point_Count].stateFlag = 1;//״̬��־λstateFlag��
                                                                      //1Ϊ��ʼ�㣬0Ϊ��ͨ��Ե�㣬2Ϊ������
                        ptr_Contour_First[ point_Count].direction = rot_Direct;
                        _Mark[ start_I*stepWidth + start_J] = 1;//���Ϊ���������ĵ�

                        direc_Count = 0;//�����������
                        rot_Direct = 0;//��ת�����������
                        rec_I = start_I;//�洢b���������
                        rec_J = start_J;//�洢b���������
                        mov_I = start_I;//�洢 c���������
                        mov_J = start_J-1;//�洢c���������
                        while ( direc_Count < 8 && mov_I > 0 && mov_I < _Height && mov_J > 0 && mov_J < _Width )
                        {
                            direc_Count ++;
                            rot_Direct ++;//˳ʱ�뷽��ת45��
                            if ( 8 == rot_Direct) rot_Direct = 0;//ʹ����ѭ���Ҳ����
                            mov_I = rec_I + direction[rot_Direct][0];//ת��֮��c���������
                            mov_J = rec_J + direction[rot_Direct][1];//ת��֮��c���������
                            if ( 0 == _Binary[ mov_I*stepWidth + mov_J]
                                && mov_I > 0 && mov_I < _Height
                                && mov_J > 0 && mov_J < _Width
                                && ( start_I != mov_I || start_J != mov_J ) )
                            {
                                if ( 0 == rot_Direct ) rot_Direct = 7;//����ص���һ���Ƕ�
                                else rot_Direct --;

                                tmp_I = rec_I;//��¼�任ǰb��������
                                tmp_J = rec_J;//��¼�任ǰb��������
                                rec_I = mov_I;//�µ�c�����긳ֵ��b��
                                rec_J = mov_J;
                                mov_I = tmp_I + direction[rot_Direct][0];//����һ��c�����긳ֵ����c������
                                mov_J = tmp_J + direction[rot_Direct][1];

                                direc_Count = 0;//���������������

                                if ( rec_I > mov_I )//��������c�����b��ķ�λ
                                    rot_Direct = 2;
                                else if ( rec_I < mov_I )
                                    rot_Direct = 6;
                                else if ( rec_J > mov_J )
                                    rot_Direct = 0;
                                else if ( rec_J < mov_J )
                                    rot_Direct = 4;

                                point_Count ++;//ѹ���µ�֮ǰ�����������Լ�

                                if ( point_Count > contour_Point_Now-1 )
                                {
                                    //�������ռ�㳬����Χ���������������ռ��
                                    ptr_Contour_First = (  contour *)malloc( contour_Point_Now * sizeof( contour) );
                                    memcpy( ptr_Contour_First, *_TracingPtr, contour_Point_Now * sizeof( contour) );
                                    *_TracingPtr = (  contour *)malloc( (contour_Point_Now + contour_Point_Add) * sizeof( contour) );
                                    memcpy( *_TracingPtr, ptr_Contour_First, contour_Point_Now * sizeof( contour) );
                                    contour_Point_Now += contour_Point_Add;
                                    free( ptr_Contour_First);
                                    ptr_Contour_First = NULL;
                                    ptr_Contour_First = *_TracingPtr;
                                }

                                ptr_Contour_First[ point_Count].height = rec_I;//������������
                                ptr_Contour_First[ point_Count].width = rec_J;
                                ptr_Contour_First[ point_Count].stateFlag = 0;//״̬��־λstateFlag��
                                                                              //1Ϊ��ʼ�㣬0Ϊ��ͨ��Ե�㣬2Ϊ������
                                ptr_Contour_First[ point_Count].direction = rot_Direct;
                                _Mark[ rec_I*stepWidth + rec_J] = 1;//���Ϊ���������ĵ�
                            }
                            else if ( start_I == mov_I && start_J == mov_J )
                            {
                                //���������ʼ�㣬���֮ǰ���һ�����Ϊ�յ㣬������ʼ�µ�����
                                ptr_Contour_First[ point_Count].stateFlag = 2;//״̬��־λstateFlag��
                                                                              //1Ϊ��ʼ�㣬0Ϊ��ͨ��Ե�㣬2Ϊ������
                                contour_Succe_Flag = 1;//������������־λ��һ
                                point_Count ++;//Ϊ��һ����������һ���µ�
                                break;
                            }
                        }
                    }

                    if ( contour_Succe_Flag )//���һ����������������ɹ�����ʼ���µ���ʼ��
                    {
                        contour_Succe_Flag = 0;//���������ɹ���־λ����
                        break;
                    }
                }
            }
        }
    }
    ptr_Contour_First[0].stateFlag = point_Count;//�����������������ڵ�һ���ռ���״̬λ
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
    if (type == 0)      //�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x��ƽ��ֵ
            averageY = sumY / sumlines;     //y��ƽ��ֵ
        }
        else
        {
            averageX = 0;     //x��ƽ��ֵ
            averageY = 0;     //y��ƽ��ֵ
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
    else if (type == 1)//�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x��ƽ��ֵ
        averageY = sumY / sumlines;     //y��ƽ��ֵ
        for (i = startline; i < endline; i++)
        {
            //SetText("lefetline"+i+" " +lefetline[i] + " averageY" +" "+ averageY);
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;//�ؾ�
    }
    else if (type == 2)//�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x��ƽ��ֵ
        averageY = sumY / sumlines;     //y��ƽ��ֵ
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

void Image_Handle(void)//ͼ�����ܺ���
{
    // ����ͷ��ʼ��

    //char txt[16];
        CAMERA_Init(50);



        //TFTSPI_P8X16Str(2, 3, "LQ 9V034 Car", u16RED, u16GREEN);
        //TFTSPI_P8X16Str(1, 5, "K2 Show Video", u16RED, u16GREEN);
        delayms(500);

        // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
        mutexCpu0TFTIsOk = 0;             // CPU1�� 0ռ��/1�ͷ� TFT
        //CircleNumber = 2;    // ������Ҫ����Բ���ĸ�����

        // ��������Ҫ���ó���⣬�����ǹ̶�ִ�У�
        // �������Ҫ�ɻɹܺ��ⲿ�ж����ʵ��
        // ���������У��ɻɹ���ͨ��Բ������������������ܴ����Ŀ�����
       // OutInGarage(OUT_GARAGE, ReadOutInGarageMode()); // ���Գ��⣬��������������⣬��֮�Ҳ�����
        //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // �������

        //TFTSPI_CLS(u16BLUE);            // ����
        // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
        mutexCpu0TFTIsOk = 0;             // CPU1�� 0ռ��/1�ͷ� TFT

        //RAllPulse = 0;                  // ȫ�ֱ����������������
        NowTime = STM_GetNowUs(STM0);   // ��ȡSTM0 ��ǰʱ��
        while (1)
        {
            tftxian();
            LED_Ctrl(LED1, RVS);     // LED��˸ ָʾ��������״̬
            if (Camera_Flag == 2)
            {
                Camera_Flag = 0;     // �������ͷ�ɼ���ɱ�־λ  �����������򲻻��ٴβɼ�����
                Get_Use_Image();     // ȡ����������ʾ����ͼ������
                Get_Bin_Image(0);
               // Show_Use_Image_zhongxian();
                //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) zhongxian);// ת��Ϊ01��ʽ���ݣ�0��1ԭͼ��2��3������ȡ
                Bin_Image_Filter();  // �˲������汻Χ�����ݽ����޸�Ϊͬһ��ֵ
                //First_Line_Find();
                //check_starting_line();
                //check_line();
                /*for(line=58;line>0;line--)//ÿѭ��һ�εõ�һ��Left[line]��һ��Right[line]ѭ��������߽縳ֵ����
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
                        TFTSPI_CLS(u16BLACK);         // ����
                        //TFTSPI_P16x16Str(0,0,(unsigned char*)"stop!",u16RED,u16BLUE);
                    }
                }*/




                for (line = 58; line >0; line--)
                {
                    //���߲���
                    if (leftfindflag[line] == 0)       //δɨ����
                    {
                        lostleft_times++;
                        if (flag_of_leftbreak == 0)     //�������һ��֮ǰû���������ߣ������
                        {
                            l_start++;
                        }
                    }
                    else  if(leftfindflag[line-1] == 1)     //ɨ����
                    {
                        //lostleft_times����������
                        flag_of_leftbreak = 1;  //break��־����
                    }
                    //���߲���
                    if (rightfindflag[line] == 0)       //δɨ����
                    {
                        lostright_times++;
                        if (flag_of_rightbreak == 0)     //�������һ��֮ǰû���������ߣ������
                        {
                            r_start++;
                        }
                    }
                    else    //ɨ����
                    {
                        //lostright_times����������
                        flag_of_rightbreak = 1;  //break��־����
                    }
                }
                for (line =(l_start<20)? (59-l_start):59; line > 0; line--)
                {
                 if(Bin_Image[line-1][middle[line]]==0)
                 {
                     break_hangshu=line;
                     flag_poding=1;
                     if (break_hangshu <=38)    //��ֹ��һ��ʼ��break
                     {
                         break;
                     }
                 }
                 else if (abs(middle[line]-middle[line-1])>3)
                 {
                     break_hangshu = line;
                     //last_break_hangshu = break_hangshu;
                     //Ҳ����˵��ʮ��һ���ǲ���break��
                     if (break_hangshu <=38)    //��ֹ��һ��ʼ��break
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


                int curvity_point1 = ((60-r_start + break_hangshu) / 2);     //�е�
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
                //���ʽӽ�0˵����ֱͦ�ģ�Ϊ������ֱ�߷���С���ص㣬�ӵ�start�п�ʼ����
                 if (curvity_right > -0.4 && curvity_right < 0.1 && r_start <= 32 && (break_hangshu - r_start) >= 7)
                 {
                     myregression(2, break_hangshu, 60-r_start);    //�������
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
                //����˵���߽������ߣ���ʱΪ��͹�Գ����ߵķ������ص㣬�ӵ�0�п�ʼ����
                 else
                 {
                     myregression(2, break_hangshu, 60);    //�������
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
                             flag_zuoguaidian=1;//�����µ�
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
                             //MotorInit(); // ���
                             t=1;

                             //OutInGarage(0,1);

                            // Expectspeed=1;
                            // STM_InitConfig(STM0, STM_Channel_1,5000);//����ж�STM_InitConfig(STM0, STM_Channel_0,100);
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
//            TFTSPI_P8X16Str(2, 6, txt, u16BLACK, u16BLUE); // ��ʾ����ʵ�����������Ա�������
            flag_of_leftbreak=0;r_start=0;
            flag_of_rightbreak=0;l_start=0;
            flag_zuoguaidian=0;flag_poding=0;j=0;
        }
}

