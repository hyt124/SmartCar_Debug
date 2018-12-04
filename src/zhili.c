#include "angle.h"
#include "common.h"
#include "math.h"
#include "pwm.h"
#include "gpio.h"
#include "zhili.h"
#include "uart.h"
#include "pit.h"

extern unsigned char Protect_flag;
extern float g_fCarSpeed;

/*陀螺仪加速度计变量定义*/
#define GYRO_Z 1
#define ACC_Z  5

float ZMAX =  4096.0 ;//1750
float  ZMIN =  -4096.0; //450
unsigned int  g_nGyroXZero;      //
float         g_fXGyro,g_fAcc;  
float         g_fZGyro;//   方向控制的转向角度积分
/*卡尔曼滤波变量定义*/
float Angle = 0, Angle_dot = 0;  //卡尔曼以后的 角度  和角速度

float Q_angle=50, Q_gyro=0.05, R_angle=0.6, dt=0.005;
float P[2][2] ={{1,0},{0,1}};
	
float  C_0 = 1;//char  C_0 = 1;
float E = 0;  
float q_bias = 0;
float Angle_err = 0;
float PCt_0 = 0, PCt_1 = 0;                  
float K_0 = 0, K_1 = 0;
float t_0 = 0, t_1 = 0;
float Pdot[4] ={0,0,0,0};

/*角度控制变量定义*/
float AngleKP_set=90;   //75
float AngleKD_set=18;   //0.07
float AngleKI_set=-28;//30;       //  D
float Angle_set=10.5;   //设定倾斜角度 3.5

#define AngleKP      AngleKP_set
#define AngleKD      AngleKD_set
#define BalanceAngle   Angle_set


//#define BalanceAngle Angle_set

 void Angle_Calculate(void);

//extern unsigned int OutData[];

/*函数名称:void GyroX_Zero_Cal();
 *函数说明:计算陀螺仪的零漂值，多次采样取平均，在初始化时调用
 *参数:void
 *返回值:void
 */

//void GyroX_Zero_Cal(void)
//{
//    unsigned int i;
//    unsigned long lGyroSum = 0;
//    for(i = 0;i < 1024;i++)
//    {
//        lGyroSum += get_angle(GYRO_Z);
//    }
// 
//    g_nGyroXZero = ( unsigned int )(lGyroSum>>10);   
//}

/*函数名称:void Kalman_Filter();
 *函数说明:卡尔曼融合
 *参数:gyro_m 陀螺仪读值,angle_m 加速度读值
 *返回值:void
 */
void Kalman_Filter(float angle_m,float gyro_m)
{
	Angle += (gyro_m-q_bias) * dt; 
	
	Pdot[0]= Q_angle - P[0][1] - P[1][0];
	Pdot[1]= -P[1][1];
	Pdot[2]= -P[1][1];
	Pdot[3]= Q_gyro;
	
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
		
	Angle_err = angle_m - Angle;   
		
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;      
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;   
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
		
	Angle += K_0 * Angle_err;  	
	q_bias += K_1 * Angle_err;  
	Angle_dot = gyro_m-q_bias;  
}


/*
*  功能说明：清华角度滤波
*  参数说明：G_angle                       加速度计角度0-90内
*            Gyro                         陀螺仪角速度转花后的数值
*            GRAVITY_ADJUST_TIME_CONSTANT  时间校正系数
*            DT                             定时器时间 单位s
*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考清华源码
*/

//float g_fCarAngle = 0,g_fGyroscopeAngleIntegral;  
//# 
//void QingHua_AngleCalaulate(float G_angle,float Gyro)
//{
//    float fDeltaValue;

//    g_fCarAngle = g_fGyroscopeAngleIntegral;   //最终融合角度
//    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_CONSTANT;  //时间系数矫正
//    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //融合角度
//}

 void Angle_Calculate(void)
{
    short nGyro_AD,nAcc_AD,nGzro_AD;
   
    //DisableInterrupts;   //关闭中断
 
    nGyro_AD = get_angle(GYRO_Z); //Read_ADC_Ave(GYRO_Z,2);
    nAcc_AD = get_angle(ACC_Z);   //Read_ADC_Ave(ACC_Z,2);
    nGzro_AD=	get_angle(2);
   // EnableInterrupts;   //开总中断
    
    if(nAcc_AD >= ZMAX)  //Acc值进行限幅
    {
        nAcc_AD = ZMAX;
    }
    if(nAcc_AD <= ZMIN)
    {
        nAcc_AD = ZMIN;
    }
    
    g_fXGyro = (((float)nGyro_AD)*7.820)/1000.0 - (((float)-43.678)*7.820/1000.0);
    g_fAcc = -(asin((float)(nAcc_AD) /4096)*57.2958); 
		
    g_fZGyro= (((float)nGzro_AD)*7.820)/1000.0 - (((float)-43.678)*7.820/1000.0);
    
//    g_fAcc_tiaoling = asin((float)(nAcc_AD) /4096)*57.2958; 
//    g_fAcc=g_fAcc_tiaoling+16.100;

    //g_fAngle = Kalman_Filter(g_fXGyro,g_fAcc);
    Kalman_Filter(g_fAcc,g_fXGyro);

   // Kalman_Filter(Angle,Angle_dot);
//    #if   0
//        OutData[0] = (unsigned int)(nAcc_AD);
//        OutData[1] = (unsigned int)(nGyro_AD);
//        OutPut_Data();
//    #elif 0
//        OutData[0] = (unsigned int)(-g_fAcc);
//        OutData[1] = (unsigned int)(-g_fXGyro);
//        OutPut_Data();
//    #elif 0
//        OutData[0] = (unsigned int)(-g_fAcc);
//        OutData[1] = (unsigned int)(-g_fXGyro);
//        OutData[2] = (unsigned int)(Angle);
//        OutData[3] = (unsigned int)(Angle_dot);
//        OutPut_Data();             
//    #endif
}

/*函数名称:float Angle_Control(void);
 *函数说明:PD控制，保持直立
 *参数:void
 *返回值:o_fAngleMotor电机的脉冲数
 */
float Angle_Control(void)
{
    float o_fAngleMotor=0.0;
    float fAngleErr,fAngleErrold=0,fAngleErrold2=0,fAngleErr_I=0;
    
    Angle_Calculate();
    
//    if(Angle>30 || Angle<-30)  //进行角度限幅
//    {
//        Protect_flag=0;  
//    }
    
                            
     fAngleErr     =  Angle - BalanceAngle ;// fAngleErr = Angle - BalanceAngle;
     o_fAngleMotor =  (AngleKP * fAngleErr + AngleKD * (g_fXGyro));  //注意极性
     
     fAngleErrold2 =  fAngleErrold;
	   fAngleErrold  =  fAngleErr;
//        
//        if(g_fCarSpeed > 3)
//    {
//        o_fAngleMotor =o_fAngleMotor*1.8;
//    }

    return o_fAngleMotor;  
}
/*函数名称:void Set_Angle(float i_fAngleValue);
 *函数说明:设定小车的倾斜角
 *参数:倾斜角
 *返回值:void
 */
//void Set_Angle(float i_fAngleValue)
//{
//    Angle_set = i_fAngleValue;   
//}


