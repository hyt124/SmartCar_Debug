#include "readspeed.h"
#include "common.h"
#include "math.h"
#include "gpio.h"
#include "uart.h"
#include "speedcontrol.h"
#include "oled.h"
#include "pwm.h"

extern short int  speed1_flag;   //速度档位标志位
extern short int  speed2_flag;
extern short int  speed3_flag;
extern short int  speed4_flag;


extern int    valuer;    //左速参数
extern int    valuel;    //右速参数
extern unsigned char Protect_flag;
extern float  Angle_set; 
extern float AngleKP_set;

extern float  suduoutjifen;
//float suduout;   看速度输出用的
#define MY_ABS(x) ((x)>0?(x):(-(x)))
/*速度采集变量定义*/
int32_t g_nRightMotorPulse;
int32_t g_nLeftMotorPulse;
//word g_nRightMotorPulseSum;
//word g_nLeftMotorPulseSum;
//byte PulseReadPeriod=0;
float g_fCarSpeed;
#define OPTICAL_ENCODE_CONSTANT  512 //光电编码盘的刻槽数量
#define SPEED_CONTROL_PERIOD     100 //速度控制周期ms
//float g_fCarSpeedRatio;
#define CAR_SPEED_RATIO         (1000.0/SPEED_CONTROL_PERIOD/OPTICAL_ENCODE_CONSTANT)

/*速度控制变量定义*/
float SpeedKP_set=52;
float SpeedKI_set=0;  //50
float SpeedKD_set=84;

float Speed_set=1;    //调用修改速度设定值
#define SpeedKP   SpeedKP_set
#define SpeedKI   SpeedKI_set
#define SpeedKD   SpeedKD_set
#define SetSpeed  Speed_set
float g_bSpeedControlPeriod;
float g_fSpeedControlOutOld,g_fSpeedControlOutNew;
float g_fIntegral=0,g_fIntegralSum=0;
float g_fSpeedErrOld=0,g_fSpeedErrNew=0,g_fDiffSpeedErr=0;
float Delt_KP=0; 


// /*模糊变量定义*/
//#define E_BOUND_LENGTH  7
//#define DE_BOUND_LENGTH 7
//#define U_BOUND_LENGTH  13
//const float  E_Bound_Speed[E_BOUND_LENGTH]={-3, -2, -1, 0 ,1, 2, 3};  //error量
//const float  DE_Bound_Speed[DE_BOUND_LENGTH]={-3, -2, -1, 0, 1, 2, 3};    //d_error微分量
////float  U_Bound_Speed_P[U_BOUND_LENGTH]={-6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6};  //speed_kp值
//const float  U_Bound_Speed_P[U_BOUND_LENGTH]={-2.4, -2.0, -1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4};  //speed_kp值
//const char   Rule_Speed_P[7][7]=
//{   
//       //NB
// /*NB*/{ 6, 5, 4, 3, 2, 0, 0},
//       { 5, 4, 3, 2, 1, 0,-1},
//       { 5, 4, 3, 1, 0,-1,-2},
//       { 4, 3, 1, 0,-1,-3,-4},
//       { 2, 1, 0,-1,-2,-4,-5},
//       { 1, 0,-1,-2,-3,-3,-5},
//       { 0 ,0,-2,-3,-4,-5,-6}, 
//};
//byte SEn=0,SDEn=0;
//char SUn=0;

//extern word OutData[];
//extern float g_fAngleOutMotor;
//extern float g_fMotorOut;
//extern float Angle;
//extern byte Protect_flag;
//extern word StartTime;
//extern float DirKP_set;
//extern float SpeedValue,AngleValue;

/*函数名称:void Pulse_Read();
 *函数说明:计算左右电机脉冲,100ms调用一次
 *参数:void
 *返回值:void
 */
void Pulse_Read(void)
{
    
    readspeed(); 
    g_nLeftMotorPulse  = valuel;
    g_nRightMotorPulse = valuer;
      
//      OLed_DisplayF(0,2,valuer);   
//      OLed_DisplayF(0,3,valuel);
    
//    g_nLeftMotorPulse = (word)cd4520_cnt();
//    g_nRightMotorPulse = PT7_cnt();
//    g_nRightMotorPulseSum += g_nRightMotorPulse;
//    g_nLeftMotorPulseSum += g_nLeftMotorPulse;
//    PulseReadPeriod++;
}

/*函数名称:float Change_Integral_Cal(float i_fNowErr);
 *函数说明:计算变速积分比例
 *参数:当前误差
 *返回值:变速积分值比例       //误差大的时候不积分
 */
float Change_Integral_Cal(float i_fNowErr)
{
    static float A=0.5,B=0.99;
    float ret,PErr;
    PErr = MY_ABS(i_fNowErr); 
    if( PErr <= B )
    {
        ret = 1;   
    }
    else if( PErr > B && PErr <= (A+B) )
    {
        ret = (A+B-PErr)/A;
    }
    else if( PErr > (A+B) )
    {
        ret = 0;
    }
    
    return ret;
} 

/*函数名称:float Speed_Control_PID(float,float);
 *函数说明:增量式PID控制
 *参数:i_fCarSpeedSet 速度设定， i_fCarSpeed 当前速度
 *返回值:fIncPID 控制增量
 */
float Speed_Control_PID(float i_fCarSpeedSet,float i_fCarSpeed)
{
    float o_fIncPID,fNowErr,fI;
    static float fLastErr,fPrevErr;
	
    fNowErr = i_fCarSpeedSet - i_fCarSpeed;
	  fI = fNowErr * SpeedKI;
	  g_fIntegral += fI; 

	
	if(speed1_flag==1)
	    {		
	//		// 1.1 m/s  zuoyou   7.7   C  超稳定   不许动                                                     -1+ 0.05*g_fCarSpeed;     -1.2 - 0.05*g_fCarSpeed;   g_fIntegral=0;
	   if  (g_fCarSpeed >9)
                {  SpeedKI_set =0;    SpeedKP_set=86;  SpeedKD_set=-300; Speed_set=8;        Angle_set=-13.2;                     AngleKP_set=89; g_fIntegral=g_fIntegral/18;}
     else if(g_fCarSpeed <4)
                {  SpeedKI_set =0.08;     SpeedKP_set=73;   SpeedKD_set=-330; Speed_set=6;     Angle_set=-12.8;                       AngleKP_set=86;}
     else 
                {  SpeedKI_set =0.05;    SpeedKP_set=76;   SpeedKD_set=-315; Speed_set=7;    Angle_set=-12.8;                     AngleKP_set=88;}
								
			}
	if(speed2_flag==1)
	    {		
	//		// 1.1 m/s  zuoyou   7.7   C  超稳定   不许动                                                     -1+ 0.05*g_fCarSpeed;     -1.2 - 0.05*g_fCarSpeed;   g_fIntegral=0;
	   if  (g_fCarSpeed >9)
                {  SpeedKI_set =0;    SpeedKP_set=83;  SpeedKD_set=-440; Speed_set=9;        Angle_set=-13.1;                     AngleKP_set=93; g_fIntegral=g_fIntegral/18;}
     else if(g_fCarSpeed <4)
                {  SpeedKI_set =0.15;     SpeedKP_set=79;   SpeedKD_set=-480; Speed_set=7;     Angle_set=-12.7;                       AngleKP_set=89;}
     else 
                {  SpeedKI_set =0.08;    SpeedKP_set=81;   SpeedKD_set=-460; Speed_set=8;    Angle_set=-12.7;                     AngleKP_set=91;}
							
			}
	if(speed3_flag==1)
	    {		
	//		// 1.1 m/s  zuoyou   7.7   C  超稳定   不许动                                                     -1+ 0.05*g_fCarSpeed;     -1.2 - 0.05*g_fCarSpeed;   g_fIntegral=0;
	   if  (g_fCarSpeed >11)
                {  SpeedKI_set =0;    SpeedKP_set=83;  SpeedKD_set=-340; Speed_set=10;        Angle_set=-13.1;                     AngleKP_set=92; g_fIntegral=g_fIntegral/18;}
     else if(g_fCarSpeed <5)
                {  SpeedKI_set =0.3;     SpeedKP_set=66;   SpeedKD_set=-380; Speed_set=8;     Angle_set=-12.7;                       AngleKP_set=86;}
     else 
                {  SpeedKI_set =0.2;    SpeedKP_set=81;   SpeedKD_set=-360; Speed_set=9;    Angle_set=-12.7;                     AngleKP_set=88;}
								
			}			
	if(speed4_flag==1)
	    {		
		//		// 1.1 m/s  zuoyou   7.7   C  超稳定   不许动                                                     -1+ 0.05*g_fCarSpeed;     -1.2 - 0.05*g_fCarSpeed;   g_fIntegral=0;
	   if  (g_fCarSpeed >14)
                {  SpeedKI_set =0;    SpeedKP_set=83;  SpeedKD_set=-400; Speed_set=13;        Angle_set=-16.8;                     AngleKP_set=93; g_fIntegral=g_fIntegral/18;}
     else if(g_fCarSpeed <7)
                {  SpeedKI_set =0.2;     SpeedKP_set=66;   SpeedKD_set=-430; Speed_set=11;     Angle_set=-16.3;                       AngleKP_set=90;}
     else 
                {  SpeedKI_set =0.1;    SpeedKP_set=81;   SpeedKD_set=-415; Speed_set=12;    Angle_set=-16.3;                     AngleKP_set=88;}
				
			}



  if(g_fIntegral>1000) g_fIntegral=1000;
  if(g_fIntegral<-1000) g_fIntegral=-1000;
  

        
    o_fIncPID = SpeedKP*(fNowErr) + g_fIntegral + SpeedKD*(fNowErr-fLastErr);
    fPrevErr = fLastErr;
    fLastErr = fNowErr;
    return o_fIncPID;     
}

/*函数名称:void Speed_Control(void);
 *函数说明:速度控制100ms调用
 *参数:void
 *返回值:void
 */
void Speed_Control(void)
{
    float fSpeedDelt;
    int nLeftMotorPulse,nRightMotorPulse;
    static float a=0.2; 
    
    Pulse_Read();
    nLeftMotorPulse    =  g_nLeftMotorPulse;
    g_nLeftMotorPulse  =  a*g_nLeftMotorPulse + (1-a)*nLeftMotorPulse;
    nRightMotorPulse   =  g_nRightMotorPulse;
    g_nRightMotorPulse =  a*g_nRightMotorPulse + (1-a)*nRightMotorPulse;//ditonglvbo
    
    g_fCarSpeed = (float)(g_nLeftMotorPulse + g_nRightMotorPulse)/2/60;//*CAR_SPEED_RATIO;
    g_fSpeedErrNew = Speed_set - g_fCarSpeed;
    g_fDiffSpeedErr = g_fSpeedErrNew - g_fSpeedErrOld;
    g_fSpeedErrOld = g_fSpeedErrNew;
    
//    if(StartTime>=300)
//    {    
//        //Delt_KP = Fuzzy_Control_Speed(g_fSpeedErrNew,g_fDiffSpeedErr,E_Bound_Speed,DE_Bound_Speed,U_Bound_Speed_P);
//    }
//    SpeedKP_set = SpeedKP_set+Delt_KP;
    /*if(SpeedKP_set > 28)
    {
        SpeedKP_set = 28;    
    }
    if(SpeedKP_set < 20)
    {
        SpeedKP_set = 20;
    } */
    fSpeedDelt = Speed_Control_PID(Speed_set,g_fCarSpeed);
    g_fSpeedControlOutOld = g_fSpeedControlOutNew;
    g_fSpeedControlOutNew = fSpeedDelt;
    

    
//        if(g_fCarSpeed < 5)
//    {
//        Protect_flag=1;
//    }
//    
//    if(g_fCarSpeed > 30)
//    {
//        Set_Angle(AngleValue - 0.05*g_fCarSpeed);
//        //Set_Angle(-9);    
//    }
//    else
//    {
//        Set_Angle(AngleValue);    
//    }
    
    
    
    //清零
    //PulseReadPeriod = 0;
    g_nLeftMotorPulse = 0;
    g_nRightMotorPulse = 0;
    g_bSpeedControlPeriod = 0;
}

/*函数名称:void Speed_Control_Output(void);
 *函数说明:对PID的控制量进行分段输出    1/20  ........
 *参数:void
 *返回值:o_fSpeedControlOut 分段PWM控制量
 */
float Speed_Control_Output(void)
{
    float fValue,o_fSpeedControlOut;
     
    fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld; 
    o_fSpeedControlOut = fValue * (g_bSpeedControlPeriod + 1)/20 + g_fSpeedControlOutOld;
    g_bSpeedControlPeriod++;
    if(g_bSpeedControlPeriod >= 20)
    {                           
        g_bSpeedControlPeriod = 0;
    }
		
		
    if(o_fSpeedControlOut>1000) {o_fSpeedControlOut=1000;}
		if(o_fSpeedControlOut<-1000){o_fSpeedControlOut=-1000;}

		 //suduout=o_fSpeedControlOut;   看输出用的
    return o_fSpeedControlOut;
}
//*===================================================================================
///*函数名称:void Set_Speed(float i_fSpeedValue,float i_fSpeedKP);//模糊算法
// *函数说明:设定小车的速度,和KP值
// *参数:速度设定值，KP值
// *返回值:void
// */
//void Set_Speed(float i_fSpeedValue,float i_fSpeedKP,float i_fSpeedKI,float i_fSpeedKD)
//{
//    Speed_set = i_fSpeedValue;
//    SpeedKP_set = i_fSpeedKP;
//    SpeedKI_set = i_fSpeedKI;
//    SpeedKD_set = i_fSpeedKD;   
//}
//void Set_SpeedValue(float i_fSpeedValue)
//{
//    Speed_set = i_fSpeedValue;  
//}

//float Fuzzy_Control_Speed(float e,float ec,const float * E_Bound,const float * DE_Bound,const float * U_Bound)
//{
//    byte i;
//    float u;
//    if(e>=E_Bound[0] && e<=E_Bound[E_BOUND_LENGTH-1])
//    {
//        for(i=0;i<E_BOUND_LENGTH;i++)
//        {
//            if(e <= E_Bound[i])
//            {
//                SEn=i;
//                break;
//            }
//        }
//    }
//    else if(e<=E_Bound[0])
//    {
//        SEn=0;   
//    }
//    else if(e>=E_Bound[E_BOUND_LENGTH-1])
//    {
//        SEn=E_BOUND_LENGTH-1;   
//    }
//    
//    if(ec>=DE_Bound[0] && ec<=DE_Bound[DE_BOUND_LENGTH-1])
//    {
//        for(i=0;i<DE_BOUND_LENGTH;i++)
//        {
//            if(ec <= DE_Bound[i])
//            {
//                SDEn=i;
//                break;
//            }
//        }
//    }
//    else if(ec<=DE_Bound[0])
//    {
//        SEn=0;   
//    }
//    else if(ec>=DE_Bound[DE_BOUND_LENGTH-1])
//    {
//        SEn=DE_BOUND_LENGTH-1;   
//    }
//    SUn = Rule_Speed_P[SEn][SDEn];
//    u = U_Bound[(byte)(SUn+6)]; 
//    return u;
//}
