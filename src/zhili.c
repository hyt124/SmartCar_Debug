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

/*�����Ǽ��ٶȼƱ�������*/
#define GYRO_Z 1
#define ACC_Z  5

float ZMAX =  4096.0 ;//1750
float  ZMIN =  -4096.0; //450
unsigned int  g_nGyroXZero;      //
float         g_fXGyro,g_fAcc;  
float         g_fZGyro;//   ������Ƶ�ת��ǶȻ���
/*�������˲���������*/
float Angle = 0, Angle_dot = 0;  //�������Ժ�� �Ƕ�  �ͽ��ٶ�

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

/*�Ƕȿ��Ʊ�������*/
float AngleKP_set=90;   //75
float AngleKD_set=18;   //0.07
float AngleKI_set=-28;//30;       //  D
float Angle_set=10.5;   //�趨��б�Ƕ� 3.5

#define AngleKP      AngleKP_set
#define AngleKD      AngleKD_set
#define BalanceAngle   Angle_set


//#define BalanceAngle Angle_set

 void Angle_Calculate(void);

//extern unsigned int OutData[];

/*��������:void GyroX_Zero_Cal();
 *����˵��:���������ǵ���Ưֵ����β���ȡƽ�����ڳ�ʼ��ʱ����
 *����:void
 *����ֵ:void
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

/*��������:void Kalman_Filter();
 *����˵��:�������ں�
 *����:gyro_m �����Ƕ�ֵ,angle_m ���ٶȶ�ֵ
 *����ֵ:void
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
*  ����˵�����廪�Ƕ��˲�
*  ����˵����G_angle                       ���ٶȼƽǶ�0-90��
*            Gyro                         �����ǽ��ٶ�ת�������ֵ
*            GRAVITY_ADJUST_TIME_CONSTANT  ʱ��У��ϵ��
*            DT                             ��ʱ��ʱ�� ��λs
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
* ��ע���ο��廪Դ��
*/

//float g_fCarAngle = 0,g_fGyroscopeAngleIntegral;  
//# 
//void QingHua_AngleCalaulate(float G_angle,float Gyro)
//{
//    float fDeltaValue;

//    g_fCarAngle = g_fGyroscopeAngleIntegral;   //�����ںϽǶ�
//    fDeltaValue = (G_angle - g_fCarAngle) / GRAVITY_CONSTANT;  //ʱ��ϵ������
//    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;                //�ںϽǶ�
//}

 void Angle_Calculate(void)
{
    short nGyro_AD,nAcc_AD,nGzro_AD;
   
    //DisableInterrupts;   //�ر��ж�
 
    nGyro_AD = get_angle(GYRO_Z); //Read_ADC_Ave(GYRO_Z,2);
    nAcc_AD = get_angle(ACC_Z);   //Read_ADC_Ave(ACC_Z,2);
    nGzro_AD=	get_angle(2);
   // EnableInterrupts;   //�����ж�
    
    if(nAcc_AD >= ZMAX)  //Accֵ�����޷�
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

/*��������:float Angle_Control(void);
 *����˵��:PD���ƣ�����ֱ��
 *����:void
 *����ֵ:o_fAngleMotor�����������
 */
float Angle_Control(void)
{
    float o_fAngleMotor=0.0;
    float fAngleErr,fAngleErrold=0,fAngleErrold2=0,fAngleErr_I=0;
    
    Angle_Calculate();
    
//    if(Angle>30 || Angle<-30)  //���нǶ��޷�
//    {
//        Protect_flag=0;  
//    }
    
                            
     fAngleErr     =  Angle - BalanceAngle ;// fAngleErr = Angle - BalanceAngle;
     o_fAngleMotor =  (AngleKP * fAngleErr + AngleKD * (g_fXGyro));  //ע�⼫��
     
     fAngleErrold2 =  fAngleErrold;
	   fAngleErrold  =  fAngleErr;
//        
//        if(g_fCarSpeed > 3)
//    {
//        o_fAngleMotor =o_fAngleMotor*1.8;
//    }

    return o_fAngleMotor;  
}
/*��������:void Set_Angle(float i_fAngleValue);
 *����˵��:�趨С������б��
 *����:��б��
 *����ֵ:void
 */
//void Set_Angle(float i_fAngleValue)
//{
//    Angle_set = i_fAngleValue;   
//}


