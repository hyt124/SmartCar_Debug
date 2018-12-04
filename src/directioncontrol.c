#include "common.h"
#include <math.h>
#include "gpio.h"
#include "uart.h"
#include "oled.h"
#include "pwm.h"
#include <stdlib.h>
#include "adc.h"
#include "ADCread.h"
#include "directioncontrol.h"

extern short int  speed1_flag;   //�ٶȵ�λ��־λ
extern short int  speed2_flag;
extern short int  speed3_flag;
extern short int  speed4_flag;
//extern short int  smallS_flag;
//extern unsigned int    SSt;

extern int32_t    valuer;    //���ٲ���
extern int32_t    valuel;    //���ٲ���

extern unsigned int A,B,C,D,EE,FF ;   //��Ÿ�Ӧ��ADC
extern unsigned int A1,B1,C1,D1,EE1,FF1,  A2,B2,C2,D2,EE2,FF2,  A3,B3,C3,D3,EE3,FF3,  A4,B4,C4,D4,EE4,FF4;;

int   Hx=0,Hs=0,H2=0,H_use=0,H_old=0,H_err=0,shizi=0;
char  H_uselook;
extern float Angle,g_fZGyro ;
extern float  Angle_set, SpeedKI_set ; 
float g_fZGyrold;
/*********************????*******************/
/*����ɼ���������*/
//#define  GYRO_Y 2
//float    g_nGyroYZero=0;
//float    g_fYGyro=0;
//byte     send_data_cnt;
int     g_bMidPoint; 

/*������Ʊ�������*/
float   DirKP_set=4.8;
float   DirKD_set=45.8;
	

#define DirKP DirKP_set
#define DirKD DirKD_set
//����4���������������ı�
int    MID_LINE=-2;

int    g_bDirControlPeriod;    
float  g_fDirControlOutOld,g_fDirControlOutNew,fangxiangoutjifen;

int    g_fMidErrorOld=0,g_fMidErrorNew=0,Errorn_errorn_1=0;

//byte    send_flag=0; 
//float   g_fDirAngle=0.0;

 /*ģ����������*/
#define E_BOUND_LENGTH  7
#define DE_BOUND_LENGTH 7
#define U_BOUND_LENGTH  13
const int  E_Bound_Dir[E_BOUND_LENGTH]={-80, -50, -20, 0 ,18, 46, 70};  //error?																										//���һ��
const int  DE_Bound_Dir[DE_BOUND_LENGTH]={-25, -18, -10, 0, 8, 15, 20};    //d_error???

// const float U_stright_P[U_BOUND_LENGTH]={5.4, 5.3, 5.1, 5.0, 4.9, 4.8, 4.6, 4.8, 4.9, 5.0, 5.1, 5.3, 5.4};    //�õ���һ��
//const float U_smallS_P[U_BOUND_LENGTH]={6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8};    //�õ���һ��

//const float OU_stright_P[U_BOUND_LENGTH]={5.4, 5.3, 5.1, 5.0, 4.9, 4.8, 4.6, 4.8, 4.9, 5.0, 5.1, 5.3, 5.4};    //һ���ٶ�
//const float OU_smallS_P[U_BOUND_LENGTH]={5.3, 5.1, 5.0, 4.9, 4.8, 4.6,4.4,4.6, 4.8, 4.9, 5.0, 5.1, 5.3};    //�õ���һ��
//const float OU_bigS_P[U_BOUND_LENGTH]={5.6, 5.4, 5.2, 5.0, 4.9, 4.8, 4.6, 4.8, 4.9, 5.0, 5.2, 5.4, 5.6};    //�õ���һ��

//const float TU_stright_P[U_BOUND_LENGTH]={6.0, 5.8, 5.6, 5.4, 5.2, 5.0, 4.8, 5.0, 5.2, 5.4, 5.6, 5.8, 6.0};    //�����ٶ�
//const float TU_smallS_P[U_BOUND_LENGTH]={ 5.8, 5.6, 5.4, 5.2, 5.0, 4.8,4.6,4.8, 5.0, 5.2, 5.4, 5.6, 5.8};       //�õ���һ��
//const float TU_bigS_P[U_BOUND_LENGTH]={6.2, 6.0, 5.8, 5.6, 5.4, 5.2, 5.0, 5.2, 5.4, 5.6, 5.8, 6.0, 6.2};    //�õ���һ��

//const float SU_stright_P[U_BOUND_LENGTH]={6.4, 6.2, 6.0, 5.9, 5.8, 5.6,5.4,5.6,5.8, 5.9, 6.0, 6.2, 6.4};    //�����ٶ�
//const float SU_smallS_P[U_BOUND_LENGTH]={6.2, 6.0, 5.8, 5.6, 5.4, 5.2, 5.0, 5.2, 5.4, 5.6, 5.8, 6.0, 6.2};     //�õ���һ��
//const float SU_bigS_P[U_BOUND_LENGTH]={6.6, 6.4, 6.2, 6.0, 5.9, 5.8, 5.6, 5.8, 5.9, 6.0, 6.2, 6.4, 6.6};     //�õ���һ��


//const float FU_stright_P[U_BOUND_LENGTH]={6.2, 6.0, 5.8, 5.6, 5.4, 5.2, 5.0, 5.2, 5.4, 5.6, 5.8, 6.0, 6.2};       //�õ���һ��
//const float FU_smallS_P[U_BOUND_LENGTH]={6.0, 5.8, 5.6, 5.4, 5.2, 5.0, 4.8, 5.0, 5.2, 5.4, 5.6, 5.8, 6.0};     //�õ���һ��
//const float FU_bigS_P[U_BOUND_LENGTH]={6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8};    //�õ���һ��

const float OU_stright_P[U_BOUND_LENGTH]={6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8};       //�����ٶ�             			//���һ��
const float OU_smallS_P[U_BOUND_LENGTH]={7.0,6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8,7.0};     //�õ���һ��
const float OU_bigS_P[U_BOUND_LENGTH]={7.4,7.2,7.0,6.8, 6.6, 6.4, 6.2, 6.4, 6.6, 6.8,7.0,7.2,7.4};         //�õ���һ��

const float TU_stright_P[U_BOUND_LENGTH]={6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8};        //�����ٶ�
const float TU_smallS_P[U_BOUND_LENGTH]={7.2,7.0,6.8, 6.6, 6.4, 6.2, 6.0, 6.2, 6.4, 6.6, 6.8,7.0,7.2};     //�õ���һ��
const float TU_bigS_P[U_BOUND_LENGTH]={7.6,7.4,7.2,7.0,6.8, 6.6, 6.4, 6.6, 6.8,7.0,7.2,7.4,7.6};         //�õ���һ��

const float SU_stright_P[U_BOUND_LENGTH]={7.0,6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8,7.0};         //�����ٶ�
const float SU_smallS_P[U_BOUND_LENGTH]={7.8,7.6,7.4,7.2,7.0,6.8, 6.6, 6.8,7.0,7.2,7.4,7.6,7.8};      //�õ���һ��
const float SU_bigS_P[U_BOUND_LENGTH]={7.6,7.4,7.2,7.0,6.8, 6.6, 6.4, 6.6, 6.8,7.0,7.2,7.4,7.6};         //�õ���һ����

const float FU_stright_P[U_BOUND_LENGTH]={7.2,7.0,6.8, 6.6, 6.4, 6.2, 6.0, 6.2, 6.4, 6.6, 6.8,7.0,7.2};      //�����ٶ�
const float FU_smallS_P[U_BOUND_LENGTH]={7.4,7.2,7.0,6.8, 6.6, 6.4, 6.2, 6.4, 6.6, 6.8,7.0,7.2,7.4};       //�õ���һ��
const float FU_bigS_P[U_BOUND_LENGTH]={7.8,7.6,7.4,7.2,7.0,6.8, 6.6, 6.8,7.0,7.2,7.4,7.6,7.8};           //�õ���һ��


//const float stright_tobigS_P[U_BOUND_LENGTH]={8.0,7.8,7.6,7.4,7.2,7.0,6.8,7.0,7.2,7.4,7.6,7.8,8.0};          //�õ���һ��
//const float stright_tobigS_P[U_BOUND_LENGTH]={8.2,8.0,7.8,7.6,7.4,7.2,7.0,7.2,7.4,7.6,7.8,8.0,8.2};          //�õ���һ��
const float stright_tobigS_P[U_BOUND_LENGTH]={8.4,8.2,8.0,7.8,7.6,7.4,7.2,7.4,7.6,7.8,8.0,8.2,8.4};          //�õ���һ��


//const float FU_stright_P[U_BOUND_LENGTH]={6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8};       //�õ���һ��
//const float FU_smallS_P[U_BOUND_LENGTH]={6.6, 6.4, 6.2, 6.0, 5.9, 5.8, 5.6, 5.8, 5.9, 6.0, 6.2, 6.4, 6.6};    //�õ���һ��
//const float FU_bigS_P[U_BOUND_LENGTH]={7.0,6.8, 6.6, 6.4, 6.2, 6.0, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8,7.0};    //�õ���һ��




float what_P[U_BOUND_LENGTH]={4.7, 4.5, 4.3, 4.0, 3.8, 3.6, 3.4, 3.6, 3.8, 4.0, 4.3, 4.5, 4.7}; 

//const int  Rule_Dir_P[7][7]=
//{
//    {-6,-5,-4,-4,-2, 1, 2}, //e?big?,de?zo,?P ???3,??5
//    {-5,-5,-3,-2,-1, 0, 1},
//    {-4,-3,-1,-1, 0, 1, 2},
//    {-3,-2,-1, 0, 1, 2, 3},
//    {-2,-1, 0, 1, 1, 3, 4},
//    {-1, 0, 1, 2, 3, 4, 5},
//    {-2,-1, 2, 4, 4, 5, 6}, 
//};

const int  Rule_Dir_P[7][7]=
{
    {-6,-5,-4,-4,-2, 1, 2}, //e?big?,de?zo,?P ???3,??5     ///���һ��
    {-6,-5,-3,-2,-1, 0, 1},
    {-4,-3, 0, 0, 0, 1, 2},
    {-3,-2, 0, 0, 0, 2, 3},
    {-2,-1, 0, 0, 0, 3, 4},
    {-1, 0, 1, 2, 3, 4, 6},
    {-2,-1, 2, 4, 4, 5, 6}, 
};

int En,DEn;
int Un;

//extern word OutData[];
//extern byte PixelLeft[128],PixelRight[128];
//extern byte Protect_flag;
//extern float g_fCarSpeed;
//extern byte Block_flag;

int Fuzzy_Control_Dir(int e,int ec,const int * E_Bound,const int * DE_Bound,const float * what_P);

																																																															//	/*????:void GyroY_Zero_Cal();
																																																															//	 *������������Ưֵ
																																																															//	 *??:void
																																																															//	 *???:void
																																																															//	 */
																																																															//	void GyroY_Zero_Cal(void)
																																																															//	{
																																																															//			word i;
																																																															//			dword lGyroSum = 0;
																																																															//			for(i = 0;i < 1024;i++)
																																																															//			{
																																																															//					lGyroSum += get_adc0_once(GYRO_Y);
																																																															//			}
																																																															//			g_nGyroYZero = (word)(lGyroSum>>10);   
																																																															//	}

/*????:float Direction_Control_PID(byte g_fMidLineErr);
 *λ��ʽPID�������
 *??:g_fMidLineErr ??????
 *???:float
 */
float Direction_Control_PID(int g_fMidLineErr,int Errorn_errorn_1 )
{    
    float o_fLocPID;
    
   /* if(g_fMidLineErr < 1 && g_fMidLineErr > -1)
    {
        i_bMidLineErr = 0;
    }*/
	
	
    o_fLocPID = DirKP*g_fMidLineErr + DirKD*(Errorn_errorn_1);  //
    return o_fLocPID;
}

void Direction_Control(void)
{
    ave();    //���ô�����ֵ
	  Hx  =  (    ((int)(278*(3*(A2-B2)+6*(C2-D2)+2*(EE2-FF2))))  / ( (int)(10*(A2+B2)+6*(C2+D2)+2*(EE2+FF2)))  ); //������Ϲ�ʽ ʮ����
   	Hs =  (    ((int)(268*(1*(A2-B2)+13*(C2-D2)+2*(EE2-FF2))))  / ( (int)(10*(A2+B2)+13*(C2+D2)+2*(EE2+FF2)))  ); //������  
			
//  H2 =  (int)(10*(A2+B2)+2*(C2+D2)+12*(EE2+FF2)); 
		

    if(EE2>70&& FF2>70 && (EE2-FF2<110||FF2-EE2<110)){		H_use = Hx; H_uselook=0; }      //ʮ�ּ��   ����ģʽ�л�   ���Ĳ�׼  ����2��ģʽ��������   Angle_set=-8.5;
		else {H_use = Hs; H_uselook=1;}
		

		
	  H_err = H_use - H_old;          //�������΢��  
	  
	  if(H_err>38||H_err<-38){ H_old = H_old;}    //������΢�ֹ���  ����ǰһ�ε����
	  else {H_old = H_use;}

		if(H_err>38||H_err<-38){ g_bMidPoint = H_old;}  //   ������ʹ�õ����ֵ  �����������PID
		else {g_bMidPoint  = H_use;}
		
    
    g_fMidErrorNew  = g_bMidPoint - MID_LINE ;       //��ʹ�õ�����ٴν���΢��    ����PID�㷨���β�
    Errorn_errorn_1 = g_fMidErrorNew - g_fMidErrorOld;
    g_fMidErrorOld  = g_fMidErrorNew;
     
		    if(speed1_flag==1)   //1���ٶ�����
	   	  {
						 if(fangxiangoutjifen<= 50 && fangxiangoutjifen>= -50)
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =OU_stright_P[i];        
								}
								DirKD_set = DirKP_set*33;
								if((valuer+valuel)>2000){DirKD_set = DirKD_set*1.2;}
								
						  if(  valuer>1333 && valuel>1333 )
								{
								 for(i=0;i<13;i++) 
								  {
										what_P[i] =stright_tobigS_P[i];        
								  }
								}
								
								//if(H_uselook==0){DirKD_set = DirKD_set/8;}
								
								
						} 
					
						 if ((fangxiangoutjifen<130 && fangxiangoutjifen>50) ||  (fangxiangoutjifen>-130 && fangxiangoutjifen<-50))
						{  

								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =OU_smallS_P[i];        
								}
								DirKD_set = DirKP_set*30;
								if((valuer+valuel)>1300){DirKD_set = DirKD_set*1.8;}
								//if(H_uselook==0){DirKD_set = DirKD_set/8;}

						} 
				   	if(fangxiangoutjifen>=130||fangxiangoutjifen<=-130)		
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =OU_bigS_P[i];        
								}
								DirKD_set = DirKP_set*50;
                  if((valuer+valuel)>1600){DirKD_set = DirKD_set*2.2;}
									SpeedKI_set=1;
						
						} 
					}
				if(speed2_flag==1)   //2���ٶ�
	   	  {
						 if(fangxiangoutjifen<= 50 && fangxiangoutjifen>= -50)
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =TU_stright_P[i];        
								}
								DirKD_set = DirKP_set*38;
								if((valuer+valuel)>2000){DirKD_set = DirKD_set*1.1;}
								if((valuer+valuel)>2200){DirKD_set = DirKD_set*1.2;}
								
							 if(  valuer>1333 && valuel>1333 )
								{
								 for(i=0;i<13;i++) 
								  {
										what_P[i] =stright_tobigS_P[i];        
								  }
								}
						} 
					
						 if ((fangxiangoutjifen<130 && fangxiangoutjifen>50) ||  (fangxiangoutjifen>-130 && fangxiangoutjifen<-50))
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =TU_smallS_P[i];        
								}
								DirKD_set = DirKP_set*42;
								if((valuer+valuel)>1800){DirKD_set = DirKD_set*1.9;}
						} 
				   	if(fangxiangoutjifen>=130||fangxiangoutjifen<=-130)		
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =TU_bigS_P[i];        
								}
								DirKD_set = DirKP_set*68;
								SpeedKI_set=1;
						
						} 
					}
				if(speed3_flag==1)   //3���ٶ�
	   	  {
						 if(fangxiangoutjifen<= 50 && fangxiangoutjifen>= -50)
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =SU_stright_P[i];        
								}
								DirKD_set = DirKP_set*36;
								if((valuer+valuel)>2000){DirKD_set = DirKD_set*1.3;}
								if((valuer+valuel)>2200){DirKD_set = DirKD_set*1.6;}
								
								 if(  valuer>1333 && valuel>1333 )
								{
								 for(i=0;i<13;i++) 
								  {
										what_P[i] =stright_tobigS_P[i];        
								  }
								}
						} 
					
						 if ((fangxiangoutjifen<130 && fangxiangoutjifen>50) ||  (fangxiangoutjifen>-130 && fangxiangoutjifen<-50))
						{  


									char i;
									for(i=0;i<13;i++) 
									{
											what_P[i] =SU_smallS_P[i];        
									}
									DirKD_set = DirKP_set*43;
									
									if((valuer+valuel)>2000){DirKD_set = DirKD_set*2.2;}
							
						} 
				   	if(fangxiangoutjifen>=130||fangxiangoutjifen<=-130)		
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =SU_bigS_P[i];        
								}
								DirKD_set = DirKP_set*45;
							//if(fangxiangoutjifen>=320||fangxiangoutjifen<=-320)			{DirKD_set = DirKD_set*1.5;}
							
						
						} 
					}
				
		    if(speed4_flag==1)   //4���ٶ����
	   	  {
						 if(fangxiangoutjifen<= 50 && fangxiangoutjifen>= -50)
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =SU_stright_P[i];        
								}
								DirKD_set = DirKP_set*36;
								if((valuer+valuel)>2000){DirKD_set = DirKD_set*1.3;}
								//if((valuer+valuel)>2200){DirKD_set = DirKD_set*1.4;}
								
								 if(  valuer>1433 && valuel>1433 )
								{
								 for(i=0;i<13;i++) 
								  {
										what_P[i] =stright_tobigS_P[i]; 
                    DirKD_set = DirKD_set*1.2;		
										SpeedKI_set=0;
								  }
								}
								SpeedKI_set=0.1;
							
								
						} 
					
						 if ((fangxiangoutjifen<130 && fangxiangoutjifen>50) ||  (fangxiangoutjifen>-130 && fangxiangoutjifen<-50))
						{  


									char i;
									for(i=0;i<13;i++) 
									{
											what_P[i] =SU_smallS_P[i];        
									}
									DirKD_set = DirKP_set*45;
									
									if((valuer+valuel)>2000){DirKD_set = DirKD_set*2.2;}
									SpeedKI_set=0.5;
							
						} 
				   	if(fangxiangoutjifen>=130||fangxiangoutjifen<=-130)		
						{  
								char i;
								for(i=0;i<13;i++) 
								{
										what_P[i] =SU_bigS_P[i];        
								}
								DirKD_set = DirKP_set*47;
							//if(fangxiangoutjifen>=320||fangxiangoutjifen<=-320)			{DirKD_set = DirKD_set*1.5;}
							  SpeedKI_set=2;
			//			
						} 
					}
				
					
		
		DirKP_set = Fuzzy_Control_Dir(H_use,Errorn_errorn_1,E_Bound_Dir,DE_Bound_Dir,what_P);//ģ������KP
					
    g_fDirControlOutOld = g_fDirControlOutNew;
    g_fDirControlOutNew = Direction_Control_PID(g_fMidErrorNew,Errorn_errorn_1);
    g_bDirControlPeriod = 0;
		
		 
//��ת�ǻ�������·��
     if((g_fZGyro/g_fZGyrold)>0){fangxiangoutjifen += g_fZGyro; }
     else  {fangxiangoutjifen=0;}
		 
		g_fZGyrold=g_fZGyro; 

//  �����������㷨   ���Ƕ�ʮ��Ӱ��Ƚϴ�      ���ز���		 
//	if(fangxiangoutjifen>500){MID_LINE=30;}
//	if(fangxiangoutjifen<-500){MID_LINE=-30;} 
//		if(fangxiangoutjifen>500){MID_LINE++; if(MID_LINE>30){MID_LINE=30;}}
//		if(fangxiangoutjifen<-500){MID_LINE--; if(MID_LINE<-30){MID_LINE=-30;}}
		 

		
//   if((g_fDirControlOutNew/g_fDirControlOutOld)>0){ fangxiangoutjifen += g_fDirControlOutNew;}
//   else  {fangxiangoutjifen=0;}
		 
}

/*????:float Direction_Control_Output(void);
 *�ֶ�ʽ����������
 *??:void
 *///:o_fDirControlOut �ֶε�PWM��
 //*/
float Direction_Control_Output(void) 
{
    float fValue,o_fDirControlOut;
    
    fValue = g_fDirControlOutNew - g_fDirControlOutOld;
    o_fDirControlOut = fValue*(g_bDirControlPeriod+1)/2 + g_fDirControlOutOld;
    g_bDirControlPeriod++;
    if(g_bDirControlPeriod >= 2)
    {
        g_bDirControlPeriod = 0;    
    }
    return o_fDirControlOut; 
}

/*????:void Set_DIR_PD(float i_fDirKP,float i_fDirKD);
 *�趨С�������PD   ģ���㷨
 *??:P?D?
 *???:void
 */
//void Set_Dir_PD(float i_fDirKP,float i_fDirKD)
//{
//    DirKP_set=i_fDirKP;
//    DirKD_set=i_fDirKD;   
//}

//void Set_Dir_Mid(byte Mid)
//{
//    MID_LINE=Mid;
//}
// 


int Fuzzy_Control_Dir(int e,int ec,const int * E_Bound,const int * DE_Bound,const float * U_Bound) //���һ��
{
    byte i;
    float u;
    if(e>=E_Bound[0] && e<=E_Bound[E_BOUND_LENGTH-1])
    {
        for(i=0;i<E_BOUND_LENGTH;i++)
        {
            if(e <= E_Bound[i])
            {
                En=i;
                break;
            }
        }
    }
    else if(e<=E_Bound[0])
    {
        En=0;   
    }
    else if(e>=E_Bound[E_BOUND_LENGTH-1])
    {
        En=E_BOUND_LENGTH-1;   
    }
    
    if(ec>=DE_Bound[0] && ec<=DE_Bound[DE_BOUND_LENGTH-1])
    {
        for(i=0;i<DE_BOUND_LENGTH;i++)
        {
            if(ec <= DE_Bound[i])
            {
                DEn=i;
                break;
            }
        }
    }
    else if(ec<=DE_Bound[0])
    {
        En=0;   
    }
    else if(ec>=DE_Bound[DE_BOUND_LENGTH-1])
    {
        En=DE_BOUND_LENGTH-1;   
    }
    Un = Rule_Dir_P[En][DEn];
    u = U_Bound[(byte)(Un+6)]; 
    return u;
}

