#include "gpio.h"
#include "common.h"
#include "i2c.h"
#include "uart.h"
#include "ftm.h"
#include "pwm.h"
#include "dma.h"
#include "pit.h"
#include "oled.h"
#include "angle.h"
#include "zhili.h"
#include "shangweiji.h"
#include "dma.h"
#include "readspeed.h"
#include "speedcontrol.h"
#include "ADCread.h"
#include "directioncontrol.h"

extern unsigned int A,B,C,D,EE,FF ;   //��Ÿ�Ӧ��ADC
extern unsigned int A1,B1,C1,D1,EE1,FF1,  A2,B2,C2,D2,EE2,FF2,  A3,B3,C3,D3,EE3,FF3,  A4,B4,C4,D4,EE4,FF4;
extern int   Hx,Hs,H2,H_use,H_uselook,g_bMidPoint,shizi;  //ʵ��ƫ��  �봦����ƫ��
extern int     g_bMidPoint,Errorn_errorn_1;    //������ƫ����ƫ���΢��
//extern float suduout;   //kansuduyongde 
extern float g_fCarSpeed;   //����
extern float g_fXGyro,g_fAcc;  //���ٶȻ���ÿ��  �ͼ��ٶ�   ���ٶ��Ѿ�ת���ɽǶ�
extern float Angle , Angle_dot ;  //������֮��Ľ��ٶȺͽǶ�
extern int32_t    valuer;    //���ٲ���
extern int32_t    valuel;    //���ٲ���
extern float   fangxiangoutjifen;   //��������Ļ���

float  X,Y,Z;    //�ٶ� �Ƕ�  ����������
int    n,m,n1,m1;//���˺���û�õ�
int    R,L;      //�����ֵ�pwm�����

unsigned char   t=2,t1=0,t2=0;   //��ʱ�������õ�   ����ͣ������������
//unsigned int    SSt=0;
	
short int zhili_flage=0;            //��߶��Ǳ�־λ  
short int speedcontrol_flage=0;  
short int dircontrol_flage=0;
short int dircontrolout_flage=0;

short int  speed1_flag=0;   //�ٶȵ�λ��־λ
short int  speed2_flag=0;
short int  speed3_flag=0;
short int  speed4_flag=0;
//short int  smallS_flag=0;

extern unsigned char Protect_flag;


	#define KEY1  PEin(6)  //����PTE�˿ڵ�6����Ϊ����
	#define KEY2  PEin(7)  //����PTE�˿ڵ�7����Ϊ����
	#define KEY3  PEin(11)  //����PTE�˿ڵ�11����Ϊ����
	#define KEY4  PEin(12)  //����PTE�˿ڵ�12����Ϊ����
	 
	 
	#define speed4  PCin(4)  //����PTE�˿ڵ�6����Ϊ����
	#define speed3  PCin(5)  //����PTE�˿ڵ�7����Ϊ����
	#define speed2  PCin(6)  //����PTE�˿ڵ�11����Ϊ����
	#define speed1  PCin(7)  //����PTE�˿ڵ�12����Ϊ����
	
	#define  Bee_on    PEout(27)=1   //  bee  ���
	#define  Bee_off   PEout(27)=0   //  bee  ���
		
 void  shineng (void)   //ʹ�ܿ��غ���
 {
  if(PAin(25)==0) {
                      PAout(14) =0;PCout(0) =1;  //ʹ�ܹص�  ��һ���Ʊ�ʾ
                      Angle_Calculate();  //�����Ǽ��ٶ�����ֵ�Ŀ������˲����ȡ
	                  	ave();
                      OLed_DisplayF(0,0,Angle_dot);   
                      OLed_DisplayF(0,1,Angle);
		                  OLed_DisplayI(0,2,A2);  OLed_DisplayI(40,2,C2);  OLed_DisplayI(80,2,EE2);
	                    OLed_DisplayI(0,3,B2);  OLed_DisplayI(40,3,D2);  OLed_DisplayI(80,3,FF2);
		                  OLed_DisplayF(0,4,H_use);
                      DelayMs(100);
		                  LCD_CLS();//����
		                 // if(KEY1==0) {printf("  %d",H);}
		                  // printf("  %f",g_fXGyro);
		
                  }
  else            {
										PAout(14) = 1;  PCout(0) = 0;
										if(t2>199&&(PCin(11)==0||PCin(12)==0)){ t1++;if(t1==2){pwm(-2000,-2000); DelayMs(500);Protect_flag=0;}}
										
										if(H_uselook==0){Bee_on;}
										else{Bee_off;}
										
		                //printf(" %d    %f \n  ",H_uselook,fangxiangoutjifen);
										//ADCread_init();
										//printf(" %d   %d     %d   %d    %d   %d       \n",A2,B2,C2,D2,EE2,FF2);
									 // printf("  %f",fangxiangoutjifen);
		              }                                                        //printf("  %f",suduout);}printf("  %f",g_fCarSpeed);
 }  
 
 //�ٶȲ��뿪���л��ٶȺ���
 void speed_change (void)
 {    
	  	if(speed1==1 && speed2==1 && speed3==1 && speed4==1) {speed1_flag=1;speed2_flag=0;speed3_flag=0;speed4_flag=0;}
			if(speed1==0 && speed2==1 && speed3==1 && speed4==1) {speed1_flag=1;speed2_flag=0;speed3_flag=0;speed4_flag=0;}
			if(speed1==1 && speed2==0 && speed3==1 && speed4==1) {speed1_flag=0;speed2_flag=1;speed3_flag=0;speed4_flag=0;}
			if(speed1==1 && speed2==1 && speed3==0 && speed4==1) {speed1_flag=0;speed2_flag=0;speed3_flag=1;speed4_flag=0;}
			if(speed1==1 && speed2==1 && speed3==1 && speed4==0) {speed1_flag=0;speed2_flag=0;speed3_flag=0;speed4_flag=1;}
			
 }

 /* PIT0�жϷ����� */
//�˺����б�д�û��ж���Ҫ��������
static void PIT_ISR(void)
{   
   t++;
	
   if((t%5)==0) {zhili_flage=1;}  //ֱ������5msһ��
	 if((t%10) ==3) {dircontrol_flage=1;}        //����10msһ��  ҲҪ��ƽ����� 
   if((t%100)==2){speedcontrol_flage=1;t=2;t2++;if(t2>200){t2=200;}}  //�ٶȿ���100ms��һ�Σ�����Ҫƽ�����
      
}
int main (void)
{
   


    DelayInit();       //��ʱ������ʼ��
		UART_QuickInit(UART0_RX_PB16_TX_PB17, 115200);  //���ڳ�ʼ��
 // UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
	
    DMA_jishu_init();  //DMA������ʼ�����ڱ�����
	  ADCread_init();    //ADC��ʼ��
	  LCD_Init();   //Һ������ʼ��

// GyroX_Zero_Cal();  //���������ֵ��ȡ�����ֵ�������һ���ò���
    
    GPIO_QuickInit(HW_GPIOA, 25, kGPIO_Mode_IFT);  //���˿��ص�IO��ʼ��
    GPIO_QuickInit(HW_GPIOA, 14, kGPIO_Mode_OPP);  //ʹ�ܿ��ƽŵĳ�ʼ��
    
    GPIO_QuickInit(HW_GPIOC, 0,  kGPIO_Mode_OPP); //LED
    GPIO_QuickInit(HW_GPIOC, 1,  kGPIO_Mode_OPP); //LED
    GPIO_QuickInit(HW_GPIOE, 27, kGPIO_Mode_OPP); //bee
	
		GPIO_QuickInit(HW_GPIOC, 11, kGPIO_Mode_IPU); //�ɻɹ�����������
	  GPIO_QuickInit(HW_GPIOC, 12, kGPIO_Mode_IPU);
	
		GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_IPU);   //��������
		GPIO_QuickInit(HW_GPIOE, 7, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOE, 11, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOE, 12, kGPIO_Mode_IPU);
		
		GPIO_QuickInit(HW_GPIOC, 4, kGPIO_Mode_IPU);  //���뿪����������
		GPIO_QuickInit(HW_GPIOC, 5, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOC, 6, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOC, 7, kGPIO_Mode_IPU);
	

   GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);  //����һ��BUG   E���IOһ��Ҫ��ʼ��һ��  ��ȻӲ��IIC�ò���

        /* ��ʼ��PITģ�� */ //��ʱ���ĳ�ʼ������
    PIT_InitTypeDef PIT_InitStruct1;  //����ṹ�����
    PIT_InitStruct1.chl = HW_PIT_CH1; /* ʹ��1�Ŷ�ʱ�� */
    PIT_InitStruct1.timeInUs = 1000*1; /* ��ʱ����1ms */
    PIT_Init(&PIT_InitStruct1); //pitģ���ʼ��
    /* ע��PIT �жϻص����� */
    PIT_CallbackInstall(HW_PIT_CH1, PIT_ISR); //1�Ŷ�ʱ�����жϴ���
    /* ����PIT1��ʱ���ж� */
    PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, true);

    /* PWM�����ʼ��  Ƶ�����ã�ռ�ձȷֳ�10000�� */
    FTM_PWM_QuickInit(FTM0_CH1_PA04, kPWM_EdgeAligned, 10000);	 
    FTM_PWM_QuickInit(FTM0_CH2_PA05, kPWM_EdgeAligned, 10000);
    FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 10000);	
    FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 10000);

          
 
//    LCD_P6x8Str(0,7,"GOGOGO");//Һ������ʾ����
   
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//IO�ڷ�ת����
		while(1)
	{ 
		   shineng ();//���ʹ�ܲ��˿���
       speed_change();//���뿪���л��ٶ�
//		

		
		
//���뿪�ز���	   
//		if(speed1_flag==1){Bee_on;} 
//		else if(speed2_flag==1){Bee_on;} 
//		else if(speed3_flag==1){Bee_on;} 
//  	else if(speed4_flag==1){Bee_on;} 
//		else {Bee_off;}

       if(zhili_flage==1)
        {
        Z = Direction_Control_Output();	   				
				X = Speed_Control_Output();
				Y = Angle_Control();
					
					
				R = (Y-X);  //�����ǽǶȼ��ٶ�
				L = R;

				//	if(fangxiangouTAtjifen>10000){}
				//	if(Z>0){	pwm((int)(R+0),(int)(L+Z));}
				//	else {	pwm((int)(R-Z),(int)(L-0));}
					
					
					pwm((int)(R-Z),(int)(L+Z));
					
					
				  zhili_flage=0;     //ֱ����־λ����ȴ���һ�ο���       
					//xunishiboqi();            
         }       
  
          if(speedcontrol_flage==1)
          {  
             Speed_Control();            
             speedcontrol_flage=0;
          }
					
			  	 if(dircontrol_flage==1)
          {  
             Direction_Control();            
             dircontrol_flage=0;
          }
          

																																// printf("  %f     %f    %f    %f\n",g_fXGyro,g_fAcc,Angle , Angle_dot );
																																 // xunishiboqi();
																															
																																// LCD_P6x8Str(0,3,"GOGOGO");
																														 
       
   }
	
}
