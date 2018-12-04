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

extern unsigned int A,B,C,D,EE,FF ;   //电磁感应的ADC
extern unsigned int A1,B1,C1,D1,EE1,FF1,  A2,B2,C2,D2,EE2,FF2,  A3,B3,C3,D3,EE3,FF3,  A4,B4,C4,D4,EE4,FF4;
extern int   Hx,Hs,H2,H_use,H_uselook,g_bMidPoint,shizi;  //实测偏差  与处理后的偏差
extern int     g_bMidPoint,Errorn_errorn_1;    //处理后的偏差与偏差的微分
//extern float suduout;   //kansuduyongde 
extern float g_fCarSpeed;   //车速
extern float g_fXGyro,g_fAcc;  //角速度弧度每秒  和加速度   加速度已经转化成角度
extern float Angle , Angle_dot ;  //卡尔曼之后的角速度和角度
extern int32_t    valuer;    //左速参数
extern int32_t    valuel;    //右速参数
extern float   fangxiangoutjifen;   //方向输出的积分

float  X,Y,Z;    //速度 角度  方向的输出量
int    n,m,n1,m1;//忘了好像没用到
int    R,L;      //左右轮的pwm输出量

unsigned char   t=2,t1=0,t2=0;   //定时器计数用的   还有停车检测次数计数
//unsigned int    SSt=0;
	
short int zhili_flage=0;            //这边都是标志位  
short int speedcontrol_flage=0;  
short int dircontrol_flage=0;
short int dircontrolout_flage=0;

short int  speed1_flag=0;   //速度档位标志位
short int  speed2_flag=0;
short int  speed3_flag=0;
short int  speed4_flag=0;
//short int  smallS_flag=0;

extern unsigned char Protect_flag;


	#define KEY1  PEin(6)  //定义PTE端口的6引脚为输入
	#define KEY2  PEin(7)  //定义PTE端口的7引脚为输入
	#define KEY3  PEin(11)  //定义PTE端口的11引脚为输入
	#define KEY4  PEin(12)  //定义PTE端口的12引脚为输入
	 
	 
	#define speed4  PCin(4)  //定义PTE端口的6引脚为输入
	#define speed3  PCin(5)  //定义PTE端口的7引脚为输入
	#define speed2  PCin(6)  //定义PTE端口的11引脚为输入
	#define speed1  PCin(7)  //定义PTE端口的12引脚为输入
	
	#define  Bee_on    PEout(27)=1   //  bee  输出
	#define  Bee_off   PEout(27)=0   //  bee  输出
		
 void  shineng (void)   //使能开关函数
 {
  if(PAin(25)==0) {
                      PAout(14) =0;PCout(0) =1;  //使能关掉  亮一个灯表示
                      Angle_Calculate();  //陀螺仪加速度两个值的卡尔曼滤波后读取
	                  	ave();
                      OLed_DisplayF(0,0,Angle_dot);   
                      OLed_DisplayF(0,1,Angle);
		                  OLed_DisplayI(0,2,A2);  OLed_DisplayI(40,2,C2);  OLed_DisplayI(80,2,EE2);
	                    OLed_DisplayI(0,3,B2);  OLed_DisplayI(40,3,D2);  OLed_DisplayI(80,3,FF2);
		                  OLed_DisplayF(0,4,H_use);
                      DelayMs(100);
		                  LCD_CLS();//清屏
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
 
 //速度拨码开关切换速度函数
 void speed_change (void)
 {    
	  	if(speed1==1 && speed2==1 && speed3==1 && speed4==1) {speed1_flag=1;speed2_flag=0;speed3_flag=0;speed4_flag=0;}
			if(speed1==0 && speed2==1 && speed3==1 && speed4==1) {speed1_flag=1;speed2_flag=0;speed3_flag=0;speed4_flag=0;}
			if(speed1==1 && speed2==0 && speed3==1 && speed4==1) {speed1_flag=0;speed2_flag=1;speed3_flag=0;speed4_flag=0;}
			if(speed1==1 && speed2==1 && speed3==0 && speed4==1) {speed1_flag=0;speed2_flag=0;speed3_flag=1;speed4_flag=0;}
			if(speed1==1 && speed2==1 && speed3==1 && speed4==0) {speed1_flag=0;speed2_flag=0;speed3_flag=0;speed4_flag=1;}
			
 }

 /* PIT0中断服务函数 */
//此函数中编写用户中断需要做的事情
static void PIT_ISR(void)
{   
   t++;
	
   if((t%5)==0) {zhili_flage=1;}  //直立控制5ms一次
	 if((t%10) ==3) {dircontrol_flage=1;}        //方向10ms一次  也要做平滑输出 
   if((t%100)==2){speedcontrol_flage=1;t=2;t2++;if(t2>200){t2=200;}}  //速度控制100ms做一次，但是要平滑输出
      
}
int main (void)
{
   


    DelayInit();       //延时函数初始化
		UART_QuickInit(UART0_RX_PB16_TX_PB17, 115200);  //串口初始化
 // UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
	
    DMA_jishu_init();  //DMA计数初始化用于编码器
	  ADCread_init();    //ADC初始化
	  LCD_Init();   //液晶屏初始化

// GyroX_Zero_Cal();  //陀螺仪零点值读取，数字的陀螺仪一般用不到
    
    GPIO_QuickInit(HW_GPIOA, 25, kGPIO_Mode_IFT);  //拨杆开关的IO初始化
    GPIO_QuickInit(HW_GPIOA, 14, kGPIO_Mode_OPP);  //使能控制脚的初始化
    
    GPIO_QuickInit(HW_GPIOC, 0,  kGPIO_Mode_OPP); //LED
    GPIO_QuickInit(HW_GPIOC, 1,  kGPIO_Mode_OPP); //LED
    GPIO_QuickInit(HW_GPIOE, 27, kGPIO_Mode_OPP); //bee
	
		GPIO_QuickInit(HW_GPIOC, 11, kGPIO_Mode_IPU); //干簧管上拉输入检测
	  GPIO_QuickInit(HW_GPIOC, 12, kGPIO_Mode_IPU);
	
		GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_IPU);   //按键输入
		GPIO_QuickInit(HW_GPIOE, 7, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOE, 11, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOE, 12, kGPIO_Mode_IPU);
		
		GPIO_QuickInit(HW_GPIOC, 4, kGPIO_Mode_IPU);  //拨码开关上拉输入
		GPIO_QuickInit(HW_GPIOC, 5, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOC, 6, kGPIO_Mode_IPU);
		GPIO_QuickInit(HW_GPIOC, 7, kGPIO_Mode_IPU);
	

   GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);  //这是一个BUG   E类的IO一定要初始化一个  不然硬件IIC用不了

        /* 初始化PIT模块 */ //定时器的初始化配置
    PIT_InitTypeDef PIT_InitStruct1;  //申请结构体变量
    PIT_InitStruct1.chl = HW_PIT_CH1; /* 使用1号定时器 */
    PIT_InitStruct1.timeInUs = 1000*1; /* 定时周期1ms */
    PIT_Init(&PIT_InitStruct1); //pit模块初始化
    /* 注册PIT 中断回调函数 */
    PIT_CallbackInstall(HW_PIT_CH1, PIT_ISR); //1号定时器的中断处理
    /* 开启PIT1定时器中断 */
    PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, true);

    /* PWM输出初始化  频率设置，占空比分成10000份 */
    FTM_PWM_QuickInit(FTM0_CH1_PA04, kPWM_EdgeAligned, 10000);	 
    FTM_PWM_QuickInit(FTM0_CH2_PA05, kPWM_EdgeAligned, 10000);
    FTM_PWM_QuickInit(FTM0_CH3_PA06, kPWM_EdgeAligned, 10000);	
    FTM_PWM_QuickInit(FTM0_CH4_PA07, kPWM_EdgeAligned, 10000);

          
 
//    LCD_P6x8Str(0,7,"GOGOGO");//液晶屏显示测试
   
    GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);//IO口翻转测试
		while(1)
	{ 
		   shineng ();//检测使能拨杆开关
       speed_change();//拨码开关切换速度
//		

		
		
//拨码开关测试	   
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
					
					
				R = (Y-X);  //必须是角度减速度
				L = R;

				//	if(fangxiangouTAtjifen>10000){}
				//	if(Z>0){	pwm((int)(R+0),(int)(L+Z));}
				//	else {	pwm((int)(R-Z),(int)(L-0));}
					
					
					pwm((int)(R-Z),(int)(L+Z));
					
					
				  zhili_flage=0;     //直立标志位至零等待下一次控制       
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
