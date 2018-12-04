#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "ftm.h"
#include "i2c.h"
#include "ssd1306.h"
#include "pwm.h"
/* CH Kinetis�̼��� V2.50 �汾 */
/* �޸���Ƶ ���޸� CMSIS��׼�ļ� system_MKxxxx.c �е� CLOCK_SETUP �� */


/* ���õ�FTMͨ����: */
/*
 FTM0_CH4_PB12   // ftm0 ģ�� 4ͨ�� PB12����
 FTM0_CH5_PB13   
 FTM0_CH5_PA00   
 FTM0_CH6_PA01   
 FTM0_CH7_PA02   
 FTM0_CH0_PA03
 FTM0_CH1_PA04   
 FTM0_CH2_PA05   
 FTM0_CH3_PA06   
 FTM0_CH4_PA07   
 FTM0_CH0_PC01   
 FTM0_CH1_PC02   
 FTM0_CH2_PC03   
 FTM0_CH3_PC04   
 FTM0_CH4_PD04   
 FTM0_CH5_PD05   
 FTM0_CH6_PD06   
 FTM0_CH7_PD07   
 FTM1_CH0_PB12   
 FTM1_CH1_PB13   
 FTM1_CH0_PA08   
 FTM1_CH1_PA09   
 FTM1_CH0_PA12   
 FTM1_CH1_PA13   
 FTM1_CH0_PB00   
 FTM1_CH1_PB01   
 FTM2_CH0_PA10   
 FTM2_CH1_PA11   
 FTM2_CH0_PB18   
 FTM2_CH1_PB19  
*/

/*
     ʵ�����ƣ�FTM���PWM
     ʵ��ƽ̨����ѻ������
     ����оƬ��MK60DN512ZVQ10
 ʵ��Ч��������PTA�˿ڵ�6������PWMģʽ
      ���ռ�ձ�Ϊ50%��Ƶ��Ϊ3KHz�ķ���  
*/

byte Protect_flag=1;
void pwm(int rightpwm,int leftpwm)
{

    
    if(rightpwm >= 0)       rightpwm += 300;
    if(rightpwm <  0)       rightpwm -= 320;
    if(leftpwm  >= 0)       leftpwm  += 300;
    if(leftpwm  <  0)       leftpwm  -= 350;
    
    if(rightpwm >=  10000)   rightpwm =  10000;
    if(rightpwm <  -10000)   rightpwm = -10000;
    if(leftpwm  >=  10000)   leftpwm  =  10000;
    if(leftpwm  <  -10000)   leftpwm  = -10000;
   
/* ռ�ձ����� */ 

    if(Protect_flag==0)
      {
        rightpwm=0;
        leftpwm=0;
      }    
    if(rightpwm>0)
     {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, rightpwm); /* 0 - 10000  to 0% - 100% */
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3, 0);
     }
    else
     {
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH4, 0);
      FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3, -rightpwm); /* 0 - 10000  to 0% - 100% */   
     }
    

    if(leftpwm>0)
    {      
     FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1, leftpwm); /* 0-10000 ��Ӧ 0-100% */
     FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2, 0);
    }  
	else
    {  
     FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH1, 0);
     FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2, -leftpwm); /* 0-10000 ��Ӧ 0-100% */
    }   
}
