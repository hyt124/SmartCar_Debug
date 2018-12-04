#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "adc.h"
#include "ADCread.h"
/* 可用的 ADC通道及引脚 - 144P*/
/*
 ADC0_SE0_DP0        
 ADC0_SE1_DP1        
 ADC0_SE3_DP3        
 ADC0_SE4B_PC2       
 ADC0_SE5B_PD1       
 ADC0_SE6B_PD5       
 ADC0_SE7B_PD6       
 ADC0_SE8_PB0        
 ADC0_SE9_PB1        
 ADC0_SE12_PB2       
 ADC0_SE13_PB3       
 ADC0_SE14_PC0       
 ADC0_SE15_PC1       
 ADC0_SE17_E24       
 ADC0_SE18_E25       
 ADC0_SE19_DM0       
 ADC0_SE20_DM1       
 ADC0_SE26_TEMP      
 ADC1_SE0_DP0        
 ADC1_SE1_DP1        
 ADC1_SE3_DP3        
 ADC1_SE4_PE0        
 ADC1_SE5_PE1        
 ADC1_SE6_PE2        
 ADC1_SE7_PE3        
 ADC1_SE4B_PC8       
 ADC1_SE5B_PC9       
 ADC1_SE6B_PC10      
 ADC1_SE7B_PC11      
 ADC1_SE8_PB0        
 ADC1_SE9_PB1        
 ADC1_SE14_PB10      
 ADC1_SE15_PB11      
 ADC1_SE17_PB117     
 ADC1_SE19_DM0       
 ADC1_SE20_DM1       
 ADC1_SE26_TEMP  
#define ADC1_SE10_PB04      (0X00504809U)
#define ADC1_SE11_PB05      (0X00584A09U)
#define ADC1_SE12_PB06      (0X00604C09U)
#define ADC1_SE13_PB07      (0X00684E09U) 
*/


 
 unsigned int A,B,C,D,EE,FF ; 
 unsigned int A1,B1,C1,D1,EE1,FF1,  A2,B2,C2,D2,EE2,FF2,  A3,B3,C3,D3,EE3,FF3,  A4,B4,C4,D4,EE4,FF4;//ad7 ad15  ad12 ad15 
	
void ADCread_init(void)
{

    
  
    /* 初始化ADC模块 ADC0_SE19_BM0 */
    ADC_InitTypeDef ADC_InitStruct1;
    ADC_InitStruct1.instance = HW_ADC1;
    ADC_InitStruct1.clockDiv = kADC_ClockDiv2; /* ADC采样时钟2分频 */
    ADC_InitStruct1.resolutionMode = kADC_SingleDiff10or11;
    ADC_InitStruct1.triggerMode = kADC_TriggerSoftware; /* 软件触发转换 */
    ADC_InitStruct1.singleOrDiffMode = kADC_Single; /*单端模式 */
    ADC_InitStruct1.continueMode = kADC_ContinueConversionDisable; /* 启动连续转换 转换一次后 自动开始下一次转换*/
    ADC_InitStruct1.hardwareAveMode = kADC_HardwareAverage_32; /*禁止 硬件平均 功能 */
    ADC_InitStruct1.vref = kADC_VoltageVREF;                       /* 使用外部VERFH VREFL 作为模拟电压参考 */
    ADC_Init(&ADC_InitStruct1);
    
    /* 初始化对应引脚 */
    /* DM0引脚为专门的模拟引脚 ADC时 无需设置复用  DM0也无法当做普通的数字引脚 */
			PORT_PinMuxConfig(HW_GPIOB, 4, kPinAlt0);
			PORT_PinMuxConfig(HW_GPIOB, 5, kPinAlt0);
			PORT_PinMuxConfig(HW_GPIOB, 6, kPinAlt0);
			PORT_PinMuxConfig(HW_GPIOB, 7, kPinAlt0);
			PORT_PinMuxConfig(HW_GPIOB, 10, kPinAlt0);
			PORT_PinMuxConfig(HW_GPIOB, 11, kPinAlt0);
			
      ADC_ChlMuxConfig(HW_ADC1 , kADC_MuxA);//设置通道口  以及AB类型
			


    /* 启动一次ADC转换 */
    //ADC_StartConversion(HW_ADC1, 11, kADC_MuxA);
}		
short ADCread(void)
{

	      ADC_StartConversion(HW_ADC1, 13, kADC_MuxA);
				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
        A= ADC_ReadValue(HW_ADC1, kADC_MuxA);
 
				ADC_StartConversion(HW_ADC1, 12, kADC_MuxA);
				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
        B= ADC_ReadValue(HW_ADC1, kADC_MuxA);
 				 
        ADC_StartConversion(HW_ADC1, 15, kADC_MuxA);/* 启动一次ADC转换 */
        while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };///* 如果ADC转换完成 */如果没转换完  等
        C= ADC_ReadValue(HW_ADC1, kADC_MuxA);/* 读取ADC的值  kADC_MuxA是每个ADC通道的转换器 默认都是 kADC_MuxA  MuxB 一般不能用于软件触发 */
				
				ADC_StartConversion(HW_ADC1, 10, kADC_MuxA);
				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
        D= ADC_ReadValue(HW_ADC1, kADC_MuxA);
			
				ADC_StartConversion(HW_ADC1, 14, kADC_MuxA);
				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
        EE= ADC_ReadValue(HW_ADC1, kADC_MuxA);
				
				ADC_StartConversion(HW_ADC1, 11, kADC_MuxA);
				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
        FF= ADC_ReadValue(HW_ADC1, kADC_MuxA);
				
	
//       	ADC_StartConversion(HW_ADC1, 12, kADC_MuxA);
//				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
//        A= ADC_ReadValue(HW_ADC1, kADC_MuxA);
// 
//				ADC_StartConversion(HW_ADC1, 13, kADC_MuxA);
//				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
//        B= ADC_ReadValue(HW_ADC1, kADC_MuxA);
// 				 
//        ADC_StartConversion(HW_ADC1, 15, kADC_MuxA);/* 启动一次ADC转换 */
//        while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };///* 如果ADC转换完成 */如果没转换完  等
//        C= ADC_ReadValue(HW_ADC1, kADC_MuxA);/* 读取ADC的值  kADC_MuxA是每个ADC通道的转换器 默认都是 kADC_MuxA  MuxB 一般不能用于软件触发 */
//				
//				ADC_StartConversion(HW_ADC1, 10, kADC_MuxA);
//				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
//        D= ADC_ReadValue(HW_ADC1, kADC_MuxA);
//			
//				ADC_StartConversion(HW_ADC1, 14, kADC_MuxA);
//				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
//        EE= ADC_ReadValue(HW_ADC1, kADC_MuxA);
//				
//				ADC_StartConversion(HW_ADC1, 11, kADC_MuxA);
//				while(ADC_IsConversionCompleted(HW_ADC1, kADC_MuxA) == 1){      };
//        FF= ADC_ReadValue(HW_ADC1, kADC_MuxA);
				
				return 0;
}

void ave()
{
	
  int a[9],b[9],c[9],d[9],e[9],f[9];
  int i=0;
	int suma=0;int sumb=0;int sumc=0;int sumd=0;int sumee=0;int sumff=0;
	int maxa,maxb,maxc,maxd,maxee,maxff,mina,minb,minc,mind,minee,minff;

					for(i=0;i<9;i++)
					{
					ADCread();
					a[i]=A,b[i]=B,c[i]=C,d[i]=D,e[i]=EE,f[i]=FF;
					maxa=mina=a[0],maxb=minb=b[0],maxc=minc=c[0],maxd=mind=d[0],maxee=minee=e[0],maxff=minff=f[0];
					if(a[i]>maxa) maxa=a[i];if(b[i]>maxb) maxb=b[i];if(c[i]>maxc) maxc=c[i];if(d[i]>maxd) maxd=d[i];if(e[i]>maxee) maxee=e[i];if(f[i]>maxff) maxff=f[i];
					if(a[i]<mina) mina=a[i];if(b[i]<minb) minb=b[i];if(c[i]<minc) minc=c[i];if(d[i]<mind) mind=d[i];if(e[i]<minee) minee=e[i];if(f[i]<minff) minff=f[i];
					suma=a[i]+suma;	sumb=b[i]+sumb;	sumc=c[i]+sumc;	sumd=d[i]+sumd;	sumee=e[i]+sumee;	sumff=f[i]+sumff;
					
					}
					
					
		  A2 = (suma-maxa-mina)/7;
			B2 = (sumb-maxb-minb)/7;	
			D2 = (sumd-maxd-mind)/7;
			EE2= (sumee-maxee-minee)/7;
					
			FF2= (sumff-maxff-minff)/7;	
		  C2 = (sumc-maxc-minc)/7;		
					
//		   	C2 = 1.133*((sumc-maxc-minc)/5-75);		
//			  FF2= 1.103*((sumff-maxff-minff)/5-20);	
//					
      //if (A2<30)		{B2=A2-5;}			
			//B2 =  B1-(A2/3);if(B2<1){B2=1;}					
					
					
//      A2 = (suma-maxa-mina)/3;
//			B2 = (sumb-maxb-minb)/3;
//			C2 = (sumc-maxc-minc)/3;
//			D2 = (sumd-maxd-mind)/3;
//			EE2= (sumee-maxee-minee)/3;
//			FF2= (sumff-maxff-minff)/3;
//					
//			A1 = mina;
//			B1 = minb;
//			C1 = minc;
//      D1 = mind;
//      EE1= minee;
//		  FF1= minff;
//					
//			A3 = maxa;
//			B3 = maxb;
//			C3 = maxc;
//      D3 = maxd;
//      EE3= maxee;
//		  FF3= maxff;

//      A4 = 100*(A2)/(A3);
//			B4 = 100*(B2)/(B3);
//			C4 = 100*(C2)/(C3);
//			D4 = 100*(D2)/(D3);
//			EE4 = 100*(EE2)/(EE3);
//			FF4 = 100*(FF2)/(FF3);
}


/*
void  AVG_ADC()
{
	int a[5],b[5],c[5],d[5],ee[5],ff[5];
	static int i=0;
	for(i=0;i<6;i++)
	{
   ADCread();
	 a[i]=A;b[i]=B;c[i]=C;d[i]=D;ee[i]=EE;ff[i]=FF;
		
		A1=A1+a[i];
		B1=B1+b[i];
		C1=C1+c[i];
		D1=D1+c[i];
		EE1=EE1+ee[i];
		FF1=FF1+ff[i];
	}
   
A2=A1/5;
B2=B1/5;
	CDEEFF
	
	
}

*/
