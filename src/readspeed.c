#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "ftm.h"
#include "dma.h"
#include "pit.h"
#include "readspeed.h"


#define Rin  PBin(22) //定义PTB端口的22引脚为输入
#define Lin  PAin(9)  //定义PTA端口的9引脚为输入
int32_t    valuer;    //左速参数
int32_t    valuel;    //右速参数

static uint32_t ChlValue[2];

/*
     实验名称：DMA脉冲计数
     实验效果：使用DMA模块实现脉冲计数功能，这期间需要使用DMA、PIT模块协同
       使用DMA的0通到连接到PTA5引脚脉冲计数
       使用DMA的1通道连接到PTB23引脚脉冲计数
       在PIT中断中处理脉冲计数结果
*/
static const uint32_t DMA_PORT_TriggerSourceTable[] = 
{
    PORTA_DMAREQ,
    PORTB_DMAREQ,
    PORTC_DMAREQ,
    PORTD_DMAREQ,
    PORTE_DMAREQ,
};

/**
 * @brief  DMA 用作脉冲计数初始化     
 * @param  dmaChl :DMA通道号
 * @param  instance :端口号 比如HW_GPIOA
 * @param  pinIndex :引脚号
 * @retval None
 */
static void DMA_PulseCountInit(uint32_t dmaChl, uint32_t instance, uint32_t pinIndex)//一路是参数叫instance，一路是pinIndex
{
    /* 开启2路引脚 配置为DMA触发 */
    GPIO_QuickInit(instance, pinIndex, kGPIO_Mode_IFT);
    /* 配置为DMA上升沿触发 */
    GPIO_ITDMAConfig(instance, pinIndex, kGPIO_DMA_RisingEdge, true);
    /* 配置DMA */
    static uint8_t dummy1, dummy2;
    DMA_InitTypeDef DMA_InitStruct1 = {0};  
    DMA_InitStruct1.chl = dmaChl;  
    DMA_InitStruct1.chlTriggerSource = DMA_PORT_TriggerSourceTable[instance];
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal; 
    DMA_InitStruct1.minorLoopByteCnt = 1;
    DMA_InitStruct1.majorLoopCnt = DMA_CITER_ELINKNO_CITER_MASK; /* 最大值 */
    
    DMA_InitStruct1.sAddr = (uint32_t)&dummy1;
    DMA_InitStruct1.sLastAddrAdj = 0; 
    DMA_InitStruct1.sAddrOffset = 0;
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;
    
    DMA_InitStruct1.dAddr = (uint32_t)&dummy2; 
    DMA_InitStruct1.dLastAddrAdj = 0;
    DMA_InitStruct1.dAddrOffset = 0; 
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;
    DMA_Init(&DMA_InitStruct1);
    /* 启动传输 */
    DMA_EnableRequest(dmaChl);

}
void DMA_jishu_init(void)
{

 
	GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IFT);//方向信号输入口初始化
	GPIO_QuickInit(HW_GPIOA, 9,  kGPIO_Mode_IFT); //方向信号输入口初始化
    
    /* 开启DMA捕捉引脚脉冲信号 (每个端口只能测量一路DMA 也就是说DMA脉冲最多只能测量5路(PTA,PTB,PTC,PTD,PTE))*/
   
    DMA_PulseCountInit(HW_DMA_CH0, HW_GPIOB, 23);
    DMA_PulseCountInit(HW_DMA_CH1, HW_GPIOA,8);
    
 
}

void readspeed(void)
{

    /* 由于DMA 是倒计数的 所需需要用最大值减一下 */
    /* CH0 */
    ChlValue[0] = DMA_CITER_ELINKNO_CITER_MASK - DMA_GetMajorLoopCount(HW_DMA_CH0);
    /* CH1 */
    ChlValue[1] = DMA_CITER_ELINKNO_CITER_MASK - DMA_GetMajorLoopCount(HW_DMA_CH1);
	
  if(Rin==0)
	{valuer = ChlValue[0];}
  if(Rin==1)
	{valuer =0- ChlValue[0];}

  if(Lin==1)
	{valuel = ChlValue[1];}
  if(Lin==0) 
	{valuel =0- ChlValue[1];} 

	
    /* 清零计数 */
    DMA_CancelTransfer();
    DMA_SetMajorLoopCounter(HW_DMA_CH0, DMA_CITER_ELINKNO_CITER_MASK);
    DMA_SetMajorLoopCounter(HW_DMA_CH1, DMA_CITER_ELINKNO_CITER_MASK);
    /* 开始下一次传输 */
    DMA_EnableRequest(HW_DMA_CH0);
    DMA_EnableRequest(HW_DMA_CH1);
}




