#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "ftm.h"
#include "dma.h"
#include "pit.h"
#include "readspeed.h"


#define Rin  PBin(22) //����PTB�˿ڵ�22����Ϊ����
#define Lin  PAin(9)  //����PTA�˿ڵ�9����Ϊ����
int32_t    valuer;    //���ٲ���
int32_t    valuel;    //���ٲ���

static uint32_t ChlValue[2];

/*
     ʵ�����ƣ�DMA�������
     ʵ��Ч����ʹ��DMAģ��ʵ������������ܣ����ڼ���Ҫʹ��DMA��PITģ��Эͬ
       ʹ��DMA��0ͨ�����ӵ�PTA5�����������
       ʹ��DMA��1ͨ�����ӵ�PTB23�����������
       ��PIT�ж��д�������������
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
 * @brief  DMA �������������ʼ��     
 * @param  dmaChl :DMAͨ����
 * @param  instance :�˿ں� ����HW_GPIOA
 * @param  pinIndex :���ź�
 * @retval None
 */
static void DMA_PulseCountInit(uint32_t dmaChl, uint32_t instance, uint32_t pinIndex)//һ·�ǲ�����instance��һ·��pinIndex
{
    /* ����2·���� ����ΪDMA���� */
    GPIO_QuickInit(instance, pinIndex, kGPIO_Mode_IFT);
    /* ����ΪDMA�����ش��� */
    GPIO_ITDMAConfig(instance, pinIndex, kGPIO_DMA_RisingEdge, true);
    /* ����DMA */
    static uint8_t dummy1, dummy2;
    DMA_InitTypeDef DMA_InitStruct1 = {0};  
    DMA_InitStruct1.chl = dmaChl;  
    DMA_InitStruct1.chlTriggerSource = DMA_PORT_TriggerSourceTable[instance];
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal; 
    DMA_InitStruct1.minorLoopByteCnt = 1;
    DMA_InitStruct1.majorLoopCnt = DMA_CITER_ELINKNO_CITER_MASK; /* ���ֵ */
    
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
    /* �������� */
    DMA_EnableRequest(dmaChl);

}
void DMA_jishu_init(void)
{

 
	GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IFT);//�����ź�����ڳ�ʼ��
	GPIO_QuickInit(HW_GPIOA, 9,  kGPIO_Mode_IFT); //�����ź�����ڳ�ʼ��
    
    /* ����DMA��׽���������ź� (ÿ���˿�ֻ�ܲ���һ·DMA Ҳ����˵DMA�������ֻ�ܲ���5·(PTA,PTB,PTC,PTD,PTE))*/
   
    DMA_PulseCountInit(HW_DMA_CH0, HW_GPIOB, 23);
    DMA_PulseCountInit(HW_DMA_CH1, HW_GPIOA,8);
    
 
}

void readspeed(void)
{

    /* ����DMA �ǵ������� ������Ҫ�����ֵ��һ�� */
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

	
    /* ������� */
    DMA_CancelTransfer();
    DMA_SetMajorLoopCounter(HW_DMA_CH0, DMA_CITER_ELINKNO_CITER_MASK);
    DMA_SetMajorLoopCounter(HW_DMA_CH1, DMA_CITER_ELINKNO_CITER_MASK);
    /* ��ʼ��һ�δ��� */
    DMA_EnableRequest(HW_DMA_CH0);
    DMA_EnableRequest(HW_DMA_CH1);
}




