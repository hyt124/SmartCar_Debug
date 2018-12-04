#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "angle.h"
#include "zhili.h"
#include "shangweiji.h"

/* CH Kinetis�̼��� V2.50 �汾 */
/* �޸���Ƶ ��ʹ�� CMSIS��׼�ļ� startup_MKxxxx.c �е� CLOCK_SETUP �� */

/* UART ���ٳ�ʼ���ṹ��֧�ֵ�����* ʹ��ʱ�����Ƽ���׼��ʼ�� */
/*
 UART1_RX_PE01_TX_PE00   
 UART0_RX_PF17_TX_PF18   
 UART3_RX_PE05_TX_PE04   
 UART5_RX_PF19_TX_PF20   
 UART5_RX_PE09_TX_PE08   
 UART2_RX_PE17_TX_PE16   
 UART4_RX_PE25_TX_PE24   
 UART0_RX_PA01_TX_PA02   
 UART0_RX_PA15_TX_PA14   
 UART3_RX_PB10_TX_PB11   
 UART0_RX_PB16_TX_PB17   
 UART1_RX_PC03_TX_PC04   
 UART4_RX_PC14_TX_PC15   
 UART3_RX_PC16_TX_PC17   
 UART2_RX_PD02_TX_PD03   
 UART0_RX_PD06_TX_PD07   
 UART2_RX_PF13_TX_PF14   
 UART5_RX_PD08_TX_PD09   
*/

extern float g_fXGyro,g_fAcc;
extern float Angle , Angle_dot ; 
extern int    valuer;    //���ٲ���
extern int    valuel;    //���ٲ���
extern float g_fCarSpeed;
void uart_putbuff (uint32_t instance, int8_t *buff, uint32_t len)
{
    while(len--)
    {
        UART_WriteByte(instance, *buff);
        buff++;
    }
}

void vcan_sendware(int8_t *wareaddr, int32_t waresize)
{
  #define CMD_WARE     3
    int8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //???? ??????
    int8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //???? ??????

    uart_putbuff(HW_UART0, cmdf, sizeof(cmdf));    //??????
    uart_putbuff(HW_UART0, wareaddr, waresize);    //????
    uart_putbuff(HW_UART0, cmdr, sizeof(cmdr));    //?????
}


int xunishiboqi(void)
{
	int8_t var[4];
    int8_t  x,y,m,n;
    //uint32_t instance; /*��� UART ��ģ��� */
    //DelayInit();
    //DelayMs(10);
    //GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    //GPIO_QuickInit(HW_GPIOD, 1, kGPIO_Mode_OPP);
	
    /* ��ʼ��UART ʹ�ÿ��ٳ�ʼ����ʽ ������ 115200 ��������Ĭ�� ���س�ʼ���� UART��ģ��� */
    UART_QuickInit(UART0_RX_PB16_TX_PB17, 115200);
    
    /* ��ʹ�ô��ڳ�ʼ���� printf ��Ĭ�����ӵ���һ������ʼ���Ĵ�����*/
   // printf("UART%d OK! Hello Kinetis\r\n", instance);
    

        /* ���� ���ֽڷ��� ���� ע�� HW_UART0�������Ѿ���ʼ������ģ�� ���� ����������*/
        /*UART_WriteByte(instance, 'h');
        UART_WriteByte(instance, 'e');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'l');
        UART_WriteByte(instance, 'o');
        UART_WriteByte(instance, '\r');
        UART_WriteByte(instance, '\n');*/
        
        
        Angle_Calculate();
        
        x=(int8_t)g_fXGyro;
        y=(int8_t)g_fAcc;
        
        m=(int8_t)Angle;
        n=(int8_t)g_fCarSpeed;
        
	      var[0] = x;
        var[1] = y;
        var[2] = m; 
        var[3] = n;
        //var[4] = 0xba;
			
	    vcan_sendware((int8_t *)var, sizeof(var));
        /* ��˸С�� */
       // GPIO_ToggleBit(HW_GPIOE, 6);
        //DelayMs(10);
        return 0;
    
}




