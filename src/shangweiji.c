#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "angle.h"
#include "zhili.h"
#include "shangweiji.h"

/* CH Kinetis固件库 V2.50 版本 */
/* 修改主频 请使用 CMSIS标准文件 startup_MKxxxx.c 中的 CLOCK_SETUP 宏 */

/* UART 快速初始化结构所支持的引脚* 使用时还是推荐标准初始化 */
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
extern int    valuer;    //左速参数
extern int    valuel;    //右速参数
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
    //uint32_t instance; /*存放 UART 的模块号 */
    //DelayInit();
    //DelayMs(10);
    //GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_OPP);
    //GPIO_QuickInit(HW_GPIOD, 1, kGPIO_Mode_OPP);
	
    /* 初始化UART 使用快速初始化方式 波特率 115200 其他配置默认 返回初始化后 UART的模块号 */
    UART_QuickInit(UART0_RX_PB16_TX_PB17, 115200);
    
    /* 当使用串口初始化后 printf 被默认连接到第一个被初始化的串口上*/
   // printf("UART%d OK! Hello Kinetis\r\n", instance);
    

        /* 串口 按字节发送 数据 注意 HW_UART0必须是已经初始化过的模块 否则 将产生错误*/
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
        /* 闪烁小灯 */
       // GPIO_ToggleBit(HW_GPIOE, 6);
        //DelayMs(10);
        return 0;
    
}




