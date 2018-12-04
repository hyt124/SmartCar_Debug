#include <math.h>
//#include "pwm.h"
//#include "gpio.h"
//#include "common.h"
//#include "uart.h"
//#include "i2c.h"
//#include "adxl345.h"
//#include "i2cc.h"

    /* exact-width signed integer types */
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;



/**********************************  IIC(引脚复用) ***************************************/
typedef enum
{
    /*  PTA端口    */ //0~31
    PTA0,  PTA1,  PTA2,  PTA3,  PTA4,  PTA5,  PTA6,  PTA7,  PTA8,  PTA9,  PTA10, PTA11, PTA12, PTA13, PTA14, PTA15,
    PTA16, PTA17, PTA18, PTA19, PTA20, PTA21, PTA22, PTA23, PTA24, PTA25, PTA26, PTA27, PTA28, PTA29, PTA30, PTA31,

    /*  PTB端口    */ //32~63
    PTB0,  PTB1,  PTB2,  PTB3,  PTB4,  PTB5,  PTB6,  PTB7,  PTB8,  PTB9,  PTB10, PTB11, PTB12, PTB13, PTB14, PTB15,
    PTB16, PTB17, PTB18, PTB19, PTB20, PTB21, PTB22, PTB23, PTB24, PTB25, PTB26, PTB27, PTB28, PTB29, PTB30, PTB31,

    /*  PTC端口    */
    PTC0,  PTC1,  PTC2,  PTC3,  PTC4,  PTC5,  PTC6,  PTC7,  PTC8,  PTC9,  PTC10, PTC11, PTC12, PTC13, PTC14, PTC15,
    PTC16, PTC17, PTC18, PTC19, PTC20, PTC21, PTC22, PTC23, PTC24, PTC25, PTC26, PTC27, PTC28, PTC29, PTC30, PTC31,

    /*  PTD端口    */
    PTD0,  PTD1,  PTD2,  PTD3,  PTD4,  PTD5,  PTD6,  PTD7,  PTD8,  PTD9,  PTD10, PTD11, PTD12, PTD13, PTD14, PTD15,
    PTD16, PTD17, PTD18, PTD19, PTD20, PTD21, PTD22, PTD23, PTD24, PTD25, PTD26, PTD27, PTD28, PTD29, PTD30, PTD31,

    /*  PTE端口    */
    PTE0,  PTE1,  PTE2,  PTE3,  PTE4,  PTE5,  PTE6,  PTE7,  PTE8,  PTE9,  PTE10, PTE11, PTE12, PTE13, PTE14, PTE15,
    PTE16, PTE17, PTE18, PTE19, PTE20, PTE21, PTE22, PTE23, PTE24, PTE25, PTE26, PTE27, PTE28, PTE29, PTE30, PTE31,
} PTXn_e;


/* C1 Bit Fields */
#define I2C_C1_DMAEN_MASK                        0x1u
#define I2C_C1_DMAEN_SHIFT                       0
#define I2C_C1_WUEN_MASK                         0x2u
#define I2C_C1_WUEN_SHIFT                        1
#define I2C_C1_RSTA_MASK                         0x4u
#define I2C_C1_RSTA_SHIFT                        2
#define I2C_C1_TXAK_MASK                         0x8u
#define I2C_C1_TXAK_SHIFT                        3
#define I2C_C1_TX_MASK                           0x10u
#define I2C_C1_TX_SHIFT                          4
#define I2C_C1_MST_MASK                          0x20u
#define I2C_C1_MST_SHIFT                         5
#define I2C_C1_IICIE_MASK                        0x40u
#define I2C_C1_IICIE_SHIFT                       6
#define I2C_C1_IICEN_MASK                        0x80u
#define I2C_C1_IICEN_SHIFT                       7
#define I2C_S_IICIF_MASK                         0x2u


//#define PORT_PCR_MUX_MASK                        0x700u
//#define PORT_PCR_MUX_SHIFT                       8
//#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_MUX_SHIFT))&PORT_PCR_MUX_MASK)

typedef struct SIM_MemMap {
  uint32_t SOPT1;                                  /*!< System Options Register 1, offset: 0x0 */
  uint8_t RESERVED_0[4096];
  uint32_t SOPT2;                                  /*!< System Options Register 2, offset: 0x1004 */
  uint8_t RESERVED_1[4];
  uint32_t SOPT4;                                  /*!< System Options Register 4, offset: 0x100C */
  uint32_t SOPT5;                                  /*!< System Options Register 5, offset: 0x1010 */
  uint32_t SOPT6;                                  /*!< System Options Register 6, offset: 0x1014 */
  uint32_t SOPT7;                                  /*!< System Options Register 7, offset: 0x1018 */
  uint8_t RESERVED_2[8];
  uint32_t SDID;                                   /*!< System Device Identification Register, offset: 0x1024 */
  uint32_t SCGC1;                                  /*!< System Clock Gating Control Register 1, offset: 0x1028 */
  uint32_t SCGC2;                                  /*!< System Clock Gating Control Register 2, offset: 0x102C */
  uint32_t SCGC3;                                  /*!< System Clock Gating Control Register 3, offset: 0x1030 */
  uint32_t SCGC4;                                  /*!< System Clock Gating Control Register 4, offset: 0x1034 */
  uint32_t SCGC5;                                  /*!< System Clock Gating Control Register 5, offset: 0x1038 */
  uint32_t SCGC6;                                  /*!< System Clock Gating Control Register 6, offset: 0x103C */
  uint32_t SCGC7;                                  /*!< System Clock Gating Control Register 7, offset: 0x1040 */
  uint32_t CLKDIV1;                                /*!< System Clock Divider Register 1, offset: 0x1044 */
  uint32_t CLKDIV2;                                /*!< System Clock Divider Register 2, offset: 0x1048 */
  uint32_t FCFG1;                                  /*!< Flash Configuration Register 1, offset: 0x104C */
  uint32_t FCFG2;                                  /*!< Flash Configuration Register 2, offset: 0x1050 */
  uint32_t UIDH;                                   /*!< Unique Identification Register High, offset: 0x1054 */
  uint32_t UIDMH;                                  /*!< Unique Identification Register Mid-High, offset: 0x1058 */
  uint32_t UIDML;                                  /*!< Unique Identification Register Mid Low, offset: 0x105C */
  uint32_t UIDL;                                   /*!< Unique Identification Register Low, offset: 0x1060 */
} volatile *SIM_MemMapPtr;
#define SIM_BASE_PTR                             ((SIM_MemMapPtr)0x40047000u)
#define SIM_SCGC4                                SIM_SCGC4_REG(SIM_BASE_PTR)
//#define SIM_SCGC4_I2C0_MASK                      0x40u


typedef struct PORT_MemMap {
  uint32_t PCR[32];                                /*!< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  uint32_t GPCLR;                                  /*!< Global Pin Control Low Register, offset: 0x80 */
  uint32_t GPCHR;                                  /*!< Global Pin Control High Register, offset: 0x84 */
  uint8_t RESERVED_0[24];
  uint32_t ISFR;                                   /*!< Interrupt Status Flag Register, offset: 0xA0 */
  uint8_t RESERVED_1[28];
  uint32_t DFER;                                   /*!< Digital Filter Enable Register, offset: 0xC0 */
  uint32_t DFCR;                                   /*!< Digital Filter Clock Register, offset: 0xC4 */
  uint32_t DFWR;                                   /*!< Digital Filter Width Register, offset: 0xC8 */
} volatile *PORT_MemMapPtr;
#define PORTE_BASE_PTR                           ((PORT_MemMapPtr)0x4004D000u)
#define PORTE_PCR1                               PORT_PCR_REG(PORTE_BASE_PTR,1)

#define PORTC_BASE_PTR                           ((PORT_MemMapPtr)0x4004B000u)
#define PORTC_PCR10                              PORT_PCR_REG(PORTC_BASE_PTR,10)

#define PORTE_BASE_PTR                           ((PORT_MemMapPtr)0x4004D000u)
#define PORTE_PCR0                               PORT_PCR_REG(PORTE_BASE_PTR,0)

#define PORTC_BASE_PTR                           ((PORT_MemMapPtr)0x4004B000u)
#define PORTC_PCR11                              PORT_PCR_REG(PORTC_BASE_PTR,11)

#define PORT_PCR_REG(base,index)                 ((base)->PCR[index])
#define I2C_F_REG(base)                          ((base)->F)
#define I2C_C1_REG(base)                         ((base)->C1)
#define I2C_S_REG(base)                          ((base)->S)
#define I2C_D_REG(base)                          ((base)->D)

#define I2C0_SCL    PTD8        // PTB0、PTB2、PTD8
#define I2C0_SDA    PTD9        // PTB1、PTB3、PTD9

#define I2C1_SCL    PTE1       // PTE1、PTC10
#define I2C1_SDA    PTE0       // PTE0、PTC11



//定义模块号
typedef enum I2Cnn
{
    I2C00 = 0,
    I2C11 = 1
} I2Cnn;

//定义读写选项
typedef enum MSmode
{
    write =   0x00,  /* Master write  */
    read =   0x01   /* Master read */
} MSmode;





#define I2C_DisableAck(I2Cnn)        I2C_C1_REG(I2Cxx[I2Cnn]) |= I2C_C1_TXAK_MASK

//
#define I2C_RepeatedStart(I2Cnn)     I2C_C1_REG(I2Cxx[I2Cnn]) |= I2C_C1_RSTA_MASK

//启动信号
#define I2C_Startt(I2Cnn)             I2C_C1_REG(I2Cxx[I2Cnn]) |=I2C_C1_TX_MASK+I2C_C1_MST_MASK;
                                    //I2C_C1_REG(I2Cx[I2Cn]) |= I2C_C1_MST_MASK

//暂停信号
#define I2C_Stopp(I2Cnn)              I2C_C1_REG(I2Cxx[I2Cnn]) &= ~(I2C_C1_MST_MASK+I2C_C1_TX_MASK);\
                                    //I2C_C1_REG(I2Cx[I2Cn]) &= ~I2C_C1_TX_MASK

//进入接收模式(应答)
#define I2C_EnterRxMode(I2Cnn)       I2C_C1_REG(I2Cxx[I2Cnn]) &= ~I2C_C1_TX_MASK;\
                                    I2C_C1_REG(I2Cxx[I2Cnn]) &= ~I2C_C1_TXAK_MASK
//进入接收模式(不应答)
#define I2C_PutinRxMode(I2Cnn)       I2C_C1_REG(I2Cxx[I2Cnn]) &= ~I2C_C1_TX_MASK

//等待 I2C0_S
#define I2C_Wait(I2Cnn)              while(( I2C_S_REG(I2Cxx[I2Cnn]) & I2C_S_IICIF_MASK)==0) {} \
                                    I2C_S_REG(I2Cxx[I2Cnn]) |= I2C_S_IICIF_MASK;

//写一个字节
#define I2C_write_byte(I2Cnn,data)   I2C_D_REG(I2Cxx[I2Cnn]) = data

#define	SlaveAddress2100	      0x20	//IIC写入时的地址字节数据，+1为读取   现在是8451的程序
#define CTRL_REG1_2100            0x13
#define CTRL_REG0_2100            0x0d
    
#define	SlaveAddress8700	      0x1e	//IIC写入时的地址字节数据，+1为读取   现在是8451的程序
#define CTRL_REG1_8700                0x2a


#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06


#define I2C0_BASE_PTR                            ((I2C_MemMapPtr)0x40066000u)
/*! Peripheral I2C1 base pointer */
#define I2C1_BASE_PTR                            ((I2C_MemMapPtr)0x40067000u)
typedef struct I2C_MemMap {
  uint8_t A1;                                      /*!< I2C Address Register 1, offset: 0x0 */
  uint8_t F;                                       /*!< I2C Frequency Divider register, offset: 0x1 */
  uint8_t C1;                                      /*!< I2C Control Register 1, offset: 0x2 */
  uint8_t S;                                       /*!< I2C Status Register, offset: 0x3 */
  uint8_t D;                                       /*!< I2C Data I/O register, offset: 0x4 */
  uint8_t C2;                                      /*!< I2C Control Register 2, offset: 0x5 */
  uint8_t FLT;                                     /*!< I2C Programmable Input Glitch Filter register, offset: 0x6 */
  uint8_t RA;                                      /*!< I2C Range Address register, offset: 0x7 */
  uint8_t SMB;                                     /*!< I2C SMBus Control and Status register, offset: 0x8 */
  uint8_t A2;                                      /*!< I2C Address Register 2, offset: 0x9 */
  uint8_t SLTH;                                    /*!< I2C SCL Low Timeout Register High, offset: 0xA */
  uint8_t SLTL;                                    /*!< I2C SCL Low Timeout Register Low, offset: 0xB */
} volatile *I2C_MemMapPtr;

volatile struct I2C_MemMap *I2Cxx[2] = {I2C0_BASE_PTR, I2C1_BASE_PTR}; //定义两个指针数组保存 I2Cx 的地址



 void I2C_StartTransmission (I2Cnn i2cnn, unsigned char SlaveID, MSmode Mode)
{
 

    SlaveID = ( SlaveID << 1 ) | Mode ;      //确定写地址和读地址

    /* send start signal */
    I2C_Startt(i2cnn);      
  
    /* send ID with W/R bit */
     I2C_write_byte(i2cnn, SlaveID);
}

//-------------------------------------------------------------------------*
//函数名: Pause
//功  能: 延时
//参  数: 无
//返  回: 无
//简  例: Pause;
//-------------------------------------------------------------------------*
void Pause(void)
{
    unsigned short n;
    for(n = 1; n < 200; n++)      //不可太小
    {
        n++;
        n--; 
        //  asm("nop");
    }
}



//-------------------------------------------------------------------------*
//函数名: I2C_ReadAddr
//功  能: 读取IIC设备指定地址寄存器的数据
//参  数: i2cn    :端口名 I2C0,I2C1
//        SlaveID :从机地址
//        Addr    :从机的寄存器地址
//返  回: result
//简  例: p[0]  = I2C_ReadAddr(i2cn,SlaveID,OUT_X_MSB_REG);
//-------------------------------------------------------------------------*
unsigned char I2C_ReadAddr(I2Cnn i2cnn, unsigned char SlaveID, unsigned char Addr)
{
    unsigned char result;

    /* Send Slave Address */
    I2C_StartTransmission (i2cnn, SlaveID, write);
    I2C_Wait(i2cnn);

    /* Write Register Address */
    I2C_write_byte(i2cnn, Addr);
    I2C_Wait(i2cnn);

    /* Do a repeated start */
    I2C_RepeatedStart(i2cnn);

    /* Send Slave Address */
    I2C_write_byte(i2cnn, ( SlaveID << 1) | read );
    I2C_Wait(i2cnn);

    /* Put in Rx Mode */
    I2C_PutinRxMode(i2cnn);

    /* Turn off ACK since this is second to last byte being read*/
    I2C_DisableAck(i2cnn); //不应答

    /* Dummy read 虚假读取*/
    result = I2C_D_REG(I2Cxx[i2cnn]);
    I2C_Wait(i2cnn);

    /* Send stop since about to read last byte */
    I2C_Stopp(i2cnn);

    /* Read byte */
    result = I2C_D_REG(I2Cxx[i2cnn]);

    return result;
}


//-------------------------------------------------------------------------*
//函数名: I2C_ReadAddr
//功  能: 读取IIC设备指定地址寄存器的数据
//参  数: i2cn    :端口名 I2C0,I2C1
//        SlaveID :从机地址
//        Addr    :从机的寄存器地址
//        Data    :数据
//返  回: 无
//简  例: I2C_WriteAddr(I2C1, SlaveAddress2100, CTRL_REG1_2100, 0x02);
//-------------------------------------------------------------------------*
void I2C_WriteAddr(I2Cnn i2cnn, unsigned char SlaveID, unsigned char Addr, unsigned char Data)
{
    /* send data to slave */
    I2C_StartTransmission(i2cnn, SlaveID, write);  //启动传输
    I2C_Wait(i2cnn);

    I2C_write_byte(i2cnn, Addr);                  //写地址
    I2C_Wait(i2cnn);   

    I2C_write_byte(i2cnn, Data);                    //写数据
    I2C_Wait(i2cnn);

    I2C_Stopp(i2cnn);

    Pause();                                        //延时太短的话，可能写出错
}

    
    void Init2100()
{ 
        I2C_WriteAddr(I2C11,SlaveAddress2100,CTRL_REG0_2100,0x03); //设置工作模式，最大量程等
        Pause();
        I2C_WriteAddr(I2C11,SlaveAddress2100,CTRL_REG1_2100,0x03); //输出模式，输出率400HZ
        Pause();

}

    void Init8700()
{

     //  I2C_WriteAddr(I2C1,SlaveAddress8700,0x0f,0x33);
     //   Pause();
        I2C_WriteAddr(I2C11,SlaveAddress8700,CTRL_REG1_8700,0x05);//读取  输出模式
        Pause();
}


void IIC_Read(unsigned char slave,unsigned char * p,short * qq,short * q)
{
  int MUP_Zero=0x0000;

//  p[0]  =    I2C_ReadAddr(I2C11,slave,OUT_X_MSB_REG);
//  Pause();
//  p[1]  =    I2C_ReadAddr(I2C11,slave,OUT_X_LSB_REG);
//  Pause();
  p[2]  =    I2C_ReadAddr(I2C11,slave,OUT_Y_MSB_REG);
  Pause();
  p[3]  =    I2C_ReadAddr(I2C11,slave,OUT_Y_LSB_REG);
  Pause();
  p[4]  =    I2C_ReadAddr(I2C11,slave,OUT_Z_MSB_REG);
  Pause();
  p[5]  =    I2C_ReadAddr(I2C11,slave,OUT_Z_LSB_REG);
  Pause();


    qq[0] = (((MUP_Zero | p[0])<<8)|p[1]);
    qq[1] = (((MUP_Zero | p[2])<<8)|p[3]);
    qq[2] = (((MUP_Zero | p[4])<<8)|p[5]);       //16位陀螺仪
    
  q[0] = qq[0]>>2;
  q[1] = qq[1]>>2;
  q[2] = qq[2]>>2;     //14位加速度
}

void Angle_collect()
{

}    
//*================================================================================
//*================================================================================
//*================================================================================

short gyro_x,gyro_y,gyro_z;
short accle_x,accle_y,accle_z;  

//*-------------------------------------------------------------------------------------------------
//*-------------------------------------------------------------------------------------------------
#define SIM_SCGC4_REG(base)                      ((base)->SCGC4)
#define SIM_SCGC4_I2C1_MASK                      0x80u

#define PORT_PCR_MUX_MASK                        0x700u
#define PORT_PCR_MUX_SHIFT                       8
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_MUX_SHIFT))&PORT_PCR_MUX_MASK)

#define I2C_F_MULT_MASK                          0xC0u
#define I2C_F_MULT_SHIFT                         6
#define I2C_F_MULT(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_F_MULT_SHIFT))&I2C_F_MULT_MASK)

#define I2C_F_ICR_MASK                           0x3Fu
#define I2C_F_ICR_SHIFT                          0
#define I2C_F_ICR(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_F_ICR_SHIFT))&I2C_F_ICR_MASK)

void I2C_init(I2Cnn i2cn)
{


    
    
        /* 开启时钟 */
        SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;         //开启 I2C1时钟

        /* 配置 I2C1功能的 GPIO 接口 */
        if(I2C1_SCL == PTE1)
        { PORTE_PCR1 = PORT_PCR_MUX(6);
          PORTE_PCR1 = PORTE_PCR1|(1<<5);//设置为开漏模式
        }
        else if(I2C1_SCL == PTC10)
        {
          PORTC_PCR10 = PORT_PCR_MUX(2);
          PORTC_PCR10 = PORTC_PCR10|(1<<5);//设置为开漏模式
        }
        else
            ;

        if(I2C1_SDA == PTE0)
        { PORTE_PCR0 = PORT_PCR_MUX(6);
          PORTE_PCR0 = PORTE_PCR0 | (1<<5);//设置成开漏模式   只设置SDA即可，两个都设置也没事
        }
        else if (I2C1_SDA == PTC11)
        {
          PORTC_PCR11 = PORT_PCR_MUX(2);
          PORTC_PCR11 = PORTC_PCR11 | (1<<5);//设置为开漏模式
        }
        else
          ;

    

    /* 设置频率 */
    I2C_F_REG(I2Cxx[i2cn])  = I2C_F_MULT(0) | I2C_F_ICR(00) ;  //0x29
    // MULT=00  即  mul = 1
    // ICR =14  ICR为Clock rate时钟频率
    // 从《k16 reference manual.pdf》P1460 可得：
    // ICR    SCL Divider   SDA Hold Value    SCL Hold (Start) Value    SCL Hold (Stop) Value
    //  29       384              33                   190                        193
    //  17       128              21                   58                         65
    // I2C baud rate = bus speed (Hz)/(mul × SCL divider)  即这里 90MHz/(1 ×128)=234.375kHz

    /* 使能 IIC1 */
    I2C_C1_REG(I2Cxx[i2cn]) = I2C_C1_IICEN_MASK;
}
//*-----------------------------------------------------------------------------------------------------
short get_angle (unsigned char i)
{
   
    I2C_init(I2C11);
    Init2100();//加速度计  
    Init8700();//陀螺仪
 
   short gyro[3];
   short acc[3];
   short jiao_jia_sudu[6];
   unsigned char bit8_data[6];
   

      
      IIC_Read(SlaveAddress2100,bit8_data,gyro,acc);//16位精度    陀螺仪 
   //    jiao_jia_sudu[0]= gyro[0];
      jiao_jia_sudu[1]= gyro[1];
      jiao_jia_sudu[2]= gyro[2];

      IIC_Read(SlaveAddress8700,bit8_data,gyro,acc);//14位精度   加速度
//      jiao_jia_sudu[3]= acc[0];
//      jiao_jia_sudu[4]= acc[1];
      jiao_jia_sudu[5]= acc[2];

      return jiao_jia_sudu[i];
}
//============================================================================
unsigned int Read_ADC_Ave(unsigned char ch,unsigned char N)
{
    short sum=0;
    unsigned char i;
    for(i=0;i<N;i++)
    {
        sum =sum+get_angle(ch);
         
    }
    return sum/N;
}
