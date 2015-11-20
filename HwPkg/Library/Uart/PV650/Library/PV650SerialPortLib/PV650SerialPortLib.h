/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCm4LaY1Scm2hXUYwJOrTRw72N6o5uLoxSzomTX
lSLrv4MJuO9Rs9UpcpOwuiRDUa+wXPrrfKHb0yf/8p5LP/GmN+5SVXT02xnlWPegL0jY6bGK
KtB21iYdwjOHnQNKEeuanocfCfYWwqRUiK4LNOZvsICa89ydiuWeNR/6Y7bCBQ==*/
/*--------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/

#ifndef __PV650_SERIALPORTLIB_H__
#define __PV650_SERIALPORTLIB_H__


#define UART_USED_CHANNELS          1           /*ʹ�õĴ�������*/
#define TCXO_CLK_FREQ               26000000    /*ʱ��Ƶ��*/

// �Ĵ�������ַͨ��PCD��ָ��
#define SERIAL_0_BASE_ADR     (PcdGet64(PcdSerialRegisterBase)) /*uart0 ��ַ*/

// �����ʴ�PCD��ȡ
#define UART_SEND_DELAY      (PcdGet32(PcdSerialPortSendDelay))     /*���͵ȴ�ʱ��*/  //modified by t00216239
#define BAUDRATE             (PcdGet64(PcdUartDefaultBaudRate))

/*���ڼĴ�����ַ����*/
#define UART_THR_REG         (SERIAL_0_BASE_ADR + UART_THR)
#define UART_RBR_REG         (SERIAL_0_BASE_ADR + UART_RBR)
#define UART_DLL_REG         (SERIAL_0_BASE_ADR + UART_DLL)
#define UART_DLH_REG         (SERIAL_0_BASE_ADR + UART_DLH)
#define UART_IEL_REG         (SERIAL_0_BASE_ADR + UART_IEL)
#define UART_IIR_REG         (SERIAL_0_BASE_ADR + UART_IIR)
#define UART_FCR_REG         (SERIAL_0_BASE_ADR + UART_FCR)
#define UART_LCR_REG         (SERIAL_0_BASE_ADR + UART_LCR)
#define UART_LSR_REG         (SERIAL_0_BASE_ADR + UART_LSR)
#define UART_USR_REG         (SERIAL_0_BASE_ADR + UART_USR)

/* �Ĵ�����ַƫ�ƶ��� */
#define UART_RBR     0x00            /* �������ݻ���Ĵ�����*/
#define UART_THR     0x00            /* �������ݻ���Ĵ���*/
#define UART_DLL     0x00            /* �����ʵ�λ��Ƶ���ӼĴ���*/
#define UART_DLH     0x04            /* �����ʸ�λ��Ƶ���ӼĴ���*/
#define UART_IEL     0x04            /* �ж�ʹ�ܼĴ���*/
#define UART_IIR     0x08            /* �жϱ�ʶ���ƼĴ���*/
#define UART_FCR     0x08            /* FIFO���ƼĴ���*/
#define UART_LCR     0x0C            /* ���Կ��ƼĴ���*/
#define UART_MCR     0x10            /*  Modem���ƼĴ���*/
#define UART_LSR     0x14            /* ����״̬�Ĵ���*/
#define UART_USR     0x7C            /* ״̬�Ĵ���*/

/* register definitions */

#define UART_FCR_EN		     0x01		/* FIFOʹ�� */
#define UART_FCR_RXCLR       0x02		/* �����FIFO */
#define UART_FCR_TXCLR       0x04		/* �巢��FIFO */
#define UART_FCR_CLEARFIFO   0x00      /* ��FIFO����ˮ��*/
#define UART_FCR_RXL1        0x00
#define UART_FCR_RXL4        0x40
#define UART_FCR_RXL8        0x80
#define UART_FCR_RXL14       0xc0
#define UART_FCR_TXL0        0x00
#define UART_FCR_TXL4        0x20
#define UART_FCR_TXL8        0x30
#define UART_FCR_TXL14       0x10

/*LCR �� Name: Line Control Register fields*/
#define UART_LCR_DLAB   0x80     /*0����ֹ��1��ʹ�ܷ���Ƶ����������*/
#define UART_LCR_EPS    0x10     /*0-��У��;1-żУ��*/
#define UART_LCR_PEN    0x08     /*0���رգ�1��ʹ����żУ�飻*/
#define UART_LCR_STOP   0x04     /* 0 = 1 ֹͣλ;1=1.5��2��ֹͣλ*/
#define UART_LCR_DLS8   0x03    /* 00=5bit;01=6bit;10=7bit;11=8bit ����*/
#define UART_LCR_DLS7   0x02    /* 00=5bit;01=6bit;10=7bit;11=8bit ����*/
#define UART_LCR_DLS6   0x01    /* 00=5bit;01=6bit;10=7bit;11=8bit ����*/
#define UART_LCR_DLS5   0x00    /* 00=5bit;01=6bit;10=7bit;11=8bit ����*/

/*DLL DLH �� Name: �����ʷ�Ƶ���ӼĴ�������*/
#define UART_DLH_AND_DLL_WIDTH 0xFF /* �ߵ�λ���������ڱ����Ƶ���ӵ�bitλ��*/

/*IER �� Name: �ж�ʹ�ܼĴ�������*/
#define UART_IER_PTIME  0x80     /*����FIFO���ж�ʹ��*/
#define UART_IER_ELSI   0x04    /* ���մ����ж�ʹ��*/
#define UART_IER_ETBEI  0x02    /* ���ͻ���Ĵ������ж�ʹ��*/
#define UART_IER_ERBFI  0x01    /* ����������Ч�ж�ʹ��*/

/*IIR �� Name: �жϱ�ʶ�Ĵ�������*/
#define UART_IIR_FIFOSE         0xC0    /* FIFOs ʹ��: 00 = ��ֹ; 11 = ʹ��;*/
/*�ж�����ָʾ:
0x1�����жϣ�
0x2�����Ϳ��жϣ�����FIFO�պͷ��ͻ���Ĵ����գ�FIFO��ʹ�ܣ������жϣ�
0x4������������Ч�жϣ�����FIFO���ͽ��ջ���Ĵ�������FIFO��ʹ�ܣ���������´������жϣ�
0x6�����մ����жϣ����������������żУ�����֡�����Լ������쳣�жϣ�
0x7��æ�жϡ���UART�շ����ݣ�����æ״̬��USR[0]=1��ʱ���������������LCR�Ĵ��������������жϣ�
0xC����ʱ�жϡ���������4���ַ��շ�ʱ����û������������������FIFO���������жϡ�*/

#define UART_IIR_InterruptID   0x01    
#define UART_IIR_INTIDTE       0x02
#define UART_IIR_INTIDRA       0x04
#define UART_IIR_INTIDRLS      0x06
#define UART_IIR_INTMASK       0x0f
#define UART_IIR_RDA           0x04
#define UART_IIR_TE            0x02
/*LSR �� Name: ����״̬�Ĵ�������*/
#define UART_LSR_TEMT       0x40    /* �������λ*/
#define UART_LSR_THRE       0x20    /* ���ͻ���Ĵ������λ*/
#define UART_LSR_BI         0x10    /* Break�ж�λ*/
#define UART_LSR_FE         0x08    /* ֡����λ*/
#define UART_LSR_PE         0x04    /* У�����λ*/
#define UART_LSR_R          0x02    /* �������λ*/
#define UART_LSR_DR         0x01    /* ����׼��λ*/

/*USR�� Name: ����״̬�Ĵ���*/
#define UART_USR_BUSY  0x01    /*����æ*/

#define FIFO_MAXSIZE    32      /* FIFO������ */

extern UINT32 UART_UartClkFreq(VOID);
extern UINT8 UART_ChkSndEnd(VOID);
extern UINT8 SerialPortReadChar(VOID);
extern VOID SerialPortWriteChar(UINT8 scShowChar);

#endif

