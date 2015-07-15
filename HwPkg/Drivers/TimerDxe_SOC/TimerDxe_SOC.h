/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCm4LaY1Scm2hXUYwJOrTRw72N6o5uLoxSzomTX
lSLrv6ZQqmxIHFjU9RVPs8oWP0fCWQPu/vYQf72+peri0N73nqcW5U6qtEodlblkR5pyA34j
SpfLnLkKMnfRlk+0rxfZPd+4T1huFulnnkm2UA2FK1ZkSO4GtAhePB7YYvHFTw==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
#ifndef _TIMERDXE_SOC_H_
#define _TIMERDXE_SOC_H_

#define TIMER_SUBCTRL_BASE   PcdGet64(PcdPeriSubctrlAddress)
#define SC_TM_CLKEN0_REG     (0x2050)

#define TIMER_SOC_FREQ       200000000


/****************** Timer32 register addresses offset start ***********************/
#define SRE_HITIMER32_LOAD_OFFSET              (0x0)      /* ������ֵ�Ĵ���*/
#define SRE_HITIMER32_VALUE_OFFSET             (0x4)      /* ��ǰ����ֵ�Ĵ���*/
#define SRE_HITIMER32_CNTL_OFFSET              (0x8)      /* ���ƼĴ���*/
#define SRE_HITIMER32_INTC_OFFSET              (0xC)      /* �ж�����Ĵ�����ֻд*/
#define SRE_HITIMER32_RIS_OFFSET               (0x10)     /* �ж�״̬�Ĵ���(RIS), ֻ��*/
#define SRE_HITIMER32_MIS_OFFSET               (0x0014)   /* �ж�״̬�Ĵ���(MIS),ֻ��*/
#define SRE_HITIMER32_BGLOAD_OFFSET            (0x18)     /* ����ģʽ������ֵ������*/
/****************** end ******************************/


#define SRE_HITIMER_ENCLK_SEL_BIT              (1 << 7)   /* Timer enable flag */



#define SRE_HITIMER_DEFAULT_TICKS              100        /* Ĭ��Timer�ĵ�tick�� */
#define SRE_HITIMER_RELOAD_TICKS               1          /* �ӳ�tick */

#define SRE_HITIMER_INT_CLEAR                  (0x01)     /* ���ж�*/
#define SRE_HITIMER_CNTL_ENABLE                (0x80)     /* Ĭ��ʹ��timer */
#define SRE_HITIMER_CNTL_MODE                  (0x40)     /* ����ģʽ 0: һ���Լ���;   1: ����ģʽ Ĭ������ģʽ*/
#define SRE_HITIMER_CNTL_IRQ_ENABLE            (0x20)     /* 0�����жϣ�1�������ж�,Ĭ�ϲ�����*/
#define SRE_HITIMER_CNTL_SIZEMODE              (0x2)      /* 32bit : 0 16bit��������1 32bit������,Ĭ��32bit������
                                                                                      64bit: 0 32bit��������1 64bit������    */

UINT32 gRegBase ;

#define TMERSOC_REG_READ(addr,value)    ((value) = (*(volatile unsigned int *)(UINTN)(addr)))    

#define TMERSOC_REG_WRITE(addr,value)   ((*(volatile unsigned int *)(UINTN)(addr)) = (value))  



#endif

