/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCTRBL0qDbp15usAFL4sRUNaa9PQnehq7qerrn2
LiJ41omhITDeCH/C4foR79uiWZBenn+D71khwAjSMtqE06geZFKbTvn0y5Ny8Gb0pRReuPVw
PohMSutCKQUqUscz5cZTh7VMSm/lzVfxl78+2sOPB7qtPbNytjaTz2/X+3ivwA==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : I2CHw.h
  �� �� ��   : v1.0
  ��    ��   : C00213799
  ��������   : 2013��03��15��
  ����޸�   :
  ��������   : ͷ�ļ�
  �޸���ʷ   :
1.   ��	  ��   : 
     ��	  ��   :
     �޸�����  :
******************************************************************************/

#ifndef _I2C_HW_H_
#define _I2C_HW_H_

#define I2C_READ_TIMEOUT             500
#define I2C_DRV_ONCE_WRITE_BYTES_NUM 8        /*I2C������,ÿ��д�������д8bytes*/
#define I2C_DRV_ONCE_READ_BYTES_NUM  8        /*I2C������,ÿ�ζ���������8bytes*/
#define I2C_READ_SIGNAL              0x0100               /*���ź�*/
#define I2C_TXRX_THRESHOLD           0x7
#define I2C_SS_SCLHCNT               0x493      /*��׼�ٶ�ģʽ��SCLʱ�Ӹߵ�ƽ������*/
#define I2C_SS_SCLLCNT               0x4fe      /*��׼�ٶ�ģʽ��SCLʱ�ӵ͵�ƽʱ��*/

#define I2C_REG_WRITE(reg,data) \
     (*((volatile UINT32 *)(reg)) = (data))

#define I2C_REG_READ(reg,result) \
     (result) = *((volatile UINT32 *)(UINTN)(reg))

 /*�Ĵ�����������ö�ٶ���*/
 #define    I2C_CON_OFFSET                 0x0 /* I2C0_CONΪI2C0���ƼĴ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_TAR_OFFSET                 0x4 /* I2C0_TARΪI2C0����Slave��ַ�Ĵ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_SAR_OFFSET                 0x8 /* I2C0_SARΪI2C0����Դ�豸��ַ�Ĵ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_DATA_CMD_OFFSET            0x10 /* I2C0_DATA_CMDΪI2C0����ͨ���Ĵ����� */
 #define    I2C_SS_SCL_HCNT_OFFSET         0x14 /* I2C0_SS_SCL_HCNTΪ��׼�ٶ��µ�SCLʱ�Ӹߵ�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_SS_SCL_LCNT_OFFSET         0x18 /* I2C0_SS_SCL_LCNTΪ��׼�ٶ��µ�SCLʱ�ӵ͵�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_FS_SCL_HCNT_OFFSET         0x1c /* I2C0_FS_SCL_HCNTΪ�����ٶ��µ�SCL ʱ�Ӹߵ�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_FS_SCL_LCNT_OFFSET         0x20 /* I2C0_FS_SCL_LCNTΪ�����ٶ��µ�SCL ʱ�ӵ͵�ƽʱ��Ĵ�����I2C0_FS_SCL_LCNTֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 #define    I2C_INTR_STAT_OFFSET           0x2c /* I2C0_INTR_STATΪI2C0�ж�״̬�Ĵ����� */
 #define    I2C_INTR_MASK_OFFSET           0x30 /* I2C0_INTR_MASKΪI2C0�ж����μĴ����� */
 #define    I2C_RAW_INTR_STAT_OFFSET       0x34 /* I2C0_RAW_INTR_STATΪI2C0ԭʼ�ж�״̬�Ĵ����� */
 #define    I2C_RX_TL_OFFSET               0x38 /* I2C0_RX_TLΪRX_FIFO��ֵ�Ĵ����� */
 #define    I2C_TX_TL_OFFSET               0x3c /* I2C0_TX_TLΪTX_FIFO��ֵ�Ĵ����� */
 #define    I2C_CLR_INTR_OFFSET            0x40 /* I2C0_CLR_INTRΪI2C0��ϼ������ж�����Ĵ����� */
 #define    I2C_CLR_RX_UNDER_OFFSET        0x44 /* I2C0_CLR_RX_UNDERΪRX_UNDER�ж�����Ĵ����� */
 #define    I2C_CLR_RX_OVER_OFFSET         0x48 /* I2C0_CLR_RX_OVERΪRX_OVER�ж�����Ĵ����� */
 #define    I2C_CLR_TX_OVER_OFFSET         0x4c /* I2C0_CLR_TX_OVERΪTX_OVER�ж�����Ĵ����� */
 #define    I2C_CLR_RD_REQ_OFFSET          0x50 /* I2C0_CLR_RD_REQΪRD_REQ�ж�����Ĵ����� */
 #define    I2C_CLR_TX_ABRT_OFFSET         0x54 /* I2C0_CLR_TX_ABRTΪTX_ABRT�ж�����Ĵ����� */
 #define    I2C_CLR_RX_DONE_OFFSET         0x58 /* I2C0_CLR_RX_DONEΪRX_DONE�ж�����Ĵ����� */
 #define    I2C_CLR_ACTIVITY_OFFSET        0x5c /* I2C0_CLR_activityΪactivity״̬�Ĵ����� */
 #define    I2C_CLR_STOP_DET_OFFSET        0x60 /* I2C0_CLR_STOP_DETΪSTOP_DET�ж�����Ĵ����� */
 #define    I2C_CLR_START_DET_OFFSET       0x64 /* I2C0_CLR_START_DETΪSTART_DET�ж�����Ĵ����� */
 #define    I2C_CLR_GEN_CALL_OFFSET        0x68 /* I2C0_CLR_GEN_CALLΪGeneral Call�ж�����Ĵ����� */
 #define    I2C_ENABLE_OFFSET              0x6c /* I2C0_ENABLEΪI2C0����ģʽʹ�ܼĴ����� */
 #define    I2C_STATUS_OFFSET              0x70 /* I2C0_STATUSΪI2C0״̬�Ĵ����� */
 #define    I2C_TXFLR_OFFSET               0x74 /* I2C0_TXFLRΪTX_FIFO�е����ݸ���ָʾ�Ĵ����� */
 #define    I2C_RXFLR_OFFSET               0x78 /* I2C0_RXFLRΪRX_FIFO�е����ݸ���ָʾ�Ĵ����� */
 #define    I2C_SDA_HOLD                   0x7c /* I2C_SDA_HOLDΪSDA�ź���SCL���½��غ��hold�ӳ�ʱ�䡣 */
 #define    I2C_TX_ABRT_SOURCE_OFFSET      0x80 /* I2C0_TX_ABRT_SOURCEΪ����ʧ���ж�Դͷ�Ĵ�������I2C0_CLR_INTR��I2C0_CLR_TX_ABRTʱ��I2C0_TX_ABRT_SOURCE���㡣 */
 #define    I2C_SLV_DATA_ONLY_OFFSET       0x84 /* I2C0_SLV_DATA_ONLYΪslave����NACK�������ƼĴ����� */
 #define    I2C_DMA_CR_OFFSET              0x88 /* I2C0_DMA_CRΪI2C0��DMAͨ���������ƼĴ����� */
 #define    I2C_DMA_TDLR_OFFSET            0x8c /* I2C0_DMA_TDLRΪTX_FIFO DMA������ֵ�Ĵ����� */
 #define    I2C_DMA_RDLR_OFFSET            0x90 /* I2C0_DMA_RDLRΪRX_FIFO DMA������ֵ�Ĵ����� */
 #define    I2C_SDA_SETUP_OFFSET           0x94 /* I2C0_SDA_SETUPΪSDA����ʱ�����üĴ����� */
 #define    I2C_ACK_GENERAL_CALL_OFFSET    0x98 /* I2C0_ACK_GENERAL_CALLΪGeneral CallӦ�����üĴ����� */
 #define    I2C_ENABLE_STATUS_OFFSET       0x9c /* I2C0_ENABLE_STATUSΪI2C0״̬�Ĵ����� */
 
 /* I2C0_CONΪI2C0���ƼĴ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0Con
 {
     struct
     {
        UINT32      master                : 1   ; /* [0] I2C0��ģʽʹ�ܡ� */
        UINT32      spedd                 : 2   ; /* [2..1] �ٶ�ѡ�� */
        UINT32      slave_10bit           : 1   ; /* [3] I2C0��ģʽʹ��ʱ����ģʽ����Ӧ��ַ���ȡ� */
        UINT32      master_10bit          : 1   ; /* [4] ��ģʽ�µ�ַ���ȡ� */
        UINT32      restart_en            : 1   ; /* [5] ��ģʽ��Restartָ���ʹ�ܡ� */
        UINT32      slave_disable         : 1   ; /* [6] I2C0��ģʽʹ�ܡ� */
        UINT32      Reserved_0            : 25  ; /* [31..7] ������ */
     } bits;
     UINT32     Val32;
 } I2C0_CON_U;
 
 /* I2C0_TARΪI2C0����Slave��ַ�Ĵ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0Tar
 {
     struct
     {
         UINT32      ic_tar                : 10  ; /* [9..0] I2C0��ΪMasterʱҪ���ʵ�Slave�ĵ�ַ�� */
         UINT32      gc_or_start           : 1   ; /* [10] ��specialλΪ1ʱ����λ����ִ�е�I2C0��� */
         UINT32      special               : 1   ; /* [11] General Call��Start Byte����ʹ�ܡ� */
         UINT32      ic_10bitaddr_master   : 1   ; /* [12] ��ģʽ�µ�ַ���ȡ� */
         UINT32      Reserved_1            : 19  ; /* [31..13] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_TAR_U;
 
 /* I2C0_DATA_CMDΪI2C0����ͨ���Ĵ����� */
 typedef union tagI2c0DataCmd
 {
     struct
     {
         UINT32      dat                   : 8   ; /* [7..0] ��Ҫ��I2C0�����Ϸ���/���յ����ݡ� */
         UINT32      cmd                   : 1   ; /* [8] I2C0��ΪMasterʱ�Ķ�д������ơ� */
         UINT32      Reserved_5            : 23  ; /* [31..9] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_DATA_CMD_U;
 
 /* I2C0_SS_SCL_HCNTΪ��׼�ٶ��µ�SCLʱ�Ӹߵ�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0����ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0SsSclHcnt
 {
     struct
     {
         UINT32      ic_ss_scl_hcnt        : 16  ; /* [15..0] ��׼�ٶ�ģʽ��SCLʱ�Ӹߵ�ƽ�������� */
         UINT32      Reserved_7            : 16  ; /* [31..16] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_SS_SCL_HCNT_U;
 
 /* I2C0_SS_SCL_LCNTΪ��׼�ٶ��µ�SCLʱ�ӵ͵�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0SsSclLcnt
 {
     struct
     {
         UINT32      ic_ss_scl_lcnt        : 16  ; /* [15..0] ��׼�ٶ�ģʽ��SCLʱ�ӵ͵�ƽʱ�䡣 */
         UINT32      Reserved_9            : 16  ; /* [31..16] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_SS_SCL_LCNT_U;
 
 /* I2C0_FS_SCL_HCNTΪ�����ٶ��µ�SCL ʱ�Ӹߵ�ƽʱ��Ĵ�����ע�⣺ֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0FsSclHcnt
 {
     struct
     {
         UINT32      ic_fs_scl_hcnt        : 16  ; /* [15..0] ����ģʽ��SCLʱ�Ӹߵ�ƽʱ�䣬��������Master Code�� Start Byte��General Call�� */
         UINT32      Reserved_11           : 16  ; /* [31..16] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_FS_SCL_HCNT_U;
 
 /* I2C0_FS_SCL_LCNTΪ�����ٶ��µ�SCL ʱ�ӵ͵�ƽʱ��Ĵ�����I2C0_FS_SCL_LCNTֻ��I2C0��ֹ����I2C0_ENABLE[enable]Ϊ0��ʱ�ſ����á� */
 typedef union tagI2c0FsSclLcnt
 {
     struct
     {
         UINT32      ic_fs_scl_lcnt        : 16  ; /* [15..0] ����ģʽ��SCLʱ�ӵ͵�ƽʱ�䣬��������Master Code�� Start Byte ��General Call�� */
         UINT32      Reserved_13           : 16  ; /* [31..16] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_FS_SCL_LCNT_U;
 
 /* I2C0_INTR_MASKΪI2C0�ж����μĴ����� */
 typedef union tagI2c0IntrMask
 {
     struct
     {
         UINT32      m_rx_under            : 1   ; /* [0] ����������ж����Ρ� */
         UINT32      m_rx_over             : 1   ; /* [1] ����FIFO����ж����Ρ� */
         UINT32      m_rx_full             : 1   ; /* [2] ����FIFO�ﵽ�򳬹���ֵ�ж����Ρ� */
         UINT32      m_tx_over             : 1   ; /* [3] ����FIFO����ж����Ρ� */
         UINT32      m_tx_empty            : 1   ; /* [4] ����FIFO������ߵ�����ֵ�ж����Ρ� */
         UINT32      m_rd_req              : 1   ; /* [5] ���������ж����Ρ� */
         UINT32      m_tx_abrt             : 1   ; /* [6] ������ֹ�ж����Ρ� */
         UINT32      m_rx_done             : 1   ; /* [7] ��������ж����Ρ� */
         UINT32      m_activity            : 1   ; /* [8] activity�ж����Ρ� */
         UINT32      m_stop_det            : 1   ; /* [9] stop detect�ж����Ρ� */
         UINT32      m_start_det           : 1   ; /* [10] start detect�ж����Ρ� */
         UINT32      m_gen_call            : 1   ; /* [11] һ��General Call���󱻽����ж����Ρ� */
         UINT32      Reserved_17           : 20  ; /* [31..12] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_INTR_MASK_U;
 
 /* I2C0_RX_TLΪRX_FIFO��ֵ�Ĵ����� */
 typedef union tagI2c0RxTl
 {
     struct
     {
         UINT32      rx_tl                 : 8   ; /* [7..0] ����FIFO��ֵ�� */
         UINT32      Reserved_21           : 24  ; /* [31..8] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_RX_TL_U;
 
 /* I2C0_TX_TLΪTX_FIFO��ֵ�Ĵ����� */
 typedef union tagI2c0TxTl
 {
     struct
     {
         UINT32      tx_tl                 : 8   ; /* [7..0] ����FIFO��ֵ�� */
         UINT32      Reserved_23           : 24  ; /* [31..8] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_TX_TL_U;
 
 /* I2C0_ENABLEΪI2C0����ģʽʹ�ܼĴ����� */
 typedef union tagI2c0Enable
 {
     struct
     {
         UINT32      enable                : 1   ; /* [0] I2C0����ģʽʹ�ܡ� */
         UINT32      Reserved_47           : 31  ; /* [31..1] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_ENABLE_U;
 
 /* I2C0_STATUSΪI2C0״̬�Ĵ����� */
 typedef union tagI2c0Status
 {
     struct
     {
         UINT32      activity              : 1   ; /* [0] I2C0��Ծ״̬�� */
         UINT32      tfnf                  : 1   ; /* [1] ����FIFOδ����־�� */
         UINT32      tfe                   : 1   ; /* [2] ����FIFO�ձ�־�� */
         UINT32      rfne                  : 1   ; /* [3] ����FIFO�ǿձ�־�� */
         UINT32      rff                   : 1   ; /* [4] ����FIFO����־�� */
         UINT32      mst_activity          : 1   ; /* [5] Master FSM�״̬�� */
         UINT32      slv_activity          : 1   ; /* [6] I2C0 Slave���ܻ״̬�� */
         UINT32      Reserved_49           : 25  ; /* [31..7] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_STATUS_U;
 
 /* I2C0_TXFLRΪTX_FIFO�е����ݸ���ָʾ�Ĵ����� */
 typedef union tagI2c0Txflr
 {
     struct
     {
         UINT32      txflr                 : 4   ; /* [3..0] ����FIFO�е���Ч���ݸ����� */
         UINT32      Reserved_51           : 28  ; /* [31..4] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_TXFLR_U;
 
 /* I2C0_RXFLRΪRX_FIFO�е����ݸ���ָʾ�Ĵ����� */
 typedef union tagI2c0Rxflr
 {
     struct
     {
         UINT32      rxflr                 : 4   ; /* [3..0] ����FIFO���ݸ����Ĵ������üĴ���������ǰ����FIFO�е���Ч���ݸ����� */
         UINT32      Reserved_53           : 28  ; /* [31..4] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_RXFLR_U;
 
 /* I2C0_ENABLE_STATUSΪI2C0״̬�Ĵ����� */
 typedef union tagI2c0EnableStatus
 {
     struct
     {
         UINT32      ic_en                 : 1   ; /* [0] I2C0ʹ�ܡ� */
         UINT32      slv_disable_while_busy: 1   ; /* [1] �����ͻ����æ��ʱ��Slave��ֹ�� */
         UINT32      slv_rx_data_lost      : 1   ; /* [2] Slave�������ݶ�ʧ״̬�� */
         UINT32      Reserved_69           : 29  ; /* [31..3] ������ */
     } bits;
     UINT32      Val32;
 } I2C0_ENABLE_STATUS_U;
 

#endif
