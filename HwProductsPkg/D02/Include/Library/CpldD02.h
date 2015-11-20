/******************************************************************************

                  ��Ȩ���� (C), 2001-2011, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : CpldD02.h
  �� �� ��   : ����
  ��    ��   : 
  ��������   : 
  ����޸�   :
  ��������   : Linaro D02 CPLD��ؼĴ�����cpldapi.h�ж��岻һ���ĵط��ȷ�������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2015/1/26
    ��    ��   : s00296804
    �޸�����   : �����ļ�
  2.��    ��   : 
    ��    ��   : 
    �޸�����   : 

******************************************************************************/

#ifndef __CPLDD02_H__
#define __CPLDD02_H__
#define CPLD_LOGIC_VERSION                (0x52)
#define PRODUCT_VERSION                   (0x53)
#define CPLD_LOGIC_COMPLIER_YEAR          (0x54)
#define CPLD_LOGIC_COMPLIER_MONTH         (0x55)
#define CPLD_LOGIC_COMPLIER_DAY           (0x56)
#define CPLD_LOGIC_COMPLIER_HOUR          (0x57)
#define CPLD_LOGIC_COMPLIER_MINUTE        (0x58)
#define BOARD_ID                          (0x59)
#define BOM_VERSION                       (0x5A)
#define CPLD_BIOS_CURRENT_CHANNEL_REG_D02 (0x5B)

//PCIE�⸴λ ���Linaro D02��
#define CPU0_PCIE1_RESET_REG                (0x12)
#define CPU0_PCIE2_RESET_REG                (0x13)
#define CPU1_PCIE1_RESET_REG                (0x14)
#define CPU1_PCIE2_RESET_REG                (0x15)

#endif /* __CPLDD02_H__ */
