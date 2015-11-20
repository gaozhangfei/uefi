/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:64z4jYnYa5t1KtRL8a/vnMxg4uGttU/wzF06xcyNtiEfsIe4UpyXkUSy93j7U7XZDdqx2rNx
p+25Dla32ZW7oqucjIznazmY6pDerNNjrJRr3zqJWqMJ5g20Wh4kav+V4LjOmY/LHDS62FHF
VyAlAx7njIhP2ajqOJoJMnWikMp6aeZYsDDNCTEd6durgfJhGERE0UQ4K0IX5pjCzmhx5KT8
JLJLy94mv/xl5MsurpJd8GDGiPjUNa6QjAd1O/3TrizuqbFyPIj/+5eqpagAdg==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

******************************************************************************
  �� �� ��   : CpldDriver.c
  �� �� ��   : ����
  ��    ��   : 
  ��������   : 
  ����޸�   :
  ��������   : CPLD����
  
  �޸���ʷ   :
  1.��    ��   : 
    ��    ��   : 
    �޸�����   : �����ļ�
******************************************************************************/

#include <Library/CpldApi.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <PlatformArch.h>
#include <Library/OemAddressMapLib.h>
#include <Library/OemMiscLib.h>

/*****************************************************************************
  �� �� ��  : CPLD_WriteReg
  ��������  : CPLD�Ĵ���д
  �������  : UINT32 ulAddr
            UINT32 ulValue
  �������  : ��
  �� �� ֵ  : VOID
*****************************************************************************/
VOID WriteCpldReg(UINTN ulRegAddr, UINT8 ulValue)
{
    *(volatile UINT8 *)(ulRegAddr + PcdGet64(PcdCpldBaseAddress)) = ulValue;
}

/*****************************************************************************
  �� �� ��  : CPLD_ReadReg
  ��������  : CPLD�Ĵ�����
  �������  : void  
  �������  : ��
  �� �� ֵ  : UINT32
*****************************************************************************/
UINT8 ReadCpldReg(UINTN ulRegAddr)
{    
    volatile UINT8 ulValue = 0;
    
    ulValue = *(volatile UINT8 *)(ulRegAddr + PcdGet64(PcdCpldBaseAddress));
    
    return (ulValue); 
}

/*******************************************************************************
 �� �� ��  : CheckCpld
 ��������  : CPLD�Լ�
 �������  : �� 
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��11��24��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*******************************************************************************/
EFI_STATUS CheckCpld(VOID)
{
    UINTN RegAddr = CPLD_LPC_DEBUG_REG;
    UINT8 TestFlag = CPLD_CPLD_CHECK_FLAG;
    UINT8 Value;

    WriteCpldReg(RegAddr, TestFlag);
    Value = ReadCpldReg(RegAddr);
    Value = ~Value;  //ȡ��
    if (TestFlag == Value)
    {
        return EFI_SUCCESS;    
    }
    else
    {
        return EFI_ABORTED;    
    }
}

/*******************************************************************************
 �� �� ��  : CheckClock
 ��������  : ʱ���Լ�
 �������  : �� 
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��11��24��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*******************************************************************************/
EFI_STATUS CheckClock(VOID)
{
    UINTN RegAddr = CPLD_CLK_DET_REG;
    UINT8 Mask = CPLD_CLOCK_CHECK_FLAG;
    UINT8 Value;

    Value = ReadCpldReg(RegAddr);
    if (CPLD_CLOCK_CHECK_FLAG == BIT_GET(Value, Mask))
    {
        return EFI_SUCCESS;    
    }
    else
    {
        return EFI_ABORTED;    
    }
}

/*****************************************************************************
 �� �� ��  : GetCurrentBiosChannel
 ��������  : ��ȡ��ǰ����Ϊ��bios���Ǳ�bios
 �������  : VOID  
 �������  : ��
 �� �� ֵ  : 0 ��BIOS
             1 ��BIOS
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��3��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*****************************************************************************/
UINT32 GetCurrentBiosChannel(VOID)
{
    UINTN RegAddr = CPLD_BIOS_CURRENT_CHANNEL_REG;
    UINT8 Value;

    Value = ReadCpldReg(RegAddr);
    if (CPLD_BIOS_CURRENT_CHANNEL_MASK == BIT_GET(Value, CPLD_BIOS_CURRENT_CHANNEL_MASK))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**************************************************
��������  :   SetSelfTestFailFlagAndAlarm()
����˵��  :   �����Լ�ʧ�ܱ�־����������澯�Ƹ澯��
�������  :   ��������
**************************************************/
VOID
SetSelfTestFailFlagAndAlarm(VOID)
{
    UINT8 uLedFlag   = 0;
    UINT8 uEventFlag = CPLD_SELFTEST_FAILURE_FLAG;
    
    WriteCpldReg(CPLD_SELFTEST_FAILURE_REG, uEventFlag);

    uLedFlag = ReadCpldReg(CPLD_BIOS_WR_REG);
    uLedFlag |= CPLD_ALARM_LED_FIG;
    WriteCpldReg(CPLD_BIOS_WR_REG, uLedFlag);
}

