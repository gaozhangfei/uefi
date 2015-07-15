/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpd1z2bQhhV6cC9dn956MYMwSDGfeyc2k9r9TKK9bTrj6YPvsPEr44pJLAZWy
NJXlVR48rJ8J8IPnmjE+kMhzoFHqWonLCYLUWmp+lFB2eXwM4liKIpp/a7eMxJTXsE5QLTFK
+VM/w2lJ6nD/IM6t8FbcpxWb+0DscgG2TvJWrCPVHsBkHUc1IiHmBbhdEJVPmQ==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/*************************************************

Copyright (C), 1988-2010, Huawei Tech. Co., Ltd.

File name: HwProductsPkg\Pv660Evb\Library\OemMiscLibEvb\OemMiscLibEvb.c


Description: 

*************************************************/

#include <Uefi.h>

#include <Library/DebugLib.h>
#include <Library/IoLib.h>

#include <Library/CpldApi.h>
#include <Library/OemMiscLib.h>
#include <PlatformArch.h>
#include <Library/PlatformSysCtrlLib.h>
#include <Library/OemAddressMapLib.h>
#include <Library/SerialPortLib.h>
#include <Library/CpldD02.h>

// Right now we only support 1P
BOOLEAN OemIsSocketPresent (UINTN Socket)
{
  if (0 == Socket)
  {
    return TRUE;
  }

  return FALSE;
}

UINTN OemGetSocketNumber (VOID)
{
    return 1;
}

UINTN OemGetDimmSlot(UINTN Socket, UINTN Channel)
{
    return 2;
}

VOID OemPostEndIndicator (VOID)
{
}

EFI_STATUS OemMemoryTest (UINTN Base, UINTN Size)
{
  return EFI_SUCCESS;
}

EFI_STATUS OemSelfTestReport(IN E_SELF_TEST_ITEM Item, IN U_SELF_TEST_PARA Para)
{
    return EFI_SUCCESS;
}

/*****************************************************************************
 �� �� ��  : OemSetSelfTestFailFlagAndAlarm
 ��������  : �����Լ�ʧ�ܱ��λ����Alarm��
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��4��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*****************************************************************************/
VOID OemSetSelfTestFailFlagAndAlarm()
{
    return;
}

/*****************************************************************************
 �� �� ��  : OemGetCurrentBiosChannel
 ��������  : 
 �������  : VOID  
 �������  : ��
 �� �� ֵ  : 0 ��BIOS
             1 ��BIOS
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��4��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*****************************************************************************/
UINT32 OemGetCurrentBiosChannel(VOID)
{
    UINTN RegAddr = CPLD_BIOS_CURRENT_CHANNEL_REG_D02;
    UINT8 Value;

    Value = ReadCpldReg(RegAddr);
    if (1==(Value&0x01))
    {
        return 1;
    }
    else
    {
        return 0;
    }
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
EFI_STATUS OemCheckCpld(VOID)
{
    return EFI_SUCCESS;
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
EFI_STATUS OemCheckClock(VOID)
{
    return EFI_SUCCESS;
}
/*****************************************************************************
 �� �� ��  : OemGetCurrentResetCause
 ��������  : ��ȡ��ǰ��λԭ��
 �������  : VOID  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���
*****************************************************************************/
E_RESET_CAUSE OemGetCurrentResetCause(VOID)
{
    return RESET_CAUSE_CPLD_BUTT;
}

/*****************************************************************************
 �� �� ��  : OemGetCurrentResetCause
 ��������  : ��ȡǰһ�θ�λԭ��
 �������  : VOID  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���
*****************************************************************************/
E_RESET_CAUSE OemGetPer2ResetCause(VOID)
{
    return RESET_CAUSE_CPLD_BUTT;
}

/*****************************************************************************
 �� �� ��  : OemGetCurrentResetCause
 ��������  : ��ȡ���ϴδθ�λԭ��
 �������  : VOID  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���
*****************************************************************************/
E_RESET_CAUSE OemGetPer3ResetCause(VOID)
{
    return RESET_CAUSE_CPLD_BUTT;
}

/*****************************************************************************
 �� �� ��  : OemIsWarmBoot
 ��������  : ��ȡ���ϴδθ�λԭ��
 �������  : UINT32  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��26��
    ��    ��   : L00228991
    �޸�����   : �����ɺ���
*****************************************************************************/
UINT32 OemIsWarmBoot(VOID)
{
    return 0;
}


/*******************************************************************************
 �� �� ��  : CoreSelectBoot
 ��������  : �ж�ĳ����CPU BIST�����ʧ�ܾ͸���
 �������  : �� 
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��15��
    ��    ��   : c00213799
    �޸�����   : �����ɺ���
*******************************************************************************/
VOID CoreSelectBoot(VOID)
{
    if (!PcdGet64 (PcdTrustedFirmwareEnable))
    {
        StartupAp ();
    }

    return;
}

/*****************************************************************************
 �� �� ��  : OemBiosSwitch
 ��������  : �л�����BIOS
 �������  : UINT32 Master 0 �л�����BIOS
                           1 �л�����BIOS 
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��18��
    ��    ��   : ZhuGeSheng
    �޸�����   : �����ɺ���

*****************************************************************************/
VOID OemBiosSwitch(UINT32 Master)
{
    return;
}
/*******************************************************************************
 �� �� ��  : SetCpldBootDeviceID
 ��������  : 
 �������  : Result  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : l00306713
    �޸�����   : �����ɺ���
*******************************************************************************/
VOID SetCpldBootDeviceID(IN UINT8 Value)
{        
    return;
}

/*******************************************************************************
 �� �� ��  : ReadCpldBootDeviceID
 ��������  : 
 �������  : Result  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : l00306713
    �޸�����   : �����ɺ���
*******************************************************************************/
UINT8 ReadCpldBootDeviceID(void)
{  
    return 0;
}


/*******************************************************************************
 �� �� ��  : SetCpldBootDeviceCount
 ��������  : 
 �������  : Result  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : l00306713
    �޸�����   : �����ɺ���
*******************************************************************************/
VOID SetCpldBootDeviceCount(IN UINT8 Value)
{       
    return;
}

/*******************************************************************************
 �� �� ��  : ReadCpldBootDeviceCount
 ��������  : 
 �������  : Result  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : l00306713
    �޸�����   : �����ɺ���
*******************************************************************************/
UINT8 ReadCpldBootDeviceCount(void)
{
    return 0;
}


VOID OemWriteProtectSet(BOOLEAN val)
{
    return;
}
/*******************************************************************************
 �� �� ��  : OemIsSpiFlashWriteProtected
 ��������  : �Ƿ���Ŀ��Ҫspi flashд�������ܣ����뷵��TRUE,�񷵻�FALSE
 �������  : ��  
 �������  : TRUE/FALSE
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��12��17��
    ��    ��   : l00306713
    �޸�����   : �����ɺ���
*******************************************************************************/
BOOLEAN OemIsSpiFlashWriteProtected(VOID)
{
    return FALSE;
}

VOID OpenAlarmLed(VOID)
{
    //UINT8 Value;
    //��������澯��
    //Value = ReadCpldReg(CPLD_BIOS_WR_REG);
    //Value |= CPLD_ALARM_LED_FIG;
    //WriteCpldReg(CPLD_BIOS_WR_REG, Value);
    
    return ;
}

EFI_STATUS OemCk420Read(UINT16 InfoOffset, UINT32 ulLength, UINT8 *pBuf)
{
    return EFI_SUCCESS;
}

EFI_STATUS OemCk420Write(UINT16 InfoOffset, UINT32 ulLength, UINT8 *pBuf)
{
    return EFI_SUCCESS;
}

VOID OemPcieCardReset(UINT32 Reset)
{
    return;
}

UINT32  SelDdrFreq(pGBL_DATA pGblData)
{
    return DDR_MAX;
}

UINT8 OemAhciStartPort(VOID)
{
    return 0;
}

EFI_STATUS GetSetupData(SETUP_PARAMS *Setup)
{
    return EFI_UNSUPPORTED;
}



UINT16 OemSataNum = 0;
UINT16 OemSataDesSize = 0;
UINT16 OemPXENum = 0;
UINT16 OemPXEDesSize = 0;
UINT8 OemFirstBootId = 0;

EFI_STATUS OemGetSataBootNum(UINTN SataDesSize)
{
    if( SataDesSize/sizeof(UINT16) > 0)
    {
        OemSataDesSize = SataDesSize;
        OemSataNum = OemSataDesSize/sizeof(UINT16);
    }
    else{    
        OemSataNum = 0;    
    }

    return EFI_SUCCESS;
}

EFI_STATUS OemGetPXEBootNum(UINTN PXESize)
{
    if( PXESize/sizeof(UINT16) > 0)
    {
        OemPXEDesSize = PXESize;
        OemPXENum = OemPXEDesSize/sizeof(UINT16);
    }else{
        OemPXENum = 0;
    }

    return EFI_SUCCESS;
}

EFI_STATUS OemPreStartBootOptionAction(UINT16 BootOptionId)
{
    return EFI_SUCCESS;
}

EFI_STATUS OemBootOrderSeting(IN OUT UINT16* BootOrderFinal, UINTN BootOrderSize, BOOLEAN *IsFlashBootFirst, BOOLEAN *FlashBootSupport)
{
    *FlashBootSupport = FALSE;
    if(BootOrderFinal == NULL || BootOrderSize == 0)
    {
        return EFI_INVALID_PARAMETER;
    }
    return EFI_SUCCESS;    
}

