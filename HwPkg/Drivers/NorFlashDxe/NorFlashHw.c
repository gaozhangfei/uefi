/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpZ4X5IrQg0R6dsKjrHb0BePRXyTmI6pqqZK/VsgQAFF+TLEhrrYdasNpB+ZM
CUefd2PjWDZN+DQz+19WNnvANCqt39BE7NKeAk+F0i6xFitynKw/xfmJ4Ihj/0qYD51oshQS
kC1Tf2LvZD/JpniKkXr9Cz0u+n8chYuymK1/O5aS30/swisQ8HFSPVmb86vXhA==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : NorFlashHw.c
  �� �� ��   : v2.0
  ��    ��   : c00213799
  ��������   : 2013��03��04��
  ����޸�   :
  ��������   : Flash����ָ�������
  �޸���ʷ   :
1.   ��	  ��   : 
     ��	  ��   :
     �޸�����  :
******************************************************************************/
#include <PiDxe.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include "NorFlashHw.h"

/*ȫ�ֱ�������*/
BOOLEAN  gFlashBusy = FALSE;  
FLASH_INDEX gIndex = {
    0,
    0,
    0,
    0,
    0,
    0
};

/*****************************************************************************
 �� �� ��  : PortReadData
 ��������  : ����λ���ȡFlash�е�����
 �������  : Index      gFlashInfo���������
            FlashAddr   Flash���Ե�ַ
 �������  : ��
 �� �� ֵ  : ��ȡ������
 �޸���ʷ  :
*****************************************************************************/
UINT32 PortReadData (
    UINT32 Index,
    UINT32 FlashAddr
  )
{
    /* ���ݶ˿ڿ�ȷ���Flash */
    switch (gFlashInfo[Index].ParallelNum)
    {
        case 2:
            return *(volatile UINT32*)(UINTN)FlashAddr;
        case 1:
            return *(volatile UINT16*)(UINTN)FlashAddr;
        /* ����ȱʡΪ����*/
        default:
            DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:illegal PortWidth!\n", __FUNCTION__,__LINE__));
            return 0xffffffff;
    }
}
/*****************************************************************************
 �� �� ��  : PortWriteData
 ��������  : ����λ��д�����ݵ�Flash��
 �������  : Index      gFlashInfo���������
            FlashAddr   Flash���Ե�ַ
            ulInputData: ��Ҫ��������
 �������  : ��
 �� �� ֵ  : �����������
 �޸���ʷ      :
*****************************************************************************/
EFI_STATUS
PortWriteData (
    UINT32 Index,
    UINT32 FlashAddr,
    UINT32 InputData
  )
{
    /* ���ݶ˿ڿ�ȷ���Flash */
    switch (gFlashInfo[Index].ParallelNum)
    {
        case 2:
             *(volatile UINT32*)(UINTN)FlashAddr = InputData;
             break;
        case 1:
             *(volatile UINT16*)(UINTN)FlashAddr = (UINT16)InputData;
             break;
        default:
             DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:illegal PortWidth!\n", __FUNCTION__,__LINE__));
             return EFI_DEVICE_ERROR;
    }
    return EFI_SUCCESS;
}
/*****************************************************************************
 �� �� ��  : PortAdjustData
 ��������  : ����λ��ȡ��λ����
 �������  : Index      gFlashInfo���������
            ulInputData: ��Ҫ��������
 �������  : ��
 �� �� ֵ  : �����������
 �޸���ʷ  :
*****************************************************************************/
UINT32 PortAdjustData( 
    UINT32 Index,
    UINT32 ulInputData
  )
{
    /* ���ݶ˿ڿ�ȵ������� */
    switch (gFlashInfo[Index].ParallelNum)
    {
        case 2:
             return ulInputData;
        case 1:
             return (0x0000ffff & ulInputData );
        default:
            DEBUG((EFI_D_ERROR,"[FLASH_S29GL256N_PortAdjustData]: Error--illegal g_ulFlashS29Gl256NPortWidth!\n\r"));
            return 0xffffffff;
    }
}

/****************************************************************************
 �� �� �� : GetCommandIndex
 ��������  : ��ȡ�����֣����浽ȫ�ֱ���gIndex��
 �������  : Index       - gFlashInfo������
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS        - �����ɹ�
           EFI_DEVICE_ERROR   - �豸����
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS GetCommandIndex(
    UINT32 Index
  )
{
    UINT32 CommandCount = 0;
    UINT32 i;
    UINT8 Flag = 1;
    
    CommandCount = sizeof(gFlashCommandReset) / sizeof(FLASH_COMMAND_RESET);
    for(i = 0;i < CommandCount; i ++ )
    {
        if(gFlashInfo[Index].CommandType & gFlashCommandReset[i].CommandType)
        {
            Flag = 0;
            gIndex.ReIndex = i;
            break;
        }
    }
    //���û���ҵ���λ�����֣�ֱ�ӷ���
    if(Flag)   
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Can not Get Reset Command!\n", __FUNCTION__,__LINE__));
        return EFI_DEVICE_ERROR;
    }
    
    CommandCount = sizeof(gFlashCommandId) / sizeof(FLASH_COMMAND_ID);
    for(Flag = 1,i = 0;i < CommandCount; i ++ )
    {
        if(gFlashInfo[Index].CommandType & gFlashCommandId[i].CommandType)
        {
            Flag = 0;
            gIndex.IdIndex = i;
            break;
        }
    }
    //���û���ҵ���ȡID�����֣�ֱ�ӷ���
    if(Flag)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Can not Get ID Command!\n", __FUNCTION__,__LINE__));
        return EFI_DEVICE_ERROR;
    }

    CommandCount = sizeof(gFlashCommandWrite) / sizeof(FLASH_COMMAND_WRITE);
    for(Flag = 1, i = 0;i < CommandCount; i ++ )
    {
        if(gFlashInfo[Index].CommandType & gFlashCommandWrite[i].CommandType)
        {
            Flag = 0;
            gIndex.WIndex = i;
            break;
        }
    }
    //���û���ҵ�д�����֣�ֱ�ӷ���
    if(Flag)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Can not Get Write Command!\n", __FUNCTION__,__LINE__));
        return EFI_DEVICE_ERROR;
    }

    CommandCount = sizeof(gFlashCommandErase) / sizeof(FLASH_COMMAND_ERASE);
    for(Flag = 1, i = 0;i < CommandCount; i ++ )
    {
        if(gFlashInfo[Index].CommandType & gFlashCommandErase[i].CommandType)
        {
            Flag = 0;
            gIndex.WIndex = i;
            break;
        }
    }
    //���û���ҵ����������֣�ֱ�ӷ���
    if(Flag)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Can not Get Erase Command!\n", __FUNCTION__,__LINE__));
        return EFI_DEVICE_ERROR;
    }

    return EFI_SUCCESS;
}

/****************************************************************************
 �� �� ��  : FlashReset
 ��������  : ��λflash���ָ�����ģʽ����������Ҫȷ��gIndex�ѳ�ʼ�����
 �������  : ��
 �������  : ��
 �� �� ֵ  : ��
 �޸���ʷ  :
****************************************************************************/
VOID FlashReset(UINT32 Base)
{
    (VOID)PortWriteData(gIndex.InfIndex, Base, gFlashCommandReset[gIndex.ReIndex].ResetData);
    (void)gBS->Stall(20000);
}

/****************************************************************************
 �� �� ��  : GetManufacturerID
 ��������  : ��ȡ����ID
 �������  : ��
 �������  : ��
 �� �� ֵ  : ��
 �޸���ʷ  :
****************************************************************************/
void GetManufacturerID(UINT32 Index, UINT32 Base, UINT8 *pbyData)
{
    
	UINT32 dwAddr;

    FlashReset(Base);
    
	dwAddr = Base +  (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep1 << gFlashInfo[Index].ParallelNum);
    (VOID)PortWriteData(Index, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep1);
    
	dwAddr = Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep2 << gFlashInfo[Index].ParallelNum);
    (VOID)PortWriteData(Index, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep2);

	dwAddr = Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep3 << gFlashInfo[Index].ParallelNum);
    (VOID)PortWriteData(Index, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep3);

    *pbyData = (UINT8)PortReadData(Index, Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddress << gFlashInfo[Index].ParallelNum));     

	FlashReset(Base);	//must reset to return to the read mode
}

/****************************************************************************
 �� �� �� : FlashInit
 ��������  : ��ʼ��flash�Ļ���ַ��ƥ��flash��Ϣ
 �������  : ��
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS        - �����ɹ�
           EFI_DEVICE_ERROR   - �豸����
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS FlashInit(UINT32 Base)
{
    UINT32 FlashCount = 0;
    UINT32 i = 0;
    EFI_STATUS Status;
    UINT8 Flag = 1;
    UINT32 TempData = 0;    
    UINT32 TempDev1 = 0;   
    UINT32 TempDev2 = 0;
    UINT32 TempDev3 = 0;    
    UINT32 dwAddr;
        
    FlashCount = sizeof(gFlashInfo) / sizeof(NOR_FLASH_INFO_TABLE);
    for(;i < FlashCount; i ++ )
    {
        //��ȡ��������������ʼ��gIndex
        Status = GetCommandIndex(i);
        if (EFI_ERROR(Status))
         {
             DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Get Command Index %r!\n", __FUNCTION__,__LINE__, Status));
             return Status;
         }
        
        FlashReset(Base);
        //���ͻ�ȡmanuID��������
        dwAddr = Base +  (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep1 << gFlashInfo[i].ParallelNum);
        (VOID)PortWriteData(i, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep1);
        
        dwAddr = Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep2 << gFlashInfo[i].ParallelNum);
        (VOID)PortWriteData(i, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep2);
        
        dwAddr = Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddressStep3 << gFlashInfo[i].ParallelNum);
        (VOID)PortWriteData(i, dwAddr, gFlashCommandId[gIndex.IdIndex].ManuIDDataStep3);
        //Get manufacture ID
        TempData = PortReadData(i, Base + (gFlashCommandId[gIndex.IdIndex].ManuIDAddress << gFlashInfo[i].ParallelNum));     

        //Get Device Id
        TempDev1 = PortReadData(i, Base + (gFlashCommandId[gIndex.IdIndex].DeviceIDAddress1 << gFlashInfo[i].ParallelNum));
        TempDev2 = PortReadData(i, Base + (gFlashCommandId[gIndex.IdIndex].DeviceIDAddress2 << gFlashInfo[i].ParallelNum));
        TempDev3 = PortReadData(i, Base + (gFlashCommandId[gIndex.IdIndex].DeviceIDAddress3 << gFlashInfo[i].ParallelNum));
        DEBUG ((EFI_D_ERROR, "[cdtest]manufactor ID 0x%x!\n",TempData));
        DEBUG ((EFI_D_ERROR, "[cdtest]Device ID 1 0x%x!\n",TempDev1));
        DEBUG ((EFI_D_ERROR, "[cdtest]Device ID 2 0x%x!\n",TempDev2));
        DEBUG ((EFI_D_ERROR, "[cdtest]Device ID 3 0x%x!\n",TempDev3));
        //�ָ�����ģʽ
        FlashReset(Base);
        
        //ƥ��manuID��DeviceID
        if((0xffffffff != TempData)
            && (PortAdjustData(i, gFlashInfo[i].ManufacturerID) == TempData))
        {
            if((0xffffffff != TempDev1)
                && (PortAdjustData(i, gFlashInfo[i].DeviceID1) == TempDev1))
            {
                if((0xffffffff != TempDev2)
                    && (PortAdjustData(i, gFlashInfo[i].DeviceID2) == TempDev2))
                {
                    if((0xffffffff != TempDev3)
                        && (PortAdjustData(i, gFlashInfo[i].DeviceID3) == TempDev3))
                    {
                        Flag = 0;
                        gIndex.InfIndex = i;
                        break;
                    }
                }
            }
         }
    }
    //���ѭ��������û�ҵ�ƥ���оƬ��Ϣ�������˳�
    if(Flag)
    {
        return EFI_DEVICE_ERROR;
    }
        
    return EFI_SUCCESS;
}

/****************************************************************************
 �� �� ��  : width8IsAll
 ��������  : �ж�ָ�������ֵ�Ƿ�ȫΪĳһ���ض�ֵ
 �������  : Base               - Flash����ַ
             Offset             - ƫ�Ƶ�ַ
             Length             - ��������
             Value              - ���������жϵ�ֵ
 �������  : ��
 �� �� ֵ  : TURE               - ��ȫ���������ֵ
             FALSE              - ���ڲ���������ֵ�ĵ�ַ
 
 �޸���ʷ  :

****************************************************************************/
static BOOLEAN width8IsAll(
    const UINT64       Base,
    const UINT64       Offset,
    const UINT64       Length,
    const UINT8        Value
)
{
    UINT64 NewAddr = Base + Offset;
    UINT64 NewLength = Length;
    while (NewLength --)
    {
        if (*(UINT8 *)(UINTN)NewAddr == Value)
        {
            NewAddr ++;
            continue;
        }
        else
        {
            return FALSE;
        }
    }
    return TRUE;
}


/****************************************************************************
 �� �� ��  : BufferWriteCommand
 ��������  : ����bufferд������
 �������  : Base              ����ַ
             Offset            ƫ�Ƶ�ַ
             pData             д�������
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
           EFI_NOT_READY           - ʧ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS BufferWriteCommand(UINTN Base, UINTN Offset, void *pData)
{
    UINT32 dwCommAddr;
    UINT32 *pdwData;
    UINT16 *pwData;
    UINT32 dwLoop;
    UINT32 ulWriteWordCount;
    UINT32 dwAddr;

    if(gFlashBusy)	
    {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL]:Flash is busy!\n", __FUNCTION__,__LINE__));
        return EFI_NOT_READY;
    }
    gFlashBusy = TRUE;

    if(2 == gFlashInfo[gIndex.InfIndex].ParallelNum)
    {
        pdwData = (UINT32 *)pData;
      
        dwAddr = (UINT32)Base + (gFlashCommandWrite[gIndex.WIndex].BufferProgramAddressStep1 << gFlashInfo[gIndex.InfIndex].ParallelNum);
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep1);

        dwAddr = (UINT32)Base + (gFlashCommandWrite[gIndex.WIndex].BufferProgramAddressStep2 << gFlashInfo[gIndex.InfIndex].ParallelNum);
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr,  gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep2);

        //dwAddr = Base + (Offset << gFlashInfo[gIndex.InfIndex].ParallelNum);
        dwAddr = (UINT32)Base + Offset;
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep3);
       
       //д�뵥λ�����������̶�
       ulWriteWordCount = ((gFlashInfo[gIndex.InfIndex].BufferProgramSize - 1) << 16) | (gFlashInfo[gIndex.InfIndex].BufferProgramSize - 1);
       (VOID)PortWriteData(gIndex.InfIndex, dwAddr, ulWriteWordCount);
       
       //д������
       for (dwLoop = 0; dwLoop < gFlashInfo[gIndex.InfIndex].BufferProgramSize; dwLoop ++)
       {
           dwCommAddr = (UINT32)Base + (UINT32)Offset + (dwLoop << gFlashInfo[gIndex.InfIndex].ParallelNum);   //16λ *2��32λ *4
           *(volatile UINT32 *)(UINTN)dwCommAddr = *pdwData;
           pdwData ++;            
       }
       
       dwAddr = (UINT32)Base + (UINT32)Offset + ((gFlashInfo[gIndex.InfIndex].BufferProgramSize - 1) << gFlashInfo[gIndex.InfIndex].ParallelNum);
       (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramtoFlash);

       
    //����д������
    }
    else
    {
        pwData  = (UINT16 *)pData;
        
        dwAddr = (UINT32)Base + (gFlashCommandWrite[gIndex.WIndex].BufferProgramAddressStep1 << gFlashInfo[gIndex.InfIndex].ParallelNum);
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep1);
        
        dwAddr = (UINT32)Base + (gFlashCommandWrite[gIndex.WIndex].BufferProgramAddressStep2 << gFlashInfo[gIndex.InfIndex].ParallelNum);
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr,  gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep2);
        
        //dwAddr = Base + (Offset << gFlashInfo[gIndex.InfIndex].ParallelNum);
        dwAddr = (UINT32)Base + Offset;
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramDataStep3);
        
        //д�뵥λ�����������̶�
        ulWriteWordCount = gFlashInfo[gIndex.InfIndex].BufferProgramSize - 1;
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, ulWriteWordCount);
        
        //д������
        for (dwLoop = 0; dwLoop < gFlashInfo[gIndex.InfIndex].BufferProgramSize; dwLoop ++)
        {
            dwCommAddr = (UINT32)Base + (UINT32)Offset + (dwLoop << gFlashInfo[gIndex.InfIndex].ParallelNum);   //16λ *2��32λ *4
            *(volatile UINT16 *)(UINTN)dwCommAddr = *pwData;
            pwData ++;
        }
        
        dwAddr = (UINT32)Base + (UINT32)Offset + ((gFlashInfo[gIndex.InfIndex].BufferProgramSize - 1) << gFlashInfo[gIndex.InfIndex].ParallelNum);
        (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandWrite[gIndex.WIndex].BufferProgramtoFlash);
        
    }

    (void)gBS->Stall(200);

    gFlashBusy = FALSE;
    return EFI_SUCCESS;

}

/****************************************************************************
 �� �� ��  : SectorEraseCommand
 ��������  : ���������ַ���ڵ�sector
 �������  : UINT32 Offset     - ����������sector�������ַ
 �������  : ��
 �� �� ֵ  : ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS SectorEraseCommand(UINTN Base, UINTN Offset)
{
    UINT32 dwAddr;

    if(gFlashBusy)	
    {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL]:Flash is busy!\n", __FUNCTION__,__LINE__));
        return EFI_NOT_READY;
    }
    
    gFlashBusy = TRUE;

    dwAddr = (UINT32)Base + (gFlashCommandErase[gIndex.EIndex].SectorEraseAddressStep1 << gFlashInfo[gIndex.InfIndex].ParallelNum);
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep1);
    
    dwAddr = (UINT32)Base + (gFlashCommandErase[gIndex.EIndex].SectorEraseAddressStep2 << gFlashInfo[gIndex.InfIndex].ParallelNum);
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep2);
    
    dwAddr = (UINT32)Base + (gFlashCommandErase[gIndex.EIndex].SectorEraseAddressStep3 << gFlashInfo[gIndex.InfIndex].ParallelNum);
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep3);

    dwAddr = (UINT32)Base + (gFlashCommandErase[gIndex.EIndex].SectorEraseAddressStep4 << gFlashInfo[gIndex.InfIndex].ParallelNum);
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep4);

    dwAddr = (UINT32)Base + (gFlashCommandErase[gIndex.EIndex].SectorEraseAddressStep5 << gFlashInfo[gIndex.InfIndex].ParallelNum);
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep5);

    dwAddr = (UINT32)Base + Offset;
    (VOID)PortWriteData(gIndex.InfIndex, dwAddr, gFlashCommandErase[gIndex.EIndex].SectorEraseDataStep6);

    (void)gBS->Stall(500000);
    
    gFlashBusy = FALSE;
    return EFI_SUCCESS;
}

/****************************************************************************
 �� �� ��  : CompleteCheck
 ��������  : ���д����������Ƿ����
             ֻ�ж����λ���ȵ�����
 �������  : Base              -flash����ַ
             UINT32 Offset     - �����ƫ�Ƶ�ַ
             void *pData       -����ֵ
             UINT32 Length       - ����
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS			- �ɹ�
            EFI_DEVICE_ERROR		- ������
 
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS CompleteCheck(UINT32 Base, UINT32 Offset, void *pData, UINT32 Length)
{
    UINT32 dwTestAddr;    //ָ�����һ����λ�ĵ�ַ
    UINT32 dwTestData;    //ȡ���д���һ����λ����
    UINT32 dwTemp = 0;    //���������жϵ�ֵ        
    UINT32 dwTemp1 = 0;
    UINT32 i;
    UINT32 dwTimeOut = 3000000;  //���ѭ������(Լ3s)

    if(gFlashBusy)	
    {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL]:Flash is busy!\n", __FUNCTION__,__LINE__));
        return EFI_NOT_READY;
    }
    gFlashBusy = TRUE;
    
    if(2 == gFlashInfo[gIndex.InfIndex].ParallelNum)
    {
        dwTestAddr = Base + Offset + Length - sizeof(UINT32);
        dwTestData = *((UINT32 *)((UINT8 *)pData + Length - sizeof(UINT32)));

        while(dwTimeOut--)
        {
            dwTemp1 = *(volatile UINT32 *)(UINTN)(dwTestAddr);        
            if (dwTestData == dwTemp1)
            {
                dwTemp = *(volatile UINT32 *)(UINTN)(dwTestAddr);
                dwTemp1 = *(volatile UINT32 *)(UINTN)(dwTestAddr);
                if ((dwTemp == dwTemp1) && (dwTestData == dwTemp1))
                {
                    gFlashBusy = FALSE;
                    return EFI_SUCCESS;
                }
            }
             
            (void)gBS->Stall(1);
        }
        //�ж���Ƭ����оƬ������һ�����
        if((UINT16)(dwTemp1 >> 16) != (UINT16)(dwTestData >> 16))
        {
            DEBUG((EFI_D_ERROR, "CompleteCheck ERROR: chip1 address %x, buffer %x, flash %x!\n", Offset, dwTestData, dwTemp1));
        }
        if((UINT16)(dwTemp1) != (UINT16)(dwTestData))
        {
            DEBUG((EFI_D_ERROR, "CompleteCheck ERROR: chip2 address %x, buffer %x, flash %x!\n", Offset, dwTestData, dwTemp1));
        }
    }
    else
    {
        dwTestAddr = Base + Offset + Length - sizeof(UINT16);
        dwTestData = *((UINT16 *)((UINT8 *)pData + Length - sizeof(UINT16)));
        
        while(dwTimeOut--)
        {
            dwTemp1 = *(volatile UINT16 *)(UINTN)(dwTestAddr);        
            if (dwTestData == dwTemp1)
            {
                dwTemp = *(volatile UINT16 *)(UINTN)(dwTestAddr);
                dwTemp1 = *(volatile UINT16 *)(UINTN)(dwTestAddr);
                if ((dwTemp == dwTemp1) && (dwTestData == dwTemp1))
                {
                    gFlashBusy = FALSE;
                    return EFI_SUCCESS;
                }
            }
             
            (void)gBS->Stall(1);
        }
    }
               
    for(i = 0; i < 5; i ++)
    {
        DEBUG((EFI_D_ERROR, "CompleteCheck ERROR: flash %x\n",PortReadData(gIndex.InfIndex, dwTestAddr)));
    }
    
    FlashReset(Base);  //���ش���֮ǰ�ȸ�λ���ָ����ɶ�״̬
    
    gFlashBusy = FALSE;
    DEBUG((EFI_D_ERROR, "CompleteCheck ERROR: timeout address %x, buffer %x, flash %x\n", Offset, dwTestData, dwTemp1));	
    return EFI_TIMEOUT;
}
/****************************************************************************
 �� �� ��  : IsNeedToWrite
 ��������  : �ж�ָ�������ֵ�Ƿ�ȫΪĳһ���ض�ֵ
 �������  : Offset             - ƫ�Ƶ�ַ
             Length             - ��������
             Value              - ���������жϵ�ֵ
 �������  : ��
 �� �� ֵ  : TURE               - ��ȫ���������ֵ
            FALSE              - ���ڲ���������ֵ�ĵ�ַ
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS IsNeedToWrite(
    IN  UINT32         Base,
    IN  UINT32       Offset,
    IN  UINT8       *Buffer,
    IN  UINT32       Length
  )
{
    UINTN NewAddr = Base + Offset;
    UINT8 FlashData = 0;        //flash�ϵ�ֵ
    UINT8 BufferData = 0;       //��flashֵ���Ӧ��bufferֵ

    for(; Length > 0; Length --)
    {
        BufferData = *Buffer;
        //lint -epn -e511
        FlashData = *(UINT8 *)NewAddr;
        if (BufferData != FlashData)
        {
            return TRUE;
        }
        NewAddr ++;
        Buffer ++;
    }
    
    return FALSE;
}

/****************************************************************************
 �� �� ��  : BufferWrite
 ��������  : д��λ���ȸ��ֽڣ�������Ƿ�д���
 �������  : Offset     ƫ�Ƶ�ַ 
             pData      д��ֵ�׵�ַ
             Length д�볤��
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
             EFI_ABORTED        - ʧ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS BufferWrite(UINT32 Offset, void *pData, UINT32 Length)
{
    EFI_STATUS Status;
    UINT32 dwLoop;
    UINT32 Retry = 3;
    
    if (FALSE == IsNeedToWrite(gIndex.Base, Offset, (UINT8 *)pData, Length))
    {
        return EFI_SUCCESS;
    }

    do
    {
        (void)BufferWriteCommand(gIndex.Base, Offset, pData);
        Status = CompleteCheck(gIndex.Base, Offset, pData, Length);
     
        //���д��ɣ��ж�д��ֵ�Ƿ���ȷ
        if (EFI_SUCCESS == Status)
        {
            for (dwLoop = 0; dwLoop < Length; dwLoop ++)
            {
                if (*(UINT8 *)(UINTN)(gIndex.Base + Offset + dwLoop) != *((UINT8 *)pData + dwLoop))
                {
                    DEBUG((EFI_D_ERROR, "Flash_WriteUnit ERROR: address %x, buffer %x, flash %x\n", Offset, *((UINT8 *)pData + dwLoop), *(UINT8 *)(UINTN)(gIndex.Base + Offset + dwLoop)));
                    Status = EFI_ABORTED;
                    continue;
                }
            }
        }
        else
        {
            DEBUG((EFI_D_ERROR, "Flash_WriteUnit ERROR: complete check failed, %r\n", Status));
            continue;
        }
    } while ((Retry--) && EFI_ERROR(Status));

    return Status;
}

/****************************************************************************
 �� �� ��  : SectorErase
 ��������  : ����һ��Sector��������Ƿ���ɣ�����Ƿ��������ֽڼ��
 �������  : Base               ����ַ
             Offset            ƫ�Ƶ�ַ
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
            EFI_ABORTED        - ʧ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS SectorErase(UINT32 Base, UINT32 Offset)
{
    UINT8 gTemp[FLASH_MAX_UNIT];
    UINT64 dwLoop = FLASH_MAX_UNIT - 1;
    UINT32 Retry = 3;
    EFI_STATUS Status;
    
    do
    {
        gTemp[dwLoop] = 0xFF;
    }while (dwLoop --);

    do  
    {
        (void)SectorEraseCommand(Base, Offset);
        Status = CompleteCheck(Base, Offset, (void *)gTemp, FLASH_MAX_UNIT);
        
        //���д��ɣ��ж�д��ֵ�Ƿ���ȷ
        if (EFI_SUCCESS == Status)
        {
            //�������Sector�Ƿ���Ϊȫ0xFF��OffsetҪָ��sector����ʼ
            if (width8IsAll(Base,Offset - (Offset % gFlashInfo[gIndex.InfIndex].BlockSize), gFlashInfo[gIndex.InfIndex].BlockSize, 0xFF))
            {
                return EFI_SUCCESS;
            }
            else
            {
                DEBUG((EFI_D_ERROR, "Flash_SectorErase ERROR: not all address equal 0xFF\n"));
                //���޸�Status��continue������while�жϻ����
                Status = EFI_ABORTED;
                continue;
            }
        }
        else
        {
            DEBUG((EFI_D_ERROR, "Flash_SectorErase ERROR: complete check failed, %r\n", Status));
            continue;
        }
    }while ((Retry--) && EFI_ERROR(Status));

    if(Retry)
    {
        //do nothing for pclint
    }
    
    return Status;
}
