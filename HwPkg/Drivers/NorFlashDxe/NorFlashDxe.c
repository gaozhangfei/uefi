/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpZ4X5IrQg0R6dsKjrHb0BePcxREXVZ5dUR67fNFdcsGWwP1ecJzNoC5Nk+8B
iJvImLuolrIPUKBGd8L2wpF6y6N3BrBWhrMiUsGHALluV0im+t4T28eAdD4NhRGenqxofQly
tiYrdQUOiCbgko7X8I9R7sXS0nvwXr8GKLUB1vDslWVM+jomrqhBcmv9hcQOHQ==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : NorFlashDxe.c
  �� �� ��   : v2.0
  ��    ��   : c00213799
  ��������   : 2013��03��04��
  ����޸�   :
  ��������   : Flash�ӿ�
  �޸���ʷ   :
1.   ��	  ��   : 
     ��	  ��   :
     �޸�����  :
******************************************************************************/
#include <Uefi.h>
//#include <PiDxe.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/ArmLib.h>
#include <Library/PcdLib.h>
#include <Protocol/NorFlashProtocol.h>
#include <Library/DxeServicesTableLib.h>
#include <Protocol/Cpu.h>
#include "NorFlashHw.h"
extern EFI_GUID gUniNorFlashProtocolGuid;


EFI_STATUS Erase(
   IN UNI_NOR_FLASH_PROTOCOL   *This,
   IN  UINT32                   Offset,
   IN  UINT32                   Length
  );

EFI_STATUS  Write(
  IN UNI_NOR_FLASH_PROTOCOL   *This,
  IN  UINT32                  Offset,
  IN  UINT8                  *Buffer,
  UINT32                     ulLength
  );

EFI_STATUS Read(   
  IN UNI_NOR_FLASH_PROTOCOL   *This,
  IN UINT32                   Offset,
  IN OUT UINT8               *Buffer,
  IN UINT32                   ulLen
  );

UNI_NOR_FLASH_PROTOCOL gUniNorFlash = {
    Erase,
    Write,
    Read
};

/****************************************************************************
 �� �� ��   :  Read
 ��������  : 
 �������  : 
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS        - �����ɹ�
           EFI_INVALID_PARAMETER  -�����������
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS
EFIAPI Read(   
  IN UNI_NOR_FLASH_PROTOCOL   *This,
  IN UINT32                   Offset,
  IN OUT UINT8               *Buffer,
  IN UINT32                    ulLen
  )
{
    UINT32       index;
    UINT64 ullAddr;
    UINT32 ullCnt = 0;
    UINT32 *puiBuffer32 = NULL;
    UINT32 *puiDst32 = NULL;
    UINT8 *pucBuffer8 = NULL;
    UINT8 *pucDst8 = NULL;

    if (Offset + ulLen > (gFlashInfo[gIndex.InfIndex].SingleChipSize * gFlashInfo[gIndex.InfIndex].ParallelNum))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Exceed the flash scope!\n", __FUNCTION__,__LINE__));
        return EFI_INVALID_PARAMETER;
    }
    if (0 == ulLen)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Length is Zero!\n", __FUNCTION__,__LINE__));
        return EFI_INVALID_PARAMETER;
    }
    if (NULL == Buffer)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Buffer is NULL!\n", __FUNCTION__,__LINE__));
        return EFI_BAD_BUFFER_SIZE;
    }


    ullAddr = gIndex.Base + Offset;

    pucBuffer8 = (UINT8 *)Buffer;
    pucDst8    = (UINT8 *)((UINTN)ullAddr);

    /* ����С��4��ֱ�Ӱ��ֽڶ�ȡ */
    if (ulLen < FOUR_BYTE_UNIT)
    {
        for(index = 0; index< ulLen; index++)
        {
            *pucBuffer8++ = *pucDst8++;
        }
    }
    else /* ���ȴ��ڵ���4�����а�4�ֽڶ����ȡ���� */
    {

        ullCnt = Offset % FOUR_BYTE_UNIT;
        ullCnt = FOUR_BYTE_UNIT - ullCnt;

        for(index = 0; index < ullCnt; index++)
        {
            *pucBuffer8++ = *pucDst8++;
        }

        ulLen -= ullCnt;

        puiBuffer32 = (UINT32 *)pucBuffer8;
        puiDst32    = (UINT32 *)pucDst8;
        ullCnt      = ulLen / FOUR_BYTE_UNIT;

        for(index = 0; index < ullCnt; index++)
        {
            *puiBuffer32++ = *puiDst32++;
        }

        ullCnt     = ulLen % FOUR_BYTE_UNIT;
        pucBuffer8 = (UINT8 *)puiBuffer32;
        pucDst8    = (UINT8 *)puiDst32;

        for(index = 0; index < ullCnt; index++)
        {
            *pucBuffer8++ = *pucDst8++;
        }
    }

    return EFI_SUCCESS;
}


/****************************************************************************
 �� �� �� : WriteAfterErase_Fill
 ��������  : ������д��һ����Ԫ������ʼ���������ַ�����ݵ�Ԫ������ʱ����
             �����߱��뱣֤��д�����ݲ��絥Ԫ
             ���ô˺���ǰ���뱣֤��д�����Ѳ���Ҫ����
 �������  : Offset             - ƫ�Ƶ�ַ
             Buffer             - ��������
             Length             - ����
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS        - �����ɹ�
             EFI_DEVICE_ERROR   - �豸����

 �޸���ʷ  :

****************************************************************************/
static EFI_STATUS WriteAfterErase_Fill(
    IN  const UINT32       Offset,
    IN  const UINT8       *Buffer,
    IN  const UINT32       Length
    )
{
    EFI_STATUS Status;
    UINT32 Loop;
    UINT32 DataOffset;       //ʵ��д�������������ݵ�Ԫ�е���ʼλ��
    UINT32 NewOffset;       //����������µ�д���ַ
    UINT8 *NewDataUnit;     //�����ݵ�Ԫ
    
    UINT32 FlashUnitLength;

    FlashUnitLength = gFlashInfo[gIndex.InfIndex].BufferProgramSize << gFlashInfo[gIndex.InfIndex].ParallelNum;
    //�������
    if (0 == Length)
    {
        return EFI_SUCCESS;
    }
    if ((Offset % FlashUnitLength + Length) > FlashUnitLength)
    {
        DEBUG ((EFI_D_INFO, "[%a]:[%dL]:Exceed the Flash Size!\n", __FUNCTION__,__LINE__));
        return EFI_UNSUPPORTED;
    }

    //Ϊ�����ݵ�Ԫ����ռ�
    Status = gBS->AllocatePool(EfiBootServicesData, FlashUnitLength, (VOID *)&NewDataUnit);
    if (EFI_ERROR(Status))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Allocate Pool failed, %r!\n", __FUNCTION__,__LINE__, Status));
        return Status;
    }

    //���������ݵ�Ԫ
    NewOffset = Offset - (Offset % FlashUnitLength);
    
    gBS->CopyMem((VOID *)NewDataUnit, (VOID *)(UINTN)(gIndex.Base + NewOffset), FlashUnitLength);
    
    DataOffset = Offset % FlashUnitLength;
    for (Loop = 0; Loop < Length; Loop ++)
    {
        NewDataUnit[(UINT32)(DataOffset + Loop)] = Buffer[Loop];
    }
    
    Status = BufferWrite(NewOffset, (void *)NewDataUnit, FlashUnitLength);
    if (EFI_ERROR(Status))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:BufferWrite %r!\n", __FUNCTION__,__LINE__, Status));
        return Status;
    }

    (void)gBS->FreePool((VOID *)NewDataUnit);
    return Status;
}

/****************************************************************************
 �� �� ��  : WriteAfterErase_Final
 ��������  : ����д����������֤��ʼ��ַΪ��Ԫ��ʼ�󣬵��ô˺���д����������
             ���ô˺���ǰ���뱣֤��д�����Ѳ���Ҫ����
             �����߱��뱣֤StartAddressΪ��Ԫ��ʼ��ַ
 �������  : Offset             - ƫ�Ƶ�ַ
             Buffer             - ��������
             Length             - ����
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS        - �����ɹ�
             EFI_DEVICE_ERROR   - �豸����

 �޸���ʷ  :

****************************************************************************/
static EFI_STATUS WriteAfterErase_Final(
    IN  UINT32       Offset,
    IN  UINT8       *Buffer,
    IN  UINT32       Length
    )
{
    EFI_STATUS Status;
    UINT32 Loop;
    UINT32 FlashUnitLength;

    FlashUnitLength = gFlashInfo[gIndex.InfIndex].BufferProgramSize << gFlashInfo[gIndex.InfIndex].ParallelNum;
    //�������
    if (0 == Length)
    {
        return EFI_SUCCESS;
    }
    //StartAddress�����ǵ�Ԫ��ʼ
    if (0 != (Offset % FlashUnitLength))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]: Offset must be a multiple of 0x%x!\n", __FUNCTION__,__LINE__,FlashUnitLength));
        return EFI_UNSUPPORTED;
    }

    //д��ǰ�沿�֣���ʣ�೤�Ȳ���һ�����ݵ�ԪΪֹ
    Loop = Length / FlashUnitLength;
    while (Loop --)
    {
        Status = BufferWrite(Offset, (void *)Buffer, FlashUnitLength);
        if (EFI_ERROR(Status))
        {
            DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:BufferWrite Failed: %r!\n", __FUNCTION__,__LINE__, Status));
            return EFI_DEVICE_ERROR;
        }
        Offset += FlashUnitLength;
        Buffer += FlashUnitLength;
    }

    //д��ʣ��Ĳ���һ����Ԫ������
    Length = Length % FlashUnitLength;
    if (Length)
    {
        Status = WriteAfterErase_Fill(Offset, Buffer, Length);
        if (EFI_ERROR(Status))
        {
            DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:WriteAfterErase_Fill failed,%r!\n", __FUNCTION__,__LINE__, Status));
            return Status;
        }
    }
    
    return EFI_SUCCESS;
}
/****************************************************************************
 �� �� ��  : WriteAfterErase
 ��������  : ʵ�����ⳤ�ȵ�д�����ô˺���ǰ���뱣֤��д�����Ѳ���Ҫ����
 �������  : Offset            ƫ�Ƶ�ַ
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
            EFI_ABORTED        - ʧ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS
WriteAfterErase(
    UINT32       TempBase,
    UINT32       Offset,
    UINT8       *Buffer,
    UINT32       Length
  )
{
    EFI_STATUS Status;
    UINT32 FlashUnitLength;

    FlashUnitLength = gFlashInfo[gIndex.InfIndex].BufferProgramSize << gFlashInfo[gIndex.InfIndex].ParallelNum;
    
    if (0 == Length)
    {
        return EFI_SUCCESS;
    }

    //��ʼ��ַ���ǵ�Ԫ��ʼ������д���һ����Ԫ��ʹ��һ��д�����ʼ��ַ�ǵ�Ԫ��ʼ
    if (Offset % FlashUnitLength)
    {
        UINT32 TempLength;

        //ֻд����ʼ��ַ���ڵ����ݵ�Ԫ
        TempLength = FlashUnitLength - (Offset % FlashUnitLength);
        if (TempLength > Length)
        {
            TempLength = Length;
        }
        Status = WriteAfterErase_Fill(Offset, Buffer, TempLength);
        if (EFI_ERROR(Status))
        {
            DEBUG ((EFI_D_ERROR, "[%a]:[%dL]: %r!\n", __FUNCTION__,__LINE__, Status));
            return Status;
        }
        //���²���
        Offset += TempLength;
        Length -= TempLength;
        Buffer += TempLength;

        //Desc:if Offset >= gOneFlashSize,modify base
        if (0 < (Offset / gFlashInfo[gIndex.InfIndex].SingleChipSize))
        {
            TempBase += gFlashInfo[gIndex.InfIndex].SingleChipSize;
            gIndex.Base = TempBase;
            Offset = 0;
        }
    }

    //������ʼ��ַΪ��Ԫ��ʼ��д��ʣ�ಿ������
    Status = WriteAfterErase_Final(Offset, Buffer, Length);
    if (EFI_ERROR(Status))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]: %r!\n", __FUNCTION__,__LINE__, Status));
        return Status;
    }

    return EFI_SUCCESS;
}

/****************************************************************************
 �� �� ��  : FlashSectorErase
 ��������  : ����ʼ��ַ��ʼ������ָ�����ȵĿռ䣬����Sector
             ע�Ȿ�������ڴ���ͷ�
             �����߱��뱣֤��Ҫ�����ĵ�ַ��ͬһ��Sector��
 �������  : Offset   - ��ʼƫ�Ƶ�ַ
             Length         - ����
 �������  : ��
 �� �� ֵ  : Status

 �޸���ʷ  :

****************************************************************************/
EFI_STATUS 
FlashSectorErase(
    UINT32      TempBase,
    UINT32      Offset,
    UINT32      Length
  )
{
    EFI_STATUS  Status;
    UINT32 SectorOffset;    //��������ʼ��ַ����flash sector�׵�ַ(���Ե�ַ)
    UINT8 *StaticBuffer;
    UINT8 *Buffer;          //��д������ָ��
    UINT32 TempOffset;      //��дʱ����ʼ��ַ
    UINT32 TempLength;       //��дʱ�ĳ���
    UINT32 LeftLength;       //��ʼ��ַ����Sector��ʣ��ռ�
    

    if (0 == Length)
    {
        return EFI_SUCCESS;
    }
    //�����Ҫ�����ĵ�ַ����ͬһ��Sector�ڣ��򷵻ش���
    LeftLength = gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum - (Offset % (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum));
    if (LeftLength < Length)
    {
        return EFI_UNSUPPORTED;
    }

    //������ʼ��ַ����Sector����ʼ��ַ
    SectorOffset = Offset - (Offset % (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum));
    
    Status = gBS->AllocatePool(EfiBootServicesData, gFlashInfo[gIndex.InfIndex].BlockSize * (UINTN)gFlashInfo[gIndex.InfIndex].ParallelNum, (VOID *)&StaticBuffer);
    if (EFI_ERROR(Status))
    {
        return Status;
    }
    
    Buffer = StaticBuffer;
    
    gBS->CopyMem((VOID *)Buffer, (VOID *)(UINTN)(TempBase + SectorOffset), 
                 (gFlashInfo[gIndex.InfIndex].BlockSize * (UINTN)gFlashInfo[gIndex.InfIndex].ParallelNum));

    //����
    Status = SectorErase(TempBase, SectorOffset);
    if (EFI_ERROR(Status))
    {
        goto DO;
    }
    
    //��дǰ�沿��
    TempOffset = SectorOffset;
    TempLength = Offset % (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum);

    Status = WriteAfterErase(TempBase, TempOffset, Buffer, TempLength);
    if (EFI_ERROR(Status))
    {
        goto DO;
    }
    
    //��д���沿�֣�ע��buffer��ƫ��
    //��ǰ���д�õ���ͬһ��buffer�����Ҫ���ϲ����ĳ���
    Buffer = Buffer + TempLength + Length;
    TempOffset = Offset + Length;
    TempLength = SectorOffset + (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum) - TempOffset;

    Status = WriteAfterErase(TempBase, TempOffset, Buffer, TempLength);
    if (EFI_ERROR(Status))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]: %r!\n", __FUNCTION__,__LINE__,Status));
        goto DO;
    }
        
    (void)gBS->FreePool((VOID *)StaticBuffer);    
    return EFI_SUCCESS;
    
DO:
    (void)gBS->FreePool((VOID *)StaticBuffer);
    return Status;
}

/****************************************************************************
 �� �� ��  : Erase
 ��������  : ʵ�����ⳤ�ȵĲ�
 �������  : Offset                 ƫ�Ƶ�ַ
             Length                ��������
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
            EFI_ABORTED        - ʧ��
 
 �޸���ʷ  :

****************************************************************************/
EFI_STATUS
EFIAPI Erase(
   IN UNI_NOR_FLASH_PROTOCOL   *This,
   IN UINT32                   Offset,
   IN UINT32                   Length
  )
{
    EFI_STATUS  Status = EFI_SUCCESS;
    UINT32 Sectors;         //��ʼ��ַ�������ַ֮������sector��
    UINT32 TempLength;
    UINT32 TempBase;
    UINT32 Loop;

    //�ж����
    if (Offset + Length > (gFlashInfo[gIndex.InfIndex].SingleChipSize * gFlashInfo[gIndex.InfIndex].ParallelNum))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Exceed the Flash Size!\n", __FUNCTION__,__LINE__));
        return EFI_ABORTED;
    }
    if (0 == Length)
    {
        return EFI_SUCCESS;
    }

    //���㱾�β����漰�Ŀ�����ֵΪ������ַ���ڿ�� - ��ʼ��ַ���ڿ�� + 1
    //������ַΪ��ʼ��ַ + ���� - 1
    Sectors = ((Offset + Length - 1) / (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum)) - (Offset / (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum)) + 1;
    TempBase = gIndex.Base;
    
    //if Offset >= gOneFlashSize,modify base
    if(0 < (Offset / gFlashInfo[gIndex.InfIndex].SingleChipSize))
    {
        TempBase +=  gFlashInfo[gIndex.InfIndex].SingleChipSize * (Offset/gFlashInfo[gIndex.InfIndex].SingleChipSize);
        Offset = Offset - (Offset & gFlashInfo[gIndex.InfIndex].SingleChipSize);
    }

    for (Loop = 0; Loop <= Sectors; Loop ++)
    {
        //��֤�������ĵ�ַ��ͬһ��Sector��
        TempLength = gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum - (Offset % (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum));

        //��ʣ��ռ��㹻������������ĳ��ȣ���ֻ����Ҫ��ĳ���
        if (TempLength > Length)
        {
            TempLength = Length;
        }
        
        Status = FlashSectorErase(TempBase, Offset, TempLength);
        if (EFI_ERROR(Status))
        {
            DEBUG ((EFI_D_ERROR, "[%a]:[%dL]: FlashErase One Sector Error, Status = %r!\n", __FUNCTION__,__LINE__,Status));
            return Status;
        }
        
        Offset += TempLength;
        
         //if Offset >= gOneFlashSize,modify base
        if (0 < (Offset / gFlashInfo[gIndex.InfIndex].SingleChipSize))
        {
            TempBase += gFlashInfo[gIndex.InfIndex].SingleChipSize;
            Offset = 0;
        }
        Length -= TempLength;
    }

    return Status;
}

/****************************************************************************
 �� �� ��  : Write
 ��������  : ʵ�����ⳤ�ȵ�д
 �������  : Offset                  д���ƫ�Ƶ�ַ
             Buffer                  д�������
             ulLength                ����
 �������  : ��
 �� �� ֵ  : EFI_SUCCESS           - �ɹ�
           EFI_INVALID_PARAMETER        - ʧ��
 �޸���ʷ  :
****************************************************************************/
EFI_STATUS
EFIAPI Write(
  IN UNI_NOR_FLASH_PROTOCOL   *This,
  IN UINT32                   Offset,
  IN UINT8                   *Buffer,
  UINT32                     ulLength
  )
{
    EFI_STATUS  Status;
    UINT32     TempLength;
    UINT32       TempBase;
    UINT32           Loop;
    UINT32        Sectors;         //��ʼ��ַ�������ַ֮������sector��

    if((Offset + ulLength) > (gFlashInfo[gIndex.InfIndex].SingleChipSize * gFlashInfo[gIndex.InfIndex].ParallelNum))
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Exceed the Flash Size!\n", __FUNCTION__,__LINE__));
        return EFI_INVALID_PARAMETER;
    }
    if (0 == ulLength)
    {
        return EFI_SUCCESS;
    }
    
    //���㱾�β����漰�Ŀ�����ֵΪ������ַ���ڿ�� - ��ʼ��ַ���ڿ�� + 1
    //������ַΪ��ʼ��ַ + ���� - 1
    Sectors = ((Offset + ulLength - 1) / (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum)) - (Offset / (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum)) + 1;
    TempBase = gIndex.Base;
    
    //if Offset >= gOneFlashSize,modify base
    if(0 < (Offset / gFlashInfo[gIndex.InfIndex].SingleChipSize))
    {
        TempBase +=  gFlashInfo[gIndex.InfIndex].SingleChipSize * (Offset/gFlashInfo[gIndex.InfIndex].SingleChipSize);
        Offset = Offset - (Offset & gFlashInfo[gIndex.InfIndex].SingleChipSize);
    }

    for (Loop = 0; Loop <= Sectors; Loop ++)
    {
        //��֤�������ĵ�ַ��ͬһ��Sector��
        TempLength = gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum - (Offset % (gFlashInfo[gIndex.InfIndex].BlockSize * gFlashInfo[gIndex.InfIndex].ParallelNum));

        //��ʣ��ռ��㹻������������ĳ��ȣ���ֻ����Ҫ��ĳ���
        if (TempLength > ulLength)
        {
            TempLength = ulLength;
        }

        //�ж��Ƿ���Ҫ����
        if (TRUE == IsNeedToWrite(TempBase, Offset, Buffer, TempLength))
        {
            Status = FlashSectorErase(TempBase, Offset, TempLength);
            if (EFI_ERROR(Status))
            {
                DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:FlashErase One Sector Error, Status = %r!\n", __FUNCTION__,__LINE__,Status));
                return Status;
            }

            //��Ҫ����������Ҫд�룬�����ַΪ����ƫ�Ƶ�ַ������Ϊ����ƫ�Ƴ���
            Status = WriteAfterErase(TempBase, Offset, Buffer, TempLength);
            if (EFI_ERROR(Status))
            {
                DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:WriteAfterErase Status = %r!\n", __FUNCTION__,__LINE__,Status));
                return Status;
            }
        }
        
        Offset += TempLength;
        Buffer += TempLength;
        
         //if Offset >= gOneFlashSize,modify base
        if (0 < (Offset / gFlashInfo[gIndex.InfIndex].SingleChipSize))
        {
            TempBase += gFlashInfo[gIndex.InfIndex].SingleChipSize;
            Offset = 0;
        }
        ulLength -= TempLength;
    }
    
    return EFI_SUCCESS;
}


VOID SetFlashAttributeToUncache(VOID)
{
    EFI_CPU_ARCH_PROTOCOL             *gCpu           = NULL;
    EFI_STATUS Status;
    
    Status = gBS->LocateProtocol (&gEfiCpuArchProtocolGuid, NULL, (VOID **)&gCpu);
    if (EFI_ERROR(Status))
    {
        DEBUG((EFI_D_ERROR, "LocateProtocol gEfiCpuArchProtocolGuid Status = %r !\n", Status)); 
    }
    
    Status = gCpu->SetMemoryAttributes(
                     gCpu,
                     PcdGet64(PcdNORFlashBase),
                     PcdGet32(PcdNORFlashCachableSize),
                     EFI_MEMORY_UC
                     );
    
    if (EFI_ERROR(Status))
    {
        DEBUG((EFI_D_ERROR, "gCpu->SetMemoryAttributes Status = %r !\n", Status)); 
    }

}

EFI_STATUS
EFIAPI InitializeFlash (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable)
{
    EFI_STATUS Status;
        
    
    gIndex.Base = (UINT32)PcdGet64(PcdNORFlashBase);   

    SetFlashAttributeToUncache();
    Status = FlashInit(gIndex.Base);
    if (EFI_ERROR(Status))
    {
        DEBUG((EFI_D_ERROR, "Init Flash Error !\n")); 
        return Status;
    }
    else
    {
        DEBUG((EFI_D_ERROR, "Init Flash OK!\n")); 
    }

    Status = gBS->InstallProtocolInterface (
                            &ImageHandle,
                            &gUniNorFlashProtocolGuid,
                            EFI_NATIVE_INTERFACE,
                            &gUniNorFlash);
    if(EFI_SUCCESS != Status)
    {
        DEBUG ((EFI_D_ERROR, "[%a]:[%dL]:Install Protocol Interface %r!\n", __FUNCTION__,__LINE__,Status));
    }
    
    return Status;
}
