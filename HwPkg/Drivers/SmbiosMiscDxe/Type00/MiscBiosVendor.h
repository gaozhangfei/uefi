/******************************************************************************

                  ��Ȩ���� (C), 2001-2011, ��Ϊ�������޹�˾

******************************************************************************
  �� �� ��   : MiscBiosVendor.h
  �� �� ��   : ����
  ��    ��   : y00216284
  ��������   : 
  ����޸�   :
  ��������   : 
  �����б�   :   
  �޸���ʷ   :
  1.��    ��   : 
    ��    ��   : 
    �޸�����   : �����ļ�  
******************************************************************************/
#ifndef _MISC_BIOS_VENDER_H
#define _MISC_BIOS_VENDER_H

#include <PiPei.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>


#define FV_CUSTDATA_ADDR            FixedPcdGet32(PcdCustDataFvAddress)


#pragma pack(1)
typedef struct _OEM_TIME{
  UINT16  Year;
  UINT8   Month;
  UINT8   Day;
  UINT8   Hour;
  UINT8   Min;
  UINT8   Sec;
}OEM_TIME;

//��Ƕ�汾��Ϣ
typedef struct {
  UINT16       structV;
  char         vendor[32];  // ������
  OEM_TIME     buildTime;   // 
  char         strV[3];     // BIOS��V�汾�ţ��ַ���"xxx\0"
  char         strR[3];     // BIOS��R�汾�ţ��ַ���"xxx\0"
  char         strC[3];     // BIOS��C�汾�ţ��ַ���"xx\0"
  char         strB[6];     // BIOS��B�汾�ţ��ַ���"xxx\0"
  UINT32       biosImageSize; 
  char         vName[32];   // �汾����
  UINT32       biosUpdateFlag; 
  UINT32       fileCheckSum;    // BIOS�ļ���Checksum����֤����BIOS�ļ��ۼƺ�Ϊ0
  UINT32       dataCheckSum;   // Checksum Region0~3��У���
  UINT32       ckRegionAddr0;  
  UINT32       ckRegionSize0;  
  UINT32       ckRegionAddr1;  
  UINT32       ckRegionSize1;  
  UINT32       ckRegionAddr2;  
  UINT32       ckRegionSize2;  
  UINT32       ckRegionAddr3;  
  UINT32       ckRegionSize3;  
  UINT32       updateRegionNum;  
  UINT32       updateAddr0;  
  UINT32       updateSize0;  
  UINT32       updateAddr1;  
  UINT32       updateSize1;  
  UINT32       updateAddr2;  
  UINT32       updateSize2;  
  UINT32       updateAddr3;  
  UINT32       updateSize3;  
} BIOS_OEM_DATA;

typedef struct _NV_DATA_HEADER {
  EFI_GUID      DataGuid;
  UINT32        CrcCheckSum;
  UINT16        DataSize;
  UINT16        Flag;
} NV_DATA_HEADER;

#pragma pack()

#endif

