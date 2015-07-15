/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : MemoryMapData.h
  �� �� ��   : v1.0
  ��    ��   : 
  ��������   : 2014��11��20��
  ����޸�   :
  ��������   : GUID used for Memory Map Data entries in the HOB list.
  �޸���ʷ   :
******************************************************************************/

#ifndef _MEMORY_MAP_GUID_H_
#define _MEMORY_MAP_GUID_H_

#define EFI_MEMORY_MAP_GUID \
  { \
    0xf8870015,0x6994,0x4b98,0x95,0xa2,0xbd,0x56,0xda,0x91,0xc0,0x7f \
  }

extern EFI_GUID gEfiMemoryMapGuid;

#endif
