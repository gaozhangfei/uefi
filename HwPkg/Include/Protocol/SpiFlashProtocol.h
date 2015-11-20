/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCTRBL0qDbp15usAFL4sRUNaa9PQnehq7qerrn2
LiJ41i0HarRkiVP9Q0CoUIFb8/RA2HzrPvNn+FXbs2A9GUyO5V1FALcj5jCLeg5/eKgM6DNq
1zOBsbrxjZR4+KssYlZ2NPkRW2d3S5ZSbWqFWaNHyJJ5BfZUOG+Vkdn8ze/1cA==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2014, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : SpiFlashProtocol.h
  �� �� ��   : 
  ��    ��   :
  ��������   : 
  ����޸�   :
  ��������   : Spi FlashЭ��
  �޸���ʷ   :
1.   ��	  ��   : 
     ��	  ��   :
     �޸�����  :
******************************************************************************/
#ifndef _SPI_FLASH_PROTOCOL_H_
#define _SPI_FLASH_PROTOCOL_H_

typedef struct _SPI_FLASH_PROTOCOL SPI_FLASH_PROTOCOL;
//erase �ӿ�
typedef 
EFI_STATUS
(EFIAPI *SPI_FLASH_ERASE_INTERFACE) (
    IN SPI_FLASH_PROTOCOL   *This,
    IN UINT32                  Offset,
    IN UINT32                  ulLength
    );
//write �ӿ�
typedef 
EFI_STATUS
(EFIAPI *SPI_FLASH_WRITE_INTERFACE) (
    IN SPI_FLASH_PROTOCOL  *This,
    IN  UINT32                 Offset,
    IN  UINT8                 *Buffer,
    IN  UINT32                 ulLength
    );
//read �ӿ�
typedef 
EFI_STATUS
(EFIAPI *SPI_FLASH_READ_INTERFACE) (
    IN SPI_FLASH_PROTOCOL   *This,
    IN UINT32                  Offset,
    IN OUT UINT8              *Buffer,
    IN UINT32                  ulLength
    );
//erase write �ӿ�
typedef 
EFI_STATUS
(EFIAPI *SPI_FLASH_ERASE_WRITE_INTERFACE) (
    IN SPI_FLASH_PROTOCOL  *This,
    IN  UINT32                 Offset,
    IN  UINT8                 *Buffer,
    IN  UINT32                 ulLength
    );
//�ӿ�����
struct _SPI_FLASH_PROTOCOL {
    SPI_FLASH_ERASE_INTERFACE             Erase;
    SPI_FLASH_WRITE_INTERFACE             Write;
    SPI_FLASH_READ_INTERFACE              Read;
    SPI_FLASH_ERASE_WRITE_INTERFACE       EraseWrite;
};

extern EFI_GUID gSpiFlashProtocolGuid;

#endif
