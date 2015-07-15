/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCTRBL0qDbp15usAFL4sRUNaa9PQnehq7qerrn2
LiJ41l5yiMtXvbMumZU+W/tkBtiWSYhmuDuczbk74t26qwrS2qVK87hyPz8szwuDjvOH0lA1
s2f7NLAcEIIEV9k+Y12x6PhyQ/NENn0PxzNaJW+8s9tSLo/H5r2AJxUKtwCZGw==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/******************************************************************************

                  ��Ȩ���� (C), 2009-2019, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : I2CLib.h
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

#ifndef _I2C_LIB_H_
#define _I2C_LIB_H_

//I2C0 or I2C1
typedef enum {
  DEVICE_TYPE_SPD = 0,
  DEVICE_TYPE_E2PROM,
  DEVICE_TYPE_CPLD_3BYTE_OPERANDS,
  DEVICE_TYPE_CPLD_4BYTE_OPERANDS
}I2C_DEVICE_TYPE;   //���ͷ��䲻����  z00201473  2014.7.15

//ģʽ����
typedef enum {
  Normal = 0,
  Fast,
  SPEED_MODE_MAX
}SPEED_MODE;


#define    I2C_PORT_MAX            9


/*I2C�豸��Ϣ����*/
typedef struct {
    UINT32           Socket;
    UINT32           Port;
    I2C_DEVICE_TYPE  DeviceType;
    UINT32           SlaveDeviceAddress;            
}I2C_DEVICE;

/*�����ⲿ���ú���*/
UINTN
EFIAPI
I2CInit(UINT32 Socket, UINT32 Port, SPEED_MODE SpeedMode);

EFI_STATUS
EFIAPI
I2CWrite(I2C_DEVICE *I2cInfo, UINT16 InfoOffset, UINT32 ulLength, UINT8 *pBuf);

EFI_STATUS
EFIAPI
I2CRead(I2C_DEVICE *I2cInfo, UINT16 InfoOffset,UINT32 ulRxLen,UINT8 *pBuf);

EFI_STATUS
EFIAPI
I2CWriteMultiByte(I2C_DEVICE *I2cInfo, UINT32 InfoOffset, UINT32 ulLength, UINT8 *pBuf);

EFI_STATUS
EFIAPI
I2CReadMultiByte(I2C_DEVICE *I2cInfo, UINT32 InfoOffset,UINT32 ulRxLen,UINT8 *pBuf);

EFI_STATUS
EFIAPI
I2CSdaConfig(UINT32 Socket, UINT32 Port);


#endif
