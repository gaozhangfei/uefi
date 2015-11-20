/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCTRBL0qDbp15usAFL4sRUNaa9PQnehq7qerrn2
LiJ41nIlwfFKj89OIPNlT8p4BEwC9MTzZ2aX+ZRoyaL5PELQhOJQErCF57+D+SnuiIpPempS
tpwbHf5yS9ytax5N6EwO4mZ8BvkQEMlYLeWwyAuT/beusl+6kVhtixIa4qgXkg==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
/*************************************************

  Copyright (C), 1988-2010, Huawei Tech. Co., Ltd.


  Author: h00132467		Version:        Date: 2011.4.18  16:59:39

  Description:

  History:

    1. Date:       Author:

       Modification:

*************************************************/
//dts2012071600238,d00214344,2012-7-16,fortify����BMM

#ifndef _OEM_FTP_PROTOCOL_H_
#define _OEM_FTP_PROTOCOL_H_

#define OEM_FTP_PROTOCOL_GUID \
    { \
        0x21655A5D, 0x1233, 0x4CE8, 0xB4, 0xD8, 0x87, 0xFD, 0xC1, 0x6C, 0x39, 0x68 \
    }

#define MAX_STR_SIZE 40

typedef struct
{
    IN EFI_HANDLE       nicInfo; // ������Ϣ
    IN EFI_IPv4_ADDRESS serverIp; // ftp������IP
    IN UINT16           port; // �˿�
    IN CHAR8            userName[MAX_STR_SIZE]; // ftp�û���, NULL��ʾ����
    IN CHAR8            BMM[MAX_STR_SIZE]; // ftp����, NULL��ʾ����
} OEM_FTP_TOKEN;

/*�����ļ�Э��ӿڶ��壬Ĭ�����ص�RamDisk�� ��Ŀ¼����������ļ�ϵͳ����ͬ�����ļ�����Ḳ��ͬ���ļ�*/
typedef
EFI_STATUS
(EFIAPI * FTP_GET_TO_FILESYS)(
    IN OEM_FTP_TOKEN *ftpToken,
    IN CHAR8   *      fileName //�ļ���
);

typedef
EFI_STATUS
(EFIAPI * FTP_GET_TO_MEM)(
    IN OEM_FTP_TOKEN *ftpToken,
    IN CHAR8    *     fileName, //�ļ���
    OUT UINTN   *     fileSize, // ������ļ���С
    OUT UINT8   **    data // Data�ں�����������ڴ棬�����سɹ�ʱ��������Ҫ�ͷ�Data
);

// ��Ramdisk�ĸ�Ŀ¼�ϴ��ļ��������������Ŀ¼�´���ͬ���ļ������ɷ��������趨�����Ƿ񸲸��ļ�
typedef
EFI_STATUS
(EFIAPI * FTP_PUT_FROM_FILESYS)(
    IN OEM_FTP_TOKEN *ftpToken,
    IN CHAR8    *     fileName //�ļ���
);

typedef
EFI_STATUS
(EFIAPI * FTP_PUT_FROM_MEM)(
    IN OEM_FTP_TOKEN *     ftpToken,
    IN UINT8              *data, //��Ҫ�ϴ�������
    IN UINTN               size, //�ϴ����ݵĴ�С
    IN CHAR8              *fileName // �ϴ������������ļ�����
);

typedef struct _OEM_FTP_PROTOCOL
{
    FTP_GET_TO_FILESYS   FtpRead;
    FTP_GET_TO_MEM       FtpReadToMem;
    FTP_PUT_FROM_FILESYS FtpWrite;
    FTP_PUT_FROM_MEM     FtpWriteFromMem;
} OEM_FTP_PROTOCOL;

extern EFI_GUID gOemFtpProtocolGuid;

#endif
