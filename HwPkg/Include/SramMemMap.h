/*--------------------------------------------------------------------------------------------------------------------------*/
/*!!Warning: This is a key information asset of Huawei Tech Co.,Ltd                                                         */
/*CODEMARK:kOyQZYzjDpyGdBAEC2GaWuVy7vy/wDnq7gJfHBOj2pBXFF9pJtpDLt9sw5WJiMsUkN5d7jr7
aK5J3kmlnl+vpat0TH65jPMlUENBvGXufQCTRBL0qDbp15usAFL4sRUNaa9PQnehq7qerrn2
LiJ41kiwWClc5LlSJlz17jzjAtzz6SN07g98M0Pt+aS/gLGohrY6d9rv8A9JhDXUFW1NU3LC
q3EyMwI8FqoLiIEigXSc/kZyHtVmDM4VoBwOrDLhrGE4WeHTepdQiwX4584Elw==*/
/*--------------------------------------------------------------------------------------------------------------------------*/
#ifndef _SRAM_MEMMAP_H_
#define _SRAM_MEMMAP_H_

/* *******��ȫ�����׶�SRAM �ռ�ʾ��ͼ************  */

/*0x20000 ------------------   */
/*       |     ����          | */
/*0x17000  ------------------  */
/*       |     SEC RSA Data  | */
/* 0x5000 ------------------   */
/*       |     stack         | */
/* 0x2000 ------------------   */
/*       |     BOOTROM����   | */
/*      0 ------------------   */
/* ************* end **************************** */


#define SRAM_BASE_ADDR       (0x81000000)   //ʹ��POE SRAM

#define SE_SRAM_SIZE         (0x20000)   //SRAM��С: A57-128KB

#define BOOTROM_SRAM_BASE    SRAM_BASE_ADDR
#define BOOTROM_SRAM_SIZE    (0x17000)

//BOOTROM���������
#define BOOTROM_CODE_START   BOOTROM_SRAM_BASE
#define BOOTROM_CODE_SIZE    (0x2000)

//��ջ��
#define SRAM_STACKPOINT_BASE (BOOTROM_SRAM_BASE+0x2000)
#define SRAM_STACKPOINT_SIZE (0x3000)
#define SRAM_STACKPOINT_TOP  (BOOTROM_SRAM_BASE+0x5000)

//��ϣ�������ݴ洢��
#define SRAM_BIOS_HASH_BASE (BOOTROM_SRAM_BASE+0x5000)
#define SRAM_BIOS_HASH_SIZE (0x10020)   //��ϣ���㣬�ռ�64KB+32B

//SEC RSA����������
#define QUE_BASE_ADDR    (BOOTROM_SRAM_BASE+0x15100)  //һ��BD��СΪ64�ֽ�
#define SEC_QUE_DEPTH    (32)  //SECģ��Ķ�����ȣ�ʵ�ʶ��������+1��ռ��0x840�ֽ� 
#define HASH1_SRAM_ADDR  (BOOTROM_SRAM_BASE+0x16000)  //һ����ϣֵ32�ֽ�
#define HASH2_SRAM_ADDR  (BOOTROM_SRAM_BASE+0x16100)
#define EN_SRAM_ADDR     (BOOTROM_SRAM_BASE+0x16200)
#define E_SRAM_ADDR      (BOOTROM_SRAM_BASE+0x16300) //E  ��С256�ֽ�
#define N_SRAM_ADDR      (BOOTROM_SRAM_BASE+0x16400) //N  ��С256�ֽ�
#define KEY_SRAM_ADDR    (BOOTROM_SRAM_BASE+0x16500)
#define IVIN_SRAM_ADDR   (BOOTROM_SRAM_BASE+0x16600)
#define SIGNATURE_ADDR   (BOOTROM_SRAM_BASE+0x16700) /*ǩ����С256�ֽ�*/
#define SEC_RSA_DATA_SIZE (0x1F00)

//MBISTʹ��
#define MBIST_DONE1       0x201a004
#define MBIST_DONE2       0x201a008

//�ڴ��ʼ���ɹ���־��ַ
#define DDRInitTagAddr     0xA201a010

#define SIDTagAddr        0xA201a020
#define ISI2CTagAddr      0xA201a024
#define ISHilink1TagAddr  0xA201a024


#endif

