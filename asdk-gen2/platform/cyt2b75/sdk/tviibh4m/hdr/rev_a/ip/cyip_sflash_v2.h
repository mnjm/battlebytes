/***************************************************************************//**
* \file cyip_sflash_v2.h
*
* \brief
* SFLASH IP definitions
*
* \note
* Generator version: 1.5.0.1299
* Database revision: TVIIBH4M_WW1937_for_CFR_MetalTO
*
********************************************************************************
* \copyright
* Copyright 2016-2019, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _CYIP_SFLASH_V2_H_
#define _CYIP_SFLASH_V2_H_

#include "cyip_headers.h"

/*******************************************************************************
*                                    SFLASH
*******************************************************************************/

#define SFLASH_SECTION_SIZE                     0x00008000UL

/**
  * \brief Indicates Silicon Revision ID of the device (SFLASH_SI_REVISION_ID)
  */
typedef struct stc_SFLASH_SI_REVISION_ID_field {
  uint8_t                          u8SI_REVISION_ID:8;
} stc_SFLASH_SI_REVISION_ID_field_t;

typedef union un_SFLASH_SI_REVISION_ID {
  uint8_t                          u8Register;
  stc_SFLASH_SI_REVISION_ID_field_t stcField;
} un_SFLASH_SI_REVISION_ID_t;

/**
  * \brief Indicates Silicon ID of the device (SFLASH_SILICON_ID)
  */
typedef struct stc_SFLASH_SILICON_ID_field {
  uint16_t                         u16ID:16;
} stc_SFLASH_SILICON_ID_field_t;

typedef union un_SFLASH_SILICON_ID {
  uint16_t                         u16Register;
  stc_SFLASH_SILICON_ID_field_t    stcField;
  uint8_t                          au8Byte[2];
} un_SFLASH_SILICON_ID_t;

/**
  * \brief Flash boot flags (SFLASH_FB_FLAGS)
  */
typedef struct stc_SFLASH_FB_FLAGS_field {
  uint32_t                         u2FB_PIN_CTL:2;
  uint32_t                         u2FB_RSA3K_CTL:2;
  uint32_t                         u2FB_RSA4K_CTL:2;
  uint32_t                         :26;
} stc_SFLASH_FB_FLAGS_field_t;

typedef union un_SFLASH_FB_FLAGS {
  uint32_t                         u32Register;
  stc_SFLASH_FB_FLAGS_field_t      stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_FB_FLAGS_t;

/**
  * \brief SRSS_PWR_OFFSET (SFLASH_SRSS_PWR_OFFSET)
  */
typedef struct stc_SFLASH_SRSS_PWR_OFFSET_field {
  uint32_t                         u5PMIC_VADJ_OFFSET:5;
  uint32_t                         :3;
  uint32_t                         u5REGHC_TRANS_VADJ_OFFSET:5;
  uint32_t                         :19;
} stc_SFLASH_SRSS_PWR_OFFSET_field_t;

typedef union un_SFLASH_SRSS_PWR_OFFSET {
  uint32_t                         u32Register;
  stc_SFLASH_SRSS_PWR_OFFSET_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_SRSS_PWR_OFFSET_t;

/**
  * \brief Trim settings when the supply is intended to come from the internal regulators (SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL)
  */
typedef struct stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_field {
  uint32_t                         u5ACT_REG_VTRIM:5;
  uint32_t                         :19;
  uint32_t                         u4DPSLP_REG_VTRIM:4;
  uint32_t                         :4;
} stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_field_t;

typedef union un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL {
  uint32_t                         u32Register;
  stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_t;

/**
  * \brief Trim settings for the regulators so they do not interfere with external supply (SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL)
  */
typedef struct stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_field {
  uint32_t                         u5ACT_REG_VTRIM:5;
  uint32_t                         :19;
  uint32_t                         u4DPSLP_REG_VTRIM:4;
  uint32_t                         :4;
} stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_field_t;

typedef union un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL {
  uint32_t                         u32Register;
  stc_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_t;

/**
  * \brief USER_FREE_ROW0 (SFLASH_USER_FREE_ROW0)
  */
typedef struct stc_SFLASH_USER_FREE_ROW0_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_USER_FREE_ROW0_field_t;

typedef union un_SFLASH_USER_FREE_ROW0 {
  uint32_t                         u32Register;
  stc_SFLASH_USER_FREE_ROW0_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_USER_FREE_ROW0_t;

/**
  * \brief USER_FREE_ROW1 (SFLASH_USER_FREE_ROW1)
  */
typedef struct stc_SFLASH_USER_FREE_ROW1_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_USER_FREE_ROW1_field_t;

typedef union un_SFLASH_USER_FREE_ROW1 {
  uint32_t                         u32Register;
  stc_SFLASH_USER_FREE_ROW1_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_USER_FREE_ROW1_t;

/**
  * \brief USER_FREE_ROW2 (SFLASH_USER_FREE_ROW2)
  */
typedef struct stc_SFLASH_USER_FREE_ROW2_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_USER_FREE_ROW2_field_t;

typedef union un_SFLASH_USER_FREE_ROW2 {
  uint32_t                         u32Register;
  stc_SFLASH_USER_FREE_ROW2_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_USER_FREE_ROW2_t;

/**
  * \brief USER_FREE_ROW3 (SFLASH_USER_FREE_ROW3)
  */
typedef struct stc_SFLASH_USER_FREE_ROW3_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_USER_FREE_ROW3_field_t;

typedef union un_SFLASH_USER_FREE_ROW3 {
  uint32_t                         u32Register;
  stc_SFLASH_USER_FREE_ROW3_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_USER_FREE_ROW3_t;

/**
  * \brief Markers for storing SFLASH programming states (SFLASH_SFLASH_UPDATE_MARKER)
  */
typedef struct stc_SFLASH_SFLASH_UPDATE_MARKER_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_SFLASH_UPDATE_MARKER_field_t;

typedef union un_SFLASH_SFLASH_UPDATE_MARKER {
  uint32_t                         u32Register;
  stc_SFLASH_SFLASH_UPDATE_MARKER_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_SFLASH_UPDATE_MARKER_t;

/**
  * \brief Flash Boot - Object Size (SFLASH_FLASH_BOOT_OBJECT_SIZE)
  */
typedef struct stc_SFLASH_FLASH_BOOT_OBJECT_SIZE_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_FLASH_BOOT_OBJECT_SIZE_field_t;

typedef union un_SFLASH_FLASH_BOOT_OBJECT_SIZE {
  uint32_t                         u32Register;
  stc_SFLASH_FLASH_BOOT_OBJECT_SIZE_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_FLASH_BOOT_OBJECT_SIZE_t;

/**
  * \brief Flash Boot - Application ID/Version (SFLASH_FLASH_BOOT_APP_ID)
  */
typedef struct stc_SFLASH_FLASH_BOOT_APP_ID_field {
  uint32_t                         u16APP_ID:16;
  uint32_t                         u8MINOR_VERSION:8;
  uint32_t                         u4MAJOR_VERSION:4;
  uint32_t                         :4;
} stc_SFLASH_FLASH_BOOT_APP_ID_field_t;

typedef union un_SFLASH_FLASH_BOOT_APP_ID {
  uint32_t                         u32Register;
  stc_SFLASH_FLASH_BOOT_APP_ID_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_FLASH_BOOT_APP_ID_t;

/**
  * \brief Flash Boot - Core Vector Table (SFLASH_FLASH_BOOT_VECTOR_TABLE)
  */
typedef struct stc_SFLASH_FLASH_BOOT_VECTOR_TABLE_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_FLASH_BOOT_VECTOR_TABLE_field_t;

typedef union un_SFLASH_FLASH_BOOT_VECTOR_TABLE {
  uint32_t                         u32Register;
  stc_SFLASH_FLASH_BOOT_VECTOR_TABLE_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_FLASH_BOOT_VECTOR_TABLE_t;

/**
  * \brief Public key for signature verification (max RSA key size 4096) (SFLASH_PUBLIC_KEY)
  */
typedef struct stc_SFLASH_PUBLIC_KEY_field {
  uint8_t                          u8DATA:8;
} stc_SFLASH_PUBLIC_KEY_field_t;

typedef union un_SFLASH_PUBLIC_KEY {
  uint8_t                          u8Register;
  stc_SFLASH_PUBLIC_KEY_field_t    stcField;
} un_SFLASH_PUBLIC_KEY_t;

/**
  * \brief Application protection settings (4*128=512 bytes) (SFLASH_APP_PROT_SETTINGS)
  */
typedef struct stc_SFLASH_APP_PROT_SETTINGS_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_APP_PROT_SETTINGS_field_t;

typedef union un_SFLASH_APP_PROT_SETTINGS {
  uint32_t                         u32Register;
  stc_SFLASH_APP_PROT_SETTINGS_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_APP_PROT_SETTINGS_t;

/**
  * \brief Object size in bytes for CRC calculation starting from offset 0x00 (SFLASH_TOC1_OBJECT_SIZE)
  */
typedef struct stc_SFLASH_TOC1_OBJECT_SIZE_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_OBJECT_SIZE_field_t;

typedef union un_SFLASH_TOC1_OBJECT_SIZE {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_OBJECT_SIZE_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_OBJECT_SIZE_t;

/**
  * \brief Magic number(0x01211219) (SFLASH_TOC1_MAGIC_NUMBER)
  */
typedef struct stc_SFLASH_TOC1_MAGIC_NUMBER_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_MAGIC_NUMBER_field_t;

typedef union un_SFLASH_TOC1_MAGIC_NUMBER {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_MAGIC_NUMBER_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_MAGIC_NUMBER_t;

/**
  * \brief Number of objects to be verified for FACTORY_HASH (SFLASH_TOC1_FHASH_OBJECTS)
  */
typedef struct stc_SFLASH_TOC1_FHASH_OBJECTS_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_FHASH_OBJECTS_field_t;

typedef union un_SFLASH_TOC1_FHASH_OBJECTS {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_FHASH_OBJECTS_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_FHASH_OBJECTS_t;

/**
  * \brief Unused (Address is Hardcoded in ROM) (SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED)
  */
typedef struct stc_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_field_t;

typedef union un_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_t;

/**
  * \brief Address of Unique ID stored in SFLASH (SFLASH_TOC1_UNIQUE_ID_ADDR)
  */
typedef struct stc_SFLASH_TOC1_UNIQUE_ID_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_UNIQUE_ID_ADDR_field_t;

typedef union un_SFLASH_TOC1_UNIQUE_ID_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_UNIQUE_ID_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_UNIQUE_ID_ADDR_t;

/**
  * \brief Addresss of FLASH Boot(FB) object that include FLASH patch also (SFLASH_TOC1_FB_OBJECT_ADDR)
  */
typedef struct stc_SFLASH_TOC1_FB_OBJECT_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_FB_OBJECT_ADDR_field_t;

typedef union un_SFLASH_TOC1_FB_OBJECT_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_FB_OBJECT_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_FB_OBJECT_ADDR_t;

/**
  * \brief Unused (Address is Hardcoded in ROM) (SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED)
  */
typedef struct stc_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_field_t;

typedef union un_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_t;

/**
  * \brief Unused (Address is Hardcoded in ROM) (SFLASH_TOC1_OBJECT_ADDR_UNUSED)
  */
typedef struct stc_SFLASH_TOC1_OBJECT_ADDR_UNUSED_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_OBJECT_ADDR_UNUSED_field_t;

typedef union un_SFLASH_TOC1_OBJECT_ADDR_UNUSED {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_OBJECT_ADDR_UNUSED_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_OBJECT_ADDR_UNUSED_t;

/**
  * \brief Address of boot protection object (SFLASH_TOC1_BOOT_PROTECTION_ADDR)
  */
typedef struct stc_SFLASH_TOC1_BOOT_PROTECTION_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_BOOT_PROTECTION_ADDR_field_t;

typedef union un_SFLASH_TOC1_BOOT_PROTECTION_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_BOOT_PROTECTION_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_BOOT_PROTECTION_ADDR_t;

/**
  * \brief Address of 8051 code OTP flash (SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR)
  */
typedef struct stc_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_field_t;

typedef union un_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_t;

/**
  * \brief Object size in bytes for CRC calculation starting from offset 0x00 (SFLASH_TOC2_OBJECT_SIZE)
  */
typedef struct stc_SFLASH_TOC2_OBJECT_SIZE_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_OBJECT_SIZE_field_t;

typedef union un_SFLASH_TOC2_OBJECT_SIZE {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_OBJECT_SIZE_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_OBJECT_SIZE_t;

/**
  * \brief Magic number(0x01211220) (SFLASH_TOC2_MAGIC_NUMBER)
  */
typedef struct stc_SFLASH_TOC2_MAGIC_NUMBER_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_MAGIC_NUMBER_field_t;

typedef union un_SFLASH_TOC2_MAGIC_NUMBER {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_MAGIC_NUMBER_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_MAGIC_NUMBER_t;

/**
  * \brief Null terminated table of pointers representing the SMIF configuration structure (SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR)
  */
typedef struct stc_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_field_t;

typedef union un_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_t;

/**
  * \brief Address of First User Application Object (SFLASH_TOC2_FIRST_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_FIRST_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_FIRST_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_FIRST_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_FIRST_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_FIRST_USER_APP_ADDR_t;

/**
  * \brief Format of First User Application Object. 0 - Basic, 1 - Cypress standard & 2 - Simplified (SFLASH_TOC2_FIRST_USER_APP_FORMAT)
  */
typedef struct stc_SFLASH_TOC2_FIRST_USER_APP_FORMAT_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_FIRST_USER_APP_FORMAT_field_t;

typedef union un_SFLASH_TOC2_FIRST_USER_APP_FORMAT {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_FIRST_USER_APP_FORMAT_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_FIRST_USER_APP_FORMAT_t;

/**
  * \brief Address of Second User Application Object (SFLASH_TOC2_SECOND_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_SECOND_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SECOND_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_SECOND_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SECOND_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SECOND_USER_APP_ADDR_t;

/**
  * \brief Format of Second User Application Object. 0 - Basic, 1 - Cypress standard & 2 - Simplified (SFLASH_TOC2_SECOND_USER_APP_FORMAT)
  */
typedef struct stc_SFLASH_TOC2_SECOND_USER_APP_FORMAT_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SECOND_USER_APP_FORMAT_field_t;

typedef union un_SFLASH_TOC2_SECOND_USER_APP_FORMAT {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SECOND_USER_APP_FORMAT_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SECOND_USER_APP_FORMAT_t;

/**
  * \brief Address of First CM4 or CM7 core1 User Application Object (SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_t;

/**
  * \brief Address of Second CM4 or CM7 core1 User Application Object (SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_t;

/**
  * \brief Address of First CM4 or CM7 core2 User Application Object (SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_t;

/**
  * \brief Address of Second CM4 or CM7 core2 User Application Object (SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR)
  */
typedef struct stc_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_field_t;

typedef union un_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_t;

/**
  * \brief Number of additional objects to be verified for SECURE_HASH (SFLASH_TOC2_SHASH_OBJECTS)
  */
typedef struct stc_SFLASH_TOC2_SHASH_OBJECTS_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SHASH_OBJECTS_field_t;

typedef union un_SFLASH_TOC2_SHASH_OBJECTS {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SHASH_OBJECTS_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SHASH_OBJECTS_t;

/**
  * \brief Address of  signature verification key (0 if none).The object is signature specific key. It is the public key in case of RSA (SFLASH_TOC2_SIGNATURE_VERIF_KEY)
  */
typedef struct stc_SFLASH_TOC2_SIGNATURE_VERIF_KEY_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_SIGNATURE_VERIF_KEY_field_t;

typedef union un_SFLASH_TOC2_SIGNATURE_VERIF_KEY {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_SIGNATURE_VERIF_KEY_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_SIGNATURE_VERIF_KEY_t;

/**
  * \brief Address of  Application Protection (SFLASH_TOC2_APP_PROTECTION_ADDR)
  */
typedef struct stc_SFLASH_TOC2_APP_PROTECTION_ADDR_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_APP_PROTECTION_ADDR_field_t;

typedef union un_SFLASH_TOC2_APP_PROTECTION_ADDR {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_APP_PROTECTION_ADDR_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_APP_PROTECTION_ADDR_t;

/**
  * \brief Indicates TOC2 Revision. It is not used now. (SFLASH_TOC2_REVISION)
  */
typedef struct stc_SFLASH_TOC2_REVISION_field {
  uint32_t                         u32DATA32:32;
} stc_SFLASH_TOC2_REVISION_field_t;

typedef union un_SFLASH_TOC2_REVISION {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_REVISION_field_t stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_REVISION_t;

/**
  * \brief Controls default configuration (SFLASH_TOC2_FLAGS)
  */
typedef struct stc_SFLASH_TOC2_FLAGS_field {
  uint32_t                         u2CLOCK_CONFIG:2;
  uint32_t                         u3LISTEN_WINDOW:3;
  uint32_t                         u2SWJ_PINS_CTL:2;
  uint32_t                         u2APP_AUTH_CTL:2;
  uint32_t                         u2FB_BOOTLOADER_CTL:2;
  uint32_t                         :21;
} stc_SFLASH_TOC2_FLAGS_field_t;

typedef union un_SFLASH_TOC2_FLAGS {
  uint32_t                         u32Register;
  stc_SFLASH_TOC2_FLAGS_field_t    stcField;
  uint8_t                          au8Byte[4];
  uint16_t                         au16Halfword[2];
} un_SFLASH_TOC2_FLAGS_t;



/**
  * \brief FLASH Supervisory Region (SFLASH)
  */
typedef struct stc_SFLASH {
  uint8_t                          u8Reserved;
  un_SFLASH_SI_REVISION_ID_t       unSI_REVISION_ID;   /*!< 0x00000001 Indicates Silicon Revision ID of the device */
  un_SFLASH_SILICON_ID_t           unSILICON_ID;       /*!< 0x00000002 Indicates Silicon ID of the device */
  uint32_t                         au32Reserved1[126];
  un_SFLASH_FB_FLAGS_t             unFB_FLAGS;         /*!< 0x000001FC Flash boot flags */
  uint32_t                         au32Reserved2[332];
  un_SFLASH_SRSS_PWR_OFFSET_t      unSRSS_PWR_OFFSET;  /*!< 0x00000730 SRSS_PWR_OFFSET */
  un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_t unSRSS_PWR_TRIM_HT_PWRSYS_INTERNAL; /*!< 0x00000734 Trim settings when the supply is intended to come from
                                                                                the internal regulators */
  un_SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_t unSRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL; /*!< 0x00000738 Trim settings for the regulators so they do not
                                                                                interfere with external supply */
  uint32_t                         au32Reserved3[49];
  un_SFLASH_USER_FREE_ROW0_t       unUSER_FREE_ROW0[128]; /*!< 0x00000800 USER_FREE_ROW0 */
  un_SFLASH_USER_FREE_ROW1_t       unUSER_FREE_ROW1[128]; /*!< 0x00000A00 USER_FREE_ROW1 */
  un_SFLASH_USER_FREE_ROW2_t       unUSER_FREE_ROW2[128]; /*!< 0x00000C00 USER_FREE_ROW2 */
  un_SFLASH_USER_FREE_ROW3_t       unUSER_FREE_ROW3[128]; /*!< 0x00000E00 USER_FREE_ROW3 */
  uint32_t                         au32Reserved4[512];
  un_SFLASH_SFLASH_UPDATE_MARKER_t unSFLASH_UPDATE_MARKER[2]; /*!< 0x00001800 Markers for storing SFLASH programming states */
  uint32_t                         au32Reserved5[510];
  un_SFLASH_FLASH_BOOT_OBJECT_SIZE_t unFLASH_BOOT_OBJECT_SIZE; /*!< 0x00002000 Flash Boot - Object Size */
  un_SFLASH_FLASH_BOOT_APP_ID_t    unFLASH_BOOT_APP_ID; /*!< 0x00002004 Flash Boot - Application ID/Version */
  uint32_t                         au32Reserved6[4];
  un_SFLASH_FLASH_BOOT_VECTOR_TABLE_t unFLASH_BOOT_VECTOR_TABLE[48]; /*!< 0x00002018 Flash Boot - Core Vector Table */
  uint32_t                         au32Reserved7[4298];
  un_SFLASH_PUBLIC_KEY_t           unPUBLIC_KEY[3072]; /*!< 0x00006400 Public key for signature verification (max RSA key size
                                                                                4096) */
  uint32_t                         au32Reserved8[384];
  un_SFLASH_APP_PROT_SETTINGS_t    unAPP_PROT_SETTINGS[128]; /*!< 0x00007600 Application protection settings (4*128=512 bytes) */
  un_SFLASH_TOC1_OBJECT_SIZE_t     unTOC1_OBJECT_SIZE; /*!< 0x00007800 Object size in bytes for CRC calculation starting from
                                                                                offset 0x00 */
  un_SFLASH_TOC1_MAGIC_NUMBER_t    unTOC1_MAGIC_NUMBER; /*!< 0x00007804 Magic number(0x01211219) */
  uint32_t                         au32Reserved9[2];
  un_SFLASH_TOC1_FHASH_OBJECTS_t   unTOC1_FHASH_OBJECTS; /*!< 0x00007810 Number of objects to be verified for FACTORY_HASH */
  un_SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_t unTOC1_GENERAL_TRIM_ADDR_UNUSED; /*!< 0x00007814 Unused (Address is Hardcoded in ROM) */
  un_SFLASH_TOC1_UNIQUE_ID_ADDR_t  unTOC1_UNIQUE_ID_ADDR; /*!< 0x00007818 Address of Unique ID stored in SFLASH */
  un_SFLASH_TOC1_FB_OBJECT_ADDR_t  unTOC1_FB_OBJECT_ADDR; /*!< 0x0000781C Addresss of FLASH Boot(FB) object that include FLASH
                                                                                patch also */
  un_SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_t unTOC1_SYSCALL_TABLE_ADDR_UNUSED; /*!< 0x00007820 Unused (Address is Hardcoded in ROM) */
  un_SFLASH_TOC1_OBJECT_ADDR_UNUSED_t unTOC1_OBJECT_ADDR_UNUSED; /*!< 0x00007824 Unused (Address is Hardcoded in ROM) */
  un_SFLASH_TOC1_BOOT_PROTECTION_ADDR_t unTOC1_BOOT_PROTECTION_ADDR; /*!< 0x00007828 Address of boot protection object */
  un_SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_t unTOC1_8051_CODE_OTP_FLASH_ADDR; /*!< 0x0000782C Address of 8051 code OTP flash */
  uint32_t                         au32Reserved10[244];
  un_SFLASH_TOC2_OBJECT_SIZE_t     unTOC2_OBJECT_SIZE; /*!< 0x00007C00 Object size in bytes for CRC calculation starting from
                                                                                offset 0x00 */
  un_SFLASH_TOC2_MAGIC_NUMBER_t    unTOC2_MAGIC_NUMBER; /*!< 0x00007C04 Magic number(0x01211220) */
  un_SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_t unTOC2_SMIF_CFG_STRUCT_ADDR; /*!< 0x00007C08 Null terminated table of pointers representing the SMIF
                                                                                configuration structure */
  un_SFLASH_TOC2_FIRST_USER_APP_ADDR_t unTOC2_FIRST_USER_APP_ADDR; /*!< 0x00007C0C Address of First User Application Object */
  un_SFLASH_TOC2_FIRST_USER_APP_FORMAT_t unTOC2_FIRST_USER_APP_FORMAT; /*!< 0x00007C10 Format of First User Application Object. 0 - Basic, 1 -
                                                                                Cypress standard & 2 - Simplified */
  un_SFLASH_TOC2_SECOND_USER_APP_ADDR_t unTOC2_SECOND_USER_APP_ADDR; /*!< 0x00007C14 Address of Second User Application Object */
  un_SFLASH_TOC2_SECOND_USER_APP_FORMAT_t unTOC2_SECOND_USER_APP_FORMAT; /*!< 0x00007C18 Format of Second User Application Object. 0 - Basic, 1 -
                                                                                Cypress standard & 2 - Simplified */
  un_SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_t unTOC2_FIRST_CMX_1_USER_APP_ADDR; /*!< 0x00007C1C Address of First CM4 or CM7 core1 User Application
                                                                                Object */
  un_SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_t unTOC2_SECOND_CMX_1_USER_APP_ADDR; /*!< 0x00007C20 Address of Second CM4 or CM7 core1 User Application
                                                                                Object */
  un_SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_t unTOC2_FIRST_CMX_2_USER_APP_ADDR; /*!< 0x00007C24 Address of First CM4 or CM7 core2 User Application
                                                                                Object */
  un_SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_t unTOC2_SECOND_CMX_2_USER_APP_ADDR; /*!< 0x00007C28 Address of Second CM4 or CM7 core2 User Application
                                                                                Object */
  uint32_t                         au32Reserved11[53];
  un_SFLASH_TOC2_SHASH_OBJECTS_t   unTOC2_SHASH_OBJECTS; /*!< 0x00007D00 Number of additional objects to be verified for
                                                                                SECURE_HASH */
  un_SFLASH_TOC2_SIGNATURE_VERIF_KEY_t unTOC2_SIGNATURE_VERIF_KEY; /*!< 0x00007D04 Address of signature verification key (0 if none).The
                                                                                object is signature specific key. It is the
                                                                                public key in case of RSA */
  un_SFLASH_TOC2_APP_PROTECTION_ADDR_t unTOC2_APP_PROTECTION_ADDR; /*!< 0x00007D08 Address of  Application Protection */
  uint32_t                         au32Reserved12[58];
  un_SFLASH_TOC2_REVISION_t        unTOC2_REVISION;    /*!< 0x00007DF4 Indicates TOC2 Revision. It is not used now. */
  un_SFLASH_TOC2_FLAGS_t           unTOC2_FLAGS;       /*!< 0x00007DF8 Controls default configuration */
} stc_SFLASH_t;                                        /*!< Size = 32252 (0x7DFC) */


/* SFLASH.SI_REVISION_ID */
#define SFLASH_SI_REVISION_ID_SI_REVISION_ID_Pos 0UL
#define SFLASH_SI_REVISION_ID_SI_REVISION_ID_Msk 0xFFUL
/* SFLASH.SILICON_ID */
#define SFLASH_SILICON_ID_ID_Pos                0UL
#define SFLASH_SILICON_ID_ID_Msk                0xFFFFUL
/* SFLASH.FB_FLAGS */
#define SFLASH_FB_FLAGS_FB_PIN_CTL_Pos          0UL
#define SFLASH_FB_FLAGS_FB_PIN_CTL_Msk          0x3UL
#define SFLASH_FB_FLAGS_FB_RSA3K_CTL_Pos        2UL
#define SFLASH_FB_FLAGS_FB_RSA3K_CTL_Msk        0xCUL
#define SFLASH_FB_FLAGS_FB_RSA4K_CTL_Pos        4UL
#define SFLASH_FB_FLAGS_FB_RSA4K_CTL_Msk        0x30UL
/* SFLASH.SRSS_PWR_OFFSET */
#define SFLASH_SRSS_PWR_OFFSET_PMIC_VADJ_OFFSET_Pos 0UL
#define SFLASH_SRSS_PWR_OFFSET_PMIC_VADJ_OFFSET_Msk 0x1FUL
#define SFLASH_SRSS_PWR_OFFSET_REGHC_TRANS_VADJ_OFFSET_Pos 8UL
#define SFLASH_SRSS_PWR_OFFSET_REGHC_TRANS_VADJ_OFFSET_Msk 0x1F00UL
/* SFLASH.SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL */
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_ACT_REG_VTRIM_Pos 0UL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_ACT_REG_VTRIM_Msk 0x1FUL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_DPSLP_REG_VTRIM_Pos 24UL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_INTERNAL_DPSLP_REG_VTRIM_Msk 0xF000000UL
/* SFLASH.SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL */
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_ACT_REG_VTRIM_Pos 0UL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_ACT_REG_VTRIM_Msk 0x1FUL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_DPSLP_REG_VTRIM_Pos 24UL
#define SFLASH_SRSS_PWR_TRIM_HT_PWRSYS_EXTERNAL_DPSLP_REG_VTRIM_Msk 0xF000000UL
/* SFLASH.USER_FREE_ROW0 */
#define SFLASH_USER_FREE_ROW0_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW0_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW1 */
#define SFLASH_USER_FREE_ROW1_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW1_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW2 */
#define SFLASH_USER_FREE_ROW2_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW2_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.USER_FREE_ROW3 */
#define SFLASH_USER_FREE_ROW3_DATA32_Pos        0UL
#define SFLASH_USER_FREE_ROW3_DATA32_Msk        0xFFFFFFFFUL
/* SFLASH.SFLASH_UPDATE_MARKER */
#define SFLASH_SFLASH_UPDATE_MARKER_DATA32_Pos  0UL
#define SFLASH_SFLASH_UPDATE_MARKER_DATA32_Msk  0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_OBJECT_SIZE */
#define SFLASH_FLASH_BOOT_OBJECT_SIZE_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_OBJECT_SIZE_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.FLASH_BOOT_APP_ID */
#define SFLASH_FLASH_BOOT_APP_ID_APP_ID_Pos     0UL
#define SFLASH_FLASH_BOOT_APP_ID_APP_ID_Msk     0xFFFFUL
#define SFLASH_FLASH_BOOT_APP_ID_MINOR_VERSION_Pos 16UL
#define SFLASH_FLASH_BOOT_APP_ID_MINOR_VERSION_Msk 0xFF0000UL
#define SFLASH_FLASH_BOOT_APP_ID_MAJOR_VERSION_Pos 24UL
#define SFLASH_FLASH_BOOT_APP_ID_MAJOR_VERSION_Msk 0xF000000UL
/* SFLASH.FLASH_BOOT_VECTOR_TABLE */
#define SFLASH_FLASH_BOOT_VECTOR_TABLE_DATA32_Pos 0UL
#define SFLASH_FLASH_BOOT_VECTOR_TABLE_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.PUBLIC_KEY */
#define SFLASH_PUBLIC_KEY_DATA_Pos              0UL
#define SFLASH_PUBLIC_KEY_DATA_Msk              0xFFUL
/* SFLASH.APP_PROT_SETTINGS */
#define SFLASH_APP_PROT_SETTINGS_DATA32_Pos     0UL
#define SFLASH_APP_PROT_SETTINGS_DATA32_Msk     0xFFFFFFFFUL
/* SFLASH.TOC1_OBJECT_SIZE */
#define SFLASH_TOC1_OBJECT_SIZE_DATA32_Pos      0UL
#define SFLASH_TOC1_OBJECT_SIZE_DATA32_Msk      0xFFFFFFFFUL
/* SFLASH.TOC1_MAGIC_NUMBER */
#define SFLASH_TOC1_MAGIC_NUMBER_DATA32_Pos     0UL
#define SFLASH_TOC1_MAGIC_NUMBER_DATA32_Msk     0xFFFFFFFFUL
/* SFLASH.TOC1_FHASH_OBJECTS */
#define SFLASH_TOC1_FHASH_OBJECTS_DATA32_Pos    0UL
#define SFLASH_TOC1_FHASH_OBJECTS_DATA32_Msk    0xFFFFFFFFUL
/* SFLASH.TOC1_GENERAL_TRIM_ADDR_UNUSED */
#define SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_DATA32_Pos 0UL
#define SFLASH_TOC1_GENERAL_TRIM_ADDR_UNUSED_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC1_UNIQUE_ID_ADDR */
#define SFLASH_TOC1_UNIQUE_ID_ADDR_DATA32_Pos   0UL
#define SFLASH_TOC1_UNIQUE_ID_ADDR_DATA32_Msk   0xFFFFFFFFUL
/* SFLASH.TOC1_FB_OBJECT_ADDR */
#define SFLASH_TOC1_FB_OBJECT_ADDR_DATA32_Pos   0UL
#define SFLASH_TOC1_FB_OBJECT_ADDR_DATA32_Msk   0xFFFFFFFFUL
/* SFLASH.TOC1_SYSCALL_TABLE_ADDR_UNUSED */
#define SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_DATA32_Pos 0UL
#define SFLASH_TOC1_SYSCALL_TABLE_ADDR_UNUSED_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC1_OBJECT_ADDR_UNUSED */
#define SFLASH_TOC1_OBJECT_ADDR_UNUSED_DATA32_Pos 0UL
#define SFLASH_TOC1_OBJECT_ADDR_UNUSED_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC1_BOOT_PROTECTION_ADDR */
#define SFLASH_TOC1_BOOT_PROTECTION_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC1_BOOT_PROTECTION_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC1_8051_CODE_OTP_FLASH_ADDR */
#define SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC1_8051_CODE_OTP_FLASH_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_OBJECT_SIZE */
#define SFLASH_TOC2_OBJECT_SIZE_DATA32_Pos      0UL
#define SFLASH_TOC2_OBJECT_SIZE_DATA32_Msk      0xFFFFFFFFUL
/* SFLASH.TOC2_MAGIC_NUMBER */
#define SFLASH_TOC2_MAGIC_NUMBER_DATA32_Pos     0UL
#define SFLASH_TOC2_MAGIC_NUMBER_DATA32_Msk     0xFFFFFFFFUL
/* SFLASH.TOC2_SMIF_CFG_STRUCT_ADDR */
#define SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_SMIF_CFG_STRUCT_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_FIRST_USER_APP_ADDR */
#define SFLASH_TOC2_FIRST_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_FIRST_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_FIRST_USER_APP_FORMAT */
#define SFLASH_TOC2_FIRST_USER_APP_FORMAT_DATA32_Pos 0UL
#define SFLASH_TOC2_FIRST_USER_APP_FORMAT_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_SECOND_USER_APP_ADDR */
#define SFLASH_TOC2_SECOND_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_SECOND_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_SECOND_USER_APP_FORMAT */
#define SFLASH_TOC2_SECOND_USER_APP_FORMAT_DATA32_Pos 0UL
#define SFLASH_TOC2_SECOND_USER_APP_FORMAT_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_FIRST_CMX_1_USER_APP_ADDR */
#define SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_FIRST_CMX_1_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_SECOND_CMX_1_USER_APP_ADDR */
#define SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_SECOND_CMX_1_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_FIRST_CMX_2_USER_APP_ADDR */
#define SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_FIRST_CMX_2_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_SECOND_CMX_2_USER_APP_ADDR */
#define SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_SECOND_CMX_2_USER_APP_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_SHASH_OBJECTS */
#define SFLASH_TOC2_SHASH_OBJECTS_DATA32_Pos    0UL
#define SFLASH_TOC2_SHASH_OBJECTS_DATA32_Msk    0xFFFFFFFFUL
/* SFLASH.TOC2_SIGNATURE_VERIF_KEY */
#define SFLASH_TOC2_SIGNATURE_VERIF_KEY_DATA32_Pos 0UL
#define SFLASH_TOC2_SIGNATURE_VERIF_KEY_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_APP_PROTECTION_ADDR */
#define SFLASH_TOC2_APP_PROTECTION_ADDR_DATA32_Pos 0UL
#define SFLASH_TOC2_APP_PROTECTION_ADDR_DATA32_Msk 0xFFFFFFFFUL
/* SFLASH.TOC2_REVISION */
#define SFLASH_TOC2_REVISION_DATA32_Pos         0UL
#define SFLASH_TOC2_REVISION_DATA32_Msk         0xFFFFFFFFUL
/* SFLASH.TOC2_FLAGS */
#define SFLASH_TOC2_FLAGS_CLOCK_CONFIG_Pos      0UL
#define SFLASH_TOC2_FLAGS_CLOCK_CONFIG_Msk      0x3UL
#define SFLASH_TOC2_FLAGS_LISTEN_WINDOW_Pos     2UL
#define SFLASH_TOC2_FLAGS_LISTEN_WINDOW_Msk     0x1CUL
#define SFLASH_TOC2_FLAGS_SWJ_PINS_CTL_Pos      5UL
#define SFLASH_TOC2_FLAGS_SWJ_PINS_CTL_Msk      0x60UL
#define SFLASH_TOC2_FLAGS_APP_AUTH_CTL_Pos      7UL
#define SFLASH_TOC2_FLAGS_APP_AUTH_CTL_Msk      0x180UL
#define SFLASH_TOC2_FLAGS_FB_BOOTLOADER_CTL_Pos 9UL
#define SFLASH_TOC2_FLAGS_FB_BOOTLOADER_CTL_Msk 0x600UL


#endif /* _CYIP_SFLASH_V2_H_ */


/* [] END OF FILE */
