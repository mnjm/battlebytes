/***************************************************************************//**
* \file main_cm4.c
*
* \version 1.0
*
* \brief Main example file for CM4
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"
#include "SMIF_TestData.h"


/***** User Modifiable definitions *****/
#define DLP_VALUE                       (0x34u)
#define SMIF_CLK_FREQ_SETUP             (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_SDR_RD            (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_SDR_FST_RD        (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_DDR_RD            (50000000UL) // Please modify as per your HW

/* Select the memory availabe on hardware to test */
#define CY_SMIF_USED                    SMIF0
#define CY_SMIF_QUAD_MEMORY             CY_SMIF_S25FXXXS

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
  #define TEST_SPI_READ_LC              CY_SMIF_S25FL_RD_LATENCY2
  #define DEVICE_SECTOR_ERASE           (0x00010000) // 64KB
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
  #define TEST_SPI_READ_LC              CY_SMIF_S25FL_RD_LATENCY8
  #define DEVICE_SECTOR_ERASE           (0x00001000) // 4KB
#endif
#define DEVICE_PROGRAM_BUFFER_SIZE      (0x100)      // 256B
#define TEST_SPI_WRITE_LC               CY_SMIF_S25FL_WT_LATENCY0
#define TEST_PROGRAM_SECTOR             (0x01240000ul)
/**************************************/

// #define DLP_SETTING_UPDATE_AUTO

#ifdef DLP_SETTING_UPDATE_AUTO
  #define SETTING_DELAY_SEL CY_SMIF_1_NEW_SEL_PER_TAP
  #define SETTING_DLP_AUTO  CY_SMIF_DLP_UPDATE_AUTO
#else
  #define SETTING_DELAY_SEL CY_SMIF_NO_DELAY_SEL
  #define SETTING_DLP_AUTO  CY_SMIF_DLP_UPDATE_MANUAL
#endif


#define USE_QUAD_MODE 0 // 0: not quad, 1: quad

#define SMIF_MPU_REG_NO    MPU_REG_SMIF0_DEV
#define SMIF_HF_CLOCK      CY_SYSCLK_HFCLK_5
#define PLL_200M_0_PATH_NO (3u)
#define PLL_200M_0_PLL_NO  (PLL_200M_0_PATH_NO-1u)

static void ChangePLLFrequency(uint32_t outputFreq)
{
#if (CY_USE_PSVP == 0u)
    uint32_t pllInputClockFreq = 0;
    CY_ASSERT(Cy_SysClk_GetPllInputFrequency(PLL_200M_0_PLL_NO, &pllInputClockFreq) == CY_SYSCLK_SUCCESS);

    cy_stc_pll_config_t pll200_1_Config = 
    {
        .inputFreq     = pllInputClockFreq,
        .outputFreq    = outputFreq,    // target PLL output
        .lfMode        = 0u,            // VCO frequency is [200MHz, 400MHz]
        .outputMode    = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
    };

    CY_ASSERT(Cy_SysClk_PllDisable(PLL_200M_0_PATH_NO) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllConfigure(PLL_200M_0_PATH_NO , &pll200_1_Config) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllEnable(PLL_200M_0_PATH_NO, 10000ul) == CY_SYSCLK_SUCCESS);
#endif
    return;
}

typedef struct
{
    volatile stc_GPIO_PRT_t*  port;
    uint8_t                   pin;
    en_hsiom_sel_t            hsiom;
    uint32_t                  driveMode;
} cy_stc_smif_port_t;

cy_stc_smif_port_t smifPortCfg[] =
{
    {CY_BSP_SMIF0_CLK_PORT,     CY_BSP_SMIF0_CLK_PIN,     CY_BSP_SMIF0_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_RWDS_PORT,    CY_BSP_SMIF0_RWDS_PIN,    CY_BSP_SMIF0_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_SELECT0_PORT, CY_BSP_SMIF0_SELECT0_PIN, CY_BSP_SMIF0_SELECT0_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_SELECT1_PORT, CY_BSP_SMIF0_SELECT1_PIN, CY_BSP_SMIF0_SELECT1_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA0_PORT,   CY_BSP_SMIF0_DATA0_PIN,   CY_BSP_SMIF0_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA1_PORT,   CY_BSP_SMIF0_DATA1_PIN,   CY_BSP_SMIF0_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA2_PORT,   CY_BSP_SMIF0_DATA2_PIN,   CY_BSP_SMIF0_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA3_PORT,   CY_BSP_SMIF0_DATA3_PIN,   CY_BSP_SMIF0_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA4_PORT,   CY_BSP_SMIF0_DATA4_PIN,   CY_BSP_SMIF0_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA5_PORT,   CY_BSP_SMIF0_DATA5_PIN,   CY_BSP_SMIF0_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA6_PORT,   CY_BSP_SMIF0_DATA6_PIN,   CY_BSP_SMIF0_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA7_PORT,   CY_BSP_SMIF0_DATA7_PIN,   CY_BSP_SMIF0_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
};
#define SIZE_OF_PORT (sizeof(smifPortCfg)/sizeof(cy_stc_smif_port_t))

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size)
{
    cy_stc_gpio_pin_config_t pinCfg = {0};
    for(uint32_t i = 0; i < size; i++)
    {
        pinCfg.driveMode = cfg[i].driveMode;
        pinCfg.hsiom     = cfg[i].hsiom;
        Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
    }
}

cy_stc_smif_context_t smifContext;
uint8_t readDataBuffer[TEST_PROGRAM_SIZE];

cy_stc_smif_config_t smifConfig =
{
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_OUTPUT_CLK, // Note
    .delaySel      = SETTING_DELAY_SEL,
    .dlpAuto       = SETTING_DLP_AUTO,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_3_CLK_PULUS_MIN,
    .mode          = CY_SMIF_NORMAL, // MMIO mode
    .blockEvent    = CY_SMIF_BUS_ERROR,
};

cy_stc_device_config_t smifDevSingle0Cfg =
{
    .xipReadCmdId       = CY_SMIF_S25FL_ID_4READ,
    .xipReadMode        = CY_SMIF_S25FL_RW_NON_CONTINUOUS,
    .writeEn            = true,
    .xipWriteCmdId      = CY_SMIF_S25FL_ID_4PP,
    .xipWriteMode       = 0u,                      // Do not care
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 1000u,
    .addr               = 0,                       // will be updated in the application
    .size               = CY_SMIF_DEVICE_32M_BYTE,
    .rdLatencyCode      = TEST_SPI_READ_LC,
    .wtLatencyCode      = 0,                       // Do not care
};

cy_stc_device_config_t smifDevSingle1Cfg =
{
    .xipReadCmdId       = CY_SMIF_S25FL_ID_4READ,
    .xipReadMode        = CY_SMIF_S25FL_RW_NON_CONTINUOUS,
    .writeEn            = true,
    .xipWriteCmdId      = CY_SMIF_S25FL_ID_4PP,
    .xipWriteMode       = 0u,                      // Do not care
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 1000u,
    .addr               = 0,                       // will be updated in the application
    .size               = CY_SMIF_DEVICE_32M_BYTE,
    .rdLatencyCode      = TEST_SPI_READ_LC,
    .wtLatencyCode      = 0,                       // Do not care
};


static void TestFor_MMIO_WRITE_Then_MMIO_READ(cy_en_smif_slave_select_t slave, uint16_t wt_cmdId ,uint16_t rd_cmdId);
static void TestFor_XIP_WRITE_Then_XIP_READ(cy_en_smif_slave_select_t slave);

static void SectorEraseProcedure(cy_en_smif_slave_select_t slave, uint32_t address);
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave);
static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode);

cy_stc_ex_dev_context_t smifDevice0Context;
cy_stc_ex_dev_context_t smifDevice1Context;


int main(void)
{
    SystemInit();

    __enable_irq();

    Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
    //Cy_SysClk_HfClockSetDivider(SMIF_HF_CLOCK, CY_SYSCLK_HFCLK_DIVIDE_BY_4);

    // Please modify according to your HW condition.
    ChangePLLFrequency(SMIF_CLK_FREQ_SETUP*2); // SMIF out clock will be 90,000,000

    SmifPortInit(smifPortCfg, SIZE_OF_PORT);

    CY_ASSERT(Cy_SMIF_InitExMemContext(CY_SMIF_S25FL, &smifDevice0Context) == CY_SMIF_SUCCESS);
    CY_ASSERT(Cy_SMIF_InitExMemContext(CY_SMIF_S25FL, &smifDevice1Context) == CY_SMIF_SUCCESS);

    /*************************/
    /* 1. Setting for SMIF 0 */
    /*************************/
    Cy_SMIF_DeInit(CY_SMIF_USED);
    Cy_SMIF_Init(CY_SMIF_USED, &smifConfig, 1000, &smifContext);

    /************************************/
    /* 2. Setting for SMIF 0 Device 0/1 */
    /************************************/
    Cy_SMIF_DeviceSetDataSelect(SMIF0_DEVICE0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_DeviceWriteEnable(SMIF0_DEVICE0);
    Cy_SMIF_DeviceSetDataSelect(SMIF0_DEVICE1, CY_SMIF_DATA_SEL2);
    Cy_SMIF_DeviceWriteEnable(SMIF0_DEVICE1);

    /********************/
    /* 3. Enable SMIF 0 */
    /********************/
    Cy_SMIF_Enable(CY_SMIF_USED, &smifContext);

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
    /***********************************/
    /* 4. Setting for external devices */
    /***********************************/
    // 4.1 Send Rest command
    Cy_SMIF_S25FL_SoftwareReset(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &smifContext);
    Cy_SMIF_S25FL_SoftwareReset(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &smifContext);

    // wait at least 32us
    Cy_SysTick_DelayInUs(32ul);

    /*********************************************************/
    /********* Read Status and Configuration registers *******/
    /*********************************************************/
    // 4.2 Read Configuration register 1 and status register 1
    // Read current configuration register 1 and status register 1
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1 = { .u8 = 0u };

    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg1, &smifContext);
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevStatusReg0, &smifContext);
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevStatusReg1, &smifContext);

    /*********************************************************/
    /******* Modify Status and Configuration registers *******/
    /*********************************************************/
    // Modify QUAD and LC field to be set
    prevConfigReg0.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg0.field.u2LC   = TEST_SPI_READ_LC;
    prevConfigReg1.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1.field.u2LC   = TEST_SPI_READ_LC;
    prevStatusReg0.field.u3BP   = 0u; // no protection
    prevStatusReg1.field.u3BP   = 0u; // no protection

    /*********************************************************/
    /******* Write Status and Configuration registers ********/
    /*********************************************************/
    // Send Write Enable command
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &smifContext);
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &smifContext);

    // Write Registers
    stc_cy_smif_s25f_write_reg_t writeRegValue_0;
    writeRegValue_0.stsReg1 = prevStatusReg0.u8;
    writeRegValue_0.confgReg1 = prevConfigReg0.u8;

    stc_cy_smif_s25f_write_reg_t writeRegValue_1;
    writeRegValue_1.stsReg1 = prevStatusReg1.u8;
    writeRegValue_1.confgReg1 = prevConfigReg1.u8;

    // Write Registers
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &writeRegValue_0, 2, &smifContext);
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &writeRegValue_1, 2, &smifContext);

    // Wait writing registers complete
    CY_ASSERT_L1(WaitDeviceBecomesReady(CY_SMIF_SLAVE_SELECT_0) == 0u);
    CY_ASSERT_L1(WaitDeviceBecomesReady(CY_SMIF_SLAVE_SELECT_1) == 0u);

    /*********************************************************/
    /******* Verify Status and Configuration registers *******/
    /*********************************************************/
    // Check the config register have been writen properly
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg1, &smifContext);
    CY_ASSERT_L1(readConfigReg0.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg0.field.u2LC == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg1.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1.field.u2LC == TEST_SPI_READ_LC);
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
    // 4.1 Send Rest command
    // YOTS: Not implemented yet.

    /*********************************************************/
    /********* Read Status and Configuration registers *******/
    /*********************************************************/
    // 4.2 Read Configulation register 1 and status register 1
    // Read current configuration register 1 and status register 1
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t prevConfigReg2_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t prevConfigReg3_0 = { .u8 = 0u };

    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t prevConfigReg2_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t prevConfigReg3_1 = { .u8 = 0u };

    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevStatusReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg2_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg3_0, &smifContext);

    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevStatusReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg2_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg3_1, &smifContext);

    /*********************************************************/
    /******* Modify Status and Configuration registers *******/
    /*********************************************************/
    prevStatusReg1_0.u8 = 0;
    prevConfigReg1_0.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1_0.field.u1CMP  = 0u;
    prevConfigReg2_0.field.u1QPI  = 0u; // Quad instruction disabled
    prevConfigReg3_0.field.u4RL   = TEST_SPI_READ_LC; // Latency code
    prevConfigReg3_0.field.u1WE   = 1u; // Wrap Disabled

    prevStatusReg1_1.u8 = 0;
    prevConfigReg1_1.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1_1.field.u1CMP  = 0u;
    prevConfigReg2_1.field.u1QPI  = 0u; // Quad instruction disabled
    prevConfigReg3_1.field.u4RL   = TEST_SPI_READ_LC; // Latency code
    prevConfigReg3_1.field.u1WE   = 1u; // Wrap Disabled

    /*********************************************************/
    /******* Write Status and Configuration registers ********/
    /*********************************************************/
    // Send Write Enable command
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &smifContext);
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &smifContext);

    stc_cy_smif_s25f_write_reg_t writeRegValue_0;
    writeRegValue_0.stsReg1 = prevStatusReg1_0.u8;
    writeRegValue_0.confgReg1 = prevConfigReg1_0.u8;
    writeRegValue_0.confgReg2 = prevConfigReg2_0.u8;
    writeRegValue_0.confgReg3 = prevConfigReg3_0.u8;

    stc_cy_smif_s25f_write_reg_t writeRegValue_1;
    writeRegValue_1.stsReg1 = prevStatusReg1_1.u8;
    writeRegValue_1.confgReg1 = prevConfigReg1_1.u8;
    writeRegValue_1.confgReg2 = prevConfigReg2_1.u8;
    writeRegValue_1.confgReg3 = prevConfigReg3_1.u8;

    // Write Registers
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &writeRegValue_0, 4, &smifContext);
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &writeRegValue_1, 4, &smifContext);

    // Wait writing registers complete
    CY_ASSERT_L1(WaitDeviceBecomesReady(CY_SMIF_SLAVE_SELECT_0) == 0u);
    CY_ASSERT_L1(WaitDeviceBecomesReady(CY_SMIF_SLAVE_SELECT_1) == 0u);

    /*********************************************************/
    /******* Verify Status and Configuration registers *******/
    /*********************************************************/
    // Check the config register have been writen properly
    volatile un_cy_smif_s25fl128_SR1_t readStatusReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t readConfigReg2_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t readConfigReg3_0 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readStatusReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg2_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg3_0, &smifContext);

    CY_ASSERT_L1(readStatusReg1_0.u8 == 0u);
    CY_ASSERT_L1(readConfigReg1_0.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1_0.field.u1CMP == 0u);
    CY_ASSERT_L1(readConfigReg2_0.field.u1QPI == 0u);
    CY_ASSERT_L1(readConfigReg3_0.field.u4RL == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg3_0.field.u1WE == 1u);

    volatile un_cy_smif_s25fl128_SR1_t readStatusReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t readConfigReg2_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t readConfigReg3_1 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readStatusReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg2_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg3_1, &smifContext);

    CY_ASSERT_L1(readStatusReg1_1.u8 == 0u);
    CY_ASSERT_L1(readConfigReg1_1.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1_1.field.u1CMP == 0u);
    CY_ASSERT_L1(readConfigReg2_1.field.u1QPI == 0u);
    CY_ASSERT_L1(readConfigReg3_1.field.u4RL == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg3_1.field.u1WE == 1u);
#endif

    // 4.4 Set DLP
    uint8_t dlp = DLP_VALUE;
    Cy_SMIF_S25FL_WriteVDLR(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, dlp, &smifContext);
    Cy_SMIF_S25FL_WriteVDLR(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, dlp, &smifContext);

    smifDevSingle0Cfg.addr = CY_SMIF_GetXIP_Address(CY_SMIF_USED);
    smifDevSingle1Cfg.addr = CY_SMIF_GetXIP_Address(CY_SMIF_USED) + 0x02000000;
    /*********************************************/
    /******** Write/Read Test (Read) *************/
    /*********************************************/
    ChangePLLFrequency(SMIF_CLK_FREQ_SDR_RD*2);

    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_0, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4READ);
    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_1, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4READ);

    smifDevSingle0Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4READ;
    smifDevSingle1Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4READ;
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE0, CY_SMIF_DEV_SINGLE_BIT_0To1, &smifDevSingle0Cfg, &smifDevice0Context);
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE1, CY_SMIF_DEV_SINGLE_BIT_4To5, &smifDevSingle1Cfg, &smifDevice1Context);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_0);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_1);

    /**************************************************/
    /******** Write/Read Test (Fast Read) *************/
    /**************************************************/
    // Please modify according to your HW condition.
    ChangePLLFrequency(SMIF_CLK_FREQ_SDR_FST_RD*2); // SMIF out clock will be 50,000,000

    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_0, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4FAST_READ);
    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_1, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4FAST_READ);

    smifDevSingle0Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4FAST_READ;
    smifDevSingle1Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4FAST_READ;
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE0, CY_SMIF_DEV_SINGLE_BIT_0To1, &smifDevSingle0Cfg, &smifDevice0Context);
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE1, CY_SMIF_DEV_SINGLE_BIT_4To5, &smifDevSingle1Cfg, &smifDevice1Context);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_0);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_1);

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
    /******************************************************/
    /******* Write/Read Test (DDR Dual I/O Read) **********/
    /******************************************************/
    // Please modify according to your HW condition.
    ChangePLLFrequency(SMIF_CLK_FREQ_DDR_RD*2);         // SMIF out clock will be 50,000,000

    smifConfig.rxClk = CY_SMIF_INV_OUTPUT_CLK;        	// in case did not work, try on CY_SMIF_INV_FEEDBACK_CLK or CY_SMIF_INV_OUTPUT_CLK. !!!!
    Cy_SMIF_Init(CY_SMIF_USED, &smifConfig, 1000, &smifContext);

    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_0, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4DDRFR);
    TestFor_MMIO_WRITE_Then_MMIO_READ(CY_SMIF_SLAVE_SELECT_1, CY_SMIF_S25FL_ID_4PP, CY_SMIF_S25FL_ID_4DDRFR);

    smifDevSingle0Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4DDRFR;
    smifDevSingle1Cfg.xipReadCmdId = CY_SMIF_S25FL_ID_4DDRFR;
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE0, CY_SMIF_DEV_SINGLE_BIT_0To1, &smifDevSingle0Cfg, &smifDevice0Context);
    Cy_SMIF_InitXIPModeForExMem(SMIF0_DEVICE1, CY_SMIF_DEV_SINGLE_BIT_4To5, &smifDevSingle1Cfg, &smifDevice1Context);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_0);
    TestFor_XIP_WRITE_Then_XIP_READ(CY_SMIF_SLAVE_SELECT_1);
#endif

    // Reaching here means this example worked properly.
    for(;;);
}

static void TestFor_MMIO_WRITE_Then_MMIO_READ(cy_en_smif_slave_select_t slave, uint16_t wt_cmdId ,uint16_t rd_cmdId)
{
    cy_stc_ex_dev_context_t* smifDeviceContext;
    if(slave == CY_SMIF_SLAVE_SELECT_0)
    {
        smifDeviceContext = &smifDevice0Context;
    }
    else if(slave == CY_SMIF_SLAVE_SELECT_1)
    {
        smifDeviceContext = &smifDevice1Context;
    }
    else
    {
        CY_ASSERT_L1(false);
    }

    SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_NORMAL);
    for(uint32_t i_EraseSectorTopAddr = TEST_PROGRAM_SECTOR; i_EraseSectorTopAddr < (TEST_PROGRAM_SECTOR+TEST_PROGRAM_SIZE); i_EraseSectorTopAddr+= DEVICE_SECTOR_ERASE)
    {
        SectorEraseProcedure(slave, i_EraseSectorTopAddr);
    }

    /* Program 64K byte data */
    volatile uint32_t programAddress    = TEST_PROGRAM_SECTOR;
    volatile uint32_t programSizeInByte = TEST_PROGRAM_SIZE;

    for(uint32_t i_WriteOffsetAddress = 0; i_WriteOffsetAddress < programSizeInByte; i_WriteOffsetAddress+=DEVICE_PROGRAM_BUFFER_SIZE)
    {
        uint32_t writeStartAddress = i_WriteOffsetAddress + programAddress;

        // Write enable
        Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, slave, &smifContext);

        // Issue program
        Cy_SMIF_MMIO_Program_ExMem(CY_SMIF_USED, slave, wt_cmdId, writeStartAddress, DEVICE_PROGRAM_BUFFER_SIZE, (uint8_t*)&programData[i_WriteOffsetAddress], CY_SMIF_BLOCKING, TEST_SPI_WRITE_LC, 0, smifDeviceContext, &smifContext);

        // Wait for completion
        CY_ASSERT_L1(WaitDeviceBecomesReady(slave) == 0u);
    }

    /* Read 64K byte data */
    volatile uint32_t readAddress    = TEST_PROGRAM_SECTOR;
    volatile uint32_t readSizeInByte = TEST_PROGRAM_SIZE;

    // Read the programmed data
    Cy_SMIF_MMIO_Read_ExMem(CY_SMIF_USED, slave, rd_cmdId, readAddress, readSizeInByte, (uint8_t*)readDataBuffer, TEST_SPI_READ_LC, CY_SMIF_S25FL_RW_NON_CONTINUOUS, CY_SMIF_BLOCKING, smifDeviceContext, &smifContext);

    // Check read data
    CY_ASSERT_L1((memcmp(readDataBuffer, programData, TEST_PROGRAM_SIZE) == 0));
}

static void TestFor_XIP_WRITE_Then_XIP_READ(cy_en_smif_slave_select_t slave)
{
    uint32_t TestAddress;
    if(slave == CY_SMIF_SLAVE_SELECT_0)
    {
        TestAddress = CY_SMIF_GetXIP_Address(CY_SMIF_USED);
    }
    else if(slave == CY_SMIF_SLAVE_SELECT_1)
    {
        TestAddress = CY_SMIF_GetXIP_Address(CY_SMIF_USED) + 0x02000000;
    }
    else
    {
        CY_ASSERT_L1(false);
    }

    SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_NORMAL);
    for(uint32_t i_EraseSectorTopAddr = TEST_PROGRAM_SECTOR; i_EraseSectorTopAddr < (TEST_PROGRAM_SECTOR+TEST_PROGRAM_SIZE); i_EraseSectorTopAddr+= DEVICE_SECTOR_ERASE)
    {
        SectorEraseProcedure(slave, i_EraseSectorTopAddr);
    }

    // Issue program
    uint32_t writeBaseAddress = TestAddress+TEST_PROGRAM_SECTOR;
    for(uint32_t i_WriteOffsetAddress = 0; i_WriteOffsetAddress < TEST_PROGRAM_SIZE; i_WriteOffsetAddress += DEVICE_PROGRAM_BUFFER_SIZE)
    {
        // Write enable
        SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_NORMAL);
        Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, slave, &smifContext);
        SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_MEMORY);

        memcpy((void*)(writeBaseAddress + i_WriteOffsetAddress), (void*)&programData[i_WriteOffsetAddress], DEVICE_PROGRAM_BUFFER_SIZE);

        SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_NORMAL);
        CY_ASSERT_L1(WaitDeviceBecomesReady(slave) == 0u);
        SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_MEMORY);
    }

    // Read the programmed data
    memcpy(readDataBuffer, (void*)writeBaseAddress, TEST_PROGRAM_SIZE);

    // Check read data
    CY_ASSERT_L1((memcmp(readDataBuffer, programData, TEST_PROGRAM_SIZE) == 0));
}

static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode)
{
    cy_en_smif_status_t status;
    do
    {
        status = Cy_SMIF_SetMode(base, mode);
    }while(status != CY_SMIF_SUCCESS);
}

// return 1, when one of selected device return error during erasing.
// return 0, when erasing completed without error.
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave)
{
    for(uint8_t i_slaveNum = 0; i_slaveNum < CY_SMIF_GetDeviceNumber(CY_SMIF_USED); i_slaveNum++)
    {
        cy_en_smif_slave_select_t slaveToBeChecked = (cy_en_smif_slave_select_t)((uint8_t)slave & (0x01 << i_slaveNum));
        if((uint8_t)slaveToBeChecked == 0u)
        {
            // not selected slave
            continue;
        }

        // only selected slave will reach hare.
        while(1)
        {
            volatile un_cy_smif_s25fl128_SR1_t sr1 = { .u8 = 0u };
            Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, slaveToBeChecked, (uint8_t*)&sr1, &smifContext);
            if(sr1.field.u1WIP == 0u)
            {
                break;
            }

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
            if((sr1.field.u1E_ERR == 1u) || (sr1.field.u1P_ERR == 1u))
            {
                return 1; // error
            }
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
            volatile un_cy_smif_s25fl128_SR2_t sr2 = { .u8 = 0u };
            Cy_SMIF_S25FL_ReadStatusRegister2(CY_SMIF_USED, slaveToBeChecked, (uint8_t*)&sr2, &smifContext);
            if((sr2.field.u1E_ERR == 1u) || (sr2.field.u1P_ERR == 1u))
            {
                 Cy_SMIF_S25FL_ClearStatusRegister(CY_SMIF_USED,
                                        slaveToBeChecked,
                                        &smifContext);

                return 1; // error
            }
#endif
        }
    }
    return 0;

}

static void SectorEraseProcedure(cy_en_smif_slave_select_t slave, uint32_t address)
{
    /*  Erase sector        */
    // Write enable
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, slave, &smifContext);

    // Sector erase
    Cy_SMIF_S25FL_SectorErase(CY_SMIF_USED, slave, address, &smifContext);

    // Wait for completion
    CY_ASSERT_L1(WaitDeviceBecomesReady(slave) == 0);

}

/* [] END OF FILE */
