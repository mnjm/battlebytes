/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
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
#include "tvii_series_smif_ex_adopter.h" 

/************ Modifiable definitions **********/
#define TEST_SECTOR_NO (0)            // modifiable 0 ~ 255
// !!!!!!! Note the sector defined by below will be erased during delay line calibration !!!!!!!!!
#define SECTOR_NO_USED_BY_CALIB (255) // modifiable 0 ~ 255
/**********************************************/

#define HB_WRITE_BUF_SIZE_WORD          (256)      // 256 word = 512 byte
#define HB_SECTOR_SIZE                  (0x00040000UL) // 256KB
#define TEST_SECTOR_ADDRESS             (HB_SECTOR_SIZE * TEST_SECTOR_NO)
#define SECTOR_ADDR_USED_BY_CALIB       (HB_SECTOR_SIZE * SECTOR_NO_USED_BY_CALIB)

cy_stc_smif_config_t const smifConfig =
{
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_OUTPUT_CLK, //CY_SMIF_OUTPUT_CLK,    //CY_SMIF_INV_RWDS,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL, 
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_2_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_3_CLK_PULUS_MIN,
    .mode          = CY_SMIF_MEMORY, // XIP mode
    .blockEvent    = CY_SMIF_BUS_ERROR,
};
cy_stc_smif_context_t smifContext;

#define HB_FLASH_TARGET_LC  CY_SMIF_HB_LC16     // CY_SMIF_HB_LC10

cy_stc_device_hb_config_t smifDevHBFlashCfg =
{
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = false,                  // will be updated in the application
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 50,
    .addr               = 0,                       // will be updated in the application
    .size               = CY_SMIF_DEVICE_64M_BYTE,
    .lc_hb              = HB_FLASH_TARGET_LC,
    .hbDevType          = CY_SMIF_HB_FLASH,
};

typedef struct
{
    volatile stc_GPIO_PRT_t*  port;
    uint8_t                   pin;
    en_hsiom_sel_t            hsiom;
    uint32_t                  driveMode;
} cy_stc_smif_port_t;

cy_stc_smif_port_t smifPortCfg[] =
{
    {CY_SMIF_CLK_PORT,         CY_SMIF_CLK_PIN,       CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
#if (CY_USE_PSVP == 0)
    {CY_SMIF_CLK_INV_PORT,     CY_SMIF_CLK_INV_PIN,   HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
#endif    
    {CY_SMIF_RWDS_PORT,        CY_SMIF_RWDS_PIN,      CY_SMIF_RWDS_PIN_MUX,    CY_GPIO_DM_PULLDOWN},
    {CY_SMIF_SELECT0_PORT,     CY_SMIF_SELECT0_PIN,   CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_PULLUP},
    {CY_SMIF_SELECT1_PORT,     CY_SMIF_SELECT1_PIN,   CY_SMIF_SELECT1_PIN_MUX, CY_GPIO_DM_PULLUP},
    {CY_SMIF_DATA0_PORT,       CY_SMIF_DATA0_PIN,     CY_SMIF_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA1_PORT,       CY_SMIF_DATA1_PIN,     CY_SMIF_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA2_PORT,       CY_SMIF_DATA2_PIN,     CY_SMIF_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA3_PORT,       CY_SMIF_DATA3_PIN,     CY_SMIF_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA4_PORT,       CY_SMIF_DATA4_PIN,     CY_SMIF_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA5_PORT,       CY_SMIF_DATA5_PIN,     CY_SMIF_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA6_PORT,       CY_SMIF_DATA6_PIN,     CY_SMIF_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA7_PORT,       CY_SMIF_DATA7_PIN,     CY_SMIF_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
};
#define SIZE_OF_PORT (sizeof(smifPortCfg)/sizeof(cy_stc_smif_port_t))

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size)
{
    cy_stc_gpio_pin_config_t pinCfg = {0};
    for(uint32_t i = 0; i < size; i++)
    {
        pinCfg.driveMode = cfg[i].driveMode;
        pinCfg.hsiom     = cfg[i].hsiom;
        Cy_GPIO_SetDriveSelTrim(cfg[i].port, cfg[i].pin, CY_GPIO_DRIVE_STRENGTH_50OHM);
        Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
    }
}

static uint8_t readBuf[TEST_PROGRAM_SIZE];

static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode);

uint16_t HyperFlashReadStatusOpInMMIO(void);
void HyperFlashEraseOpInMMIO(uint32_t address);
void HyperFlashWordProgramOpInMMIO(uint32_t address, uint16_t* writeData, uint16_t halfWordCount);
bool SetSmifFrequencyWithDelayLineTapNumCalibration_HFLASH(uint32_t safeFreq, uint32_t freq, uint32_t usedAddr, cy_en_smif_slave_select_t slave);
void CalibrateDelayLineSelect(uint32_t freq, bool isDDR);


int main(void)
{
    SystemInit();

    __enable_irq();
    
    Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
    //Cy_SysClk_HfClockSetDivider(SMIF_HF_CLOCK, CY_SYSCLK_HFCLK_DIVIDE_BY_2);
    
    ChangePLLFrequency(64000000);            // SMIF out clock will be 32,000,000
    SmifPortInit(smifPortCfg ,SIZE_OF_PORT); 

    /*************************************/
    /*       SMIF Initialization         */
    /*************************************/
    Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);

#if defined (CY_MCU_rev_b)
    // Capture config
    Cy_SMIF_SetCaptureConfig(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, true);
#endif

    //Cy_SMIF_Set_DelayTapSel(SMIF_USED, (CY_SMIF_GetDelayTapsNumber(SMIF_USED) - 1));
    Cy_SMIF_Enable(SMIF_USED, &smifContext);
    Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
    Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH); // this is required when checking status register of external device
    volatile CY_SMIF_FLASHDATA* pHyperFlashBaseAddr = (CY_SMIF_FLASHDATA*)(CY_SMIF_GetXIP_Address(SMIF_USED));

    /*************************************/
    /*     SMIF DEVICE Initialization    */
    /*************************************/
    smifDevHBFlashCfg.addr = (uint32_t)pHyperFlashBaseAddr;
    Cy_SMIF_InitDeviceHyperBus(SMIF_DEVICE0, &smifDevHBFlashCfg);

    /*************************************/
    /*      Hyper Bus Initialization     */
    /*************************************/
    /***** Set Configuration register 1 *****/
    // Load Configuration register 1
    cy_un_h_flash_cfg1_reg_t flashVCR1  = { .u16 = CFG_REG1_DEFAULT_S26K };
    flashVCR1.fld.readLatency           = smifDevHBFlashCfg.lc_hb;
    flashVCR1.fld.driveStrength         = CY_SMIF_HB_S26K_DRV_34_20_OHMS;
    CY_SMIF_HbFlash_LoadVolatileConfigReg(pHyperFlashBaseAddr, flashVCR1.u16);

    // Read Configuration register 1
    uint16_t readConfig1RegVal = CY_SMIF_HbFlash_ReadVolatileConfigReg(pHyperFlashBaseAddr);

    // Verify Configuration register 1
    CY_ASSERT(readConfig1RegVal == flashVCR1.u16);

    /*************************************/
    /*  Calibration for Delay RWDS line  */
    /*************************************/
    
#if (CY_USE_PSVP == 0)
    /*********  Calibrate Delay Line Select   *******/
    // Please modify according to your HW condition.
    uint32_t targetSmifFreqFlash = 100000000;
    uint32_t safeSmifFreqFlash = 32000000;

    CalibrateDelayLineSelect(targetSmifFreqFlash, true);
    SetSmifFrequencyWithDelayLineTapNumCalibration_HFLASH(safeSmifFreqFlash, targetSmifFreqFlash, SECTOR_ADDR_USED_BY_CALIB, CY_SMIF_SLAVE_SELECT_0);
#endif

    /**********************************************/
    /*       Hyper Flash Test In XIP mode         */
    /**********************************************/
    /*********  Erase Top Sector   *******/
    CY_SMIF_HbFlash_SectorEraseOp(pHyperFlashBaseAddr, TEST_SECTOR_ADDRESS);

    /*********  Write Data to the Top Sector   *******/
    for(uint32_t i_WriteAddr = TEST_SECTOR_ADDRESS; i_WriteAddr < (TEST_PROGRAM_SIZE / 2);i_WriteAddr += HB_WRITE_BUF_SIZE_WORD)
    {
        // Write buffer size = 512 byte = 256 word
        CY_SMIF_HbFlash_WriteBufferProgramOp(pHyperFlashBaseAddr, i_WriteAddr, HB_WRITE_BUF_SIZE_WORD, (CY_SMIF_FLASHDATA*)&programData[i_WriteAddr*2]);
    }

    /*********  Marge mode enable   *******/
    // If more than 1 cycle merge time accepted, there will be long CS# low duration when burst reading.
    // It may cause error because Low/High ratio of CLK should be around 50/50 during reading because of Memory device restriction.
    Cy_SMIF_DeviceTransferMergingEnable(SMIF_DEVICE0,CY_SMIF_MER_TIMEOUT_1_CYCLE);
    while(Cy_SMIF_IsBusy(SMIF_USED));

    /*********  SMIF cache enable   *******/
    Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);
    Cy_SMIF_CachePrefetchingEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);
    
    /*********  Read back and verify   *******/
    memcpy(readBuf, (void*)pHyperFlashBaseAddr, TEST_PROGRAM_SIZE);

    // Verify data
    CY_ASSERT(memcmp(readBuf, programData, TEST_PROGRAM_SIZE) == 0); 

    /**********************************************/
    /*       Hyper Flash Test In MMIO mode        */
    /**********************************************/
    /*********  SMIF cache disable   *******/
    Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
    Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);

    SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

    /*********  Erase Top Sector   *******/
    HyperFlashEraseOpInMMIO(TEST_SECTOR_ADDRESS);
    
    /*********  Write Data to the Top Sector   *******/
    for(uint32_t i_WriteAddr = TEST_SECTOR_ADDRESS; i_WriteAddr < (TEST_PROGRAM_SIZE / 2);i_WriteAddr += HB_WRITE_BUF_SIZE_WORD)
    {
        // write buffer size = 512 byte = 256 word
        HyperFlashWordProgramOpInMMIO((TEST_SECTOR_ADDRESS + i_WriteAddr), (uint16_t*)&programData[i_WriteAddr*2], HB_WRITE_BUF_SIZE_WORD);
    }
    
    memset(readBuf, 0, TEST_PROGRAM_SIZE);
    
    /*********  Read back and verify   *******/
    for(uint32_t i_WriteAddr = TEST_SECTOR_ADDRESS; i_WriteAddr < (TEST_PROGRAM_SIZE / 2);i_WriteAddr += 0x100)
    {
        static uint16_t readBuffer[256];
        Cy_SMIF_HB_MMIO_Read(SMIF_USED,
                             CY_SMIF_SLAVE_SELECT_0,
                             CY_SMIF_HB_COUTINUOUS_BURST,
                             (i_WriteAddr+TEST_SECTOR_ADDRESS), // address
                             0x100,                             // size in half word
                             readBuffer,
                             smifDevHBFlashCfg.lc_hb,
                             false,                             // Single initial latency
                             true,                              // Blocking mode
                             &smifContext
                             );
        CY_ASSERT(memcmp(readBuffer, &programData[i_WriteAddr*2], 0x200) == 0);
    }

    for(;;);
}


static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode)
{
    cy_en_smif_status_t status;
    do
    {
        status = Cy_SMIF_SetMode(base, mode);
    }while(status != CY_SMIF_SUCCESS);
}

uint16_t HyperFlashReadStatusOpInMMIO(void)
{
    uint16_t writeBuf   = 0x0070;
    uint16_t readStatus = 0x0000;

    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST,
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    Cy_SMIF_HB_MMIO_Read(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST,
                         0x0000,                   // address
                         1,                        // size in half word
                         &readStatus,
                         smifDevHBFlashCfg.lc_hb,
                         false,                      // Single initial latency
                         true,                       // Blocking mode
                         &smifContext
                         );

    return (readStatus);
}


void HyperFlashEraseOpInMMIO(uint32_t address)
{
    uint16_t writeBuf = 0x0000;

    writeBuf = 0x00AA;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x0055;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x2AA,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x0080;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x00AA;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x0055;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x2AA,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x0030;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         address,                 // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );


    cy_un_h_flash_stat_reg_t flashStatus;
    do
    {
        flashStatus.u16 = HyperFlashReadStatusOpInMMIO();
    } while(flashStatus.fld.DeviceReady == 0); // wait for the device becoming ready

    return;
}


void HyperFlashWordProgramOpInMMIO(uint32_t address, uint16_t* writeData, uint16_t halfWordCount)
{
    uint16_t writeBuf = 0x0000;

    writeBuf = 0x00AA;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x0055;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x2AA,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    writeBuf = 0x00A0;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         0x555,                   // address
                         1,                       // size in half word
                         &writeBuf,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_0,
                         CY_SMIF_HB_COUTINUOUS_BURST, // not cared
                         address,                 // address
                         halfWordCount,                       // size in half word
                         writeData,
                         CY_SMIF_HB_FLASH,
                         CY_SMIF_HB_LC5,          // latency code, do not care
                         true,                    // Blocking mode
                         &smifContext
                         );

    cy_un_h_flash_stat_reg_t flashStatus;
    do
    {
        flashStatus.u16 = HyperFlashReadStatusOpInMMIO();
    } while(flashStatus.fld.DeviceReady == 0); // wait for the device becoming ready

    return;
}

bool SetSmifFrequencyWithDelayLineTapNumCalibration_HFLASH(uint32_t safeFreq, uint32_t freq, uint32_t usedAddr, cy_en_smif_slave_select_t slave)
{
    uint8_t readTest[12]      = {0};
    uint8_t testData[12]      = {0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x55, 0xAA};
    bool    isMatch[32]       = {false};

    cy_en_smif_mode_t savedMode = Cy_SMIF_GetMode(SMIF_USED);

    SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

    ChangePLLFrequency(safeFreq*2);

    HyperFlashEraseOpInMMIO(usedAddr);
    HyperFlashWordProgramOpInMMIO(usedAddr, (uint16_t*)testData/*Data*/, 0x6);

    ChangePLLFrequency(freq*2);

    for(uint8_t i_tapNum = 0; i_tapNum < CY_SMIF_GetDelayTapsNumber(SMIF_USED); i_tapNum++)
    {
        while(Cy_SMIF_IsBusy(SMIF_USED));
        Cy_SMIF_Set_DelayTapSel(SMIF_USED, i_tapNum);
        Cy_SMIF_HB_MMIO_Read(SMIF_USED,
                         slave,
                         CY_SMIF_HB_COUTINUOUS_BURST,
                         usedAddr,                 // address
                         0x6,                      // size in half word
                         (uint16_t*)readTest,
                         smifDevHBFlashCfg.lc_hb,
                         false,                      // Single initial latency
                         true,                       // Blocking mode
                         &smifContext
                         );
        if(memcmp(&readTest, testData, 12) == 0)
        {
            isMatch[i_tapNum] = true;
        }
        else
        {
            isMatch[i_tapNum] = false;
        }
    }

    uint8_t bestTapNum = 0xFF;
    uint8_t consectiveMatchNum    = 0;
    uint8_t maxConsectiveMatchNum = 0;
    for(uint8_t i_tapNum = 0; i_tapNum < CY_SMIF_GetDelayTapsNumber(SMIF_USED); i_tapNum++)
    {
        if(isMatch[i_tapNum] == true)
        {
            consectiveMatchNum += 1;
        }
        else
        {
            if(maxConsectiveMatchNum < consectiveMatchNum)
            {
                maxConsectiveMatchNum = consectiveMatchNum;
                bestTapNum = i_tapNum - ((maxConsectiveMatchNum + 1) / 2);
                consectiveMatchNum = 0;
            }
        }
    }

    if(maxConsectiveMatchNum < consectiveMatchNum)
    {
        bestTapNum = CY_SMIF_GetDelayTapsNumber(SMIF_USED) - ((consectiveMatchNum + 1) / 2);
    }

    if(bestTapNum != 0xFF)
    {
        Cy_SMIF_Set_DelayTapSel(SMIF_USED, bestTapNum);
    }
    else
    {
        CY_ASSERT(false); // No right tap number
    }

    HyperFlashEraseOpInMMIO(usedAddr);

    SetModeWithBusyCheck(SMIF_USED, savedMode);

    return true;
}


void CalibrateDelayLineSelect(uint32_t freq, bool isDDR)
{
    static const float minDelayPerTapInNs[4] = {0.0559, 0.1197, 0.1935, 0.3973};

    /*********  Calibrate Delay Line Select   *******/
    // disable delay line
    uint32_t tapLineChecked = 0;
    for(; tapLineChecked < 4; tapLineChecked++)
    {
        float maxTapNum;
        if(isDDR == true)
        {
            maxTapNum = (uint32_t)((1000000000.0 / (freq * 4 * minDelayPerTapInNs[tapLineChecked])) + 0.5);
        }
        else
        {
            maxTapNum = (uint32_t)((1000000000.0 / (freq * 2 * minDelayPerTapInNs[tapLineChecked])) + 0.5);
        }

        if(maxTapNum <= CY_SMIF_GetDelayTapsNumber(SMIF_USED))
        {
            break;
        }
    }

    if(tapLineChecked == 4)
    {
        tapLineChecked = 3;
    }

    while(Cy_SMIF_IsBusy(SMIF_USED));
    Cy_SMIF_Disable(SMIF_USED);
    Cy_SMIF_Set_DelayLineSel(SMIF_USED, (cy_en_smif_delay_line_t)tapLineChecked);
    Cy_SMIF_Enable(SMIF_USED, &smifContext);
}

/* [] END OF FILE */

