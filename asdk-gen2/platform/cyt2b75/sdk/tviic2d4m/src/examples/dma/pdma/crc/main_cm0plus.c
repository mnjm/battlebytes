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


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "crc32_emulator.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define BUFFER_SIZE (5ul)
#define DW_CHANNEL  (7)
#define DW_CH_INTR  (cpuss_interrupts_dw1_7_IRQn)

/*****************************************************************************/
/* Parameters depending on TVII MCU series                                   */
/*****************************************************************************/
#define DW_SW_TRIG (TRIG_OUT_MUX_2_PDMA1_TR_IN7)

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
void DW1_Ch_IntHandler(void);

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static cy_stc_pdma_descr_t stcDescr;
const  uint8_t             au8SrcBuffer[] = {0x12u, 0x34u, 0x56u, 0x78u, 0x9au};
static uint32_t crcLfsrResulttDst = 0ul;
const cy_stc_pdma_crc_config_t stcCrcConfig[] =
{
//  { data_reverse, rem_reverse, data_xor,   polynomial,       lfsr32,      rem_xor }
    {          1ul,         1ul,       0u, 0x04c11db7ul, 0xFFFFFFFFul, 0xFFFFFFFFul },
    {          0ul,         1ul,       0u, 0x04c11db7ul, 0xFFFFFFFFul, 0xFFFFFFFFul },
    {          1ul,         0ul,       0u, 0x04c11db7ul, 0xFFFFFFFFul, 0xFFFFFFFFul },
    {          1ul,         1ul,    0x5Au, 0x04c11db7ul, 0xFFFFFFFFul, 0xFFFFFFFFul },
    {          1ul,         1ul,       0u, 0xABCDEF01ul, 0xFFFFFFFFul, 0xFFFFFFFFul },
    {          1ul,         1ul,       0u, 0x04c11db7ul, 0x12345678ul, 0xFFFFFFFFul },
    {          1ul,         1ul,       0u, 0x04c11db7ul, 0xFFFFFFFFul, 0x5A5A1234ul },
};
#define TEST_PATTERN_NUM (sizeof(stcCrcConfig)/sizeof(cy_stc_pdma_crc_config_t))

const   cy_stc_pdma_chnl_config_t chnlConfig =
{
    .PDMA_Descriptor =   &stcDescr,
    .preemptable    =   0ul,
    .priority       =   0ul,
    .enable         =   1ul,  /*enabled after initialization*/
};

static  cy_stc_pdma_descr_config_t stcDmaDescrConfig =
{
    .deact          = 0ul,                              /*Do not wait for trigger de-activation*/
    .intrType       = CY_PDMA_INTR_DESCR_CMPLT,
    .trigoutType    = CY_PDMA_TRIGOUT_DESCR_CMPLT,
    .chStateAtCmplt = CY_PDMA_CH_DISABLED,
    .triginType     = CY_PDMA_TRIGIN_DESCR,
    .dataSize       = CY_PDMA_BYTE,
    .srcTxfrSize    = 0ul,                              /*= dataSize*/
    .destTxfrSize   = 1ul,                              /*= 32bit*/
    .descrType      = CY_PDMA_CRC_TRANSFER,
    .srcAddr        = (void*)au8SrcBuffer,
    .destAddr       = &crcLfsrResulttDst,
    .srcXincr       = 1ul,
    .xCount         = BUFFER_SIZE,
    .descrNext      = &stcDescr
};

static const cy_stc_sysint_irq_t stc_sysint_irq_cfg =
{
    .sysIntSrc = DW_CH_INTR,
    .intIdx    = CPUIntIdx2_IRQn,
    .isEnabled = true,
};

static bool isComplete = false;

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
static uint32_t CRCEmulate32bit(stc_crc32_emu_config_t * crc_params);
int main(void)
{
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */

    Cy_PDMA_Disable(DW1);
    Cy_PDMA_Chnl_DeInit(DW1, DW_CHANNEL);
    Cy_PDMA_Descr_Init(&stcDescr,&stcDmaDescrConfig);
    Cy_PDMA_Chnl_Init(DW1, DW_CHANNEL, &chnlConfig);
    Cy_PDMA_Chnl_SetInterruptMask(DW1, DW_CHANNEL);
    Cy_PDMA_Enable(DW1);

    /* Interrupt Initialization */
    Cy_SysInt_InitIRQ(&stc_sysint_irq_cfg);
    Cy_SysInt_SetSystemIrqVector(stc_sysint_irq_cfg.sysIntSrc, DW1_Ch_IntHandler);
    NVIC_EnableIRQ(stc_sysint_irq_cfg.intIdx);

    for(uint32_t i = 0; i < TEST_PATTERN_NUM; i++)
    {
        Cy_PDMA_CRC_Config( DW1, &stcCrcConfig[i]);

        SetCrcSeed(stcCrcConfig[i].lfsr32);
        stc_crc32_emu_config_t crcParams = { 0ul };
        crcParams.indata       = au8SrcBuffer;
        crcParams.size         = sizeof(au8SrcBuffer);
        crcParams.poly.u32     = stcCrcConfig[i].polynomial;
        crcParams.data_xor     = stcCrcConfig[i].data_xor;
        crcParams.rem_xor.u32  = stcCrcConfig[i].rem_xor;
        crcParams.option.ROBIT = (stcCrcConfig[i].rem_reverse == 0) ? CRC_MSB_FIRST: CRC_LSB_FIRST;
        crcParams.option.RIBIT = (stcCrcConfig[i].data_reverse == 0) ? CRC_MSB_FIRST: CRC_LSB_FIRST;

        for(uint32_t j = 0; j < 5; j++) // try 5 times with same parameter
        {
            /*Trigger DMA by SW */
            isComplete = false;
            Cy_PDMA_Chnl_Enable(DW1, DW_CHANNEL);
            Cy_TrigMux_SwTrigger(DW_SW_TRIG, TRIGGER_TYPE_EDGE, 1ul);

            // wait for DMA completion
            while(isComplete == false);

            /* Check CRC result */
            uint32_t generatedCrcResult = Cy_PDMA_GetCrcRemainderResult(DW1);

            /* Emulate generating CRC */
            uint32_t emulatedCrcResult = CRCEmulate32bit(&crcParams);

            CY_ASSERT(generatedCrcResult == emulatedCrcResult);
        }
    }

    for(;;);
}

void DW1_Ch_IntHandler(void)
{
    uint32_t masked;

    masked = Cy_PDMA_Chnl_GetInterruptStatusMasked(DW1, DW_CHANNEL);
    if ((masked & CY_PDMA_INTRCAUSE_COMPLETION) != 0ul)
    {
        /* Clear Complete DMA interrupt flag */
        Cy_PDMA_Chnl_ClearInterrupt(DW1, DW_CHANNEL);

        /* Mark the transmission as complete */
        isComplete = true;
    }
    else
    {
        CY_ASSERT(false);
    }
}
/* [] END OF FILE */
