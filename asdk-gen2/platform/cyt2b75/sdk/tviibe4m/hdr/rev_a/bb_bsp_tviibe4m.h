/***************************************************************************//**
* \file bb_bsp_tviibe4m.h
*
* \brief
* Project specific header
*
* \note
* It contains references to all generated header files and should 
* not be modified.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef BB_BSP_TVIIBE4M_H
#define BB_BSP_TVIIBE4M_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined (CPU_BOARD_REVA)
    #include "bb_bsp_tviibe4m_reva.h"
#elif defined (CPU_BOARD_REVC)
    #include "bb_bsp_tviibe4m_revc.h"
#else
    #error Undefined Revision, Please select either Rev A or C
#endif  
  
#ifdef __cplusplus
}
#endif

#endif /* BB_BSP_TVIIBE4M_H */

/* [] END OF FILE */
