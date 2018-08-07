/*******************************************************************************
* File Name: DB2.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_DB2_H) /* Pins DB2_H */
#define CY_PINS_DB2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DB2_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DB2__PORT == 15 && ((DB2__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DB2_Write(uint8 value);
void    DB2_SetDriveMode(uint8 mode);
uint8   DB2_ReadDataReg(void);
uint8   DB2_Read(void);
void    DB2_SetInterruptMode(uint16 position, uint16 mode);
uint8   DB2_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DB2_SetDriveMode() function.
     *  @{
     */
        #define DB2_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DB2_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DB2_DM_RES_UP          PIN_DM_RES_UP
        #define DB2_DM_RES_DWN         PIN_DM_RES_DWN
        #define DB2_DM_OD_LO           PIN_DM_OD_LO
        #define DB2_DM_OD_HI           PIN_DM_OD_HI
        #define DB2_DM_STRONG          PIN_DM_STRONG
        #define DB2_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DB2_MASK               DB2__MASK
#define DB2_SHIFT              DB2__SHIFT
#define DB2_WIDTH              1u

/* Interrupt constants */
#if defined(DB2__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DB2_SetInterruptMode() function.
     *  @{
     */
        #define DB2_INTR_NONE      (uint16)(0x0000u)
        #define DB2_INTR_RISING    (uint16)(0x0001u)
        #define DB2_INTR_FALLING   (uint16)(0x0002u)
        #define DB2_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DB2_INTR_MASK      (0x01u) 
#endif /* (DB2__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DB2_PS                     (* (reg8 *) DB2__PS)
/* Data Register */
#define DB2_DR                     (* (reg8 *) DB2__DR)
/* Port Number */
#define DB2_PRT_NUM                (* (reg8 *) DB2__PRT) 
/* Connect to Analog Globals */                                                  
#define DB2_AG                     (* (reg8 *) DB2__AG)                       
/* Analog MUX bux enable */
#define DB2_AMUX                   (* (reg8 *) DB2__AMUX) 
/* Bidirectional Enable */                                                        
#define DB2_BIE                    (* (reg8 *) DB2__BIE)
/* Bit-mask for Aliased Register Access */
#define DB2_BIT_MASK               (* (reg8 *) DB2__BIT_MASK)
/* Bypass Enable */
#define DB2_BYP                    (* (reg8 *) DB2__BYP)
/* Port wide control signals */                                                   
#define DB2_CTL                    (* (reg8 *) DB2__CTL)
/* Drive Modes */
#define DB2_DM0                    (* (reg8 *) DB2__DM0) 
#define DB2_DM1                    (* (reg8 *) DB2__DM1)
#define DB2_DM2                    (* (reg8 *) DB2__DM2) 
/* Input Buffer Disable Override */
#define DB2_INP_DIS                (* (reg8 *) DB2__INP_DIS)
/* LCD Common or Segment Drive */
#define DB2_LCD_COM_SEG            (* (reg8 *) DB2__LCD_COM_SEG)
/* Enable Segment LCD */
#define DB2_LCD_EN                 (* (reg8 *) DB2__LCD_EN)
/* Slew Rate Control */
#define DB2_SLW                    (* (reg8 *) DB2__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DB2_PRTDSI__CAPS_SEL       (* (reg8 *) DB2__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DB2_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DB2__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DB2_PRTDSI__OE_SEL0        (* (reg8 *) DB2__PRTDSI__OE_SEL0) 
#define DB2_PRTDSI__OE_SEL1        (* (reg8 *) DB2__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DB2_PRTDSI__OUT_SEL0       (* (reg8 *) DB2__PRTDSI__OUT_SEL0) 
#define DB2_PRTDSI__OUT_SEL1       (* (reg8 *) DB2__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DB2_PRTDSI__SYNC_OUT       (* (reg8 *) DB2__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DB2__SIO_CFG)
    #define DB2_SIO_HYST_EN        (* (reg8 *) DB2__SIO_HYST_EN)
    #define DB2_SIO_REG_HIFREQ     (* (reg8 *) DB2__SIO_REG_HIFREQ)
    #define DB2_SIO_CFG            (* (reg8 *) DB2__SIO_CFG)
    #define DB2_SIO_DIFF           (* (reg8 *) DB2__SIO_DIFF)
#endif /* (DB2__SIO_CFG) */

/* Interrupt Registers */
#if defined(DB2__INTSTAT)
    #define DB2_INTSTAT            (* (reg8 *) DB2__INTSTAT)
    #define DB2_SNAP               (* (reg8 *) DB2__SNAP)
    
	#define DB2_0_INTTYPE_REG 		(* (reg8 *) DB2__0__INTTYPE)
#endif /* (DB2__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DB2_H */


/* [] END OF FILE */
