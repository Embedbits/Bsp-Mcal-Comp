/**
 * \author Mr.Nobody
 * \file Comp_Reg.h
 * \ingroup Comp
 * \brief Comp module Reg component header file
 *
 * This component contains the functionality for register operations. It is
 * private to the module (the register access is not shared with other
 * modules). The COMP instances are multi-instance: every accessor takes the
 * instance identification and addresses the register of that one instance
 * only. STM32H503 has one register block with the status register, the
 * interrupt clear register and two configuration registers, the other lines
 * share the status and the interrupt clear register of both comparators and
 * have one configuration register per comparator.
 *
 */

#ifndef COMP_REG_COMP_REG_H
#define COMP_REG_COMP_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Comp_Types.h"                     /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief Raw value or bit mask of a COMP register or of a selection code */
typedef uint32_t comp_RegValue_t;

/** \brief List of the COMP registers used by the module (one set per instance) */
typedef enum
{
    COMP_REG_SR = 0u, /**< Comparator status register                          */
    COMP_REG_ICFR,    /**< Comparator interrupt clear flag register            */
    COMP_REG_CFGR,    /**< Comparator configuration register (CFGR1)           */
#if defined(STM32H503xx)
    COMP_REG_CFGR2,   /**< Comparator configuration register 2 (INPSEL0, LOCK) */
#endif
    COMP_REG_CNT      /**< Count of available registers                        */
}   comp_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Selection code bit of the inverting input: SCALEN (voltage scaler enable) */
#define COMP_SEL_MINUS_SCALEN               ( 0x10u )

/** Selection code bit of the inverting input: BRGEN (scaler bridge enable) */
#define COMP_SEL_MINUS_BRGEN                ( 0x20u )

/** Selection code field mask of the inverting input: INMSEL[3:0] (not shifted) */
#define COMP_SEL_MINUS_INMSEL_MASK          ( 0xFu )

/** Selection code bit of the non-inverting input: INPSEL0 */
#define COMP_SEL_PLUS_INPSEL0               ( 0x1u )

/** Selection code bit of the non-inverting input: INPSEL1 */
#define COMP_SEL_PLUS_INPSEL1               ( 0x2u )

/** Selection code bit of the non-inverting input: INPSEL2 (STM32H503 only) */
#define COMP_SEL_PLUS_INPSEL2               ( 0x4u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void            Comp_Set_RegBit   ( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask );
void            Comp_Reset_RegBit ( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask );
comp_RegValue_t Comp_Get_RegBit   ( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask );

void            Comp_Set_RegVal   ( comp_PeriphId_t periphId,
                                    comp_RegId_t regId,
                                    comp_RegValue_t regMask,
                                    comp_RegValue_t regValue );
comp_RegValue_t Comp_Get_RegVal   ( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t regMask );

/* ----------------------------- Register protocol -------------------------- */

comp_RequestState_t Comp_Reg_Wait_RegVal       ( comp_PeriphId_t periphId,
                                                 comp_RegId_t regId,
                                                 comp_RegValue_t regMask,
                                                 comp_RegValue_t regValue );

/* ------------------------------ Input selection --------------------------- */

void                Comp_Reg_Set_InputPlusSel  ( comp_PeriphId_t periphId, comp_RegValue_t selCode );
comp_RegValue_t     Comp_Reg_Get_InputPlusSel  ( comp_PeriphId_t periphId );

void                Comp_Reg_Set_InputMinusSel ( comp_PeriphId_t periphId, comp_RegValue_t selCode );
comp_RegValue_t     Comp_Reg_Get_InputMinusSel ( comp_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* COMP_REG_COMP_REG_H */
