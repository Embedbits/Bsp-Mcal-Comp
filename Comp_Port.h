/**
 * \author Mr.Nobody
 * \file Comp_Port.h
 * \ingroup Comp
 * \brief Comp module public interface header file
 *
 * This file contains the public interface of the module, identical for all
 * supported families. Include this file to use the module from an application
 * or from another module. Every function is documented at its definition in
 * Comp.c.
 *
 */

#ifndef COMP_COMP_PORT_H
#define COMP_COMP_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Comp_Types.h"                     /* Module types definition        */
/* ============================= TYPEDEFS =================================== */
/* ========================= SYMBOLIC CONSTANTS ============================= */
/* ========================= EXPORTED MACROS ================================ */
/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

/* --------------------------- Module management ----------------------------- */

comp_ModuleVersion_t Comp_Get_ModuleVersion  ( void );

comp_RequestState_t  Comp_Init               ( const comp_Config_t * const compConfig );
comp_RequestState_t  Comp_Deinit             ( comp_PeriphId_t periphId );
void                 Comp_Task               ( void );
comp_RequestState_t  Comp_Get_DefaultConfig  ( comp_Config_t * const compConfig );

/* -------------------------------- Control ---------------------------------- */

comp_RequestState_t  Comp_Set_PeriphActive   ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_PeriphInactive ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Get_PeriphState    ( comp_PeriphId_t periphId, comp_State_t * const compState );

/* ---------------------------- Inputs and settings --------------------------- */

comp_RequestState_t  Comp_Set_InputPlus      ( comp_PeriphId_t periphId, comp_InputPlus_t inputPlus );
comp_RequestState_t  Comp_Get_InputPlus      ( comp_PeriphId_t periphId, comp_InputPlus_t * const inputPlus );

comp_RequestState_t  Comp_Set_InputMinus     ( comp_PeriphId_t periphId, comp_InputMinus_t inputMinus );
comp_RequestState_t  Comp_Get_InputMinus     ( comp_PeriphId_t periphId, comp_InputMinus_t * const inputMinus );

comp_RequestState_t  Comp_Set_Hysteresis     ( comp_PeriphId_t periphId, comp_Hysteresis_t hysteresis );
comp_RequestState_t  Comp_Get_Hysteresis     ( comp_PeriphId_t periphId, comp_Hysteresis_t * const hysteresis );

comp_RequestState_t  Comp_Set_PowerMode      ( comp_PeriphId_t periphId, comp_PowerMode_t powerMode );
comp_RequestState_t  Comp_Get_PowerMode      ( comp_PeriphId_t periphId, comp_PowerMode_t * const powerMode );

comp_RequestState_t  Comp_Set_Polarity       ( comp_PeriphId_t periphId, comp_Polarity_t polarity );
comp_RequestState_t  Comp_Get_Polarity       ( comp_PeriphId_t periphId, comp_Polarity_t * const polarity );

comp_RequestState_t  Comp_Set_Blanking       ( comp_PeriphId_t periphId, comp_Blanking_t blanking );
comp_RequestState_t  Comp_Get_Blanking       ( comp_PeriphId_t periphId, comp_Blanking_t * const blanking );

comp_RequestState_t  Comp_Set_WindowMode     ( comp_PeriphId_t periphId, comp_WindowMode_t windowMode );
comp_RequestState_t  Comp_Get_WindowMode     ( comp_PeriphId_t periphId, comp_WindowMode_t * const windowMode );

/* --------------------------------- Output ---------------------------------- */

comp_RequestState_t  Comp_Get_OutputLevel    ( comp_PeriphId_t periphId, comp_Level_t * const outputLevel );

/* -------------------------------- Interrupt -------------------------------- */

comp_RequestState_t  Comp_Set_TriggerEdge    ( comp_PeriphId_t periphId, comp_Edge_t edge );
comp_RequestState_t  Comp_Get_TriggerEdge    ( comp_PeriphId_t periphId, comp_Edge_t * const edge );

comp_RequestState_t  Comp_Set_IrqActive      ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_IrqInactive    ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_IrqCallback    ( comp_PeriphId_t periphId, comp_IrqCallback_t * const callback );

comp_RequestState_t  Comp_Set_IrqPriority    ( comp_PeriphId_t periphId, comp_IrqPrio_t irqPriority );
comp_RequestState_t  Comp_Get_IrqPriority    ( comp_PeriphId_t periphId, comp_IrqPrio_t * const irqPriority );

/* ---------------------------------- Lock ----------------------------------- */

comp_RequestState_t  Comp_Set_Lock           ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Get_LockState      ( comp_PeriphId_t periphId, comp_FunctionState_t * const lockState );

/* ---------------------------------- Pins ----------------------------------- */

comp_RequestState_t  Comp_InitInputPlusGpio  ( comp_InputPlus_t inputPlus );
comp_RequestState_t  Comp_InitInputMinusGpio ( comp_InputMinus_t inputMinus );
comp_RequestState_t  Comp_InitOutputGpio     ( comp_OutPin_t outputPin );

#ifdef __cplusplus
}
#endif

#endif /* COMP_COMP_PORT_H */
