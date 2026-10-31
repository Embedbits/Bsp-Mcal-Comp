/**
 * \author Mr.Nobody
 * \file Comp.h
 * \ingroup Comp
 * \brief Comp module private header file
 *
 * This file contains the functionality private to the module (not shared
 * with the application or other modules through the Port file).
 *
 */

#ifndef COMP_COMP_H
#define COMP_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Comp_Port.h"                      /* Module public interface        */
#include "Comp_Reg.h"                       /* Module register component      */
#include "Dwt_Port.h"                       /* Cycle counter (time base)      */
/* ============================= TYPEDEFS =================================== */

/** \brief Time in microseconds (start-up time) */
typedef uint32_t comp_Time_us_t;

/** \brief Run-time state kept by the module for a single comparator instance */
typedef struct
{
    comp_FunctionState_t Initialized;   /**< COMP_FUNCTION_ACTIVE once \ref Comp_Init succeeded               */
    comp_FunctionState_t Enabled;       /**< The comparator is enabled (EN set)                               */
    comp_FunctionState_t Ready;         /**< The start-up time has elapsed, the output is valid               */
    comp_FunctionState_t LockRequested; /**< The lock is applied together with the enable                     */
    comp_FunctionState_t Locked;        /**< The configuration is locked until the reset                      */
    comp_FunctionState_t IrqRequested;  /**< The application asked for the edge interrupt                     */
    comp_FunctionState_t IrqArmed;      /**< The interrupt source is armed (start-up time elapsed)            */
    comp_InputPlus_t     InputPlus;     /**< Selected non-inverting input                                     */
    comp_InputMinus_t    InputMinus;    /**< Selected inverting input                                         */
    comp_Hysteresis_t    Hysteresis;    /**< Selected hysteresis                                              */
    comp_PowerMode_t     PowerMode;     /**< Selected power mode                                              */
    comp_Polarity_t      Polarity;      /**< Selected polarity                                                */
    comp_Blanking_t      Blanking;      /**< Selected blanking source                                         */
    comp_Edge_t          Edge;          /**< Selected interrupt edge                                          */
    comp_IrqCallback_t * Callback;      /**< Edge callback, NULL for none                                     */
    dwt_CounterValue_t   StartupCycle;  /**< Value of the cycle counter at the enable                         */
    comp_Time_us_t       StartupTimeUs; /**< Start-up time of the power mode selected at the enable           */
}   comp_Runtime_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Start-up time tSTART of the high-speed mode (DS14053 table 76, maximum), in microseconds */
#define COMP_STARTUP_HIGH_SPEED_US          ( 5u )

/** Start-up time tSTART of the medium mode (DS14053 table 76, maximum), in microseconds */
#define COMP_STARTUP_MEDIUM_US              ( 20u )

/** Start-up time tSTART of the ultra-low-power mode (DS14053 table 76, maximum), in microseconds */
#define COMP_STARTUP_ULTRA_LOW_POWER_US     ( 80u )

/** Start-up time tSTART_SCALER of the voltage scaler (DS14053 table 76, maximum), in microseconds. The comparator
 *  whose inverting input is the scaler (SCALEN set: VREFINT and its fractions) is ready after this time at the
 *  earliest, whatever the power mode is */
#define COMP_STARTUP_SCALER_US              ( 250u )

/** Interrupt priority written by \ref Comp_Get_DefaultConfig (lowest priority) */
#define COMP_IRQ_PRIORITY_DEFAULT           ( 15u )

/** Module version, major component */
#define COMP_MODULE_VERSION_MAJOR           ( 1u )
/** Module version, minor component */
#define COMP_MODULE_VERSION_MINOR           ( 0u )
/** Module version, patch component */
#define COMP_MODULE_VERSION_PATCH           ( 0u )

/* ========================= EXPORTED MACROS ================================ */

/** NVIC interrupt of the comparators: shared by COMP1 and COMP2 where both exist */
#if defined(COMP2)
#define COMP_NVIC_IRQ                       ( NVIC_PERIPH_IRQ_COMP )
#else
#define COMP_NVIC_IRQ                       ( NVIC_PERIPH_IRQ_COMP1 )
#endif

/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

void Comp_IsrHandler( void );

#ifdef __cplusplus
}
#endif

#endif /* COMP_COMP_H */
