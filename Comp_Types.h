/**
 * \author Mr.Nobody
 * \defgroup Comp Comp
 * \brief Comp module
 */

/**
 * \author Mr.Nobody
 * \file Comp_Types.h
 * \ingroup Comp
 * \brief Comp module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file. STM32H5 has COMP1 on
 * STM32H503 and COMP1 with COMP2 on STM32H543, STM32H553 and the STM32H5E / STM32H5F lines;
 * the other lines have no comparator (every function returns an error there).
 *
 * The inputs (comp_InputPlus_t, comp_InputMinus_t) and the output pins (comp_OutPin_t) are
 * generated lists (Comp_Pins.h): an item holds the instance mask, the selection code of the
 * register fields and the pin, so the application names only the input and the module
 * configures the pin.
 *
 */

#ifndef COMP_COMP_TYPES_H
#define COMP_COMP_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Stm32.h"                          /* Device peripheral definitions  */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define COMP_NULL_PTR                       ( ( void* ) 0u )

/** Instance mask of an encoded item: no instance (unused item) */
#define COMP_INST_NONE                      ( 0x0u )

/** Instance mask of an encoded item: COMP1 only */
#define COMP_INST_1                         ( 0x1u )

/** Instance mask of an encoded item: COMP2 only */
#define COMP_INST_2                         ( 0x2u )

/** Instance mask of an encoded item: both instances */
#define COMP_INST_ALL                       ( 0x3u )

/** Instance mask bit offset in an encoded input, output pin or blanking item */
#define COMP_ITEM_INST_BIT_OFFSET           ( 0u )

/** Instance mask field mask of an encoded item (not shifted) */
#define COMP_ITEM_INST_FIELD_MASK           ( 0x3u )

/** Selection code bit offset in an encoded input item */
#define COMP_INPUT_SEL_BIT_OFFSET           ( 2u )

/** Selection code field mask of an encoded input item (not shifted) */
#define COMP_INPUT_SEL_FIELD_MASK           ( 0x3Fu )

/** Port identification bit offset in an encoded input item */
#define COMP_INPUT_PORT_BIT_OFFSET          ( 8u )

/** Pin identification bit offset in an encoded input item */
#define COMP_INPUT_PIN_BIT_OFFSET           ( 13u )

/** Port and pin identification field mask of an encoded item (not shifted) */
#define COMP_ITEM_GPIO_FIELD_MASK           ( 0x1Fu )

/** Port identification bit offset in an encoded output pin item */
#define COMP_OUT_PORT_BIT_OFFSET            ( 2u )

/** Pin identification bit offset in an encoded output pin item */
#define COMP_OUT_PIN_BIT_OFFSET             ( 7u )

/** Alternate function identification bit offset in an encoded output pin item */
#define COMP_OUT_AF_BIT_OFFSET              ( 12u )

/** Code bit offset in an encoded blanking item */
#define COMP_BLANKING_CODE_BIT_OFFSET       ( 2u )

/** Code field mask of an encoded blanking item (not shifted) */
#define COMP_BLANKING_CODE_FIELD_MASK       ( 0xFu )

/* ========================== EXPORTED MACROS =============================== */

/** Encode an input with a pin (instance mask, selection code, port, pin) into one value */
#define COMP_INPUT_IO_ENC( INST, SEL, PORT, PIN ) \
            ( ( (uint32_t)(INST) << COMP_ITEM_INST_BIT_OFFSET ) | \
              ( (uint32_t)(SEL)  << COMP_INPUT_SEL_BIT_OFFSET ) | \
              ( (uint32_t)(PORT) << COMP_INPUT_PORT_BIT_OFFSET ) | \
              ( (uint32_t)(PIN)  << COMP_INPUT_PIN_BIT_OFFSET )   )

/** Encode an input without a pin (internal signal, instance mask, selection code) into one value */
#define COMP_INPUT_INT_ENC( INST, SEL ) \
            COMP_INPUT_IO_ENC( INST, SEL, GPIO_PORT_CNT, GPIO_PIN_ID_CNT )

/** Encode an output pin (instance mask, port, pin, alternate function) into one value */
#define COMP_OUT_ENC( INST, PORT, PIN, AF ) \
            ( ( (uint32_t)(INST) << COMP_ITEM_INST_BIT_OFFSET ) | \
              ( (uint32_t)(PORT) << COMP_OUT_PORT_BIT_OFFSET ) | \
              ( (uint32_t)(PIN)  << COMP_OUT_PIN_BIT_OFFSET ) | \
              ( (uint32_t)(AF)   << COMP_OUT_AF_BIT_OFFSET )   )

/** Value of the unused output pin item */
#define COMP_OUT_PIN_NONE_CODE \
            COMP_OUT_ENC( COMP_INST_NONE, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )

/** Encode a blanking source (instance mask, blanking field code) into one value */
#define COMP_BLANKING_ENC( INST, CODE ) \
            ( ( (uint32_t)(INST) << COMP_ITEM_INST_BIT_OFFSET ) | \
              ( (uint32_t)(CODE) << COMP_BLANKING_CODE_BIT_OFFSET ) )

/** Extract the instance mask from an encoded item */
#define COMP_ITEM_DECODE_INST( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_ITEM_INST_BIT_OFFSET ) & COMP_ITEM_INST_FIELD_MASK )

/** Extract the selection code from an encoded input item */
#define COMP_INPUT_DECODE_SEL( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_INPUT_SEL_BIT_OFFSET ) & COMP_INPUT_SEL_FIELD_MASK )

/** Extract the port identification from an encoded input item */
#define COMP_INPUT_DECODE_PORT( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_INPUT_PORT_BIT_OFFSET ) & COMP_ITEM_GPIO_FIELD_MASK )

/** Extract the pin identification from an encoded input item */
#define COMP_INPUT_DECODE_PIN( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_INPUT_PIN_BIT_OFFSET ) & COMP_ITEM_GPIO_FIELD_MASK )

/** Extract the port identification from an encoded output pin item */
#define COMP_OUT_DECODE_PORT( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_OUT_PORT_BIT_OFFSET ) & COMP_ITEM_GPIO_FIELD_MASK )

/** Extract the pin identification from an encoded output pin item */
#define COMP_OUT_DECODE_PIN( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_OUT_PIN_BIT_OFFSET ) & COMP_ITEM_GPIO_FIELD_MASK )

/** Extract the alternate function identification from an encoded output pin item */
#define COMP_OUT_DECODE_AF( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_OUT_AF_BIT_OFFSET ) & COMP_ITEM_GPIO_FIELD_MASK )

/** Extract the blanking field code from an encoded blanking item */
#define COMP_BLANKING_DECODE_CODE( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> COMP_BLANKING_CODE_BIT_OFFSET ) & COMP_BLANKING_CODE_FIELD_MASK )

/* ============================== TYPEDEFS ================================== */

/** Major version number of the module */
typedef uint8_t comp_MajorVersion_t;

/** Minor version number of the module */
typedef uint8_t comp_MinorVersion_t;

/** Patch version number of the module */
typedef uint8_t comp_PatchVersion_t;

/** Module version */
typedef struct
{
    comp_MajorVersion_t Major; /**< Major version number */
    comp_MinorVersion_t Minor; /**< Minor version number */
    comp_PatchVersion_t Patch; /**< Patch version number */
}   comp_ModuleVersion_t;

/** Result of a module request */
typedef enum
{
    COMP_REQUEST_OK = 0u, /**< The request succeeded                                           */
    COMP_REQUEST_ERROR    /**< The request failed (bad parameter, bad state, hardware timeout) */
}   comp_RequestState_t;

/** Generic active / inactive function state of the module */
typedef enum
{
    COMP_FUNCTION_INACTIVE = 0u, /**< The function is inactive */
    COMP_FUNCTION_ACTIVE         /**< The function is active   */
}   comp_FunctionState_t;

/** Generic active / inactive flag state of the module */
typedef enum
{
    COMP_FLAG_INACTIVE = 0u, /**< The flag is not set */
    COMP_FLAG_ACTIVE         /**< The flag is set     */
}   comp_FlagState_t;

/** List of the COMP instances of the device line */
typedef enum
{
#if defined(COMP1)
    COMP_PERIPH_1 = 0u, /**< COMP1 instance                              */
#endif
#if defined(COMP2)
    COMP_PERIPH_2,      /**< COMP2 instance                              */
#endif
    COMP_PERIPH_CNT     /**< Number of COMP instances of the device line */
}   comp_PeriphId_t;

/** State of a comparator instance */
typedef enum
{
    COMP_STATE_DISABLED = 0u, /**< The comparator is disabled                                      */
    COMP_STATE_STARTING,      /**< The comparator is enabled, the start-up time has not elapsed    */
    COMP_STATE_READY,         /**< The comparator is enabled, the output is valid                  */
    COMP_STATE_LOCKED,        /**< The comparator is enabled and its configuration is locked       */
    COMP_STATE_CNT            /**< Number of comparator state items                                */
}   comp_State_t;

/** Level of the comparator output (after the polarity and the blanking) */
typedef enum
{
    COMP_LEVEL_LOW = 0u, /**< The output is low (non-inverting input below the inverting input)  */
    COMP_LEVEL_HIGH,     /**< The output is high (non-inverting input above the inverting input) */
    COMP_LEVEL_INVALID   /**< The output is not valid (comparator disabled or in the start-up)   */
}   comp_Level_t;

/** Hysteresis levels of the comparator (the values in mV are in the README of the family) */
typedef enum
{
    COMP_HYSTERESIS_NONE = 0u, /**< No hysteresis     */
    COMP_HYSTERESIS_LOW,       /**< Low hysteresis    */
    COMP_HYSTERESIS_MEDIUM,    /**< Medium hysteresis */
    COMP_HYSTERESIS_HIGH,      /**< High hysteresis   */
    COMP_HYSTERESIS_CNT        /**< Number of hysteresis items */
}   comp_Hysteresis_t;

/** Speed / consumption modes of the comparator (STM32H5 has no LOW_POWER mode, the item is rejected) */
typedef enum
{
    COMP_POWER_HIGH_SPEED = 0u, /**< High speed, high consumption                      */
    COMP_POWER_MEDIUM,          /**< Medium speed, medium consumption                  */
    COMP_POWER_LOW_POWER,       /**< Low power (not available on STM32H5)              */
    COMP_POWER_ULTRA_LOW_POWER, /**< Ultra-low power, lowest speed                     */
    COMP_POWER_CNT              /**< Number of power mode items                        */
}   comp_PowerMode_t;

/** Polarity of the comparator output */
typedef enum
{
    COMP_POLARITY_NORMAL = 0u, /**< The output is high when the non-inverting input is above the inverting one */
    COMP_POLARITY_INVERTED,    /**< The output is inverted                                                     */
    COMP_POLARITY_CNT          /**< Number of polarity items                                                   */
}   comp_Polarity_t;

/** Blanking sources of the comparator output (the signal of a timer that hides the switching spikes) */
typedef enum
{
    COMP_BLANKING_NONE       = COMP_BLANKING_ENC( COMP_INST_ALL, 0x0u ), /**< No blanking                    */
    COMP_BLANKING_TIM2_OC3   = COMP_BLANKING_ENC( COMP_INST_ALL, 0x2u ), /**< TIM2 output compare 3          */
    COMP_BLANKING_TIM3_OC3   = COMP_BLANKING_ENC( COMP_INST_ALL, 0x3u ), /**< TIM3 output compare 3          */
    COMP_BLANKING_TIM3_OC4   = COMP_BLANKING_ENC( COMP_INST_ALL, 0x4u ), /**< TIM3 output compare 4          */
#if defined(COMP1)
    COMP_BLANKING_TIM1_OC5   = COMP_BLANKING_ENC( COMP_INST_1  , 0x1u ), /**< TIM1 output compare 5 (COMP1)  */
    COMP_BLANKING_LPTIM1_CH2 = COMP_BLANKING_ENC( COMP_INST_1  , 0x5u ), /**< LPTIM1 channel 2 (COMP1)       */
    COMP_BLANKING_LPTIM2_CH2 = COMP_BLANKING_ENC( COMP_INST_1  , 0x6u ), /**< LPTIM2 channel 2 (COMP1)       */
#endif
#if defined(COMP2)
    COMP_BLANKING_TIM1_OC6   = COMP_BLANKING_ENC( COMP_INST_2  , 0x1u ), /**< TIM1 output compare 6 (COMP2)  */
    COMP_BLANKING_TIM8_OC5   = COMP_BLANKING_ENC( COMP_INST_2  , 0x5u ), /**< TIM8 output compare 5 (COMP2)  */
    COMP_BLANKING_TIM15_OC2  = COMP_BLANKING_ENC( COMP_INST_2  , 0x6u ), /**< TIM15 output compare 2 (COMP2) */
#endif
    COMP_BLANKING_UNUSED     = COMP_BLANKING_ENC( COMP_INST_NONE, 0xFu )  /**< No blanking source assigned    */
}   comp_Blanking_t;

/** Window mode of the comparator pair (COMP1 with COMP2) */
typedef enum
{
    COMP_WINDOW_MODE_INACTIVE = 0u, /**< The comparators work independently                              */
    COMP_WINDOW_MODE_ACTIVE,        /**< COMP2 takes the non-inverting input of COMP1, COMP1 shows the
                                         window output (XOR of both comparator outputs)                  */
    COMP_WINDOW_MODE_CNT            /**< Number of window mode items                                     */
}   comp_WindowMode_t;

/** Edge of the comparator output that raises the interrupt */
typedef enum
{
    COMP_EDGE_RISING = 0u, /**< Rising edge of the output  */
    COMP_EDGE_FALLING,     /**< Falling edge of the output */
    COMP_EDGE_BOTH,        /**< Both edges of the output   */
    COMP_EDGE_CNT          /**< Number of edge items       */
}   comp_Edge_t;

/** Interrupt priority type of the comparator interrupt (forwarded to Nvic) */
typedef nvic_IrqPrio_t comp_IrqPrio_t;

/* ========================= INPUT AND OUTPUT LISTS ========================== */
/* Comp_Pins.h uses COMP_INST_* and the encoding macros defined above and its items
 * (comp_InputPlus_t, comp_InputMinus_t, comp_OutPin_t) are used by comp_Config_t below. */
#include "Comp_Pins.h"                      /* Generated input / output lists */

/** Callback called from the interrupt on the selected edge of the output */
typedef void ( comp_IrqCallback_t )( comp_PeriphId_t periphId, comp_Level_t outputLevel );

/** Configuration structure of one comparator instance, see Comp_Init / Comp_Get_DefaultConfig */
typedef struct
{
    comp_PeriphId_t      PeriphId;    /**< Instance to configure                                          */
    comp_InputPlus_t     InputPlus;   /**< Non-inverting input                                            */
    comp_InputMinus_t    InputMinus;  /**< Inverting input                                                */
    comp_Hysteresis_t    Hysteresis;  /**< Hysteresis level                                               */
    comp_PowerMode_t     PowerMode;   /**< Speed / consumption mode                                       */
    comp_Polarity_t      Polarity;    /**< Polarity of the output                                         */
    comp_Blanking_t      Blanking;    /**< Blanking source of the output                                  */
    comp_WindowMode_t    WindowMode;  /**< Window mode (COMP_PERIPH_1 only, the pair must be initialised)  */
    comp_OutPin_t        OutputPin;   /**< Output pin, COMP_OUT_PIN_UNUSED when the output stays internal */
    comp_Edge_t          Edge;        /**< Edge of the output that raises the interrupt                   */
    comp_IrqCallback_t * Callback;    /**< Edge callback, NULL for none                                   */
    comp_IrqPrio_t       IrqPriority; /**< NVIC priority of the (shared) comparator interrupt             */
    comp_FunctionState_t Lock;        /**< COMP_FUNCTION_ACTIVE locks the configuration at the enable     */
}   comp_Config_t;

#endif /* COMP_COMP_TYPES_H */
