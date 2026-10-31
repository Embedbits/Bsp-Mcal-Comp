/**
 * \author Mr.Nobody
 * \file Comp.c
 * \ingroup Comp
 * \brief Comp module main functionality (STM32H5)
 *
 * Implements \ref Comp_Port.h against the COMP registers (via \ref Comp_Reg.h), the
 * peripheral clock (Rcc module), the interrupt vector and its priority (Nvic module), the
 * wake-up line of the comparator output (Exti module) and the pins (Gpio module).
 * STM32H503 has COMP1, STM32H543, STM32H553 and the STM32H5E / STM32H5F lines have COMP1 and
 * COMP2, the other lines have no comparator: every function checks the instance macro
 * (defined(COMP1)) and returns COMP_REQUEST_ERROR there (Comp_Get_ModuleVersion returns the
 * version, Comp_Task does nothing).
 *
 * Interrupt. The edge of the output is detected by the EXTI line of the comparator (the lines
 * of COMP1 / COMP2 are configurable lines on STM32H543, STM32H553 and the STM32H5E / STM32H5F
 * lines: rising, falling or both edges, pending flag in the EXTI) and reaches the CPU through the
 * comparator vector of the Nvic module (one vector for COMP1 and COMP2 where both exist). The
 * comparator interrupt enable bit (ITEN, level interrupt of the NVIC route) is not used there.
 * The EXTI line of STM32H503 is a direct line without edge selection: the interrupt is the
 * comparator interrupt (ITEN, the flag C1IF is set by the rising edge of the output) and only
 * the rising edge is available (Comp_Set_TriggerEdge rejects the other edges).
 *
 * Start-up. The comparator has no ready flag. The start-up time tSTART of the datasheet depends on
 * the power mode (maximum 5 us in the high-speed, 20 us in the medium and 80 us in the
 * ultra-low-power mode) and is measured with the cycle counter of the Dwt module: Comp_Set_PeriphActive
 * stores the counter value, Comp_Task compares the elapsed time (Dwt_Get_ElapsedTime_us) with the
 * start-up time of the mode, so the length of the start-up does not depend on how often Comp_Task is
 * called. Comp_Init returns COMP_REQUEST_ERROR when the cycle counter does not run (Dwt_Get_State).
 * Until the start-up time has elapsed the state is COMP_STATE_STARTING, the level is
 * COMP_LEVEL_INVALID and the edge interrupt stays disarmed. When the cycle counter is stopped after
 * the initialization the start-up does not end (it is never cut short).
 *
 */
/* ============================= INCLUDES =================================== */
#include "Comp.h"                           /* Module private declarations    */
#include "Comp_Reg.h"                       /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
#include "Rcc_Port.h"                       /* RCC Port module definitions    */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Exti_Port.h"                      /* EXTI Port module definitions   */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */
#include "Dwt_Port.h"                       /* DWT Port module definitions    */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Value of a lookup table item that is not available on the device (rejected) */
#define COMP_RAW_INVALID                    ( 0xFFFFFFFFu )

/** \brief Register value without any bit set */
#define COMP_BITS_CLEAR                     ( 0u )

/** \brief Count of initialized instances that shares the clock and the interrupt with nobody */
#define COMP_NO_INSTANCE                    ( 0u )

#if defined(STM32H503xx)
/** \brief LOCK bit of the second configuration register (bit 31, RM0492 22.7.4; CMSIS defines COMP_CFGR1_LOCK only) */
#define COMP_CFGR2_LOCK_MASK                ( COMP_CFGR1_LOCK )
#endif

#if defined(COMP2)
/** \brief Window mode bits of the configuration register */
#define COMP_CFGR_WINDOW_MASK               ( COMP_CFGR1_WINMODE | COMP_CFGR1_WINOUT )
#else
/** \brief No window mode bits on a line with one comparator */
#define COMP_CFGR_WINDOW_MASK               ( 0u )
#endif

/** \brief Bits of the configuration register that Comp_Init writes (the lock and the inputs are not part of it) */
#define COMP_CFGR_CONFIG_MASK               ( COMP_CFGR1_BLANKING | \
                                              COMP_CFGR1_HYST     | \
                                              COMP_CFGR1_PWRMODE  | \
                                              COMP_CFGR1_POLARITY | \
                                              COMP_CFGR1_ITEN     | \
                                              COMP_CFGR1_EN       | \
                                              COMP_CFGR_WINDOW_MASK )

/** \brief Bits of the configuration register that the inputs and the scaler use */
#define COMP_CFGR_INPUT_MASK                ( COMP_CFGR1_INMSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN )

/* ============================== TYPEDEFS ================================== */
/* =========================== LOCAL VARIABLES ============================== */

#if defined(COMP1)

/** \brief Run-time state of every comparator instance */
static comp_Runtime_t comp_Runtime[ COMP_PERIPH_CNT ];

/** \brief COMP_FUNCTION_ACTIVE when the module enabled the peripheral clock (and has to disable it) */
static comp_FunctionState_t comp_ClockOwned = COMP_FUNCTION_INACTIVE;

/** \brief State of the window mode of the pair */
static comp_WindowMode_t comp_WindowMode = COMP_WINDOW_MODE_INACTIVE;

/** \brief Output value flag of every instance in the status register */
static const comp_RegValue_t comp_SrValMask[ COMP_PERIPH_CNT ] =
{
    COMP_SR_C1VAL,   /**< COMP1 output status bit */
#if defined(COMP2)
    COMP_SR_C2VAL,   /**< COMP2 output status bit */
#endif
};

/** \brief Interrupt flag clear bit of every instance in the interrupt clear register */
static const comp_RegValue_t comp_IcfrClearMask[ COMP_PERIPH_CNT ] =
{
    COMP_ICFR_CC1IF, /**< COMP1 interrupt flag clear bit */
#if defined(COMP2)
    COMP_ICFR_CC2IF, /**< COMP2 interrupt flag clear bit */
#endif
};

#if defined(STM32H503xx)
/** \brief Interrupt flag of every instance in the status register (direct EXTI line: no pending flag in the EXTI) */
static const comp_RegValue_t comp_SrFlagMask[ COMP_PERIPH_CNT ] =
{
    COMP_SR_C1IF,    /**< COMP1 interrupt flag */
};
#endif

/** \brief EXTI line of every instance */
static const exti_LineId_t comp_ExtiLine[ COMP_PERIPH_CNT ] =
{
    EXTI_LINE_COMP1, /**< COMP1 output line */
#if defined(COMP2)
    EXTI_LINE_COMP2, /**< COMP2 output line */
#endif
};

/** \brief Register field value of every hysteresis level */
static const comp_RegValue_t comp_HystReg[ COMP_HYSTERESIS_CNT ] =
{
    COMP_BITS_CLEAR,    /**< COMP_HYSTERESIS_NONE   */
    COMP_CFGR1_HYST_0,  /**< COMP_HYSTERESIS_LOW    */
    COMP_CFGR1_HYST_1,  /**< COMP_HYSTERESIS_MEDIUM */
    COMP_CFGR1_HYST,    /**< COMP_HYSTERESIS_HIGH   */
};

/** \brief Register field value of every power mode (the device has three distinct modes) */
static const comp_RegValue_t comp_PowerReg[ COMP_POWER_CNT ] =
{
    COMP_BITS_CLEAR,       /**< COMP_POWER_HIGH_SPEED      */
    COMP_CFGR1_PWRMODE_0,  /**< COMP_POWER_MEDIUM          */
    COMP_RAW_INVALID,      /**< COMP_POWER_LOW_POWER       */
    COMP_CFGR1_PWRMODE,    /**< COMP_POWER_ULTRA_LOW_POWER */
};

/** \brief Start-up time tSTART of every power mode, in microseconds (maximum of the datasheet) */
static const comp_Time_us_t comp_StartupTime_us[ COMP_POWER_CNT ] =
{
    COMP_STARTUP_HIGH_SPEED_US,      /**< COMP_POWER_HIGH_SPEED      */
    COMP_STARTUP_MEDIUM_US,          /**< COMP_POWER_MEDIUM          */
    COMP_STARTUP_ULTRA_LOW_POWER_US, /**< COMP_POWER_LOW_POWER (rejected by the device, longest time kept) */
    COMP_STARTUP_ULTRA_LOW_POWER_US, /**< COMP_POWER_ULTRA_LOW_POWER */
};

/** \brief Register field value of every polarity */
static const comp_RegValue_t comp_PolarityReg[ COMP_POLARITY_CNT ] =
{
    COMP_BITS_CLEAR,     /**< COMP_POLARITY_NORMAL   */
    COMP_CFGR1_POLARITY, /**< COMP_POLARITY_INVERTED */
};

#if !defined(STM32H503xx)
/** \brief EXTI trigger edge of every comparator edge (configurable EXTI lines only) */
static const exti_TriggerEdge_t comp_EdgeExti[ COMP_EDGE_CNT ] =
{
    EXTI_TRIGGER_EDGE_RAISING, /**< COMP_EDGE_RISING  */
    EXTI_TRIGGER_EDGE_FALLING, /**< COMP_EDGE_FALLING */
    EXTI_TRIGGER_EDGE_BOTH,    /**< COMP_EDGE_BOTH    */
};
#endif

/* ========================= FORWARD DECLARATIONS =========================== */

static comp_Runtime_t *    Comp_Get_Runtime         ( comp_PeriphId_t periphId );
static comp_Runtime_t *    Comp_Get_ConfigRuntime   ( comp_PeriphId_t periphId );
static uint32_t            Comp_Get_InitializedCnt  ( void );
static comp_FlagState_t    Comp_Get_InstanceMatch   ( comp_PeriphId_t periphId, uint32_t instMask );
static comp_RequestState_t Comp_Check_Config        ( const comp_Config_t * const compConfig );
static comp_RequestState_t Comp_Check_Edge          ( comp_Edge_t edge );
static comp_RequestState_t Comp_Check_TimeBase      ( void );
static comp_FunctionState_t Comp_Get_StartupOver    ( const comp_Runtime_t * const runtime );
static comp_RequestState_t Comp_Write_CfgrField     ( comp_PeriphId_t periphId,
                                                      comp_RegValue_t regMask,
                                                      comp_RegValue_t regValue );
static comp_RequestState_t Comp_Write_InputPlus     ( comp_PeriphId_t periphId, comp_InputPlus_t inputPlus );
static comp_RequestState_t Comp_Write_InputMinus    ( comp_PeriphId_t periphId, comp_InputMinus_t inputMinus );
static comp_RequestState_t Comp_Setup_Instance      ( const comp_Config_t * const compConfig );
static void                Comp_Cleanup_Instance    ( comp_PeriphId_t periphId );
static comp_RequestState_t Comp_Init_AnalogPin      ( gpio_PortId_t portId, gpio_PinId_t pinId );
static comp_RequestState_t Comp_Init_AltFuncPin     ( gpio_PortId_t portId,
                                                      gpio_PinId_t pinId,
                                                      gpio_AltFunction_t altFunc );
static comp_RequestState_t Comp_Enable_Shared       ( comp_IrqPrio_t irqPriority );
static void                Comp_Release_Shared      ( void );
static comp_RequestState_t Comp_Arm_Irq             ( comp_PeriphId_t periphId, comp_Edge_t edge );
static void                Comp_Disarm_Irq          ( comp_PeriphId_t periphId );
static comp_Level_t        Comp_Read_Level          ( comp_PeriphId_t periphId );
static void                Comp_Handle_Irq          ( comp_PeriphId_t periphId );
#if defined(COMP2)
static comp_RequestState_t Comp_Write_WindowMode    ( comp_WindowMode_t windowMode );
#endif
#endif /* COMP1 */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns the version of the module.
 *
 * \return Module version (major, minor, patch).
 */
comp_ModuleVersion_t Comp_Get_ModuleVersion( void )
{
    const comp_ModuleVersion_t version =
    {
        .Major = COMP_MODULE_VERSION_MAJOR,
        .Minor = COMP_MODULE_VERSION_MINOR,
        .Patch = COMP_MODULE_VERSION_PATCH,
    };

    return ( version );
}


/**
 * \brief Initializes one comparator instance: the shared clock and interrupt resources, the
 *        configuration register (hysteresis, power mode, polarity, blanking, window mode), the
 *        inputs, the pins and the interrupt priority. The comparator is left disabled, use
 *        \ref Comp_Set_PeriphActive; a requested lock is applied together with the enable.
 *        The result is verified by reading the registers back, on a failure the instance is
 *        deinitialized.
 *
 * \param compConfig [in] : Pointer to the configuration structure.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise.
 */
comp_RequestState_t Comp_Init( const comp_Config_t * const compConfig )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_RequestState_t checkState = Comp_Check_Config( compConfig );
    const comp_RequestState_t timeState  = Comp_Check_TimeBase();

    if( ( COMP_REQUEST_OK == checkState ) &&
        ( COMP_REQUEST_OK == timeState  )    )
    {
        const comp_PeriphId_t periphId = compConfig->PeriphId;
        comp_Runtime_t * const runtime = &comp_Runtime[periphId];

        if( COMP_FUNCTION_INACTIVE == runtime->Initialized )
        {
            const comp_RequestState_t sharedState = Comp_Enable_Shared( compConfig->IrqPriority );

            if( COMP_REQUEST_OK == sharedState )
            {
                const comp_RequestState_t setupState = Comp_Setup_Instance( compConfig );

                if( COMP_REQUEST_OK == setupState )
                {
                    retState = COMP_REQUEST_OK;
                }
                else
                {
                    /* The configuration did not read back, the instance is deinitialized */
                    Comp_Cleanup_Instance( periphId );
                }
            }
            else
            {
                /* The shared resources are not available, no register is touched */
            }
        }
        else
        {
            /* The instance is initialized already, nothing changes */
        }
    }
    else
    {
        /* Invalid configuration or the cycle counter does not run, no action required */
    }
#else
    (void)compConfig;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Deinitializes one comparator instance: the interrupt is disarmed, the registers return to the
 *        reset values, the clock and the interrupt are released when no other instance is initialized.
 *        A locked comparator cannot be deinitialized (the configuration is frozen until the reset).
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized or locked instance.
 */
comp_RequestState_t Comp_Deinit( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        if( COMP_FUNCTION_INACTIVE == runtime->Locked )
        {
            Comp_Cleanup_Instance( periphId );
            retState = COMP_REQUEST_OK;
        }
        else
        {
            /* The configuration is locked until the reset, nothing changes */
        }
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Serves the start-up state machine: the comparators in COMP_STATE_STARTING move to
 *        COMP_STATE_READY when the start-up time of their power mode has elapsed (measured with the cycle
 *        counter), a requested edge interrupt is armed then.
 */
void Comp_Task( void )
{
#if defined(COMP1)
    for( uint32_t idx = 0u; COMP_PERIPH_CNT > idx; idx++ )
    {
        comp_Runtime_t * const runtime = &comp_Runtime[idx];

        if( ( COMP_FUNCTION_ACTIVE   == runtime->Initialized ) &&
            ( COMP_FUNCTION_ACTIVE   == runtime->Enabled     ) &&
            ( COMP_FUNCTION_INACTIVE == runtime->Ready       )    )
        {
            const comp_FunctionState_t startupOver = Comp_Get_StartupOver( runtime );

            if( COMP_FUNCTION_ACTIVE == startupOver )
            {
                runtime->Ready = COMP_FUNCTION_ACTIVE;

                if( COMP_FUNCTION_ACTIVE == runtime->IrqRequested )
                {
                    /* The flag is set before the source is armed: the handler may be called as soon as it is */
                    runtime->IrqArmed = COMP_FUNCTION_ACTIVE;

                    const comp_RequestState_t armState = Comp_Arm_Irq( (comp_PeriphId_t)idx, runtime->Edge );

                    if( COMP_REQUEST_OK == armState )
                    {
                        /* The interrupt is armed, no action required */
                    }
                    else
                    {
                        /* The interrupt could not be armed, it stays requested but not armed */
                        runtime->IrqArmed = COMP_FUNCTION_INACTIVE;
                    }
                }
                else
                {
                    /* No edge interrupt requested, no action required */
                }
            }
            else
            {
                /* The start-up time is not over, no action required */
            }
        }
        else
        {
            /* The instance is not in the start-up, no action required */
        }
    }
#endif /* COMP1 */

    return;
}


/**
 * \brief Fills a configuration structure with safe default values (COMP_PERIPH_1, IO1 against the
 *        internal reference, no hysteresis, high speed, normal polarity, no blanking, no window
 *        mode, no output pin, rising edge, no callback, lowest priority, no lock).
 *
 * \param compConfig [out] : Pointer to the configuration structure.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an invalid parameter.
 */
comp_RequestState_t Comp_Get_DefaultConfig( comp_Config_t * const compConfig )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    if( COMP_NULL_PTR != compConfig )
    {
        compConfig->PeriphId    = COMP_PERIPH_1;
        compConfig->InputPlus   = COMP_INPUT_PLUS_COMP1_IO1;
        compConfig->InputMinus  = COMP_INPUT_MINUS_VREFINT;
        compConfig->Hysteresis  = COMP_HYSTERESIS_NONE;
        compConfig->PowerMode   = COMP_POWER_HIGH_SPEED;
        compConfig->Polarity    = COMP_POLARITY_NORMAL;
        compConfig->Blanking    = COMP_BLANKING_NONE;
        compConfig->WindowMode  = COMP_WINDOW_MODE_INACTIVE;
        compConfig->OutputPin   = COMP_OUT_PIN_UNUSED;
        compConfig->Edge        = COMP_EDGE_RISING;
        compConfig->Callback    = COMP_NULL_PTR;
        compConfig->IrqPriority = COMP_IRQ_PRIORITY_DEFAULT;
        compConfig->Lock        = COMP_FUNCTION_INACTIVE;
        retState                = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
    }
#else
    (void)compConfig;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Enables the comparator (and locks the configuration when the lock was requested in the
 *        configuration, both in one register write). The state is COMP_STATE_STARTING until
 *        \ref Comp_Task has found that the start-up time of the power mode has elapsed.
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized, enabled or locked instance.
 */
comp_RequestState_t Comp_Set_PeriphActive( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        comp_RegValue_t enableMask = COMP_CFGR1_EN;

        if( COMP_FUNCTION_ACTIVE == runtime->LockRequested )
        {
            enableMask |= COMP_CFGR1_LOCK;
        }
        else
        {
            /* No lock requested, only the enable bit is written */
        }

        Comp_Set_RegBit( periphId, COMP_REG_CFGR, enableMask );
        retState = Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR, enableMask, enableMask );

        if( COMP_REQUEST_OK == retState )
        {
            runtime->Enabled       = COMP_FUNCTION_ACTIVE;
            runtime->Ready         = COMP_FUNCTION_INACTIVE;
            runtime->StartupCycle  = Dwt_Get_CycleCounter();
            runtime->StartupTimeUs = comp_StartupTime_us[ runtime->PowerMode ];

            const comp_RegValue_t scalerBit = Comp_Get_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_SCALEN );

            if( COMP_BITS_CLEAR != scalerBit )
            {
                /* The scaler is the inverting input, its start-up time is longer than the one of the comparator */
                runtime->StartupTimeUs = COMP_STARTUP_SCALER_US;
            }
            else
            {
                /* No scaler, the start-up time of the power mode is valid */
            }
            runtime->Locked        = runtime->LockRequested;

#if defined(STM32H503xx)
            if( COMP_FUNCTION_ACTIVE == runtime->Locked )
            {
                Comp_Set_RegBit( periphId, COMP_REG_CFGR2, COMP_CFGR2_LOCK_MASK );
                retState = Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR2, COMP_CFGR2_LOCK_MASK, COMP_CFGR2_LOCK_MASK );
            }
            else
            {
                /* No lock requested, the second configuration register stays writable */
            }
#endif
        }
        else
        {
            /* The enable did not read back, the comparator stays disabled for the module */
            Comp_Reset_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_EN );
        }
    }
    else
    {
        /* Invalid, uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Disables the comparator (the edge interrupt is disarmed first). A locked comparator cannot be disabled.
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized, disabled or locked instance.
 */
comp_RequestState_t Comp_Set_PeriphInactive( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        if( ( COMP_FUNCTION_ACTIVE   == runtime->Enabled ) &&
            ( COMP_FUNCTION_INACTIVE == runtime->Locked  )    )
        {
            if( COMP_FUNCTION_ACTIVE == runtime->IrqArmed )
            {
                Comp_Disarm_Irq( periphId );
                runtime->IrqArmed = COMP_FUNCTION_INACTIVE;
            }
            else
            {
                /* The interrupt source is not armed, no action required */
            }

            Comp_Reset_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_EN );
            retState = Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR, COMP_CFGR1_EN, COMP_BITS_CLEAR );

            if( COMP_REQUEST_OK == retState )
            {
                runtime->Enabled       = COMP_FUNCTION_INACTIVE;
                runtime->Ready         = COMP_FUNCTION_INACTIVE;
                runtime->StartupCycle  = 0u;
                runtime->StartupTimeUs = 0u;
            }
            else
            {
                /* The disable did not read back, the comparator stays enabled for the module */
            }
        }
        else
        {
            /* The comparator is disabled or locked, nothing changes */
        }
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reports the state of the comparator.
 *
 * \param periphId  [in]  : Instance identification.
 * \param compState [out] : Pointer receiving the state.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_PeriphState( comp_PeriphId_t periphId, comp_State_t * const compState )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime   ) &&
        ( COMP_NULL_PTR != compState )    )
    {
        if( COMP_FUNCTION_INACTIVE == runtime->Enabled )
        {
            *compState = COMP_STATE_DISABLED;
        }
        else if( COMP_FUNCTION_ACTIVE == runtime->Locked )
        {
            *compState = COMP_STATE_LOCKED;
        }
        else if( COMP_FUNCTION_INACTIVE == runtime->Ready )
        {
            *compState = COMP_STATE_STARTING;
        }
        else
        {
            *compState = COMP_STATE_READY;
        }

        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)compState;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the non-inverting input and configures its pin as analog input. The comparator must be
 *        disabled and unlocked; the selection is verified by reading it back.
 *
 * \param periphId  [in] : Instance identification.
 * \param inputPlus [in] : Input of the instance (a COMPx item of \ref comp_InputPlus_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_InputPlus( comp_PeriphId_t periphId, comp_InputPlus_t inputPlus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        const comp_FlagState_t match     = Comp_Get_InstanceMatch( periphId, COMP_ITEM_DECODE_INST( inputPlus ) );
        comp_FlagState_t       inputFree = COMP_FLAG_ACTIVE;

        if( ( COMP_PERIPH_1             != periphId       ) &&
            ( COMP_WINDOW_MODE_INACTIVE != comp_WindowMode )    )
        {
            /* COMP2 takes the non-inverting input of COMP1 in the window mode */
            inputFree = COMP_FLAG_INACTIVE;
        }
        else
        {
            /* The instance has its own non-inverting input */
        }

        if( ( COMP_FLAG_ACTIVE == match     ) &&
            ( COMP_FLAG_ACTIVE == inputFree )    )
        {
            retState = Comp_Write_InputPlus( periphId, inputPlus );

            if( COMP_REQUEST_OK == retState )
            {
                runtime->InputPlus = inputPlus;
            }
            else
            {
                /* The selection did not read back or the pin is not available, the stored input is kept */
            }
        }
        else
        {
            /* The input is not an input of the instance, or COMP2 takes the input of COMP1 in the window mode */
        }
    }
    else
    {
        /* Invalid, uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)inputPlus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected non-inverting input.
 *
 * \param periphId  [in]  : Instance identification.
 * \param inputPlus [out] : Pointer receiving the input.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_InputPlus( comp_PeriphId_t periphId, comp_InputPlus_t * const inputPlus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime   ) &&
        ( COMP_NULL_PTR != inputPlus )    )
    {
        *inputPlus = runtime->InputPlus;
        retState   = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)inputPlus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the inverting input and configures its pin as analog input (the scaler of the
 *        internal reference fractions is enabled by the selection). The comparator must be
 *        disabled and unlocked; the selection is verified by reading it back.
 *
 * \param periphId   [in] : Instance identification.
 * \param inputMinus [in] : Input of the instance (an item of \ref comp_InputMinus_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_InputMinus( comp_PeriphId_t periphId, comp_InputMinus_t inputMinus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        const comp_FlagState_t match = Comp_Get_InstanceMatch( periphId, COMP_ITEM_DECODE_INST( inputMinus ) );

        if( COMP_FLAG_ACTIVE == match )
        {
            retState = Comp_Write_InputMinus( periphId, inputMinus );

            if( COMP_REQUEST_OK == retState )
            {
                runtime->InputMinus = inputMinus;
            }
            else
            {
                /* The selection did not read back or the pin is not available, the stored input is kept */
            }
        }
        else
        {
            /* The input is not an input of the instance, no action required */
        }
    }
    else
    {
        /* Invalid, uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)inputMinus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected inverting input.
 *
 * \param periphId   [in]  : Instance identification.
 * \param inputMinus [out] : Pointer receiving the input.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_InputMinus( comp_PeriphId_t periphId, comp_InputMinus_t * const inputMinus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime    ) &&
        ( COMP_NULL_PTR != inputMinus )    )
    {
        *inputMinus = runtime->InputMinus;
        retState    = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)inputMinus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the hysteresis. The comparator must be disabled and unlocked.
 *
 * \param periphId   [in] : Instance identification.
 * \param hysteresis [in] : Hysteresis level.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_Hysteresis( comp_PeriphId_t periphId, comp_Hysteresis_t hysteresis )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( ( COMP_NULL_PTR       != runtime    ) &&
        ( COMP_HYSTERESIS_CNT >  hysteresis )    )
    {
        retState = Comp_Write_CfgrField( periphId, COMP_CFGR1_HYST, comp_HystReg[hysteresis] );

        if( COMP_REQUEST_OK == retState )
        {
            runtime->Hysteresis = hysteresis;
        }
        else
        {
            /* The value did not read back, the stored hysteresis is kept */
        }
    }
    else
    {
        /* Invalid parameter, or uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)hysteresis;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected hysteresis.
 *
 * \param periphId   [in]  : Instance identification.
 * \param hysteresis [out] : Pointer receiving the level.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_Hysteresis( comp_PeriphId_t periphId, comp_Hysteresis_t * const hysteresis )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime    ) &&
        ( COMP_NULL_PTR != hysteresis )    )
    {
        *hysteresis = runtime->Hysteresis;
        retState    = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)hysteresis;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the speed / consumption mode. The comparator must be disabled and unlocked. STM32H5 has no
 *        COMP_POWER_LOW_POWER mode (the item is rejected).
 *
 * \param periphId  [in] : Instance identification.
 * \param powerMode [in] : Power mode.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_PowerMode( comp_PeriphId_t periphId, comp_PowerMode_t powerMode )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( ( COMP_NULL_PTR  != runtime   ) &&
        ( COMP_POWER_CNT >  powerMode )    )
    {
        const comp_RegValue_t regValue = comp_PowerReg[powerMode];

        if( COMP_RAW_INVALID != regValue )
        {
            retState = Comp_Write_CfgrField( periphId, COMP_CFGR1_PWRMODE, regValue );

            if( COMP_REQUEST_OK == retState )
            {
                runtime->PowerMode = powerMode;
            }
            else
            {
                /* The value did not read back, the stored power mode is kept */
            }
        }
        else
        {
            /* The device has no such power mode, no action required */
        }
    }
    else
    {
        /* Invalid parameter, or uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)powerMode;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected power mode.
 *
 * \param periphId  [in]  : Instance identification.
 * \param powerMode [out] : Pointer receiving the mode.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_PowerMode( comp_PeriphId_t periphId, comp_PowerMode_t * const powerMode )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime   ) &&
        ( COMP_NULL_PTR != powerMode )    )
    {
        *powerMode = runtime->PowerMode;
        retState   = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)powerMode;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the polarity of the output. The comparator must be disabled and unlocked.
 *
 * \param periphId [in] : Instance identification.
 * \param polarity [in] : Polarity.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_Polarity( comp_PeriphId_t periphId, comp_Polarity_t polarity )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( ( COMP_NULL_PTR       != runtime  ) &&
        ( COMP_POLARITY_CNT   >  polarity )    )
    {
        retState = Comp_Write_CfgrField( periphId, COMP_CFGR1_POLARITY, comp_PolarityReg[polarity] );

        if( COMP_REQUEST_OK == retState )
        {
            runtime->Polarity = polarity;
        }
        else
        {
            /* The value did not read back, the stored polarity is kept */
        }
    }
    else
    {
        /* Invalid parameter, or uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)polarity;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected polarity.
 *
 * \param periphId [in]  : Instance identification.
 * \param polarity [out] : Pointer receiving the polarity.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_Polarity( comp_PeriphId_t periphId, comp_Polarity_t * const polarity )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime  ) &&
        ( COMP_NULL_PTR != polarity )    )
    {
        *polarity = runtime->Polarity;
        retState  = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)polarity;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the blanking source of the output. The comparator must be disabled and unlocked.
 *
 * \param periphId [in] : Instance identification.
 * \param blanking [in] : Blanking source of the instance (an item of \ref comp_Blanking_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_Blanking( comp_PeriphId_t periphId, comp_Blanking_t blanking )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_ConfigRuntime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        const comp_FlagState_t match = Comp_Get_InstanceMatch( periphId, COMP_ITEM_DECODE_INST( blanking ) );

        if( COMP_FLAG_ACTIVE == match )
        {
            const comp_RegValue_t regValue = COMP_BLANKING_DECODE_CODE( blanking ) << COMP_CFGR1_BLANKING_Pos;

            retState = Comp_Write_CfgrField( periphId, COMP_CFGR1_BLANKING, regValue );

            if( COMP_REQUEST_OK == retState )
            {
                runtime->Blanking = blanking;
            }
            else
            {
                /* The value did not read back, the stored blanking source is kept */
            }
        }
        else
        {
            /* The source is not a blanking source of the instance, no action required */
        }
    }
    else
    {
        /* Invalid, uninitialized, enabled or locked instance, no action required */
    }
#else
    (void)periphId;
    (void)blanking;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected blanking source.
 *
 * \param periphId [in]  : Instance identification.
 * \param blanking [out] : Pointer receiving the blanking source.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_Blanking( comp_PeriphId_t periphId, comp_Blanking_t * const blanking )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime  ) &&
        ( COMP_NULL_PTR != blanking )    )
    {
        *blanking = runtime->Blanking;
        retState  = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)blanking;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Activates or deactivates the window mode of the pair. Set on COMP_PERIPH_1 (the first comparator,
 *        its non-inverting input is the monitored voltage): COMP2 takes the non-inverting input of COMP1
 *        and COMP1 shows the window output (XOR of both outputs). Both comparators must be initialized,
 *        disabled and unlocked. A line with one comparator has no window mode (error).
 *
 * \param periphId   [in] : Instance identification, COMP_PERIPH_1 only.
 * \param windowMode [in] : Window mode.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_WindowMode( comp_PeriphId_t periphId, comp_WindowMode_t windowMode )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP2)
    const comp_Runtime_t * const first  = Comp_Get_ConfigRuntime( COMP_PERIPH_1 );
    const comp_Runtime_t * const second = Comp_Get_ConfigRuntime( COMP_PERIPH_2 );

    if( ( COMP_PERIPH_1       == periphId   ) &&
        ( COMP_NULL_PTR       != first      ) &&
        ( COMP_NULL_PTR       != second     ) &&
        ( COMP_WINDOW_MODE_CNT > windowMode )    )
    {
        retState = Comp_Write_WindowMode( windowMode );

        if( COMP_REQUEST_OK == retState )
        {
            comp_WindowMode = windowMode;
        }
        else
        {
            /* The bits did not read back, the stored window mode is kept */
        }
    }
    else
    {
        /* Invalid parameter, or the pair is not initialized, disabled and unlocked, no action required */
    }
#else
    (void)periphId;
    (void)windowMode;
#endif /* COMP2 */

    return ( retState );
}


/**
 * \brief Reads the window mode of the pair.
 *
 * \param periphId   [in]  : Instance identification.
 * \param windowMode [out] : Pointer receiving the window mode.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_WindowMode( comp_PeriphId_t periphId, comp_WindowMode_t * const windowMode )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime    ) &&
        ( COMP_NULL_PTR != windowMode )    )
    {
        *windowMode = comp_WindowMode;
        retState    = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)windowMode;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the level of the comparator output (after the polarity and the blanking). The level is
 *        COMP_LEVEL_INVALID while the comparator is disabled or in the start-up.
 *
 * \param periphId    [in]  : Instance identification.
 * \param outputLevel [out] : Pointer receiving the level.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_OutputLevel( comp_PeriphId_t periphId, comp_Level_t * const outputLevel )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime     ) &&
        ( COMP_NULL_PTR != outputLevel )    )
    {
        if( COMP_FUNCTION_ACTIVE == runtime->Ready )
        {
            *outputLevel = Comp_Read_Level( periphId );
        }
        else
        {
            /* Disabled or in the start-up, the output is not valid */
            *outputLevel = COMP_LEVEL_INVALID;
        }

        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)outputLevel;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Selects the edge of the output that raises the interrupt (an armed interrupt is armed again with
 *        the new edge). STM32H503 has only the rising edge (the EXTI line of the comparator is a direct line).
 *
 * \param periphId [in] : Instance identification.
 * \param edge     [in] : Edge.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR otherwise (nothing changes for a rejected parameter).
 */
comp_RequestState_t Comp_Set_TriggerEdge( comp_PeriphId_t periphId, comp_Edge_t edge )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime   = Comp_Get_Runtime( periphId );
    const comp_RequestState_t edgeState = Comp_Check_Edge( edge );

    if( ( COMP_NULL_PTR    != runtime   ) &&
        ( COMP_REQUEST_OK  == edgeState )    )
    {
        if( COMP_FUNCTION_ACTIVE == runtime->IrqArmed )
        {
            retState = Comp_Arm_Irq( periphId, edge );
        }
        else
        {
            /* The interrupt is not armed, only the stored edge changes */
            retState = COMP_REQUEST_OK;
        }

        if( COMP_REQUEST_OK == retState )
        {
            runtime->Edge = edge;
        }
        else
        {
            /* The line did not accept the edge, the stored edge is kept */
        }
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)edge;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the selected interrupt edge.
 *
 * \param periphId [in]  : Instance identification.
 * \param edge     [out] : Pointer receiving the edge.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_TriggerEdge( comp_PeriphId_t periphId, comp_Edge_t * const edge )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime ) &&
        ( COMP_NULL_PTR != edge    )    )
    {
        *edge    = runtime->Edge;
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)edge;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Requests the edge interrupt (and the wake-up from the low-power modes) of the comparator. The comparator must
 *        be enabled; during the start-up time the interrupt is only requested and armed by \ref Comp_Task when the
 *        start-up is over, so the start-up causes no false event. The interrupt is allowed on a locked comparator
 *        (the EXTI line is outside the lock) except on STM32H503, where the enable bit is a locked bit.
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized or disabled instance or when the
 *         interrupt source cannot be armed.
 */
comp_RequestState_t Comp_Set_IrqActive( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        if( COMP_FUNCTION_ACTIVE == runtime->Enabled )
        {
            if( COMP_FUNCTION_ACTIVE == runtime->Ready )
            {
                /* The flag is set before the source is armed: the handler may be called as soon as it is */
                runtime->IrqArmed = COMP_FUNCTION_ACTIVE;
                retState          = Comp_Arm_Irq( periphId, runtime->Edge );

                if( COMP_REQUEST_OK == retState )
                {
                    runtime->IrqRequested = COMP_FUNCTION_ACTIVE;
                }
                else
                {
                    /* The interrupt source could not be armed, the request is not stored */
                    runtime->IrqArmed = COMP_FUNCTION_INACTIVE;
                }
            }
            else
            {
                /* Start-up: the request is stored and armed by Comp_Task at the end of the start-up */
                runtime->IrqRequested = COMP_FUNCTION_ACTIVE;
                retState              = COMP_REQUEST_OK;
            }
        }
        else
        {
            /* The comparator is disabled, there is no output edge, no action required */
        }
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Cancels the edge interrupt request and disarms the interrupt source.
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance.
 */
comp_RequestState_t Comp_Set_IrqInactive( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        Comp_Disarm_Irq( periphId );
        runtime->IrqRequested = COMP_FUNCTION_INACTIVE;
        runtime->IrqArmed     = COMP_FUNCTION_INACTIVE;
        retState              = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Sets the edge callback (called from the interrupt with the instance and the output level).
 *
 * \param periphId [in] : Instance identification.
 * \param callback [in] : Callback, NULL for none.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance.
 */
comp_RequestState_t Comp_Set_IrqCallback( comp_PeriphId_t periphId, comp_IrqCallback_t * const callback )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        runtime->Callback = callback;
        retState          = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)callback;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Sets the NVIC priority of the comparator interrupt (one vector for COMP1 and COMP2 where both exist).
 *
 * \param periphId    [in] : Instance identification.
 * \param irqPriority [in] : Priority.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a rejected priority.
 */
comp_RequestState_t Comp_Set_IrqPriority( comp_PeriphId_t periphId, comp_IrqPrio_t irqPriority )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( COMP_NVIC_IRQ, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = COMP_REQUEST_OK;
        }
        else
        {
            /* The priority is rejected by the NVIC, no action required */
        }
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)irqPriority;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reads the NVIC priority of the comparator interrupt.
 *
 * \param periphId    [in]  : Instance identification.
 * \param irqPriority [out] : Pointer receiving the priority.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_IrqPriority( comp_PeriphId_t periphId, comp_IrqPrio_t * const irqPriority )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime     ) &&
        ( COMP_NULL_PTR != irqPriority )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( COMP_NVIC_IRQ, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = COMP_REQUEST_OK;
        }
        else
        {
            /* The NVIC could not report the priority, no action required */
        }
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)irqPriority;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Locks the configuration of an enabled comparator until the reset (every configuration call and
 *        the disable return an error afterwards). A disabled comparator cannot be locked (it could never
 *        be enabled again).
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized, disabled or locked instance.
 */
comp_RequestState_t Comp_Set_Lock( comp_PeriphId_t periphId )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        if( ( COMP_FUNCTION_ACTIVE   == runtime->Enabled ) &&
            ( COMP_FUNCTION_INACTIVE == runtime->Locked  )    )
        {
            Comp_Set_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_LOCK );
            retState = Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR, COMP_CFGR1_LOCK, COMP_CFGR1_LOCK );

#if defined(STM32H503xx)
            if( COMP_REQUEST_OK == retState )
            {
                Comp_Set_RegBit( periphId, COMP_REG_CFGR2, COMP_CFGR2_LOCK_MASK );
                retState = Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR2, COMP_CFGR2_LOCK_MASK, COMP_CFGR2_LOCK_MASK );
            }
            else
            {
                /* The first configuration register did not lock, the second one is not touched */
            }
#endif

            if( COMP_REQUEST_OK == retState )
            {
                runtime->Locked = COMP_FUNCTION_ACTIVE;
            }
            else
            {
                /* The lock did not read back, the stored lock state is kept */
            }
        }
        else
        {
            /* The comparator is disabled or locked already, nothing changes */
        }
    }
    else
    {
        /* Invalid or uninitialized instance, no action required */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Reports whether the configuration is locked.
 *
 * \param periphId  [in]  : Instance identification.
 * \param lockState [out] : Pointer receiving the lock state.
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an uninitialized instance or a NULL pointer.
 */
comp_RequestState_t Comp_Get_LockState( comp_PeriphId_t periphId, comp_FunctionState_t * const lockState )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const comp_Runtime_t * const runtime = Comp_Get_Runtime( periphId );

    if( ( COMP_NULL_PTR != runtime   ) &&
        ( COMP_NULL_PTR != lockState )    )
    {
        *lockState = runtime->Locked;
        retState   = COMP_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or uninitialized instance, no action required */
    }
#else
    (void)periphId;
    (void)lockState;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Configures the pin of a non-inverting input as analog input (decoded from the input item).
 *
 * \param inputPlus [in] : Input item with a pin (an IO item of \ref comp_InputPlus_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an input without a pin.
 */
comp_RequestState_t Comp_InitInputPlusGpio( comp_InputPlus_t inputPlus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const uint32_t instMask = COMP_ITEM_DECODE_INST( inputPlus );
    const uint32_t portId   = COMP_INPUT_DECODE_PORT( inputPlus );
    const uint32_t pinId    = COMP_INPUT_DECODE_PIN( inputPlus );

    if( ( COMP_INST_NONE   != instMask ) &&
        ( GPIO_PORT_CNT    >  portId   ) &&
        ( GPIO_PIN_ID_CNT  >  pinId    )    )
    {
        retState = Comp_Init_AnalogPin( (gpio_PortId_t)portId, (gpio_PinId_t)pinId );
    }
    else
    {
        /* The input is an internal signal or has no pin, no action required */
    }
#else
    (void)inputPlus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Configures the pin of an inverting input as analog input (decoded from the input item).
 *
 * \param inputMinus [in] : Input item with a pin (an IO item of \ref comp_InputMinus_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for an input without a pin.
 */
comp_RequestState_t Comp_InitInputMinusGpio( comp_InputMinus_t inputMinus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const uint32_t instMask = COMP_ITEM_DECODE_INST( inputMinus );
    const uint32_t portId   = COMP_INPUT_DECODE_PORT( inputMinus );
    const uint32_t pinId    = COMP_INPUT_DECODE_PIN( inputMinus );

    if( ( COMP_INST_NONE   != instMask ) &&
        ( GPIO_PORT_CNT    >  portId   ) &&
        ( GPIO_PIN_ID_CNT  >  pinId    )    )
    {
        retState = Comp_Init_AnalogPin( (gpio_PortId_t)portId, (gpio_PinId_t)pinId );
    }
    else
    {
        /* The input is an internal signal or has no pin, no action required */
    }
#else
    (void)inputMinus;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Configures the output pin of a comparator in the alternate function mode (decoded from the pin item).
 *
 * \param outputPin [in] : Output pin item (an item of \ref comp_OutPin_t).
 *
 * \return COMP_REQUEST_OK on success, COMP_REQUEST_ERROR for the unused pin item.
 */
comp_RequestState_t Comp_InitOutputGpio( comp_OutPin_t outputPin )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    const uint32_t instMask = COMP_ITEM_DECODE_INST( outputPin );
    const uint32_t portId   = COMP_OUT_DECODE_PORT( outputPin );
    const uint32_t pinId    = COMP_OUT_DECODE_PIN( outputPin );
    const uint32_t altFunc  = COMP_OUT_DECODE_AF( outputPin );

    if( ( COMP_INST_NONE      != instMask ) &&
        ( GPIO_PORT_CNT       >  portId   ) &&
        ( GPIO_PIN_ID_CNT     >  pinId    ) &&
        ( GPIO_ALT_FUNC_CNT   >  altFunc  )    )
    {
        retState = Comp_Init_AltFuncPin( (gpio_PortId_t)portId, (gpio_PinId_t)pinId, (gpio_AltFunction_t)altFunc );
    }
    else
    {
        /* The unused pin item, no action required */
    }
#else
    (void)outputPin;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Interrupt handler of the comparator vector: serves every comparator with an armed interrupt
 *        (the vector is shared by COMP1 and COMP2 where both exist), clears the flag of the edge and
 *        calls the edge callback with the output level.
 */
void Comp_IsrHandler( void )
{
#if defined(COMP1)
    for( uint32_t idx = 0u; COMP_PERIPH_CNT > idx; idx++ )
    {
        Comp_Handle_Irq( (comp_PeriphId_t)idx );
    }
#endif /* COMP1 */

    return;
}


/* =========================== LOCAL FUNCTIONS ============================== */
#if defined(COMP1)

/**
 * \brief Returns the run-time state of an initialized instance.
 *
 * \param periphId [in] : Instance identification.
 *
 * \return Pointer to the state, NULL for an invalid or uninitialized instance.
 */
static comp_Runtime_t * Comp_Get_Runtime( comp_PeriphId_t periphId )
{
    comp_Runtime_t *runtime = COMP_NULL_PTR;

    if( COMP_PERIPH_CNT > periphId )
    {
        if( COMP_FUNCTION_ACTIVE == comp_Runtime[periphId].Initialized )
        {
            runtime = &comp_Runtime[periphId];
        }
        else
        {
            /* The instance is not initialized, runtime stays NULL */
        }
    }
    else
    {
        /* Invalid instance, runtime stays NULL */
    }

    return ( runtime );
}


/**
 * \brief Returns the run-time state of an initialized instance whose configuration can be changed
 *        (the comparator is disabled and not locked).
 *
 * \param periphId [in] : Instance identification.
 *
 * \return Pointer to the state, NULL when the configuration cannot be changed.
 */
static comp_Runtime_t * Comp_Get_ConfigRuntime( comp_PeriphId_t periphId )
{
    comp_Runtime_t *runtime = Comp_Get_Runtime( periphId );

    if( COMP_NULL_PTR != runtime )
    {
        if( ( COMP_FUNCTION_INACTIVE == runtime->Enabled ) &&
            ( COMP_FUNCTION_INACTIVE == runtime->Locked  )    )
        {
            /* The configuration can be changed, no action required */
        }
        else
        {
            /* The comparator is enabled or locked, the configuration is frozen */
            runtime = COMP_NULL_PTR;
        }
    }
    else
    {
        /* The instance is not available, runtime stays NULL */
    }

    return ( runtime );
}


/**
 * \brief Counts the initialized instances.
 *
 * \return Count of the initialized instances.
 */
static uint32_t Comp_Get_InitializedCnt( void )
{
    uint32_t initializedCnt = 0u;

    for( uint32_t idx = 0u; COMP_PERIPH_CNT > idx; idx++ )
    {
        if( COMP_FUNCTION_ACTIVE == comp_Runtime[idx].Initialized )
        {
            initializedCnt++;
        }
        else
        {
            /* The instance is not initialized, it is not counted */
        }
    }

    return ( initializedCnt );
}


/**
 * \brief Checks whether an encoded item (input, output pin, blanking source) belongs to an instance.
 *
 * \param periphId [in] : Instance identification.
 * \param instMask [in] : Instance mask of the item.
 *
 * \return COMP_FLAG_ACTIVE when the instance is in the mask.
 */
static comp_FlagState_t Comp_Get_InstanceMatch( comp_PeriphId_t periphId, uint32_t instMask )
{
    comp_FlagState_t match = COMP_FLAG_INACTIVE;

    if( COMP_PERIPH_CNT > periphId )
    {
        if( COMP_BITS_CLEAR != ( instMask & ( 1u << periphId ) ) )
        {
            match = COMP_FLAG_ACTIVE;
        }
        else
        {
            /* The item is not an item of the instance, match stays inactive */
        }
    }
    else
    {
        /* Invalid instance, match stays inactive */
    }

    return ( match );
}


/**
 * \brief Checks that the cycle counter of the Dwt module runs, which the start-up time needs.
 *
 * \return COMP_REQUEST_OK when the cycle counter runs, COMP_REQUEST_ERROR otherwise.
 */
static comp_RequestState_t Comp_Check_TimeBase( void )
{
    comp_RequestState_t      retState = COMP_REQUEST_ERROR;
    dwt_FunctionState_t      dwtState = DWT_FUNCTION_INACTIVE;
    const dwt_RequestState_t dwtReq   = Dwt_Get_State( &dwtState );

    if( ( DWT_REQUEST_OK      == dwtReq   ) &&
        ( DWT_FUNCTION_ACTIVE == dwtState )    )
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* The cycle counter does not run */
        retState = COMP_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Tells whether the start-up time of an enabled comparator has elapsed.
 *
 * When the elapsed time is not available (the cycle counter was stopped), the start-up is not over: it is
 * never cut short.
 *
 * \param runtime [in] : Run-time data of the instance.
 *
 * \return COMP_FUNCTION_ACTIVE when the start-up time has elapsed, COMP_FUNCTION_INACTIVE otherwise.
 */
static comp_FunctionState_t Comp_Get_StartupOver( const comp_Runtime_t * const runtime )
{
    comp_FunctionState_t     retState = COMP_FUNCTION_INACTIVE;
    comp_Time_us_t           elapsed  = 0u;
    const dwt_RequestState_t dwtReq   = Dwt_Get_ElapsedTime_us( runtime->StartupCycle, &elapsed );

    if( ( DWT_REQUEST_OK          == dwtReq  ) &&
        ( runtime->StartupTimeUs <= elapsed )    )
    {
        retState = COMP_FUNCTION_ACTIVE;
    }
    else
    {
        retState = COMP_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Checks an interrupt edge against the edges of the device line.
 *
 * \param edge [in] : Edge.
 *
 * \return COMP_REQUEST_OK for an available edge.
 */
static comp_RequestState_t Comp_Check_Edge( comp_Edge_t edge )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(STM32H503xx)
    /* The EXTI line of the comparator is a direct line: only the rising edge sets the flag */
    if( COMP_EDGE_RISING == edge )
#else
    if( COMP_EDGE_CNT > edge )
#endif
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* The edge is not available, no action required */
    }

    return ( retState );
}


/**
 * \brief Checks every field of a configuration structure before any register is written.
 *
 * \param compConfig [in] : Pointer to the configuration structure.
 *
 * \return COMP_REQUEST_OK for a valid configuration.
 */
static comp_RequestState_t Comp_Check_Config( const comp_Config_t * const compConfig )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

    if( COMP_NULL_PTR != compConfig )
    {
        const comp_PeriphId_t     periphId   = compConfig->PeriphId;
        const uint32_t            plusInst   = COMP_ITEM_DECODE_INST( compConfig->InputPlus );
        const uint32_t            minusInst  = COMP_ITEM_DECODE_INST( compConfig->InputMinus );
        const uint32_t            blankInst  = COMP_ITEM_DECODE_INST( compConfig->Blanking );
        const uint32_t            outInst    = COMP_ITEM_DECODE_INST( compConfig->OutputPin );
        const comp_FlagState_t    plusMatch  = Comp_Get_InstanceMatch( periphId, plusInst );
        const comp_FlagState_t    minusMatch = Comp_Get_InstanceMatch( periphId, minusInst );
        const comp_FlagState_t    blankMatch = Comp_Get_InstanceMatch( periphId, blankInst );
        const comp_RequestState_t edgeState  = Comp_Check_Edge( compConfig->Edge );
        comp_FlagState_t          outputOk   = Comp_Get_InstanceMatch( periphId, outInst );
        comp_FlagState_t          powerOk    = COMP_FLAG_INACTIVE;
        comp_FlagState_t          windowOk   = COMP_FLAG_INACTIVE;

        if( COMP_OUT_PIN_UNUSED == compConfig->OutputPin )
        {
            /* The output stays internal */
            outputOk = COMP_FLAG_ACTIVE;
        }
        else
        {
            /* The output pin must be a pin of the instance */
        }

        if( COMP_POWER_CNT > compConfig->PowerMode )
        {
            if( COMP_RAW_INVALID != comp_PowerReg[compConfig->PowerMode] )
            {
                powerOk = COMP_FLAG_ACTIVE;
            }
            else
            {
                /* The device has no such power mode, powerOk stays inactive */
            }
        }
        else
        {
            /* Invalid power mode, powerOk stays inactive */
        }

        if( COMP_WINDOW_MODE_INACTIVE == compConfig->WindowMode )
        {
            windowOk = COMP_FLAG_ACTIVE;
        }
        else if( COMP_WINDOW_MODE_ACTIVE == compConfig->WindowMode )
        {
#if defined(COMP2)
            /* The window mode is set on the first comparator, the second one must be initialized already */
            if( ( COMP_PERIPH_1                == periphId                                  ) &&
                ( COMP_FUNCTION_ACTIVE         == comp_Runtime[COMP_PERIPH_2].Initialized   ) &&
                ( COMP_FUNCTION_INACTIVE       == comp_Runtime[COMP_PERIPH_2].Enabled       ) &&
                ( COMP_FUNCTION_INACTIVE       == comp_Runtime[COMP_PERIPH_2].Locked        )    )
            {
                windowOk = COMP_FLAG_ACTIVE;
            }
            else
            {
                /* The pair is not ready for the window mode, windowOk stays inactive */
            }
#endif
        }
        else
        {
            /* Invalid window mode, windowOk stays inactive */
        }

        if( ( COMP_PERIPH_CNT       >  periphId                 ) &&
            ( COMP_FLAG_ACTIVE      == plusMatch                ) &&
            ( COMP_FLAG_ACTIVE      == minusMatch               ) &&
            ( COMP_FLAG_ACTIVE      == blankMatch               ) &&
            ( COMP_FLAG_ACTIVE      == outputOk                 ) &&
            ( COMP_FLAG_ACTIVE      == powerOk                  ) &&
            ( COMP_FLAG_ACTIVE      == windowOk                 ) &&
            ( COMP_REQUEST_OK       == edgeState                ) &&
            ( COMP_HYSTERESIS_CNT   >  compConfig->Hysteresis   ) &&
            ( COMP_POLARITY_CNT     >  compConfig->Polarity     ) &&
            ( COMP_FUNCTION_ACTIVE  >= compConfig->Lock         )    )
        {
            retState = COMP_REQUEST_OK;
        }
        else
        {
            /* A field of the configuration is not valid for the instance, no action required */
        }
    }
    else
    {
        /* NULL pointer, no action required */
    }

    return ( retState );
}


/**
 * \brief Writes a field of the configuration register and verifies it by reading it back.
 *
 * \param periphId [in] : Instance identification.
 * \param regMask  [in] : Mask of the field.
 * \param regValue [in] : Value of the field (in the position of the mask).
 *
 * \return COMP_REQUEST_OK when the value was read back.
 */
static comp_RequestState_t Comp_Write_CfgrField( comp_PeriphId_t periphId,
                                                 comp_RegValue_t regMask,
                                                 comp_RegValue_t regValue )
{
    Comp_Set_RegVal( periphId, COMP_REG_CFGR, regMask, regValue );

    return ( Comp_Reg_Wait_RegVal( periphId, COMP_REG_CFGR, regMask, regValue ) );
}


/**
 * \brief Selects a non-inverting input (selection bits verified by reading them back) and configures its pin.
 *
 * \param periphId  [in] : Instance identification.
 * \param inputPlus [in] : Input of the instance.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Write_InputPlus( comp_PeriphId_t periphId, comp_InputPlus_t inputPlus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;
    const comp_RegValue_t selCode = COMP_INPUT_DECODE_SEL( inputPlus );
    const uint32_t        portId  = COMP_INPUT_DECODE_PORT( inputPlus );
    const uint32_t        pinId   = COMP_INPUT_DECODE_PIN( inputPlus );

    Comp_Reg_Set_InputPlusSel( periphId, selCode );

    const comp_RegValue_t readBack = Comp_Reg_Get_InputPlusSel( periphId );

    if( selCode == readBack )
    {
        if( ( GPIO_PORT_CNT   > portId ) &&
            ( GPIO_PIN_ID_CNT > pinId  )    )
        {
            retState = Comp_Init_AnalogPin( (gpio_PortId_t)portId, (gpio_PinId_t)pinId );
        }
        else
        {
            /* Internal signal or an input without an assigned pin, no pin to configure */
            retState = COMP_REQUEST_OK;
        }
    }
    else
    {
        /* The selection did not read back, retState stays COMP_REQUEST_ERROR */
    }

    return ( retState );
}


/**
 * \brief Selects an inverting input (selection bits verified by reading them back) and configures its pin.
 *
 * \param periphId   [in] : Instance identification.
 * \param inputMinus [in] : Input of the instance.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Write_InputMinus( comp_PeriphId_t periphId, comp_InputMinus_t inputMinus )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;
    const comp_RegValue_t selCode = COMP_INPUT_DECODE_SEL( inputMinus );
    const uint32_t        portId  = COMP_INPUT_DECODE_PORT( inputMinus );
    const uint32_t        pinId   = COMP_INPUT_DECODE_PIN( inputMinus );

    Comp_Reg_Set_InputMinusSel( periphId, selCode );

    const comp_RegValue_t readBack = Comp_Reg_Get_InputMinusSel( periphId );

    if( selCode == readBack )
    {
        if( ( GPIO_PORT_CNT   > portId ) &&
            ( GPIO_PIN_ID_CNT > pinId  )    )
        {
            retState = Comp_Init_AnalogPin( (gpio_PortId_t)portId, (gpio_PinId_t)pinId );
        }
        else
        {
            /* Internal signal or an input without an assigned pin, no pin to configure */
            retState = COMP_REQUEST_OK;
        }
    }
    else
    {
        /* The selection did not read back, retState stays COMP_REQUEST_ERROR */
    }

    return ( retState );
}


/**
 * \brief Stores the configuration in the run-time state and writes it to the registers (every step is
 *        verified by reading back). The instance is marked initialized at the start, the caller
 *        deinitializes it when this function fails.
 *
 * \param compConfig [in] : Pointer to a checked configuration structure.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Setup_Instance( const comp_Config_t * const compConfig )
{
    const comp_PeriphId_t periphId = compConfig->PeriphId;
    comp_Runtime_t * const runtime = &comp_Runtime[periphId];
    const comp_RegValue_t blankingValue = COMP_BLANKING_DECODE_CODE( compConfig->Blanking ) << COMP_CFGR1_BLANKING_Pos;
    const comp_RegValue_t cfgrValue = comp_HystReg[compConfig->Hysteresis]   |
                                      comp_PowerReg[compConfig->PowerMode]   |
                                      comp_PolarityReg[compConfig->Polarity] |
                                      blankingValue;
    comp_RequestState_t   retState  = Comp_Write_CfgrField( periphId, COMP_CFGR_CONFIG_MASK, cfgrValue );

    runtime->Initialized   = COMP_FUNCTION_ACTIVE;
    runtime->LockRequested = compConfig->Lock;
    runtime->Hysteresis    = compConfig->Hysteresis;
    runtime->PowerMode     = compConfig->PowerMode;
    runtime->Polarity      = compConfig->Polarity;
    runtime->Blanking      = compConfig->Blanking;
    runtime->Edge          = compConfig->Edge;
    runtime->Callback      = compConfig->Callback;

    if( COMP_REQUEST_OK == retState )
    {
        retState = Comp_Write_InputPlus( periphId, compConfig->InputPlus );
        runtime->InputPlus = compConfig->InputPlus;
    }
    else
    {
        /* The configuration register did not read back, the inputs are not written */
    }

    if( COMP_REQUEST_OK == retState )
    {
        retState = Comp_Write_InputMinus( periphId, compConfig->InputMinus );
        runtime->InputMinus = compConfig->InputMinus;
    }
    else
    {
        /* The non-inverting input was not written, the inverting input is not written */
    }

    if( COMP_REQUEST_OK == retState )
    {
        if( COMP_OUT_PIN_UNUSED != compConfig->OutputPin )
        {
            retState = Comp_InitOutputGpio( compConfig->OutputPin );
        }
        else
        {
            /* The output stays internal, no pin to configure */
        }
    }
    else
    {
        /* The inputs were not written, the output pin is not configured */
    }

    if( COMP_REQUEST_OK == retState )
    {
        /* A flag left from before is cleared, the start state of the comparator is clean */
        Comp_Set_RegBit( periphId, COMP_REG_ICFR, comp_IcfrClearMask[periphId] );
    }
    else
    {
        /* The output pin was not configured, the flag is not touched */
    }

#if defined(COMP2)
    if( ( COMP_REQUEST_OK          == retState                ) &&
        ( COMP_WINDOW_MODE_ACTIVE  == compConfig->WindowMode  )    )
    {
        retState = Comp_Write_WindowMode( COMP_WINDOW_MODE_ACTIVE );

        if( COMP_REQUEST_OK == retState )
        {
            comp_WindowMode = COMP_WINDOW_MODE_ACTIVE;
        }
        else
        {
            /* The window mode bits did not read back, the stored window mode is kept */
        }
    }
    else
    {
        /* No window mode requested or an earlier step failed, no action required */
    }
#endif

    return ( retState );
}


/**
 * \brief Deinitializes an instance: disarms the interrupt, returns the registers to the reset
 *        values, clears the window mode of the pair and releases the shared resources.
 *
 * \param periphId [in] : Instance identification.
 */
static void Comp_Cleanup_Instance( comp_PeriphId_t periphId )
{
    const comp_Runtime_t resetRuntime =
    {
        .Initialized   = COMP_FUNCTION_INACTIVE,
        .Enabled       = COMP_FUNCTION_INACTIVE,
        .Ready         = COMP_FUNCTION_INACTIVE,
        .LockRequested = COMP_FUNCTION_INACTIVE,
        .Locked        = COMP_FUNCTION_INACTIVE,
        .IrqRequested  = COMP_FUNCTION_INACTIVE,
        .IrqArmed      = COMP_FUNCTION_INACTIVE,
        .InputPlus     = COMP_INPUT_PLUS_UNUSED,
        .InputMinus    = COMP_INPUT_MINUS_UNUSED,
        .Hysteresis    = COMP_HYSTERESIS_NONE,
        .PowerMode     = COMP_POWER_HIGH_SPEED,
        .Polarity      = COMP_POLARITY_NORMAL,
        .Blanking      = COMP_BLANKING_NONE,
        .Edge          = COMP_EDGE_RISING,
        .Callback      = COMP_NULL_PTR,
        .StartupCycle  = 0u,
        .StartupTimeUs = 0u,
    };

    if( ( COMP_FUNCTION_ACTIVE == comp_Runtime[periphId].IrqArmed     ) ||
        ( COMP_FUNCTION_ACTIVE == comp_Runtime[periphId].IrqRequested )    )
    {
        Comp_Disarm_Irq( periphId );
    }
    else
    {
        /* No interrupt source was armed, no action required */
    }

#if defined(COMP2)
    if( COMP_WINDOW_MODE_ACTIVE == comp_WindowMode )
    {
        (void)Comp_Write_WindowMode( COMP_WINDOW_MODE_INACTIVE );
        comp_WindowMode = COMP_WINDOW_MODE_INACTIVE;
    }
    else
    {
        /* The window mode is not active, no action required */
    }
#endif

    Comp_Set_RegVal( periphId, COMP_REG_CFGR, COMP_CFGR_CONFIG_MASK | COMP_CFGR_INPUT_MASK, COMP_BITS_CLEAR );
    Comp_Reg_Set_InputPlusSel( periphId, COMP_BITS_CLEAR );
    Comp_Set_RegBit( periphId, COMP_REG_ICFR, comp_IcfrClearMask[periphId] );

    comp_Runtime[periphId] = resetRuntime;
    Comp_Release_Shared();

    return;
}


/**
 * \brief Configures a pin as analog input.
 *
 * \param portId [in] : Port of the pin.
 * \param pinId  [in] : Pin of the port.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Init_AnalogPin( gpio_PortId_t portId, gpio_PinId_t pinId )
{
    comp_RequestState_t retState  = COMP_REQUEST_ERROR;
    gpio_Config_t       gpioConfig;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;

    gpioConfig.PortId         = portId;
    gpioConfig.PinId          = pinId;
    gpioConfig.PinMode        = GPIO_PIN_MODE_ANALOG;
    gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
    gpioConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
    gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    gpioConfig.PinAltFunction = GPIO_ALT_FUNC_0;
    gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;
    gpioState                 = Gpio_Init( &gpioConfig );

    if( GPIO_REQUEST_OK == gpioState )
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* The Gpio module rejected the configuration, no action required */
    }

    return ( retState );
}


/**
 * \brief Configures a pin in the alternate function mode (comparator output).
 *
 * \param portId  [in] : Port of the pin.
 * \param pinId   [in] : Pin of the port.
 * \param altFunc [in] : Alternate function of the pin.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Init_AltFuncPin( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_AltFunction_t altFunc )
{
    comp_RequestState_t retState  = COMP_REQUEST_ERROR;
    gpio_Config_t       gpioConfig;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;

    gpioConfig.PortId         = portId;
    gpioConfig.PinId          = pinId;
    gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
    gpioConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
    gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    gpioConfig.PinAltFunction = altFunc;
    gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;
    gpioState                 = Gpio_Init( &gpioConfig );

    if( GPIO_REQUEST_OK == gpioState )
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* The Gpio module rejected the configuration, no action required */
    }

    return ( retState );
}


/**
 * \brief Enables the resources shared by the instances: the peripheral clock and the interrupt vector
 *        (the first instance), and sets the priority of the vector (every instance).
 *
 * \param irqPriority [in] : Priority of the comparator vector.
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Enable_Shared( comp_IrqPrio_t irqPriority )
{
    comp_RequestState_t  retState       = COMP_REQUEST_ERROR;
    const uint32_t       initializedCnt = Comp_Get_InitializedCnt();
    nvic_RequestState_t  prioState      = NVIC_REQUEST_ERROR;

    if( COMP_NO_INSTANCE == initializedCnt )
    {
        rcc_FunctionState_t clockState    = RCC_FUNCTION_INACTIVE;
        const rcc_RequestState_t readState = Rcc_Get_PeriphState( RCC_PERIPH_COMP, &clockState );
        rcc_RequestState_t  clockSetState = RCC_REQUEST_ERROR;

        if( RCC_REQUEST_OK == readState )
        {
            if( RCC_FUNCTION_ACTIVE == clockState )
            {
                /* The clock is enabled by somebody else, the module does not disable it */
                clockSetState = RCC_REQUEST_OK;
            }
            else
            {
                clockSetState = Rcc_Set_PeriphActive( RCC_PERIPH_COMP );

                if( RCC_REQUEST_OK == clockSetState )
                {
                    comp_ClockOwned = COMP_FUNCTION_ACTIVE;
                }
                else
                {
                    /* The clock could not be enabled, clockSetState reports the error */
                }
            }
        }
        else
        {
            /* The clock state is not known, clockSetState stays RCC_REQUEST_ERROR */
        }

        if( RCC_REQUEST_OK == clockSetState )
        {
            const nvic_RequestState_t handlerState = Nvic_Set_PeriphIrq_Handler( COMP_NVIC_IRQ, Comp_IsrHandler );

            prioState = Nvic_Set_PeriphIrq_Prio( COMP_NVIC_IRQ, irqPriority );

            const nvic_RequestState_t activeState = Nvic_Set_PeriphIrq_Active( COMP_NVIC_IRQ );

            if( ( NVIC_REQUEST_OK == handlerState ) &&
                ( NVIC_REQUEST_OK == prioState    ) &&
                ( NVIC_REQUEST_OK == activeState  )    )
            {
                retState = COMP_REQUEST_OK;
            }
            else
            {
                /* The vector is not available, the clock is released again */
                Comp_Release_Shared();
            }
        }
        else
        {
            /* The clock is not available, no action required */
        }
    }
    else
    {
        /* The clock and the vector are enabled by another instance, only the priority is set */
        prioState = Nvic_Set_PeriphIrq_Prio( COMP_NVIC_IRQ, irqPriority );

        if( NVIC_REQUEST_OK == prioState )
        {
            retState = COMP_REQUEST_OK;
        }
        else
        {
            /* The priority is rejected by the NVIC, no action required */
        }
    }

    return ( retState );
}


/**
 * \brief Releases the shared resources when no instance is initialized: the interrupt vector is
 *        deactivated and the clock is disabled when the module enabled it.
 */
static void Comp_Release_Shared( void )
{
    const uint32_t initializedCnt = Comp_Get_InitializedCnt();

    if( COMP_NO_INSTANCE == initializedCnt )
    {
        (void)Nvic_Set_PeriphIrq_Inactive( COMP_NVIC_IRQ );

        if( COMP_FUNCTION_ACTIVE == comp_ClockOwned )
        {
            (void)Rcc_Set_PeriphInactive( RCC_PERIPH_COMP );
            comp_ClockOwned = COMP_FUNCTION_INACTIVE;
        }
        else
        {
            /* The clock was enabled by somebody else, the module does not disable it */
        }
    }
    else
    {
        /* Another instance uses the shared resources, no action required */
    }

    return;
}


/**
 * \brief Arms the interrupt source of an enabled comparator whose start-up time is over.
 *        STM32H503 (direct EXTI line): the stale flag is cleared, ITEN is set and the line is
 *        enabled as wake-up with interrupt. The other lines (configurable EXTI line): the stale
 *        pending flag is cleared and the line is enabled with the selected edge.
 *
 * \param periphId [in] : Instance identification.
 * \param edge     [in] : Edge of the output (a checked edge).
 *
 * \return COMP_REQUEST_OK on success.
 */
static comp_RequestState_t Comp_Arm_Irq( comp_PeriphId_t periphId, comp_Edge_t edge )
{
    comp_RequestState_t retState  = COMP_REQUEST_ERROR;
    exti_LineConfig_t   lineConfig;
    exti_RequestState_t extiState = EXTI_REQUEST_ERROR;

    lineConfig.LineMode = EXTI_LINE_MODE_INTERRUPT;

#if defined(STM32H503xx)
    (void)edge;
    lineConfig.TriggerEdge = EXTI_TRIGGER_EDGE_NONE;

    Comp_Set_RegBit( periphId, COMP_REG_ICFR, comp_IcfrClearMask[periphId] );
    Comp_Set_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_ITEN );

    const comp_RequestState_t itenState = Comp_Reg_Wait_RegVal( periphId,
                                                                COMP_REG_CFGR,
                                                                COMP_CFGR1_ITEN,
                                                                COMP_CFGR1_ITEN );

    if( COMP_REQUEST_OK == itenState )
    {
        extiState = Exti_Set_LineActive( comp_ExtiLine[periphId], &lineConfig );
    }
    else
    {
        /* The interrupt enable bit did not read back (locked), the line is not enabled */
    }
#else
    lineConfig.TriggerEdge = comp_EdgeExti[edge];
    extiState              = Exti_Set_LinePendingClear( comp_ExtiLine[periphId] );

    if( EXTI_REQUEST_OK == extiState )
    {
        extiState = Exti_Set_LineActive( comp_ExtiLine[periphId], &lineConfig );
    }
    else
    {
        /* The stale pending flag could not be cleared, the line is not enabled */
    }
#endif

    if( EXTI_REQUEST_OK == extiState )
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* The line is not armed, no action required */
    }

    return ( retState );
}


/**
 * \brief Disarms the interrupt source of a comparator (the line is masked, the flags are cleared).
 *
 * \param periphId [in] : Instance identification.
 */
static void Comp_Disarm_Irq( comp_PeriphId_t periphId )
{
    (void)Exti_Set_LineInactive( comp_ExtiLine[periphId] );

#if defined(STM32H503xx)
    Comp_Reset_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_ITEN );
    Comp_Set_RegBit( periphId, COMP_REG_ICFR, comp_IcfrClearMask[periphId] );
#else
    (void)Exti_Set_LinePendingClear( comp_ExtiLine[periphId] );
#endif

    return;
}


/**
 * \brief Reads the output level of a comparator (after the polarity and the blanking).
 *
 * \param periphId [in] : Instance identification.
 *
 * \return COMP_LEVEL_HIGH or COMP_LEVEL_LOW.
 */
static comp_Level_t Comp_Read_Level( comp_PeriphId_t periphId )
{
    comp_Level_t          level  = COMP_LEVEL_LOW;
    const comp_RegValue_t srBits = Comp_Get_RegBit( periphId, COMP_REG_SR, comp_SrValMask[periphId] );

    if( COMP_BITS_CLEAR != srBits )
    {
        level = COMP_LEVEL_HIGH;
    }
    else
    {
        /* The output is low, level stays COMP_LEVEL_LOW */
    }

    return ( level );
}


/**
 * \brief Serves the interrupt of one comparator: when an edge is pending the flag is cleared and the
 *        callback is called with the output level.
 *
 * \param periphId [in] : Instance identification.
 */
static void Comp_Handle_Irq( comp_PeriphId_t periphId )
{
    const comp_Runtime_t * const runtime = &comp_Runtime[periphId];

    if( ( COMP_FUNCTION_ACTIVE == runtime->Initialized ) &&
        ( COMP_FUNCTION_ACTIVE == runtime->IrqArmed    )    )
    {
        comp_FlagState_t pendingState = COMP_FLAG_INACTIVE;

#if defined(STM32H503xx)
        const comp_RegValue_t flagBits = Comp_Get_RegBit( periphId, COMP_REG_SR, comp_SrFlagMask[periphId] );

        if( COMP_BITS_CLEAR != flagBits )
        {
            /* The interrupt request is the level of the comparator output and the hardware does not clear it (RM0492
             * 22.5.2): the handler removes it with ITEN = 0, otherwise it is called again at once. The request is
             * armed again by Comp_Set_IrqActive. */
            Comp_Reset_RegBit( periphId, COMP_REG_CFGR, COMP_CFGR1_ITEN );
            Comp_Set_RegBit( periphId, COMP_REG_ICFR, comp_IcfrClearMask[periphId] );
            pendingState = COMP_FLAG_ACTIVE;
        }
        else
        {
            /* No edge pending on this comparator, no action required */
        }
#else
        exti_TriggerEdge_t        pendingEdge = EXTI_TRIGGER_EDGE_NONE;
        const exti_RequestState_t readState   = Exti_Get_LinePending( comp_ExtiLine[periphId], &pendingEdge );

        if( ( EXTI_REQUEST_OK        == readState   ) &&
            ( EXTI_TRIGGER_EDGE_NONE != pendingEdge )    )
        {
            (void)Exti_Set_LinePendingClear( comp_ExtiLine[periphId] );
            pendingState = COMP_FLAG_ACTIVE;
        }
        else
        {
            /* No edge pending on this comparator, no action required */
        }
#endif

        if( COMP_FLAG_ACTIVE == pendingState )
        {
            comp_IrqCallback_t * const callback = runtime->Callback;

            if( COMP_NULL_PTR != callback )
            {
                callback( periphId, Comp_Read_Level( periphId ) );
            }
            else
            {
                /* No callback is set, the flag is cleared only */
            }
        }
        else
        {
            /* No edge pending, no action required */
        }
    }
    else
    {
        /* The comparator is not initialized or its interrupt is not armed, no action required */
    }

    return;
}

#if defined(COMP2)
/**
 * \brief Writes the window mode bits of the pair (COMP2 takes the non-inverting input of COMP1, COMP1 shows
 *        the XOR of both outputs) and verifies them by reading them back.
 *
 * \param windowMode [in] : Window mode.
 *
 * \return COMP_REQUEST_OK when both registers read back.
 */
static comp_RequestState_t Comp_Write_WindowMode( comp_WindowMode_t windowMode )
{
    comp_RequestState_t retState   = COMP_REQUEST_ERROR;
    comp_RegValue_t     firstValue = COMP_BITS_CLEAR;
    comp_RegValue_t     secondValue = COMP_BITS_CLEAR;

    if( COMP_WINDOW_MODE_ACTIVE == windowMode )
    {
        firstValue  = COMP_CFGR1_WINOUT;
        secondValue = COMP_CFGR1_WINMODE;
    }
    else
    {
        /* The window mode is inactive, both bits are cleared */
    }

    const comp_RequestState_t firstState  = Comp_Write_CfgrField( COMP_PERIPH_1, COMP_CFGR1_WINOUT, firstValue );
    const comp_RequestState_t secondState = Comp_Write_CfgrField( COMP_PERIPH_2, COMP_CFGR1_WINMODE, secondValue );

    if( ( COMP_REQUEST_OK == firstState  ) &&
        ( COMP_REQUEST_OK == secondState )    )
    {
        retState = COMP_REQUEST_OK;
    }
    else
    {
        /* A window mode bit did not read back, no action required */
    }

    return ( retState );
}
#endif /* COMP2 */

#endif /* COMP1 */
