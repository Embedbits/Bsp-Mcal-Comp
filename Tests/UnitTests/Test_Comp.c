/**
 * \author Mr.Nobody
 * \file Test_Comp.c
 * \ingroup Comp
 * \brief Unit tests of the Comp module (analog comparators).
 *
 * Comp.c and Comp_Reg.c are compiled unchanged. The COMP registers are emulated by RegMem, Rcc, Nvic, Exti
 * and Gpio are mocked by CMock. Plain memory does not emulate the lock bit, the interrupt flags and the
 * output level: the tests preset the status register where the code under test reads it, the rejected
 * calls of an enabled or locked comparator are proven by the unchanged register and the rejected write
 * protection of a locked comparator by the hardware model UtComp_HwModel_IgnoreWrites (every write is
 * undone, so the read-back of the code under test fails).
 *
 * The lines without a comparator (H523, H533, H562, H563, H573) run the test of the error returns only,
 * the other tests are ignored there. STM32H503 (one comparator, CFGR1 / CFGR2, direct EXTI line, rising
 * edge only) and the other lines (two comparators, configurable EXTI lines) differ in the expected values.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Comp_Port.h"                      /* Module under test              */
#include "Comp.h"                           /* Private header (interrupt handler, vector)  */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockExti_Port.h"                  /* EXTI module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDwt_Port.h"                   /* DWT module mock               */
#include "Stm32.h"                          /* COMP registers definition      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(COMP1)
static comp_Config_t         UtComp_Get_Config          ( comp_PeriphId_t periphId );
static volatile uint32_t *   UtComp_Get_CfgrAddr        ( comp_PeriphId_t periphId );
static volatile uint32_t *   UtComp_Get_SrAddr          ( void );
static volatile uint32_t *   UtComp_Get_IcfrAddr        ( void );
static uint32_t              UtComp_Get_SrValMask       ( comp_PeriphId_t periphId );
static exti_LineId_t         UtComp_Get_ExtiLine        ( comp_PeriphId_t periphId );
static void                  UtComp_Expect_SharedInit   ( void );
static void                  UtComp_Expect_AnalogPin    ( gpio_PortId_t portId, gpio_PinId_t pinId );
static void                  UtComp_Expect_AltFuncPin   ( gpio_PortId_t portId,
                                                          gpio_PinId_t pinId,
                                                          gpio_AltFunction_t altFunc );
static void                  UtComp_Expect_ItemPin      ( uint32_t portCode, uint32_t pinCode );
static void                  UtComp_Init_Instance       ( comp_PeriphId_t periphId, comp_FunctionState_t lock );
static void                  UtComp_Start_Instance      ( comp_PeriphId_t periphId );
static dwt_RequestState_t    UtComp_DwtGetStateStub     ( dwt_FunctionState_t * state, int callCnt );
static dwt_CounterValue_t    UtComp_DwtGetCycleStub     ( int callCnt );
static dwt_RequestState_t    UtComp_DwtElapsedStub      ( dwt_CounterValue_t startCycle,
                                                          dwt_Time_us_t *    elapsed,
                                                          int                callCnt );
static void                  UtComp_Advance_Time        ( uint32_t time_us );
static void                  UtComp_Expect_Arm          ( comp_PeriphId_t periphId, exti_TriggerEdge_t edge );
static void                  UtComp_Expect_Disarm       ( comp_PeriphId_t periphId );
static void                  UtComp_HwModel_IgnoreWrites ( void );
static void                  UtComp_IrqCallback         ( comp_PeriphId_t periphId, comp_Level_t outputLevel );
#endif /* COMP1 */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by the tests */
#define UT_COMP_IRQ_PRIO                    ( 5u )

/** Interrupt priority used by the priority tests */
#define UT_COMP_OTHER_IRQ_PRIO              ( 9u )

/** Start-up times of the power modes in microseconds (DS14053 table 76 maxima, independent copy of Comp.h) */
#define UT_COMP_STARTUP_HS_US               ( 5u )
#define UT_COMP_STARTUP_MEDIUM_US           ( 20u )
#define UT_COMP_STARTUP_ULP_US              ( 80u )

/** Start-up time of the voltage scaler in microseconds (DS14053 table 76 maximum, independent copy of Comp.h); the
 *  default inverting input VREFINT uses the scaler, so the instances of the tests start up in this time */
#define UT_COMP_STARTUP_SCALER_US           ( 250u )

/** Count of the Comp_Task calls made without time passing */
#define UT_COMP_MANY_CALLS                  ( 50u )

/** Raw register value without any bit */
#define UT_COMP_BITS_CLEAR                  ( 0u )

/** INMSEL code 0000 */
#define UT_COMP_INMSEL_0000                    ( 0u )
/** INMSEL code 0001 */
#define UT_COMP_INMSEL_0001                    ( 1u )
/** INMSEL code 0010 */
#define UT_COMP_INMSEL_0010                    ( 2u )
/** INMSEL code 0011 */
#define UT_COMP_INMSEL_0011                    ( 3u )
/** INMSEL code 0100 */
#define UT_COMP_INMSEL_0100                    ( 4u )
/** INMSEL code 0101 */
#define UT_COMP_INMSEL_0101                    ( 5u )
/** INMSEL code 0110 */
#define UT_COMP_INMSEL_0110                    ( 6u )
/** INMSEL code 1001 */
#define UT_COMP_INMSEL_1001                    ( 9u )

/** INMSEL field value of an inverting input code (not shifted) */
#define UT_COMP_INMSEL( CODE )              ( (uint32_t)( CODE ) << COMP_CFGR1_INMSEL_Pos )

/** CFGR1 value of the default configuration: INP IO1 (all zero), VREFINT as inverting input (INMSEL 0011, SCALEN) */
#define UT_COMP_DEFAULT_CFGR                ( UT_COMP_INMSEL( 3u ) | COMP_CFGR1_SCALEN )

#if defined(STM32H503xx)
/** Pin of the first non-inverting input of COMP1 (LL driver: IO1 = PB0) */
#define UT_COMP1_IO1_PORT                   ( GPIO_PORT_B )
#define UT_COMP1_IO1_PIN                    ( GPIO_PIN_ID_0 )
/** Output pin used by the output pin tests (PB15, AF12) */
#define UT_COMP_OUT_PIN                     ( COMP_OUT_PIN_COMP1_PB15 )
#define UT_COMP_OUT_PORT                    ( GPIO_PORT_B )
#define UT_COMP_OUT_PINID                   ( GPIO_PIN_ID_15 )
#define UT_COMP_OUT_AF                      ( GPIO_ALT_FUNC_12 )
#elif defined(STM32H543xx) || \
      defined(STM32H553xx)
/** Pin of the first non-inverting input of COMP1 (LL driver: IO1 = PA7) */
#define UT_COMP1_IO1_PORT                   ( GPIO_PORT_A )
#define UT_COMP1_IO1_PIN                    ( GPIO_PIN_ID_7 )
/** Output pin used by the output pin tests (PA6, AF13) */
#define UT_COMP_OUT_PIN                     ( COMP_OUT_PIN_COMP1_PA6 )
#define UT_COMP_OUT_PORT                    ( GPIO_PORT_A )
#define UT_COMP_OUT_PINID                   ( GPIO_PIN_ID_6 )
#define UT_COMP_OUT_AF                      ( GPIO_ALT_FUNC_13 )
#else
/** Pin of the first non-inverting input of COMP1 (LL driver: IO1 = PA7) */
#define UT_COMP1_IO1_PORT                   ( GPIO_PORT_A )
#define UT_COMP1_IO1_PIN                    ( GPIO_PIN_ID_7 )
/** Output pin used by the output pin tests (PC11, AF3) */
#define UT_COMP_OUT_PIN                     ( COMP_OUT_PIN_COMP1_PC11 )
#define UT_COMP_OUT_PORT                    ( GPIO_PORT_C )
#define UT_COMP_OUT_PINID                   ( GPIO_PIN_ID_11 )
#define UT_COMP_OUT_AF                      ( GPIO_ALT_FUNC_3 )
#endif

/** Number of gpio configuration records a test can expect */
#define UT_COMP_GPIO_EXPECT_MAX             ( 4u )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(COMP1)
/** Expected configurations of the mocked Gpio_Init (CMock keeps the pointer, the records live here) */
static gpio_Config_t        utComp_GpioExpected[ UT_COMP_GPIO_EXPECT_MAX ];
#endif

#if defined(COMP1)
/** Simulated cycle counter of the Dwt module, one cycle is one microsecond */
static volatile uint32_t    utComp_NowCycle;

/** State of the simulated cycle counter of the Dwt module */
static volatile dwt_FunctionState_t utComp_DwtState;
#endif

/** Count of the used records of utComp_GpioExpected */
static uint32_t             utComp_GpioExpectedCnt;

/** Count of the edge callback calls, last instance and level reported */
static uint32_t             utComp_CallbackCnt;
static comp_PeriphId_t      utComp_CallbackPeriph;
static comp_Level_t         utComp_CallbackLevel;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utComp_GpioExpectedCnt = 0u;
    utComp_CallbackCnt     = 0u;
    utComp_CallbackPeriph  = COMP_PERIPH_CNT;
    utComp_CallbackLevel   = COMP_LEVEL_INVALID;

#if defined(COMP1)
    utComp_NowCycle = 0u;
    utComp_DwtState = DWT_FUNCTION_ACTIVE;
    Dwt_Get_State_StubWithCallback( UtComp_DwtGetStateStub );
    Dwt_Get_CycleCounter_StubWithCallback( UtComp_DwtGetCycleStub );
    Dwt_Get_ElapsedTime_us_StubWithCallback( UtComp_DwtElapsedStub );
#endif
}


void tearDown( void )
{
    /* Mocks are verified by the generated runner, a hardware model is switched off */
    (void)RegMem_Set_ModelInactive();
}

/* ========================== LOCAL HELPERS ================================== */
#if defined(COMP1)

/** \brief Default configuration of an instance (non-inverting input IO1 of the instance, the other items default). */
static comp_Config_t UtComp_Get_Config( comp_PeriphId_t periphId )
{
    comp_Config_t config;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_DefaultConfig( &config ) );
    config.PeriphId    = periphId;
    config.IrqPriority = UT_COMP_IRQ_PRIO;

#if defined(COMP2)
    if( COMP_PERIPH_2 == periphId )
    {
        config.InputPlus = COMP_INPUT_PLUS_COMP2_IO1;
    }
    else
    {
        /* COMP1 keeps its default input */
    }
#endif

    return ( config );
}


/** \brief Address of the configuration register of an instance. */
static volatile uint32_t * UtComp_Get_CfgrAddr( comp_PeriphId_t periphId )
{
    volatile uint32_t *addr = &COMP1->CFGR1;

    (void)periphId;
#if defined(COMP2)
    if( COMP_PERIPH_2 == periphId )
    {
        addr = &COMP2->CFGR1;
    }
    else
    {
        /* COMP1 register */
    }
#endif

    return ( addr );
}


/** \brief Address of the status register. */
static volatile uint32_t * UtComp_Get_SrAddr( void )
{
#if defined(STM32H503xx)
    return ( &COMP1->SR );
#else
    return ( &COMP12->SR );
#endif
}


/** \brief Address of the interrupt clear register. */
static volatile uint32_t * UtComp_Get_IcfrAddr( void )
{
#if defined(STM32H503xx)
    return ( &COMP1->ICFR );
#else
    return ( &COMP12->ICFR );
#endif
}


/** \brief Output value bit of an instance in the status register. */
static uint32_t UtComp_Get_SrValMask( comp_PeriphId_t periphId )
{
    uint32_t mask = COMP_SR_C1VAL;

    (void)periphId;
#if defined(COMP2)
    if( COMP_PERIPH_2 == periphId )
    {
        mask = COMP_SR_C2VAL;
    }
    else
    {
        /* COMP1 bit */
    }
#endif

    return ( mask );
}


/** \brief Expects the Rcc and Nvic calls of the first initialized instance. */
static void UtComp_Expect_SharedInit( void )
{
    /* static: CMock stores the pointer, not a copy, and dereferences it when the mocked function is called */
    static rcc_FunctionState_t clockState;

    clockState = RCC_FUNCTION_INACTIVE;

    Rcc_Get_PeriphState_ExpectAndReturn( RCC_PERIPH_COMP, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &clockState );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( COMP_NVIC_IRQ, Comp_IsrHandler, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
}


/** \brief Expects a Gpio_Init of an analog input pin. */
static void UtComp_Expect_AnalogPin( gpio_PortId_t portId, gpio_PinId_t pinId )
{
    gpio_Config_t * const expected = &utComp_GpioExpected[utComp_GpioExpectedCnt];

    TEST_ASSERT_TRUE( UT_COMP_GPIO_EXPECT_MAX > utComp_GpioExpectedCnt );
    utComp_GpioExpectedCnt++;

    expected->PortId         = portId;
    expected->PinId          = pinId;
    expected->PinMode        = GPIO_PIN_MODE_ANALOG;
    expected->PinPull        = GPIO_PIN_PULL_NONE;
    expected->PinSpeed       = GPIO_PIN_SPEED_LOW;
    expected->PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expected->PinAltFunction = GPIO_ALT_FUNC_0;
    expected->PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( expected, GPIO_REQUEST_OK );
}


/** \brief Expects a Gpio_Init of an output pin in the alternate function mode. */
static void UtComp_Expect_AltFuncPin( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_AltFunction_t altFunc )
{
    gpio_Config_t * const expected = &utComp_GpioExpected[utComp_GpioExpectedCnt];

    TEST_ASSERT_TRUE( UT_COMP_GPIO_EXPECT_MAX > utComp_GpioExpectedCnt );
    utComp_GpioExpectedCnt++;

    expected->PortId         = portId;
    expected->PinId          = pinId;
    expected->PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expected->PinPull        = GPIO_PIN_PULL_NONE;
    expected->PinSpeed       = GPIO_PIN_SPEED_HIGH;
    expected->PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expected->PinAltFunction = altFunc;
    expected->PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( expected, GPIO_REQUEST_OK );
}


/** \brief Expects a Gpio_Init of the analog pin that an input item holds (nothing for an input without a pin). */
static void UtComp_Expect_ItemPin( uint32_t portCode, uint32_t pinCode )
{
    if( ( GPIO_PORT_CNT   > portCode ) &&
        ( GPIO_PIN_ID_CNT > pinCode  )    )
    {
        UtComp_Expect_AnalogPin( (gpio_PortId_t)portCode, (gpio_PinId_t)pinCode );
    }
    else
    {
        /* The input has no pin, no Gpio call is expected */
    }
}


/** \brief Initializes the first instance with the default configuration and the given lock request. */
static void UtComp_Init_Instance( comp_PeriphId_t periphId, comp_FunctionState_t lock )
{
    comp_Config_t config = UtComp_Get_Config( periphId );

    config.Lock = lock;

    UtComp_Expect_SharedInit();
    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( config.InputPlus ), COMP_INPUT_DECODE_PIN( config.InputPlus ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
}


/** \brief Enables an initialized instance and runs the start-up to the end (state READY). */
static void UtComp_Start_Instance( comp_PeriphId_t periphId )
{
    comp_State_t state = COMP_STATE_CNT;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( periphId ) );

    UtComp_Advance_Time( UT_COMP_STARTUP_SCALER_US );
    Comp_Task();

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( periphId, &state ) );
    TEST_ASSERT_NOT_EQUAL( COMP_STATE_DISABLED, state );
    TEST_ASSERT_NOT_EQUAL( COMP_STATE_STARTING, state );
}


/** \brief Simulated cycle counter of the Dwt module: one cycle is one microsecond. */
static dwt_CounterValue_t UtComp_DwtGetCycleStub( int callCnt )
{
    (void)callCnt;

    return ( utComp_NowCycle );
}


/** \brief Simulated state of the cycle counter of the Dwt module. */
static dwt_RequestState_t UtComp_DwtGetStateStub( dwt_FunctionState_t * state, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( state );
    *state = utComp_DwtState;

    return ( DWT_REQUEST_OK );
}


/** \brief Simulated elapsed time of the Dwt module: the error value while the counter does not run. */
static dwt_RequestState_t UtComp_DwtElapsedStub( dwt_CounterValue_t startCycle, dwt_Time_us_t * elapsed, int callCnt )
{
    dwt_RequestState_t retState = DWT_REQUEST_ERROR;

    (void)callCnt;

    TEST_ASSERT_NOT_NULL( elapsed );

    if( DWT_FUNCTION_ACTIVE == utComp_DwtState )
    {
        *elapsed = (dwt_Time_us_t)( utComp_NowCycle - startCycle );
        retState = DWT_REQUEST_OK;
    }
    else
    {
        retState = DWT_REQUEST_ERROR;
    }

    return ( retState );
}


/** \brief Moves the simulated clock by a given time in microseconds. */
static void UtComp_Advance_Time( uint32_t time_us )
{
    utComp_NowCycle = utComp_NowCycle + time_us;
}


/** \brief EXTI line of an instance (independent table of the tests). */
static exti_LineId_t UtComp_Get_ExtiLine( comp_PeriphId_t periphId )
{
    exti_LineId_t line = EXTI_LINE_COMP1;

    (void)periphId;
#if defined(COMP2)
    if( COMP_PERIPH_2 == periphId )
    {
        line = EXTI_LINE_COMP2;
    }
    else
    {
        /* COMP1 line */
    }
#endif

    return ( line );
}


/** \brief Expects the Exti calls that arm the interrupt source of an instance. */
static void UtComp_Expect_Arm( comp_PeriphId_t periphId, exti_TriggerEdge_t edge )
{
    static exti_LineConfig_t lineConfig;

    lineConfig.LineMode    = EXTI_LINE_MODE_INTERRUPT;
    lineConfig.TriggerEdge = edge;

#if defined(STM32H503xx)
    /* A direct line has no trigger edge */
    (void)periphId;
    lineConfig.TriggerEdge = EXTI_TRIGGER_EDGE_NONE;
    Exti_Set_LineActive_ExpectAndReturn( EXTI_LINE_COMP1, &lineConfig, EXTI_REQUEST_OK );
#else
    const exti_LineId_t line = UtComp_Get_ExtiLine( periphId );

    Exti_Set_LinePendingClear_ExpectAndReturn( line, EXTI_REQUEST_OK );
    Exti_Set_LineActive_ExpectAndReturn( line, &lineConfig, EXTI_REQUEST_OK );
#endif
}


/** \brief Expects the Exti calls that disarm the interrupt source of an instance. */
static void UtComp_Expect_Disarm( comp_PeriphId_t periphId )
{
#if defined(STM32H503xx)
    (void)periphId;
    Exti_Set_LineInactive_ExpectAndReturn( EXTI_LINE_COMP1, EXTI_REQUEST_OK );
#else
    const exti_LineId_t line = UtComp_Get_ExtiLine( periphId );

    Exti_Set_LineInactive_ExpectAndReturn( line, EXTI_REQUEST_OK );
    Exti_Set_LinePendingClear_ExpectAndReturn( line, EXTI_REQUEST_OK );
#endif
}


/** \brief Hardware model of a configuration register that ignores every write (the code under test reads 0). */
static void UtComp_HwModel_IgnoreWrites( void )
{
    COMP1->CFGR1 = 0u;
#if defined(COMP2)
    COMP2->CFGR1 = 0u;
#endif
}


/** \brief Edge callback of the tests: counts the calls and keeps the last instance and level. */
static void UtComp_IrqCallback( comp_PeriphId_t periphId, comp_Level_t outputLevel )
{
    utComp_CallbackCnt++;
    utComp_CallbackPeriph = periphId;
    utComp_CallbackLevel  = outputLevel;
}
#endif /* COMP1 */

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Comp_Get_ModuleVersion() returns the version of the module.
 * \par Expected results
 * Major 1, minor 0, patch 0 (no register or module is touched, valid on every device line).
 */
void Ut_Comp_Get_ModuleVersion_Always_ReturnsVersion( void )
{
    const comp_ModuleVersion_t version = Comp_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   Every function of a device line without a comparator returns the module error value.
 * \par Expected results
 * Init, Get_DefaultConfig, Deinit, Set_PeriphActive, Get_PeriphState and the pin functions return
 * COMP_REQUEST_ERROR, Comp_Task returns without effect. The test is ignored on the lines with a comparator.
 */
void Ut_Comp_AllFunctions_LineWithoutComp_ReturnError( void )
{
#if !defined(COMP1)
    comp_Config_t config;
    comp_State_t  state = COMP_STATE_CNT;

    config.PeriphId    = COMP_PERIPH_CNT;
    config.InputPlus   = COMP_INPUT_PLUS_UNUSED;
    config.InputMinus  = COMP_INPUT_MINUS_UNUSED;
    config.Hysteresis  = COMP_HYSTERESIS_NONE;
    config.PowerMode   = COMP_POWER_HIGH_SPEED;
    config.Polarity    = COMP_POLARITY_NORMAL;
    config.Blanking    = COMP_BLANKING_NONE;
    config.WindowMode  = COMP_WINDOW_MODE_INACTIVE;
    config.OutputPin   = COMP_OUT_PIN_UNUSED;
    config.Edge        = COMP_EDGE_RISING;
    config.Callback    = NULL;
    config.IrqPriority = UT_COMP_IRQ_PRIO;
    config.Lock        = COMP_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Deinit( COMP_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphActive( COMP_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputPlusGpio( COMP_INPUT_PLUS_UNUSED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitOutputGpio( COMP_OUT_PIN_UNUSED ) );
    Comp_Task();
#else
    TEST_IGNORE_MESSAGE( "MCU with a comparator" );
#endif
}

/* ========================== DEFAULT CONFIGURATION ========================= */

/**
 * \brief   Comp_Get_DefaultConfig() fills every field with a safe default.
 * \par Expected results
 * COMP1, IO1 against the internal reference, no hysteresis, high speed, normal polarity, no blanking,
 * no window mode, no output pin, rising edge, no callback, lowest priority, no lock; COMP_REQUEST_OK.
 */
void Ut_Comp_Get_DefaultConfig_ValidPointer_FillsDefaults( void )
{
#if defined(COMP1)
    comp_Config_t config;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( COMP_PERIPH_1, config.PeriphId );
    TEST_ASSERT_EQUAL( COMP_INPUT_PLUS_COMP1_IO1, config.InputPlus );
    TEST_ASSERT_EQUAL( COMP_INPUT_MINUS_VREFINT, config.InputMinus );
    TEST_ASSERT_EQUAL( COMP_HYSTERESIS_NONE, config.Hysteresis );
    TEST_ASSERT_EQUAL( COMP_POWER_HIGH_SPEED, config.PowerMode );
    TEST_ASSERT_EQUAL( COMP_POLARITY_NORMAL, config.Polarity );
    TEST_ASSERT_EQUAL( COMP_BLANKING_NONE, config.Blanking );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_INACTIVE, config.WindowMode );
    TEST_ASSERT_EQUAL( COMP_OUT_PIN_UNUSED, config.OutputPin );
    TEST_ASSERT_EQUAL( COMP_EDGE_RISING, config.Edge );
    TEST_ASSERT_NULL( config.Callback );
    TEST_ASSERT_EQUAL_UINT32( COMP_IRQ_PRIORITY_DEFAULT, config.IrqPriority );
    TEST_ASSERT_EQUAL( COMP_FUNCTION_INACTIVE, config.Lock );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Get_DefaultConfig() rejects a NULL pointer.
 * \par Expected results
 * COMP_REQUEST_ERROR.
 */
void Ut_Comp_Get_DefaultConfig_NullPointer_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_DefaultConfig( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/* ============================== INIT ====================================== */

/**
 * \brief   Comp_Init() with the default configuration configures the first instance.
 * \par Expected results
 * Rcc clock enabled, Nvic handler / priority / activation, the analog pin of IO1 configured, the configuration
 * register holds the selection of the internal reference (INMSEL 0011, SCALEN) and the enable bit is clear,
 * the clear register holds the interrupt flag clear bit, the state is DISABLED, the Exti is not used.
 */
void Ut_Comp_Init_DefaultConfig_ConfiguresRegisters( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;

    UtComp_Expect_SharedInit();
    UtComp_Expect_AnalogPin( UT_COMP1_IO1_PORT, UT_COMP1_IO1_PIN );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_ICFR_CC1IF, *UtComp_Get_IcfrAddr() & COMP_ICFR_CC1IF );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, COMP1->CFGR2 );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_DISABLED, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() writes every setting of a complete configuration.
 * \par Expected results
 * Hysteresis HIGH, ultra-low power, inverted polarity, blanking TIM2_OC3, input IO2 and half of VREFINT with
 * the scaler bridge, the output pin in the alternate function mode: the configuration register holds exactly
 * the expected bits, the getters return the configured items, the pins of the items are configured.
 */
void Ut_Comp_Init_FullConfig_WritesEverySetting( void )
{
#if defined(COMP1)
    comp_Config_t       config   = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_InputPlus_t    inputPlus   = COMP_INPUT_PLUS_UNUSED;
    comp_InputMinus_t   inputMinus  = COMP_INPUT_MINUS_UNUSED;
    comp_Hysteresis_t   hysteresis  = COMP_HYSTERESIS_NONE;
    comp_PowerMode_t    powerMode   = COMP_POWER_HIGH_SPEED;
    comp_Polarity_t     polarity    = COMP_POLARITY_NORMAL;
    comp_Blanking_t     blanking    = COMP_BLANKING_NONE;
    comp_Edge_t         edge        = COMP_EDGE_BOTH;
    uint32_t            expected    = COMP_CFGR1_HYST | COMP_CFGR1_PWRMODE | COMP_CFGR1_POLARITY |
                                      COMP_CFGR1_BLANKING_1;

    expected         |= UT_COMP_INMSEL( UT_COMP_INMSEL_0001 ) | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN;
    config.InputPlus  = COMP_INPUT_PLUS_COMP1_IO2;
    config.InputMinus = COMP_INPUT_MINUS_VREFINT_HALF;
    config.Hysteresis = COMP_HYSTERESIS_HIGH;
    config.PowerMode  = COMP_POWER_ULTRA_LOW_POWER;
    config.Polarity   = COMP_POLARITY_INVERTED;
    config.Blanking   = COMP_BLANKING_TIM2_OC3;
    config.OutputPin  = UT_COMP_OUT_PIN;
#if defined(STM32H503xx)
    config.Edge       = COMP_EDGE_RISING;
#else
    config.Edge       = COMP_EDGE_BOTH;
    expected         |= COMP_CFGR1_INPSEL0;
#endif

    UtComp_Expect_SharedInit();
    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( config.InputPlus ), COMP_INPUT_DECODE_PIN( config.InputPlus ) );
    UtComp_Expect_AltFuncPin( UT_COMP_OUT_PORT, UT_COMP_OUT_PINID, UT_COMP_OUT_AF );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( expected, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR2_INPSEL0, COMP1->CFGR2 );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_InputPlus( COMP_PERIPH_1, &inputPlus ) );
    TEST_ASSERT_EQUAL( COMP_INPUT_PLUS_COMP1_IO2, inputPlus );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_InputMinus( COMP_PERIPH_1, &inputMinus ) );
    TEST_ASSERT_EQUAL( COMP_INPUT_MINUS_VREFINT_HALF, inputMinus );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Hysteresis( COMP_PERIPH_1, &hysteresis ) );
    TEST_ASSERT_EQUAL( COMP_HYSTERESIS_HIGH, hysteresis );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PowerMode( COMP_PERIPH_1, &powerMode ) );
    TEST_ASSERT_EQUAL( COMP_POWER_ULTRA_LOW_POWER, powerMode );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Polarity( COMP_PERIPH_1, &polarity ) );
    TEST_ASSERT_EQUAL( COMP_POLARITY_INVERTED, polarity );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Blanking( COMP_PERIPH_1, &blanking ) );
    TEST_ASSERT_EQUAL( COMP_BLANKING_TIM2_OC3, blanking );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_TriggerEdge( COMP_PERIPH_1, &edge ) );
    TEST_ASSERT_EQUAL( config.Edge, edge );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() rejects a NULL configuration before any module or register is touched.
 * \par Expected results
 * COMP_REQUEST_ERROR, no mock call, the configuration register stays zero.
 */
void Ut_Comp_Init_NullPointer_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( NULL ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() rejects an invalid instance.
 * \par Expected results
 * COMP_REQUEST_ERROR for COMP_PERIPH_CNT, no mock call.
 */
void Ut_Comp_Init_InvalidPeriph_ReturnsError( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

    config.PeriphId = COMP_PERIPH_CNT;

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() rejects an input that does not belong to the instance.
 * \par Expected results
 * A COMP2 input in the configuration of COMP1 returns COMP_REQUEST_ERROR, no mock call. On the line with one
 * comparator the unused input item (no instance) is rejected instead.
 */
void Ut_Comp_Init_InputOfOtherInstance_ReturnsError( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

#if defined(COMP2)
    config.InputPlus = COMP_INPUT_PLUS_COMP2_IO1;
#else
    config.InputPlus = COMP_INPUT_PLUS_UNUSED;
#endif

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() rejects the unused items, an invalid hysteresis, the power mode the device does not have,
 *          the invalid polarity, edge and window mode.
 * \par Expected results
 * Every call returns COMP_REQUEST_ERROR, no mock call.
 */
void Ut_Comp_Init_InvalidItems_ReturnsError( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

    config.InputMinus = COMP_INPUT_MINUS_UNUSED;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.Blanking = COMP_BLANKING_UNUSED;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.Hysteresis = COMP_HYSTERESIS_CNT;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.PowerMode = COMP_POWER_LOW_POWER;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.PowerMode = COMP_POWER_CNT;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.Polarity = COMP_POLARITY_CNT;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.Edge = COMP_EDGE_CNT;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    config = UtComp_Get_Config( COMP_PERIPH_1 );
    config.WindowMode = COMP_WINDOW_MODE_CNT;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() accepts only the rising edge on STM32H503 (direct EXTI line) and every edge elsewhere.
 * \par Expected results
 * STM32H503: the falling edge is rejected with COMP_REQUEST_ERROR, no mock call. The other lines: the
 * falling edge is accepted.
 */
void Ut_Comp_Init_FallingEdge_DependsOnExtiLineType( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

    config.Edge = COMP_EDGE_FALLING;

#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
#else
    UtComp_Expect_SharedInit();
    UtComp_Expect_AnalogPin( UT_COMP1_IO1_PORT, UT_COMP1_IO1_PIN );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() of an instance that is initialized already.
 * \par Expected results
 * The second call returns COMP_REQUEST_ERROR without any mock call and the configuration is unchanged.
 */
void Ut_Comp_Init_AlreadyInitialized_ReturnsError( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    config.Hysteresis = COMP_HYSTERESIS_HIGH;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() when the clock state of the Rcc cannot be read.
 * \par Expected results
 * COMP_REQUEST_ERROR, no Nvic call, no register written.
 */
void Ut_Comp_Init_RccStateReadFails_ReturnsError( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );

    Rcc_Get_PeriphState_ExpectAndReturn( RCC_PERIPH_COMP, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphState_IgnoreArg_funcState();

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() when the Nvic cannot activate the vector.
 * \par Expected results
 * COMP_REQUEST_ERROR, the vector is deactivated and the clock that the module enabled is disabled again,
 * no register written.
 */
void Ut_Comp_Init_NvicActiveFails_ReleasesSharedResources( void )
{
#if defined(COMP1)
    comp_Config_t               config     = UtComp_Get_Config( COMP_PERIPH_1 );
    static rcc_FunctionState_t  clockState = RCC_FUNCTION_INACTIVE;

    Rcc_Get_PeriphState_ExpectAndReturn( RCC_PERIPH_COMP, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &clockState );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( COMP_NVIC_IRQ, Comp_IsrHandler, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() when the configuration register does not keep the written value (write ignored).
 * \par Expected results
 * The read-back fails: COMP_REQUEST_ERROR, the instance is deinitialized (the shared resources are released),
 * the state getter rejects the instance afterwards.
 */
void Ut_Comp_Init_ReadBackFails_DeinitializesInstance( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;

    config.Hysteresis = COMP_HYSTERESIS_MEDIUM;

    UtComp_Expect_SharedInit();
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtComp_HwModel_IgnoreWrites ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() of the second instance shares the clock and the vector with the first one.
 * \par Expected results
 * COMP2 after COMP1: only the Nvic priority and the pin of COMP2 are configured (no clock, no handler); the
 * registers of the two instances are independent. Ignored on the line with one comparator.
 */
void Ut_Comp_Init_SecondInstance_SharesClockAndVector( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    config.Hysteresis = COMP_HYSTERESIS_LOW;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_HYST_0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}

/* ============================== DEINIT ==================================== */

/**
 * \brief   Comp_Deinit() of the only initialized instance.
 * \par Expected results
 * The registers return to zero, the vector is deactivated and the clock is disabled, COMP_REQUEST_OK, the
 * getters reject the instance afterwards.
 */
void Ut_Comp_Deinit_OnlyInstance_ResetsRegistersAndReleasesResources( void )
{
#if defined(COMP1)
    comp_State_t state = COMP_STATE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Deinit( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Deinit() of an instance while the other one is initialized.
 * \par Expected results
 * The first Deinit releases nothing (no Rcc / Nvic call), the second one deactivates the vector and disables the
 * clock. Ignored on the line with one comparator.
 */
void Ut_Comp_Deinit_TwoInstances_ReleasesResourcesWithTheLast( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Deinit( COMP_PERIPH_1 ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Deinit( COMP_PERIPH_2 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   Comp_Deinit() rejects an uninitialized and an invalid instance.
 * \par Expected results
 * COMP_REQUEST_ERROR, no mock call.
 */
void Ut_Comp_Deinit_UninitializedOrInvalid_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Deinit( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Deinit( COMP_PERIPH_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Deinit() of a locked comparator.
 * \par Expected results
 * COMP_REQUEST_ERROR (the configuration is frozen until the reset), no mock call, the registers are unchanged.
 */
void Ut_Comp_Deinit_LockedInstance_ReturnsError( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_ACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Deinit( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_LOCK, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_LOCK );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* ============================== CONTROL =================================== */

/**
 * \brief   Comp_Set_PeriphActive() enables an initialized comparator.
 * \par Expected results
 * The enable bit is set, the state is STARTING (also after a call of Comp_Task at once and after 249 us),
 * Comp_Task ends
 * the start-up at the first call after the start-up time of the scaler (250 us) which the default inverting input
 * VREFINT uses (it is longer than the 5 us of the high-speed mode).
 */
void Ut_Comp_Set_PeriphActive_Initialized_StartsUpInTime( void )
{
#if defined(COMP1)
    comp_State_t state = COMP_STATE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_EN, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_EN );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( UT_COMP_STARTUP_HS_US );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( UT_COMP_STARTUP_SCALER_US - UT_COMP_STARTUP_HS_US - 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The start-up time follows the power mode selected at the enable and the time, not the count of the calls.
 * \par Expected results
 * The inverting input is a pin (no scaler). Many calls of Comp_Task without time passing do not end the start-up.
 * Medium mode: READY at 20 us (not at 19 us), ultra-low-power mode: READY at 80 us (not at 79 us). Time spent before
 * the enable does not count.
 */
void Ut_Comp_Task_StartupTime_PowerModes_FollowTimeNotCallCount( void )
{
#if defined(COMP1)
    comp_State_t state = COMP_STATE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( COMP_INPUT_MINUS_COMP1_IO1 ),
                           COMP_INPUT_DECODE_PIN( COMP_INPUT_MINUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_MEDIUM ) );
    UtComp_Advance_Time( UT_COMP_STARTUP_ULP_US );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );

    for( uint32_t idx = 0u; UT_COMP_MANY_CALLS > idx; idx++ )
    {
        Comp_Task();
    }

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( UT_COMP_STARTUP_MEDIUM_US - 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, state );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_ULTRA_LOW_POWER ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );

    UtComp_Advance_Time( UT_COMP_STARTUP_ULP_US - 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );

    UtComp_Advance_Time( 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() fails when the cycle counter of the Dwt module does not run.
 * \par Expected results
 * COMP_REQUEST_ERROR without any Rcc, Nvic, Exti or Gpio call and without a register write. With the counter stopped
 * after the enable the state stays STARTING for any time and any count of the calls.
 */
void Ut_Comp_Init_CycleCounterNotRunning_ReturnsErrorWritesNothing( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;

    utComp_DwtState = DWT_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );

    utComp_DwtState = DWT_FUNCTION_ACTIVE;
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );

    utComp_DwtState = DWT_FUNCTION_INACTIVE;
    UtComp_Advance_Time( UT_COMP_STARTUP_SCALER_US );

    for( uint32_t idx = 0u; UT_COMP_MANY_CALLS > idx; idx++ )
    {
        Comp_Task();
    }

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_PeriphActive() with a lock request writes the enable and the lock bit together.
 * \par Expected results
 * The configuration register holds EN and LOCK (STM32H503: the second register holds LOCK too), the state is
 * LOCKED and the lock state active.
 */
void Ut_Comp_Set_PeriphActive_LockRequested_EnablesAndLocks( void )
{
#if defined(COMP1)
    const uint32_t       enableLock = COMP_CFGR1_EN | COMP_CFGR1_LOCK;
    comp_State_t         state     = COMP_STATE_CNT;
    comp_FunctionState_t lockState = COMP_FUNCTION_INACTIVE;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( enableLock, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & enableLock );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_LOCK, COMP1->CFGR2 & COMP_CFGR1_LOCK );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_LOCKED, state );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_LockState( COMP_PERIPH_1, &lockState ) );
    TEST_ASSERT_EQUAL( COMP_FUNCTION_ACTIVE, lockState );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_PeriphActive() of an uninitialized, an enabled and an invalid instance.
 * \par Expected results
 * COMP_REQUEST_ERROR in all three cases, the enabled comparator stays enabled.
 */
void Ut_Comp_Set_PeriphActive_InvalidState_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphActive( COMP_PERIPH_CNT ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_EN, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_EN );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_PeriphInactive() disables an enabled comparator.
 * \par Expected results
 * The enable bit is cleared, the state is DISABLED, the level is invalid; the comparator can be enabled again and
 * starts up from the beginning.
 */
void Ut_Comp_Set_PeriphInactive_Enabled_DisablesComparator( void )
{
#if defined(COMP1)
    comp_State_t state = COMP_STATE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_EN );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_DISABLED, state );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_STARTING, state );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_PeriphInactive() of a disabled, a locked and an uninitialized comparator.
 * \par Expected results
 * COMP_REQUEST_ERROR in all cases, the locked comparator keeps its enable bit.
 */
void Ut_Comp_Set_PeriphInactive_InvalidState_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );

    UtComp_Start_Instance( COMP_PERIPH_1 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_EN, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_EN );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Get_PeriphState() rejects a NULL pointer and an uninitialized instance.
 * \par Expected results
 * COMP_REQUEST_ERROR in both cases.
 */
void Ut_Comp_Get_PeriphState_InvalidParameter_ReturnsError( void )
{
#if defined(COMP1)
    comp_State_t state = COMP_STATE_CNT;

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* ============================ INPUTS ====================================== */

/**
 * \brief   Comp_Set_InputPlus() selects the second pin input and configures its pin.
 * \par Expected results
 * The selection bit of IO2 is set (STM32H503: INPSEL0 in the second configuration register, the other lines:
 * INPSEL0 of the configuration register), the pin of the item is configured as analog input, the getter
 * returns the item; selecting IO1 clears the selection again.
 */
void Ut_Comp_Set_InputPlus_Io2_WritesSelectionAndConfiguresPin( void )
{
#if defined(COMP1)
    comp_InputPlus_t inputPlus = COMP_INPUT_PLUS_UNUSED;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( COMP_INPUT_PLUS_COMP1_IO2 ),
                           COMP_INPUT_DECODE_PIN( COMP_INPUT_PLUS_COMP1_IO2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO2 ) );

#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR2_INPSEL0, COMP1->CFGR2 );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_INPSEL0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_InputPlus( COMP_PERIPH_1, &inputPlus ) );
    TEST_ASSERT_EQUAL( COMP_INPUT_PLUS_COMP1_IO2, inputPlus );

    UtComp_Expect_AnalogPin( UT_COMP1_IO1_PORT, UT_COMP1_IO1_PIN );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, COMP1->CFGR2 );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputPlus() selects the DAC output (an internal signal, no pin).
 * \par Expected results
 * STM32H503: INPSEL2 of the configuration register, the other lines INPSEL1; no Gpio call, the getter returns the item.
 */
void Ut_Comp_Set_InputPlus_Dac_WritesSelectionWithoutPin( void )
{
#if defined(COMP1)
    comp_InputPlus_t inputPlus = COMP_INPUT_PLUS_UNUSED;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_DAC1_CH1 ) );

#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_INPSEL2 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_INPSEL1 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_InputPlus( COMP_PERIPH_1, &inputPlus ) );
    TEST_ASSERT_EQUAL( COMP_INPUT_PLUS_COMP1_DAC1_CH1, inputPlus );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputPlus() selects the third pin input of STM32H503 (PB2).
 * \par Expected results
 * INPSEL1 of the configuration register is set, the pin PB2 is configured as analog input. Ignored on the other lines.
 */
void Ut_Comp_Set_InputPlus_Io3_Stm32H503_SelectsThirdPin( void )
{
#if defined(STM32H503xx)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_2 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO3 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_INPSEL1 ), COMP1->CFGR1 );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, COMP1->CFGR2 );
#else
    TEST_IGNORE_MESSAGE( "MCU other than STM32H503" );
#endif
}


/**
 * \brief   Comp_Set_InputPlus() rejects an input of the other instance, the unused item and an uninitialized instance.
 * \par Expected results
 * COMP_REQUEST_ERROR, no Gpio call, the register is unchanged. On the line with one comparator the other
 * instance is replaced by the unused item.
 */
void Ut_Comp_Set_InputPlus_InvalidInput_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO2 ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

#if defined(COMP2)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP2_IO2 ) );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_UNUSED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_CNT, COMP_INPUT_PLUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputMinus() selects the internal reference and its fractions.
 * \par Expected results
 * 1/4: INMSEL 0000 with SCALEN and BRGEN, 1/2: 0001 with both, 3/4: 0010 with both, VREFINT: 0011 with SCALEN
 * only. The other bits of the register are kept, no Gpio call.
 */
void Ut_Comp_Set_InputMinus_VrefintFractions_WritesSelectionAndScaler( void )
{
#if defined(COMP1)
    const uint32_t scalerBoth = COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN;
    comp_InputMinus_t inputMinus = COMP_INPUT_MINUS_UNUSED;
    volatile uint32_t * const cfgr = UtComp_Get_CfgrAddr( COMP_PERIPH_1 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_LOW ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT_QUARTER ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_INMSEL( UT_COMP_INMSEL_0000 ) | scalerBoth | COMP_CFGR1_HYST_0 ), *cfgr );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT_HALF ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_INMSEL( UT_COMP_INMSEL_0001 ) | scalerBoth | COMP_CFGR1_HYST_0 ), *cfgr );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT_THREE_QUARTERS ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_INMSEL( UT_COMP_INMSEL_0010 ) | scalerBoth | COMP_CFGR1_HYST_0 ), *cfgr );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_INMSEL( UT_COMP_INMSEL_0011 ) | COMP_CFGR1_SCALEN | COMP_CFGR1_HYST_0 ), *cfgr );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_InputMinus( COMP_PERIPH_1, &inputMinus ) );
    TEST_ASSERT_EQUAL( COMP_INPUT_MINUS_VREFINT, inputMinus );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputMinus() selects the DAC outputs.
 * \par Expected results
 * DAC1 channel 1: INMSEL 0100 on STM32H503, 0101 on the other lines; DAC1 channel 2 (not on STM32H503): 0100;
 * the scaler bits are cleared, no Gpio call.
 */
void Ut_Comp_Set_InputMinus_Dac_WritesSelection( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_DAC1_CH1 ) );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0100 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0101 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_DAC1_CH2 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0100 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputMinus() selects the first pin input of COMP1 and configures its pin.
 * \par Expected results
 * STM32H503: INMSEL 0101 and pin PC4; STM32H5E / H5F: INMSEL 0110 and pin PA6; STM32H543 / H553: INMSEL 0110, the
 * item holds no pin (not assigned), so no Gpio call.
 */
void Ut_Comp_Set_InputMinus_Io1_WritesSelectionAndConfiguresPin( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

#if defined(STM32H503xx)
    UtComp_Expect_AnalogPin( GPIO_PORT_C, GPIO_PIN_ID_4 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0101 ), COMP1->CFGR1 );
#elif defined(STM32H543xx) || \
      defined(STM32H553xx)
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0110 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    UtComp_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_6 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_COMP1_IO1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_0110 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputMinus() selects the third pin input of COMP2 (INMSEL 1001).
 * \par Expected results
 * COMP2 register: INMSEL 1001 (the INMSEL codes of COMP2 differ from COMP1), the pin of the item is configured where
 * the item holds a pin. Ignored on the line with one comparator.
 */
void Ut_Comp_Set_InputMinus_Comp2Io3_WritesSelection( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    UtComp_Expect_SharedInit();
    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( config.InputPlus ), COMP_INPUT_DECODE_PIN( config.InputPlus ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( COMP_INPUT_MINUS_COMP2_IO3 ),
                           COMP_INPUT_DECODE_PIN( COMP_INPUT_MINUS_COMP2_IO3 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_2, COMP_INPUT_MINUS_COMP2_IO3 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_INMSEL( UT_COMP_INMSEL_1001 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_InputMinus() rejects an input of the other instance and the unused item.
 * \par Expected results
 * COMP_REQUEST_ERROR, no Gpio call, the register is unchanged.
 */
void Ut_Comp_Set_InputMinus_InvalidInput_ReturnsError( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

#if defined(COMP2)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_COMP2_IO1 ) );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_UNUSED ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* =========================== SETTINGS ===================================== */

/**
 * \brief   Comp_Set_Hysteresis() writes the four levels.
 * \par Expected results
 * NONE: 00, LOW: 01, MEDIUM: 10, HIGH: 11 in the HYST field, the other bits are kept; the getter returns the
 * level; an invalid level returns COMP_REQUEST_ERROR and changes nothing.
 */
void Ut_Comp_Set_Hysteresis_AllLevels_WritesField( void )
{
#if defined(COMP1)
    comp_Hysteresis_t hysteresis = COMP_HYSTERESIS_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_LOW ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_HYST_0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_MEDIUM ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_HYST_1 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_HIGH ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_HYST ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Hysteresis( COMP_PERIPH_1, &hysteresis ) );
    TEST_ASSERT_EQUAL( COMP_HYSTERESIS_HIGH, hysteresis );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_HYST ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_NONE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_PowerMode() writes the power modes of the device and rejects the one it does not have.
 * \par Expected results
 * HIGH_SPEED: 00, MEDIUM: 01, ULTRA_LOW_POWER: 11 in the PWRMODE field; LOW_POWER and an invalid mode return
 * COMP_REQUEST_ERROR and change nothing; the getter returns the mode.
 */
void Ut_Comp_Set_PowerMode_Modes_WritesField( void )
{
#if defined(COMP1)
    comp_PowerMode_t powerMode = COMP_POWER_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_MEDIUM ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_PWRMODE_0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_ULTRA_LOW_POWER ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_PWRMODE ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PowerMode( COMP_PERIPH_1, &powerMode ) );
    TEST_ASSERT_EQUAL( COMP_POWER_ULTRA_LOW_POWER, powerMode );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_LOW_POWER ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_PWRMODE ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_HIGH_SPEED ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_Polarity() inverts and restores the output polarity.
 * \par Expected results
 * INVERTED sets the POLARITY bit, NORMAL clears it, the getter returns the polarity, an invalid value returns
 * COMP_REQUEST_ERROR.
 */
void Ut_Comp_Set_Polarity_InvertedAndNormal_WritesBit( void )
{
#if defined(COMP1)
    comp_Polarity_t polarity = COMP_POLARITY_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_INVERTED ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_POLARITY ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Polarity( COMP_PERIPH_1, &polarity ) );
    TEST_ASSERT_EQUAL( COMP_POLARITY_INVERTED, polarity );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_CNT ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_NORMAL ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_Blanking() writes the blanking sources of COMP1.
 * \par Expected results
 * TIM1_OC5: 0001, TIM2_OC3: 0010, TIM3_OC3: 0011, TIM3_OC4: 0100, LPTIM1_CH2: 0101, LPTIM2_CH2: 0110 in the BLANKING
 * field, NONE clears it; the getter returns the source.
 */
void Ut_Comp_Set_Blanking_Comp1Sources_WritesField( void )
{
#if defined(COMP1)
    comp_Blanking_t blanking = COMP_BLANKING_UNUSED;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM1_OC5 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM2_OC3 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_1 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM3_OC3 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_0 | COMP_CFGR1_BLANKING_1 ),
                             *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM3_OC4 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_2 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_LPTIM1_CH2 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_2 | COMP_CFGR1_BLANKING_0 ),
                             *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_LPTIM2_CH2 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_2 | COMP_CFGR1_BLANKING_1 ),
                             *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Blanking( COMP_PERIPH_1, &blanking ) );
    TEST_ASSERT_EQUAL( COMP_BLANKING_LPTIM2_CH2, blanking );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_NONE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_Blanking() writes the blanking sources of COMP2 and rejects the sources of the other instance.
 * \par Expected results
 * COMP2: TIM1_OC6: 0001, TIM8_OC5: 0101, TIM15_OC2: 0110; the COMP1 source TIM1_OC5 is rejected on COMP2 and the COMP2
 * source TIM1_OC6 on COMP1 (COMP_REQUEST_ERROR, register unchanged). Ignored on the line with one comparator.
 */
void Ut_Comp_Set_Blanking_Comp2Sources_WritesFieldAndRejectsOtherInstance( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_2, COMP_BLANKING_TIM1_OC6 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_0 ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_2, COMP_BLANKING_TIM8_OC5 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_2 | COMP_CFGR1_BLANKING_0 ),
                             *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_2, COMP_BLANKING_TIM15_OC2 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_BLANKING_2 | COMP_CFGR1_BLANKING_1 ),
                             *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Blanking( COMP_PERIPH_2, COMP_BLANKING_TIM1_OC5 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM1_OC6 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_UNUSED ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   The settings of an enabled comparator are rejected.
 * \par Expected results
 * Hysteresis, power mode, polarity, blanking and both inputs return COMP_REQUEST_ERROR, the register keeps its value.
 */
void Ut_Comp_Settings_EnabledComparator_ReturnError( void )
{
#if defined(COMP1)
    uint32_t snapshot = UT_COMP_BITS_CLEAR;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    snapshot = *UtComp_Get_CfgrAddr( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_HIGH ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_MEDIUM ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_INVERTED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM2_OC3 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT_HALF ) );
    TEST_ASSERT_EQUAL_HEX32( snapshot, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The settings of a locked comparator are rejected.
 * \par Expected results
 * After Comp_Set_Lock every setting returns COMP_REQUEST_ERROR, the register keeps its value.
 */
void Ut_Comp_Settings_LockedComparator_ReturnError( void )
{
#if defined(COMP1)
    uint32_t snapshot = UT_COMP_BITS_CLEAR;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Lock( COMP_PERIPH_1 ) );
    snapshot = *UtComp_Get_CfgrAddr( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_HIGH ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_MEDIUM ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_INVERTED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM2_OC3 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT_HALF ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( snapshot, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The getters reject a NULL pointer and an uninitialized instance.
 * \par Expected results
 * COMP_REQUEST_ERROR for every getter.
 */
void Ut_Comp_Getters_InvalidParameter_ReturnError( void )
{
#if defined(COMP1)
    comp_InputPlus_t      inputPlus  = COMP_INPUT_PLUS_UNUSED;
    comp_InputMinus_t     inputMinus = COMP_INPUT_MINUS_UNUSED;
    comp_Hysteresis_t     hysteresis = COMP_HYSTERESIS_NONE;
    comp_PowerMode_t      powerMode  = COMP_POWER_HIGH_SPEED;
    comp_Polarity_t       polarity   = COMP_POLARITY_NORMAL;
    comp_Blanking_t       blanking   = COMP_BLANKING_NONE;
    comp_WindowMode_t     window     = COMP_WINDOW_MODE_INACTIVE;
    comp_Edge_t           edge       = COMP_EDGE_RISING;
    comp_Level_t          level      = COMP_LEVEL_LOW;
    comp_FunctionState_t  lockState  = COMP_FUNCTION_INACTIVE;
    comp_IrqPrio_t        irqPrio    = UT_COMP_IRQ_PRIO;

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_InputPlus( COMP_PERIPH_1, &inputPlus ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_InputMinus( COMP_PERIPH_1, &inputMinus ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Hysteresis( COMP_PERIPH_1, &hysteresis ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PowerMode( COMP_PERIPH_1, &powerMode ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Polarity( COMP_PERIPH_1, &polarity ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Blanking( COMP_PERIPH_1, &blanking ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_WindowMode( COMP_PERIPH_1, &window ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_TriggerEdge( COMP_PERIPH_1, &edge ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_LockState( COMP_PERIPH_1, &lockState ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_IrqPriority( COMP_PERIPH_1, &irqPrio ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_InputPlus( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_InputMinus( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Hysteresis( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PowerMode( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Polarity( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_Blanking( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_WindowMode( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_TriggerEdge( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_OutputLevel( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_LockState( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_IrqPriority( COMP_PERIPH_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* ============================== OUTPUT ==================================== */

/**
 * \brief   Comp_Get_OutputLevel() of a disabled and of a starting comparator.
 * \par Expected results
 * COMP_REQUEST_OK with COMP_LEVEL_INVALID in both states (the status register is not used).
 */
void Ut_Comp_Get_OutputLevel_DisabledOrStarting_ReturnsInvalid( void )
{
#if defined(COMP1)
    comp_Level_t level = COMP_LEVEL_LOW;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    *UtComp_Get_SrAddr() = UtComp_Get_SrValMask( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_INVALID, level );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );
    level = COMP_LEVEL_LOW;
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_INVALID, level );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Get_OutputLevel() of a comparator whose start-up is over.
 * \par Expected results
 * The output value bit of the status register (set / cleared by the test) is reported as COMP_LEVEL_HIGH /
 * COMP_LEVEL_LOW.
 */
void Ut_Comp_Get_OutputLevel_Ready_ReturnsOutputValue( void )
{
#if defined(COMP1)
    comp_Level_t level = COMP_LEVEL_INVALID;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    *UtComp_Get_SrAddr() = UtComp_Get_SrValMask( COMP_PERIPH_1 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_HIGH, level );

    *UtComp_Get_SrAddr() = UT_COMP_BITS_CLEAR;
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_LOW, level );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* ============================= INTERRUPT ================================== */

/**
 * \brief   Comp_Set_IrqActive() during the start-up only requests the interrupt, Comp_Task arms it at the end.
 * \par Expected results
 * No Exti call and no interrupt enable bit during the start-up (a Comp_Task call before the start-up time elapsed
 * included); the call after the start-up time (5 us) arms the source: configurable lines: pending flag cleared and the
 * line enabled with the rising edge; STM32H503: the flag clear bit written, ITEN set and the direct line enabled.
 */
void Ut_Comp_Set_IrqActive_Starting_ArmsAtEndOfStartup( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
    Comp_Task();
    UtComp_Advance_Time( UT_COMP_STARTUP_SCALER_US - 1u );
    Comp_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_ITEN );

    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
#if defined(STM32H503xx)
    *UtComp_Get_IcfrAddr() = UT_COMP_BITS_CLEAR;
#endif
    UtComp_Advance_Time( 1u );
    Comp_Task();
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_ITEN, COMP1->CFGR1 & COMP_CFGR1_ITEN );
    TEST_ASSERT_EQUAL_HEX32( COMP_ICFR_CC1IF, *UtComp_Get_IcfrAddr() );
#else
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_ITEN );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqActive() of a comparator whose start-up is over arms the source immediately.
 * \par Expected results
 * COMP_REQUEST_OK, the Exti calls of the arming happen inside the call.
 */
void Ut_Comp_Set_IrqActive_Ready_ArmsImmediately( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqActive() of a disabled, an uninitialized and an invalid instance.
 * \par Expected results
 * COMP_REQUEST_ERROR, no Exti call.
 */
void Ut_Comp_Set_IrqActive_DisabledOrInvalid_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqActive( COMP_PERIPH_CNT ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqActive() when the Exti rejects the line.
 * \par Expected results
 * COMP_REQUEST_ERROR, the interrupt is not requested (the end of a later start-up arms nothing).
 */
void Ut_Comp_Set_IrqActive_ExtiRejectsLine_ReturnsError( void )
{
#if defined(COMP1)
    static exti_LineConfig_t lineConfig;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    lineConfig.LineMode = EXTI_LINE_MODE_INTERRUPT;
#if defined(STM32H503xx)
    lineConfig.TriggerEdge = EXTI_TRIGGER_EDGE_NONE;
    Exti_Set_LineActive_ExpectAndReturn( EXTI_LINE_COMP1, &lineConfig, EXTI_REQUEST_ERROR );
#else
    lineConfig.TriggerEdge = EXTI_TRIGGER_EDGE_RAISING;
    Exti_Set_LinePendingClear_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), EXTI_REQUEST_OK );
    Exti_Set_LineActive_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), &lineConfig, EXTI_REQUEST_ERROR );
#endif

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqInactive() disarms an armed interrupt.
 * \par Expected results
 * The Exti line is deactivated (and its pending flag cleared / STM32H503: ITEN cleared and the flag clear bit
 * written), COMP_REQUEST_OK; after it the interrupt handler does not call the callback.
 */
void Ut_Comp_Set_IrqInactive_Armed_DisarmsSource( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );

    UtComp_Expect_Disarm( COMP_PERIPH_1 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqInactive( COMP_PERIPH_1 ) );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, COMP1->CFGR1 & COMP_CFGR1_ITEN );
#endif

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );
    *UtComp_Get_SrAddr() = COMP_SR_C1IF;
    Comp_IsrHandler();
    TEST_ASSERT_EQUAL_UINT32( 0u, utComp_CallbackCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqInactive() of an uninitialized instance.
 * \par Expected results
 * COMP_REQUEST_ERROR, no Exti call.
 */
void Ut_Comp_Set_IrqInactive_Uninitialized_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqInactive( COMP_PERIPH_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_TriggerEdge() changes the edge of the lines that have an edge selection (not STM32H503).
 * \par Expected results
 * Before the interrupt is armed only the stored edge changes (no Exti call); with the armed interrupt the line is
 * armed again with the falling edge; the getter returns the edge. An invalid edge returns COMP_REQUEST_ERROR.
 * Ignored on STM32H503.
 */
void Ut_Comp_Set_TriggerEdge_ConfigurableLine_ReArmsWithNewEdge( void )
{
#if defined(COMP1) && \
    !defined(STM32H503xx)
    comp_Edge_t edge = COMP_EDGE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_BOTH ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_TriggerEdge( COMP_PERIPH_1, &edge ) );
    TEST_ASSERT_EQUAL( COMP_EDGE_BOTH, edge );

    UtComp_Start_Instance( COMP_PERIPH_1 );
    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_BOTH );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );

    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_FALLING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_FALLING ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_TriggerEdge( COMP_PERIPH_1, &edge ) );
    TEST_ASSERT_EQUAL( COMP_EDGE_FALLING, edge );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator or STM32H503" );
#endif
}


/**
 * \brief   Comp_Set_TriggerEdge() of STM32H503 accepts only the rising edge.
 * \par Expected results
 * The falling and the both-edges selection return COMP_REQUEST_ERROR, the rising edge COMP_REQUEST_OK; the
 * stored edge stays rising. Ignored on the other lines.
 */
void Ut_Comp_Set_TriggerEdge_Stm32H503_AcceptsOnlyRising( void )
{
#if defined(STM32H503xx)
    comp_Edge_t edge = COMP_EDGE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_FALLING ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_BOTH ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_RISING ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_TriggerEdge( COMP_PERIPH_1, &edge ) );
    TEST_ASSERT_EQUAL( COMP_EDGE_RISING, edge );
#else
    TEST_IGNORE_MESSAGE( "MCU other than STM32H503" );
#endif
}


/**
 * \brief   The interrupt handler calls the callback with the instance and the output level when an edge is pending.
 * \par Expected results
 * Configurable lines: the pending edge of the Exti is read and cleared; STM32H503: the flag in the status register is
 * seen and cleared by the clear register and the enable bit of the interrupt (ITEN) is cleared, because the request is
 * the level of the output and is not cleared by the hardware (RM0492 22.5.2). The callback is called once with
 * COMP_PERIPH_1 and COMP_LEVEL_HIGH (the output value bit is set).
 */
void Ut_Comp_IsrHandler_EdgePending_CallsCallbackWithLevel( void )
{
#if defined(COMP1)
    static exti_TriggerEdge_t pendingEdge = EXTI_TRIGGER_EDGE_RAISING;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );

#if defined(STM32H503xx)
    (void)pendingEdge;
    *UtComp_Get_SrAddr() = COMP_SR_C1IF | COMP_SR_C1VAL;
    *UtComp_Get_IcfrAddr() = UT_COMP_BITS_CLEAR;
#else
    *UtComp_Get_SrAddr() = UtComp_Get_SrValMask( COMP_PERIPH_1 );
    Exti_Get_LinePending_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), NULL, EXTI_REQUEST_OK );
    Exti_Get_LinePending_IgnoreArg_pendingEdge();
    Exti_Get_LinePending_ReturnThruPtr_pendingEdge( &pendingEdge );
    Exti_Set_LinePendingClear_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), EXTI_REQUEST_OK );
#endif

    Comp_IsrHandler();

    TEST_ASSERT_EQUAL_UINT32( 1u, utComp_CallbackCnt );
    TEST_ASSERT_EQUAL( COMP_PERIPH_1, utComp_CallbackPeriph );
    TEST_ASSERT_EQUAL( COMP_LEVEL_HIGH, utComp_CallbackLevel );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_ICFR_CC1IF, *UtComp_Get_IcfrAddr() );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_ITEN );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The interrupt handler does nothing when no edge is pending.
 * \par Expected results
 * The callback is not called, nothing is cleared (configurable lines: the Exti reports no pending edge; STM32H503:
 * the flag of the status register is clear).
 */
void Ut_Comp_IsrHandler_NoEdgePending_DoesNotCallCallback( void )
{
#if defined(COMP1)
    static exti_TriggerEdge_t pendingEdge = EXTI_TRIGGER_EDGE_NONE;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );

#if defined(STM32H503xx)
    (void)pendingEdge;
    *UtComp_Get_SrAddr() = UT_COMP_BITS_CLEAR;
#else
    Exti_Get_LinePending_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), NULL, EXTI_REQUEST_OK );
    Exti_Get_LinePending_IgnoreArg_pendingEdge();
    Exti_Get_LinePending_ReturnThruPtr_pendingEdge( &pendingEdge );
#endif

    Comp_IsrHandler();

    TEST_ASSERT_EQUAL_UINT32( 0u, utComp_CallbackCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The interrupt handler ignores a comparator whose interrupt is not armed.
 * \par Expected results
 * No Exti call, the callback is not called (initialized, enabled, ready, interrupt not requested).
 */
void Ut_Comp_IsrHandler_NotArmed_DoesNothing( void )
{
#if defined(COMP1)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    *UtComp_Get_SrAddr() = COMP_SR_C1IF;

    Comp_IsrHandler();

    TEST_ASSERT_EQUAL_UINT32( 0u, utComp_CallbackCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The interrupt handler without a callback only clears the flag.
 * \par Expected results
 * No crash, the pending flag is cleared (configurable lines: Exti_Set_LinePendingClear).
 */
void Ut_Comp_IsrHandler_NoCallback_ClearsFlagOnly( void )
{
#if defined(COMP1)
    static exti_TriggerEdge_t pendingEdge = EXTI_TRIGGER_EDGE_FALLING;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );
    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );

#if defined(STM32H503xx)
    (void)pendingEdge;
    *UtComp_Get_SrAddr() = COMP_SR_C1IF;
    *UtComp_Get_IcfrAddr() = UT_COMP_BITS_CLEAR;
#else
    Exti_Get_LinePending_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), NULL, EXTI_REQUEST_OK );
    Exti_Get_LinePending_IgnoreArg_pendingEdge();
    Exti_Get_LinePending_ReturnThruPtr_pendingEdge( &pendingEdge );
    Exti_Set_LinePendingClear_ExpectAndReturn( UtComp_Get_ExtiLine( COMP_PERIPH_1 ), EXTI_REQUEST_OK );
#endif

    Comp_IsrHandler();

#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_ICFR_CC1IF, *UtComp_Get_IcfrAddr() );
#endif
    TEST_ASSERT_EQUAL_UINT32( 0u, utComp_CallbackCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqCallback() stores or removes the callback and rejects an uninitialized instance.
 * \par Expected results
 * COMP_REQUEST_OK for a callback and for NULL on an initialized instance, COMP_REQUEST_ERROR otherwise.
 */
void Ut_Comp_Set_IrqCallback_NullAndUninitialized_Handled( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, UtComp_IrqCallback ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqCallback( COMP_PERIPH_1, NULL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqCallback( COMP_PERIPH_CNT, UtComp_IrqCallback ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_IrqPriority() and Comp_Get_IrqPriority() forward to the Nvic.
 * \par Expected results
 * The priority of the comparator vector is set and read through the Nvic module; an uninitialized instance and a
 * rejection of the Nvic return COMP_REQUEST_ERROR.
 */
void Ut_Comp_IrqPriority_SetAndGet_ForwardToNvic( void )
{
#if defined(COMP1)
    static comp_IrqPrio_t irqPrio = UT_COMP_OTHER_IRQ_PRIO;
    comp_IrqPrio_t        readPrio = 0u;

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqPriority( COMP_PERIPH_1, UT_COMP_OTHER_IRQ_PRIO ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_OTHER_IRQ_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqPriority( COMP_PERIPH_1, UT_COMP_OTHER_IRQ_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_OTHER_IRQ_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_IrqPriority( COMP_PERIPH_1, UT_COMP_OTHER_IRQ_PRIO ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &irqPrio );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_IrqPriority( COMP_PERIPH_1, &readPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_COMP_OTHER_IRQ_PRIO, readPrio );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}

/* ================================ LOCK ==================================== */

/**
 * \brief   Comp_Set_Lock() locks an enabled comparator.
 * \par Expected results
 * The LOCK bit is set (STM32H503: in both configuration registers), the state is LOCKED, the lock state is active; a
 * second call returns COMP_REQUEST_ERROR.
 */
void Ut_Comp_Set_Lock_Enabled_LocksConfiguration( void )
{
#if defined(COMP1)
    comp_State_t         state     = COMP_STATE_CNT;
    comp_FunctionState_t lockState = COMP_FUNCTION_INACTIVE;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Lock( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_LOCK, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_LOCK );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL_HEX32( COMP_CFGR1_LOCK, COMP1->CFGR2 & COMP_CFGR1_LOCK );
#endif
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_LOCKED, state );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_LockState( COMP_PERIPH_1, &lockState ) );
    TEST_ASSERT_EQUAL( COMP_FUNCTION_ACTIVE, lockState );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Lock( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_Lock() of a disabled and of an uninitialized comparator.
 * \par Expected results
 * COMP_REQUEST_ERROR, the LOCK bit stays clear (a locked disabled comparator could never be enabled).
 */
void Ut_Comp_Set_Lock_DisabledOrUninitialized_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Lock( COMP_PERIPH_1 ) );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Lock( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) & COMP_CFGR1_LOCK );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_Lock() when the lock bit is not kept by the register (write ignored).
 * \par Expected results
 * COMP_REQUEST_ERROR and the lock state of the module stays inactive.
 */
void Ut_Comp_Set_Lock_ReadBackFails_ReturnsError( void )
{
#if defined(COMP1)
    comp_FunctionState_t lockState = COMP_FUNCTION_ACTIVE;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtComp_HwModel_IgnoreWrites ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Lock( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_LockState( COMP_PERIPH_1, &lockState ) );
    TEST_ASSERT_EQUAL( COMP_FUNCTION_INACTIVE, lockState );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The edge interrupt can be armed on a locked comparator of the lines with a configurable EXTI line.
 * \par Expected results
 * Comp_Set_IrqActive returns COMP_REQUEST_OK (the EXTI line is outside the lock) and arms the line. Ignored on
 * STM32H503 (the interrupt enable bit is a locked bit).
 */
void Ut_Comp_Set_IrqActive_LockedConfigurableLine_ArmsLine( void )
{
#if defined(COMP1) && \
    !defined(STM32H503xx)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_ACTIVE );
    UtComp_Start_Instance( COMP_PERIPH_1 );

    UtComp_Expect_Arm( COMP_PERIPH_1, EXTI_TRIGGER_EDGE_RAISING );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator or STM32H503" );
#endif
}

/* ============================ WINDOW MODE ================================= */

/**
 * \brief   Comp_Set_WindowMode() on COMP_PERIPH_1 sets the window bits of the pair.
 * \par Expected results
 * COMP2 gets WINMODE and COMP1 gets WINOUT; both getters report ACTIVE; INACTIVE clears both bits. Ignored on the line
 * with one comparator.
 */
void Ut_Comp_Set_WindowMode_ActiveAndInactive_WritesPairBits( void )
{
#if defined(COMP2)
    comp_Config_t     config = UtComp_Get_Config( COMP_PERIPH_2 );
    comp_WindowMode_t mode   = COMP_WINDOW_MODE_CNT;

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_WINOUT ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_WINMODE ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_WindowMode( COMP_PERIPH_1, &mode ) );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_ACTIVE, mode );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_WindowMode( COMP_PERIPH_2, &mode ) );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_ACTIVE, mode );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_WindowMode() rejects the second comparator, an enabled pair, a missing partner and a bad mode.
 * \par Expected results
 * COMP_REQUEST_ERROR in all cases, the registers are unchanged. Ignored on the line with one comparator.
 */
void Ut_Comp_Set_WindowMode_InvalidCases_ReturnError( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_WindowMode( COMP_PERIPH_2, COMP_WINDOW_MODE_ACTIVE ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_CNT ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );

    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_EN ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   The non-inverting input of COMP2 cannot be changed while the window mode is active.
 * \par Expected results
 * Comp_Set_InputPlus( COMP_PERIPH_2, ... ) returns COMP_REQUEST_ERROR (COMP2 takes the input of COMP1), COMP1 can
 * still change its input. Ignored on the line with one comparator.
 */
void Ut_Comp_Set_InputPlus_Comp2InWindowMode_ReturnsError( void )
{
#if defined(COMP2)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_2 );

    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_InputPlus( COMP_PERIPH_2, COMP_INPUT_PLUS_COMP2_IO2 ) );

    UtComp_Expect_ItemPin( COMP_INPUT_DECODE_PORT( COMP_INPUT_PLUS_COMP1_IO2 ),
                           COMP_INPUT_DECODE_PIN( COMP_INPUT_PLUS_COMP1_IO2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputPlus( COMP_PERIPH_1, COMP_INPUT_PLUS_COMP1_IO2 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() of COMP1 with the window mode requires COMP2 to be initialized, Comp_Deinit() clears the mode.
 * \par Expected results
 * Without COMP2 the Init returns COMP_REQUEST_ERROR (no mock call); with COMP2 initialized first it succeeds and the
 * window bits are set; Comp_Deinit( COMP_PERIPH_1 ) clears them again. Ignored on the line with one comparator.
 */
void Ut_Comp_Init_WindowMode_NeedsSecondComparator( void )
{
#if defined(COMP2)
    comp_Config_t     config2 = UtComp_Get_Config( COMP_PERIPH_2 );
    comp_Config_t     config1 = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_WindowMode_t mode    = COMP_WINDOW_MODE_CNT;

    config1.WindowMode = COMP_WINDOW_MODE_ACTIVE;
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config1 ) );

    UtComp_Expect_SharedInit();
    UtComp_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_0 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config2 ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( COMP_NVIC_IRQ, UT_COMP_IRQ_PRIO, NVIC_REQUEST_OK );
    UtComp_Expect_AnalogPin( UT_COMP1_IO1_PORT, UT_COMP1_IO1_PIN );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_WINOUT ), *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_COMP_DEFAULT_CFGR | COMP_CFGR1_WINMODE ), *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Deinit( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_DEFAULT_CFGR, *UtComp_Get_CfgrAddr( COMP_PERIPH_2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_WindowMode( COMP_PERIPH_2, &mode ) );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_INACTIVE, mode );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   Comp_Set_WindowMode() of the line with one comparator.
 * \par Expected results
 * COMP_REQUEST_ERROR (a function without a meaning keeps its signature). Ignored on the other lines.
 */
void Ut_Comp_Set_WindowMode_OneComparatorLine_ReturnsError( void )
{
#if defined(COMP1) && \
    !defined(COMP2)
    UtComp_Init_Instance( COMP_PERIPH_1, COMP_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with two comparators or without a comparator" );
#endif
}

/* ================================ PINS ==================================== */

/**
 * \brief   Comp_InitInputPlusGpio() configures the pin of an input as analog pin.
 * \par Expected results
 * Gpio_Init is called once with the analog mode, no pull and the port and the pin of the first input of COMP1.
 */
void Ut_Comp_InitInputPlusGpio_PinInput_ConfiguresAnalogPin( void )
{
#if defined(COMP1)
    UtComp_Expect_AnalogPin( UT_COMP1_IO1_PORT, UT_COMP1_IO1_PIN );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_InitInputPlusGpio( COMP_INPUT_PLUS_COMP1_IO1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_InitInputPlusGpio() and Comp_InitInputMinusGpio() of the inputs without a pin.
 * \par Expected results
 * The DAC input, the internal reference and the unused items return COMP_REQUEST_ERROR, no Gpio call.
 */
void Ut_Comp_InitInputGpio_InternalInput_ReturnsError( void )
{
#if defined(COMP1)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputPlusGpio( COMP_INPUT_PLUS_COMP1_DAC1_CH1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputPlusGpio( COMP_INPUT_PLUS_UNUSED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_VREFINT ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_DAC1_CH1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_InitInputMinusGpio() configures the pin of an inverting input where the item holds a pin.
 * \par Expected results
 * STM32H503: PC4, STM32H5E / H5F: PA6 configured as analog pin; STM32H543 / H553: the item holds no pin, the call
 * returns COMP_REQUEST_ERROR without a Gpio call.
 */
void Ut_Comp_InitInputMinusGpio_PinInput_ConfiguresAnalogPin( void )
{
#if defined(COMP1)
#if defined(STM32H503xx)
    UtComp_Expect_AnalogPin( GPIO_PORT_C, GPIO_PIN_ID_4 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_COMP1_IO1 ) );
#elif defined(STM32H543xx) || \
      defined(STM32H553xx)
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_COMP1_IO1 ) );
#else
    UtComp_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_6 );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_InitInputMinusGpio( COMP_INPUT_MINUS_COMP1_IO1 ) );
#endif
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_InitOutputGpio() configures the output pin in the alternate function mode.
 * \par Expected results
 * Gpio_Init is called once with the alternate function mode, high speed, push-pull, no pull and the port, the pin and
 * the alternate function of the first output pin of the line; the unused item returns COMP_REQUEST_ERROR.
 */
void Ut_Comp_InitOutputGpio_PinItem_ConfiguresAlternateFunction( void )
{
#if defined(COMP1)
    UtComp_Expect_AltFuncPin( UT_COMP_OUT_PORT, UT_COMP_OUT_PINID, UT_COMP_OUT_AF );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_InitOutputGpio( UT_COMP_OUT_PIN ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_InitOutputGpio( COMP_OUT_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   Comp_Init() when the Gpio module rejects the pin of the input.
 * \par Expected results
 * COMP_REQUEST_ERROR, the instance is deinitialized (vector deactivated, clock disabled, registers cleared).
 */
void Ut_Comp_Init_GpioRejectsPin_DeinitializesInstance( void )
{
#if defined(COMP1)
    comp_Config_t config = UtComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;

    UtComp_Expect_SharedInit();
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( COMP_NVIC_IRQ, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_COMP, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_COMP_BITS_CLEAR, *UtComp_Get_CfgrAddr( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}
