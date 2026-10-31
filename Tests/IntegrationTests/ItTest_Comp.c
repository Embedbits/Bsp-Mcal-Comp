/**
 * \author Mr.Nobody
 * \file ItTest_Comp.c
 * \ingroup Comp
 * \brief Integration tests of the analog comparator (COMP) module on target.
 *
 * Comp module runs on the MCU with the real COMP registers and the real Rcc, Nvic, Exti and Gpio modules.
 * The tests verify what the unit tests cannot (emulated registers): the start-up of the comparator, the
 * settings against the real configuration register, the hardware lock, the window mode bits and the
 * arming of the EXTI line. Every test case runs after a system reset of the target.
 *
 * Used resources:
 * - COMP1 (and COMP2 on the lines that have it), the analog input pin of the default input of the
 *   instance (configured as analog pin by the module) and the internal reference VREFINT
 * - the level test uses the fixture feature ADC_DIVIDERS (resistor divider on PA0, about 0.318 VDDA)
 *   and runs on STM32H503 only (PA0 is the input IO2 of COMP1 there); a board without the fixture ignores it
 * - no output pin is configured (a free pin cannot be assumed on every board)
 * - the interrupt tests arm the EXTI line and disarm it again, no interrupt is awaited
 *
 * \note The tests run on the STM32H503 board of the IT agent (the only tested line with a comparator). The firmware
 *       starts the cycle counter of the Dwt module on every boot (ItTarget_Init), Comp_Init needs it.
 * \note The lines without a comparator (STM32H523, H533, H562, H563, H573) ignore every test case except the
 *       version test; the generated test runner is not preprocessor aware, so every test case function is
 *       defined unconditionally.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Comp_Port.h"                      /* Module under test              */
#include "Stm32.h"                          /* COMP registers (hardware lock test) */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(COMP1)
static comp_Config_t ItComp_Get_Config    ( comp_PeriphId_t periphId );
static comp_State_t  ItComp_Run_Startup   ( comp_PeriphId_t periphId );
#endif
#if defined(STM32H503xx)
static void          ItComp_RequireDividers ( void );
#endif

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Bound of the Comp_Task calls that run the start-up of a comparator (the start-up of the scaler, 250 us, takes
 *  several thousand calls on the fastest core clock) */
#define IT_COMP_STARTUP_LOOPS               ( 100000u )

/** Interrupt priority used by the tests */
#define IT_COMP_IRQ_PRIO                    ( 5u )

/* ========================== LOCAL VARIABLES =============================== */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* Nothing to reset before a test case (fresh boot, see ItTarget_SystemReset) */
}


void tearDown( void )
{
    /* Peripheral state is cleared by the system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief   Comp_Get_ModuleVersion() returns the version of the module on target.
 *
 * \par Expected results
 * Version 1.0.0.
 */
void It_Comp_Get_ModuleVersion_ReturnsVersion( void )
{
    const comp_ModuleVersion_t version = Comp_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   The default configuration initializes, enables and starts up the first comparator.
 *
 * \par Expected results
 * Comp_Init returns COMP_REQUEST_OK and the state is COMP_STATE_DISABLED; after Comp_Set_PeriphActive and
 * the Comp_Task calls the state is COMP_STATE_READY and the output level is valid; Comp_Deinit returns
 * COMP_REQUEST_OK.
 */
void It_Comp_Init_DefaultConfig_StartsUpToReady( void )
{
#if defined(COMP1)
    comp_Config_t config = ItComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;
    comp_Level_t  level  = COMP_LEVEL_INVALID;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_DISABLED, state );

    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_NOT_EQUAL( COMP_LEVEL_INVALID, level );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Deinit( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The settings of a disabled comparator are written to the real configuration register and read back.
 *
 * \par Expected results
 * Hysteresis, power mode, polarity and blanking are set and the getters return them; the same calls on the
 * enabled comparator return COMP_REQUEST_ERROR and change nothing.
 */
void It_Comp_Settings_Roundtrip_ReadsBackSettings( void )
{
#if defined(COMP1)
    comp_Config_t     config     = ItComp_Get_Config( COMP_PERIPH_1 );
    comp_Hysteresis_t hysteresis = COMP_HYSTERESIS_NONE;
    comp_PowerMode_t  powerMode  = COMP_POWER_HIGH_SPEED;
    comp_Polarity_t   polarity   = COMP_POLARITY_NORMAL;
    comp_Blanking_t   blanking   = COMP_BLANKING_NONE;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_MEDIUM ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PowerMode( COMP_PERIPH_1, COMP_POWER_MEDIUM ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_INVERTED ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Blanking( COMP_PERIPH_1, COMP_BLANKING_TIM2_OC3 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Hysteresis( COMP_PERIPH_1, &hysteresis ) );
    TEST_ASSERT_EQUAL( COMP_HYSTERESIS_MEDIUM, hysteresis );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PowerMode( COMP_PERIPH_1, &powerMode ) );
    TEST_ASSERT_EQUAL( COMP_POWER_MEDIUM, powerMode );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Polarity( COMP_PERIPH_1, &polarity ) );
    TEST_ASSERT_EQUAL( COMP_POLARITY_INVERTED, polarity );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Blanking( COMP_PERIPH_1, &blanking ) );
    TEST_ASSERT_EQUAL( COMP_BLANKING_TIM2_OC3, blanking );

    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_Hysteresis( COMP_PERIPH_1, COMP_HYSTERESIS_HIGH ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_Hysteresis( COMP_PERIPH_1, &hysteresis ) );
    TEST_ASSERT_EQUAL( COMP_HYSTERESIS_MEDIUM, hysteresis );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The hardware lock freezes the configuration register until the reset.
 *
 * \par Expected results
 * After Comp_Set_Lock the state is COMP_STATE_LOCKED, Comp_Set_PeriphInactive and Comp_Deinit return
 * COMP_REQUEST_ERROR and a direct write of the configuration register does not change it (the hardware
 * lock); the configuration register holds the same value before and after the write.
 */
void It_Comp_Lock_FreezesConfigurationRegister( void )
{
#if defined(COMP1)
    comp_Config_t config = ItComp_Get_Config( COMP_PERIPH_1 );
    comp_State_t  state  = COMP_STATE_CNT;
    uint32_t      before = 0u;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Lock( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( COMP_PERIPH_1, &state ) );
    TEST_ASSERT_EQUAL( COMP_STATE_LOCKED, state );

    before        = COMP1->CFGR1;
    COMP1->CFGR1 &= ~COMP_CFGR1_EN;
    COMP1->CFGR1 |= COMP_CFGR1_HYST;
    TEST_ASSERT_EQUAL_HEX32( before, COMP1->CFGR1 );

    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_ERROR, Comp_Deinit( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The edge interrupt is armed and disarmed through the EXTI line of the comparator.
 *
 * \par Expected results
 * Comp_Set_IrqActive on the started comparator and Comp_Set_IrqInactive return COMP_REQUEST_OK; the edge
 * selection (both edges, STM32H503: rising only) is accepted by the line; no interrupt is awaited.
 */
void It_Comp_Irq_ArmAndDisarm_ReturnsOk( void )
{
#if defined(COMP1)
    comp_Config_t config = ItComp_Get_Config( COMP_PERIPH_1 );

    config.IrqPriority = IT_COMP_IRQ_PRIO;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );
#if defined(STM32H503xx)
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_RISING ) );
#else
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_TriggerEdge( COMP_PERIPH_1, COMP_EDGE_BOTH ) );
#endif
    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqActive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_IrqInactive( COMP_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without a comparator" );
#endif
}


/**
 * \brief   The window mode of the pair sets and clears the window bits of the two comparators.
 *
 * \par Expected results
 * Both comparators initialized: Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) returns
 * COMP_REQUEST_OK and the getter reports the active window mode; the inactive mode restores it. Ignored on the
 * lines with one comparator.
 */
void It_Comp_WindowMode_PairRoundtrip_ReadsBackMode( void )
{
#if defined(COMP2)
    comp_Config_t     config2 = ItComp_Get_Config( COMP_PERIPH_2 );
    comp_Config_t     config1 = ItComp_Get_Config( COMP_PERIPH_1 );
    comp_WindowMode_t mode    = COMP_WINDOW_MODE_CNT;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config2 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config1 ) );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_ACTIVE ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_WindowMode( COMP_PERIPH_2, &mode ) );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_ACTIVE, mode );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_WindowMode( COMP_PERIPH_1, COMP_WINDOW_MODE_INACTIVE ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_WindowMode( COMP_PERIPH_2, &mode ) );
    TEST_ASSERT_EQUAL( COMP_WINDOW_MODE_INACTIVE, mode );
#else
    TEST_IGNORE_MESSAGE( "MCU with one comparator or without a comparator" );
#endif
}


/**
 * \brief   The divider on PA0 (about 1.05 V at 3.3 V) is compared with the fractions of the internal reference.
 *
 * \details Fixture feature ADC_DIVIDERS (PA0, 10k / 4.7k). STM32H503 only (PA0 is the input IO2 of COMP1).
 *
 * \par Expected results
 * Against 3/4 of VREFINT (about 0.9 V) the output is COMP_LEVEL_HIGH, against VREFINT (about 1.2 V) it is
 * COMP_LEVEL_LOW; the inverted polarity against 3/4 of VREFINT gives COMP_LEVEL_LOW.
 */
void It_Comp_Output_DividerAgainstVrefint_ReportsLevels( void )
{
#if defined(STM32H503xx)
    comp_Config_t config = ItComp_Get_Config( COMP_PERIPH_1 );
    comp_Level_t  level  = COMP_LEVEL_INVALID;

    ItComp_RequireDividers();

    config.InputPlus  = COMP_INPUT_PLUS_COMP1_IO2;
    config.InputMinus = COMP_INPUT_MINUS_VREFINT_THREE_QUARTERS;
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Init( &config ) );

    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_HIGH, level );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_INVERTED ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_LOW, level );

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphInactive( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_Polarity( COMP_PERIPH_1, COMP_POLARITY_NORMAL ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_InputMinus( COMP_PERIPH_1, COMP_INPUT_MINUS_VREFINT ) );
    TEST_ASSERT_EQUAL( COMP_STATE_READY, ItComp_Run_Startup( COMP_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_OutputLevel( COMP_PERIPH_1, &level ) );
    TEST_ASSERT_EQUAL( COMP_LEVEL_LOW, level );
#else
    TEST_IGNORE_MESSAGE( "MCU other than STM32H503 (the divider on PA0 is an input of COMP1 there)" );
#endif
}

/* ========================== LOCAL FUNCTIONS =============================== */

#if defined(COMP1)
/**
 * \brief   Default configuration of an instance (the input IO1 of the instance against the internal reference).
 *
 * \param   periphId [in] : Instance identification.
 *
 * \return  Configuration structure.
 */
static comp_Config_t ItComp_Get_Config( comp_PeriphId_t periphId )
{
    comp_Config_t config;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_DefaultConfig( &config ) );
    config.PeriphId = periphId;

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


/**
 * \brief   Enables the comparator and calls Comp_Task until the start-up is over (bounded).
 *
 * \param   periphId [in] : Instance identification.
 *
 * \return  State of the comparator after the loop.
 */
static comp_State_t ItComp_Run_Startup( comp_PeriphId_t periphId )
{
    comp_State_t state = COMP_STATE_DISABLED;

    TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Set_PeriphActive( periphId ) );

    for( uint32_t idx = 0u;
         ( IT_COMP_STARTUP_LOOPS > idx ) &&
         ( COMP_STATE_READY != state );
         idx++ )
    {
        Comp_Task();
        TEST_ASSERT_EQUAL( COMP_REQUEST_OK, Comp_Get_PeriphState( periphId, &state ) );
    }

    return ( state );
}
#endif /* COMP1 */

#if defined(STM32H503xx)
/**
 * \brief   Ignores the test on a board without the fixture ADC_DIVIDERS (PA0 / PC0 resistor dividers).
 *
 * \details A board of the inventory without a fixture (IT_FIXTURE defined, the feature not listed) ignores
 *          the test, a board without any inventory (IT_FIXTURE not defined) runs it.
 */
static void ItComp_RequireDividers( void )
{
#if defined(IT_FEATURE_ADC_DIVIDERS) || \
    !defined(IT_FIXTURE)
    /* The divider on PA0 is wired (or the board is not in the inventory), the test runs */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture ADC_DIVIDERS (divider on PA0)" );
#endif
}
#endif /* STM32H503xx */
