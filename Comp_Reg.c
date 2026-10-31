/**
 * \author Mr.Nobody
 * \file Comp_Reg.c
 * \ingroup Comp
 * \brief Comp module Reg component source file
 *
 * This component contains the functionality for register operations of the
 * multi-instance COMP peripheral: every accessor takes the instance
 * identification in addition to the register identification and addresses the
 * register of that one instance. STM32H503 has one register block (SR, ICFR,
 * CFGR1, CFGR2), the other lines share SR and ICFR of both comparators and have
 * the CFGR1 register of every comparator at its own address.
 *
 * The lines without a comparator (no COMP1 in the CMSIS header) have no register
 * table: every accessor keeps its signature, suppresses the unused parameters and
 * returns the module error / "no action" value.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Comp_Reg.h"                       /* Module Reg component header    */
#include "Stm32.h"                          /* COMP registers definition      */
/* ============================= TYPEDEFS =================================== */

/** \brief Structure type used to store the physical address of a COMP register. The registers
 *         are referenced through the \ref comp_RegId_t enumeration. */
typedef struct
{
    comp_RegId_t        RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   comp_RegList_t;

/** \brief Structure type binding one bit of the non-inverting input selection code to its register bit */
typedef struct
{
    comp_RegId_t    RegId;   /**< Register holding the selection bit  */
    comp_RegValue_t RegMask; /**< Bit mask of the selection bit       */
    comp_RegValue_t CodeBit; /**< Bit of the selection code           */
}   comp_PlusBit_t;

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Register value without any bit set */
#define COMP_REG_BITS_CLEAR                 ( 0u )

/** \brief Count of read-back iterations of a wait on a register value */
#define COMP_REG_READBACK_LIMIT             ( 1000u )

/* =========================== LOCAL VARIABLES ============================== */

#if defined(COMP1)

/** \brief COMP registers of every available instance, indexed by comp_PeriphId_t and comp_RegId_t */
static const comp_RegList_t comp_RegList[ COMP_PERIPH_CNT ][ COMP_REG_CNT ] =
{
#if defined(STM32H503xx)
    {
        { .RegId = COMP_REG_SR   , .RegAddr = &(COMP1->SR   ) }, /**< COMP1 status register              */
        { .RegId = COMP_REG_ICFR , .RegAddr = &(COMP1->ICFR ) }, /**< COMP1 interrupt clear register     */
        { .RegId = COMP_REG_CFGR , .RegAddr = &(COMP1->CFGR1) }, /**< COMP1 configuration register 1     */
        { .RegId = COMP_REG_CFGR2, .RegAddr = &(COMP1->CFGR2) }, /**< COMP1 configuration register 2     */
    },
#else
    {
        { .RegId = COMP_REG_SR   , .RegAddr = &(COMP12->SR  ) }, /**< COMP1 / COMP2 status register          */
        { .RegId = COMP_REG_ICFR , .RegAddr = &(COMP12->ICFR) }, /**< COMP1 / COMP2 interrupt clear register */
        { .RegId = COMP_REG_CFGR , .RegAddr = &(COMP1->CFGR1) }, /**< COMP1 configuration register           */
    },
#endif
#if defined(COMP2)
    {
        { .RegId = COMP_REG_SR   , .RegAddr = &(COMP12->SR  ) }, /**< COMP1 / COMP2 status register          */
        { .RegId = COMP_REG_ICFR , .RegAddr = &(COMP12->ICFR) }, /**< COMP1 / COMP2 interrupt clear register */
        { .RegId = COMP_REG_CFGR , .RegAddr = &(COMP2->CFGR1) }, /**< COMP2 configuration register           */
    },
#endif
};

_Static_assert( ( sizeof( comp_RegList ) / sizeof( comp_RegList[0] ) ) == COMP_PERIPH_CNT,
                 "Comp_Reg: comp_RegList has incorrect instance count." );

_Static_assert( ( sizeof( comp_RegList[0] ) / sizeof( comp_RegList_t ) ) == COMP_REG_CNT,
                 "Comp_Reg: comp_RegList has incorrect register count." );

/** \brief Bits of the non-inverting input selection code, with the register bit that holds them */
static const comp_PlusBit_t comp_PlusBit[] =
{
#if defined(STM32H503xx)
    { .RegId = COMP_REG_CFGR2, .RegMask = COMP_CFGR2_INPSEL0, .CodeBit = COMP_SEL_PLUS_INPSEL0 },
    { .RegId = COMP_REG_CFGR , .RegMask = COMP_CFGR1_INPSEL1, .CodeBit = COMP_SEL_PLUS_INPSEL1 },
    { .RegId = COMP_REG_CFGR , .RegMask = COMP_CFGR1_INPSEL2, .CodeBit = COMP_SEL_PLUS_INPSEL2 },
#else
    { .RegId = COMP_REG_CFGR , .RegMask = COMP_CFGR1_INPSEL0, .CodeBit = COMP_SEL_PLUS_INPSEL0 },
    { .RegId = COMP_REG_CFGR , .RegMask = COMP_CFGR1_INPSEL1, .CodeBit = COMP_SEL_PLUS_INPSEL1 },
#endif
};

/** \brief Count of the bits of the non-inverting input selection code */
#define COMP_PLUS_BIT_CNT                   ( sizeof( comp_PlusBit ) / sizeof( comp_PlusBit_t ) )

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Looks up the address of the register of an instance / register pair
 *
 * \param periphId [in] : Instance identification.
 * \param regId    [in] : Register identification.
 *
 * \return Address of the register, or COMP_NULL_PTR for an invalid parameter.
 */
static volatile uint32_t * Comp_Reg_GetAddr( comp_PeriphId_t periphId, comp_RegId_t regId )
{
    volatile uint32_t *regAddr = COMP_NULL_PTR;

    if( ( COMP_PERIPH_CNT > periphId ) &&
        ( COMP_REG_CNT    > regId    )    )
    {
        regAddr = comp_RegList[periphId][regId].RegAddr;
    }
    else
    {
        /* Invalid instance or register ID, no address is returned */
        regAddr = COMP_NULL_PTR;
    }

    return ( regAddr );
}
#endif /* COMP1 */

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Sets the register bit(s) with the given mask.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Comp_Set_RegBit( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask )
{
#if defined(COMP1)
    volatile uint32_t *regAddr = Comp_Reg_GetAddr( periphId, regId );

    if( COMP_NULL_PTR != regAddr )
    {
        *regAddr |= bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
#else
    (void)periphId;
    (void)regId;
    (void)bitMask;
#endif /* COMP1 */

    return;
}


/**
 * \brief Resets the register bit(s) with the given mask.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Comp_Reset_RegBit( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask )
{
#if defined(COMP1)
    volatile uint32_t *regAddr = Comp_Reg_GetAddr( periphId, regId );

    if( COMP_NULL_PTR != regAddr )
    {
        *regAddr &= ~bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
#else
    (void)periphId;
    (void)regId;
    (void)bitMask;
#endif /* COMP1 */

    return;
}


/**
 * \brief Returns the register bit(s) with the given mask.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param bitMask  [in] : Mask to apply to the register
 *
 * \return Masked register value, zero for an invalid instance or register ID.
 */
comp_RegValue_t Comp_Get_RegBit( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t bitMask )
{
    comp_RegValue_t regValue = COMP_REG_BITS_CLEAR;

#if defined(COMP1)
    volatile uint32_t *regAddr = Comp_Reg_GetAddr( periphId, regId );

    if( COMP_NULL_PTR != regAddr )
    {
        regValue = *regAddr & bitMask;
    }
    else
    {
        /* Invalid instance or register ID, regValue stays cleared */
    }
#else
    (void)periphId;
    (void)regId;
    (void)bitMask;
#endif /* COMP1 */

    return ( regValue );
}


/**
 * \brief Writes a value to the bits selected by the mask, the other bits are kept.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param regMask  [in] : Mask of the bits to modify
 * \param regValue [in] : Value of the bits to modify (already in the position of the mask)
 */
void Comp_Set_RegVal( comp_PeriphId_t periphId,
                      comp_RegId_t regId,
                      comp_RegValue_t regMask,
                      comp_RegValue_t regValue )
{
#if defined(COMP1)
    volatile uint32_t *regAddr = Comp_Reg_GetAddr( periphId, regId );

    if( COMP_NULL_PTR != regAddr )
    {
        *regAddr = ( *regAddr & ~regMask ) | ( regValue & regMask );
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
#else
    (void)periphId;
    (void)regId;
    (void)regMask;
    (void)regValue;
#endif /* COMP1 */

    return;
}


/**
 * \brief Returns the bits of the register selected by the mask.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param regMask  [in] : Mask of the bits to read
 *
 * \return Masked register value (still in the position of the mask), zero for an invalid parameter.
 */
comp_RegValue_t Comp_Get_RegVal( comp_PeriphId_t periphId, comp_RegId_t regId, comp_RegValue_t regMask )
{
    return ( Comp_Get_RegBit( periphId, regId, regMask ) );
}


/**
 * \brief Waits (bounded) until the bits of the register selected by the mask have the given value.
 *
 * \param periphId [in] : COMP instance ID
 * \param regId    [in] : COMP register ID
 * \param regMask  [in] : Mask of the bits to compare
 * \param regValue [in] : Expected value of the masked bits
 *
 * \return COMP_REQUEST_OK when the value was read back, COMP_REQUEST_ERROR on a timeout or an invalid parameter.
 */
comp_RequestState_t Comp_Reg_Wait_RegVal( comp_PeriphId_t periphId,
                                          comp_RegId_t regId,
                                          comp_RegValue_t regMask,
                                          comp_RegValue_t regValue )
{
    comp_RequestState_t retState = COMP_REQUEST_ERROR;

#if defined(COMP1)
    volatile uint32_t *regAddr = Comp_Reg_GetAddr( periphId, regId );

    if( COMP_NULL_PTR != regAddr )
    {
        for( uint32_t idx = 0u;
             ( COMP_REG_READBACK_LIMIT > idx ) &&
             ( COMP_REQUEST_OK != retState );
             idx++ )
        {
            const comp_RegValue_t readBack = *regAddr & regMask;

            if( ( regValue & regMask ) == readBack )
            {
                retState = COMP_REQUEST_OK;
            }
            else
            {
                /* Value not read back yet, no action required */
            }
        }
    }
    else
    {
        /* Invalid instance or register ID, retState stays COMP_REQUEST_ERROR */
    }
#else
    (void)periphId;
    (void)regId;
    (void)regMask;
    (void)regValue;
#endif /* COMP1 */

    return ( retState );
}


/**
 * \brief Writes the selection code of the non-inverting input to the INPSEL bits.
 *
 * \param periphId [in] : COMP instance ID
 * \param selCode  [in] : Selection code (bit 0 INPSEL0, bit 1 INPSEL1, bit 2 INPSEL2)
 */
void Comp_Reg_Set_InputPlusSel( comp_PeriphId_t periphId, comp_RegValue_t selCode )
{
#if defined(COMP1)
    for( uint32_t idx = 0u; COMP_PLUS_BIT_CNT > idx; idx++ )
    {
        comp_RegValue_t bitValue = COMP_REG_BITS_CLEAR;

        if( COMP_REG_BITS_CLEAR != ( selCode & comp_PlusBit[idx].CodeBit ) )
        {
            bitValue = comp_PlusBit[idx].RegMask;
        }
        else
        {
            /* The selection bit is cleared, bitValue stays cleared */
        }

        Comp_Set_RegVal( periphId, comp_PlusBit[idx].RegId, comp_PlusBit[idx].RegMask, bitValue );
    }
#else
    (void)periphId;
    (void)selCode;
#endif /* COMP1 */

    return;
}


/**
 * \brief Reads the selection code of the non-inverting input from the INPSEL bits.
 *
 * \param periphId [in] : COMP instance ID
 *
 * \return Selection code (bit 0 INPSEL0, bit 1 INPSEL1, bit 2 INPSEL2), zero for an invalid instance.
 */
comp_RegValue_t Comp_Reg_Get_InputPlusSel( comp_PeriphId_t periphId )
{
    comp_RegValue_t selCode = COMP_REG_BITS_CLEAR;

#if defined(COMP1)
    for( uint32_t idx = 0u; COMP_PLUS_BIT_CNT > idx; idx++ )
    {
        const comp_RegValue_t bitValue = Comp_Get_RegBit( periphId,
                                                          comp_PlusBit[idx].RegId,
                                                          comp_PlusBit[idx].RegMask );

        if( COMP_REG_BITS_CLEAR != bitValue )
        {
            selCode |= comp_PlusBit[idx].CodeBit;
        }
        else
        {
            /* The selection bit is cleared, no code bit is added */
        }
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( selCode );
}


/**
 * \brief Writes the selection code of the inverting input to INMSEL, SCALEN and BRGEN.
 *
 * \param periphId [in] : COMP instance ID
 * \param selCode  [in] : Selection code (INMSEL[3:0], bit 4 SCALEN, bit 5 BRGEN)
 */
void Comp_Reg_Set_InputMinusSel( comp_PeriphId_t periphId, comp_RegValue_t selCode )
{
#if defined(COMP1)
    comp_RegValue_t regValue = ( selCode & COMP_SEL_MINUS_INMSEL_MASK ) << COMP_CFGR1_INMSEL_Pos;

    if( COMP_REG_BITS_CLEAR != ( selCode & COMP_SEL_MINUS_SCALEN ) )
    {
        regValue |= COMP_CFGR1_SCALEN;
    }
    else
    {
        /* The scaler stays disabled */
    }

    if( COMP_REG_BITS_CLEAR != ( selCode & COMP_SEL_MINUS_BRGEN ) )
    {
        regValue |= COMP_CFGR1_BRGEN;
    }
    else
    {
        /* The scaler bridge stays disabled */
    }

    Comp_Set_RegVal( periphId, COMP_REG_CFGR, COMP_CFGR1_INMSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN, regValue );
#else
    (void)periphId;
    (void)selCode;
#endif /* COMP1 */

    return;
}


/**
 * \brief Reads the selection code of the inverting input from INMSEL, SCALEN and BRGEN.
 *
 * \param periphId [in] : COMP instance ID
 *
 * \return Selection code (INMSEL[3:0], bit 4 SCALEN, bit 5 BRGEN), zero for an invalid instance.
 */
comp_RegValue_t Comp_Reg_Get_InputMinusSel( comp_PeriphId_t periphId )
{
    comp_RegValue_t selCode = COMP_REG_BITS_CLEAR;

#if defined(COMP1)
    const comp_RegValue_t regValue = Comp_Get_RegVal( periphId,
                                                      COMP_REG_CFGR,
                                                      COMP_CFGR1_INMSEL | COMP_CFGR1_SCALEN | COMP_CFGR1_BRGEN );

    selCode = ( regValue & COMP_CFGR1_INMSEL ) >> COMP_CFGR1_INMSEL_Pos;

    if( COMP_REG_BITS_CLEAR != ( regValue & COMP_CFGR1_SCALEN ) )
    {
        selCode |= COMP_SEL_MINUS_SCALEN;
    }
    else
    {
        /* The scaler is disabled, no code bit is added */
    }

    if( COMP_REG_BITS_CLEAR != ( regValue & COMP_CFGR1_BRGEN ) )
    {
        selCode |= COMP_SEL_MINUS_BRGEN;
    }
    else
    {
        /* The scaler bridge is disabled, no code bit is added */
    }
#else
    (void)periphId;
#endif /* COMP1 */

    return ( selCode );
}
