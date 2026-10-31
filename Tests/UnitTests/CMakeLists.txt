#======================== Unit tests configuration ============================#
# Unit tests of Comp module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(COMP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${COMP_DIR}/.." ABSOLUTE)

# The COMP registers are plain memory (RegMem). Two tests activate the hardware model
# UtComp_HwModel_IgnoreWrites (a configuration register that ignores every write), so the
# set is registered with RUN_SERIAL. Rcc, Nvic, Exti, Gpio and Dwt (cycle counter) are mocked.
UnitTesting_Add_Test(
    NAME            Comp
    TEST_SOURCE     Test_Comp.c
    UUT_SOURCES     ${COMP_DIR}/Comp.c
                    ${COMP_DIR}/Comp_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Exti/Exti_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Core/Dwt/Dwt_Port.h
    INCLUDE_DIRS    ${COMP_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Exti
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Core/Dwt
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
