#===================== Integration tests configuration ========================#
# Integration tests of Comp module on target. Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Comp.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Comp.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

IntegrationTesting_Add_Test(
    NAME            Comp
    TEST_SOURCE     ItTest_Comp.c
    SOURCES         ItTarget_Comp.c     # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget), comparator vector
                    Rcc_Lib             # Peripheral clock of the comparators
                    Gpio_Lib            # Analog input pins
                    Exti_Lib            # Wake-up line of the comparator output
                    Dwt_Lib             # Cycle counter (time base of the start-up, ItTarget)
                    Comp_Lib            # Module under test
    TIMEOUT         60
)
