# COMP Peripheral Driver

This module provides an abstraction layer for the **analog comparators** (`COMPx`) of STM32 MCUs: the selection of the
inputs (pins, DAC outputs, the internal reference), the hysteresis, the speed / consumption mode, the polarity and the
blanking of the output, the routing of the output to a pin, the **edge interrupt** that also wakes the MCU from the
low-power modes, the window mode of the comparator pairs and the lock of the configuration. It is a low-level MCAL module:
it owns the comparator hardware, the application and the other modules never touch the registers.

The public interface (`Comp_Port.h`, `Comp_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- A comparator answers one question in hardware, without the CPU and within tens of nanoseconds: *"is this voltage above
  that one?"* Typical users are the **overcurrent and fault protection** (the output shuts the PWM down through the break
  input of a timer), the **cycle-by-cycle current limiting** of a converter (the output clears the output compare
  reference of a timer), the **zero-crossing detection** (mains, back-EMF), the **threshold wake-up** (the MCU sleeps in
  Stop mode and the comparator wakes it when a battery, a sensor or a light level crosses the threshold) and the
  **window supervision** of a voltage.
- The configuration of a comparator is a handful of choices that depend on each other (which inputs exist on this
  instance, which of them can be combined, which levels of the hysteresis the device has, how long the start-up takes).
  The user chooses *what* is compared, the module hides *how*: the selection bits, the start-up time, the order in
  which the lock bit is set, the EXTI line of the output.
- The module only owns the comparator side. The **consumer of the output** is configured by its own module: the
  timer break input and the `ETR` / `OCREF_CLR` source in the Tim module, the DAC channel that supplies a threshold in the
  Dac module, the EXTI wake-up in the Exti module. The Comp module documents the connection and checks that the instance
  has it.
- The module hides the differences between the families: the number of the comparators, their inputs, the hysteresis
  levels, the power modes, the blanking sources, the window pairs and the lines of the interrupt differ, the interface
  stays the same.

What the module is **not**: it does not generate the threshold (the Dac module, or the internal reference selected as an
input) and it does not drive the protection reaction (the timer break configuration is a Tim module task). It does not
measure the voltage - that is the Adc module.

---

## Concept

An instance is identified by `comp_PeriphId_t`. The configuration `comp_Config_t` describes:

| Item | Meaning |
|---|---|
| **Non-inverting input** (`InputPlus`) | A pin of the instance (up to three per instance), or an internal signal where the device has it. Only the inputs of the instance are in the list `comp_InputPlus_t`. |
| **Inverting input** (`InputMinus`) | A pin, an output of the DAC channel, the internal reference `VREFINT` or its fractions 1/4, 1/2, 3/4 from the scaler. Only the inputs of the instance are in the list `comp_InputMinus_t`. |
| **Hysteresis** | `COMP_HYSTERESIS_NONE`, `LOW`, `MEDIUM`, `HIGH` - the levels of the device (the family README gives the values in mV). |
| **Power mode** | `COMP_POWER_HIGH_SPEED`, `MEDIUM`, `LOW_POWER`, `ULTRA_LOW_POWER` - speed against consumption; the propagation delay and the start-up time follow from it. |
| **Polarity** | `COMP_POLARITY_NORMAL` (output high when `+` is above `-`) or `COMP_POLARITY_INVERTED`. |
| **Blanking** | The output is ignored while the selected timer signal (`comp_Blanking_t`, for example an output compare of the PWM timer) is active - hides the switching spikes of a converter. |
| **Output** | Optionally routed to a pin (`comp_OutPin_t`); the internal routing to the timers and the EXTI exists always. |
| **Interrupt** | The edge (`COMP_EDGE_RISING`, `FALLING`, `BOTH`) on which the callback is called and the MCU wakes up. |
| **Window mode** | Two comparators of a pair share the non-inverting input and compare it with two thresholds. |
| **Lock** | The configuration is frozen until the next reset (a protection against a software that would disable the protection). |

Rules:

1. `Comp_Init` configures the instance, the pins and the interrupt and leaves the comparator **disabled**;
   `Comp_Set_PeriphActive` enables it. The output is valid after the start-up time of the selected power mode; until then the
   state is `COMP_STATE_STARTING` and the edge interrupt is blocked, so the start-up does not cause a false event.
2. The inputs, the hysteresis, the power mode, the polarity and the blanking are changed on a disabled comparator
   (hardware rule of most families); the call on an enabled comparator returns an error and nothing changes.
3. `Comp_Get_OutputLevel` returns the level after the polarity; the level read before the start-up ends is reported as invalid.
4. The edge callback is called from the interrupt (the EXTI line of the comparator); the flag is cleared by the module.
   The wake-up from Stop mode needs the instance, the power mode and the EXTI line that the family allows - the module
   checks it at `Comp_Set_IrqActive`.
5. A pair in the window mode is configured as a pair: the non-inverting input is set on the first comparator and the module
   takes care of the second one; the output of the window is the output of the pair as the family defines it.
6. After `Comp_Set_Lock` every configuration call (and the disable) returns an error until the reset; the lock is not
   removable by software.
7. All parameters are checked before a register is written.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Comparators | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `COMP1` on STM32H503, H543, H553 and the H5E / H5F lines: pins, `DAC` output and `VREFINT` (and 1/4, 1/2, 3/4) as the inverting input, three non-inverting pins, hysteresis, power modes, polarity, blanking by timer and LPTIM outputs, outputs to the timer capture, `OCREF_CLR` and break inputs, edge interrupt with wake-up | planned (first) |
| STM32G4 | `Dev/STM32G4` | `COMP1` ... `COMP7`, window mode of the pairs, hysteresis levels, blanking, the inputs include the internal DAC channels and `VREFINT` fractions | planned |
| STM32L4 | `Dev/STM32L4` | `COMP1`, `COMP2`, window mode, power modes | planned |
| STM32H7 | `Dev/STM32H7` | `COMP1`, `COMP2`, window mode | planned |
| STM32U5 | `Dev/STM32U5` | `COMP1`, `COMP2`, window mode | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | no comparator on the device - the module is a stub that returns `COMP_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `COMP_REQUEST_ERROR` and suppresses the unused
parameters (for example the window mode on an instance without a pair, a hysteresis level or a power mode that the device
does not have, the blanking source that the instance does not offer).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Comp_Get_ModuleVersion`, `Comp_Init`, `Comp_Deinit`, `Comp_Task`, `Comp_Get_DefaultConfig` |
| Control | `Comp_Set_PeriphActive`, `Comp_Set_PeriphInactive`, `Comp_Get_PeriphState` |
| Inputs and settings | `Comp_Set_InputPlus` / `Get`, `Comp_Set_InputMinus` / `Get`, `Comp_Set_Hysteresis` / `Get`, `Comp_Set_PowerMode` / `Get`, `Comp_Set_Polarity` / `Get`, `Comp_Set_Blanking` / `Get`, `Comp_Set_WindowMode` / `Get` |
| Output | `Comp_Get_OutputLevel` |
| Interrupt | `Comp_Set_TriggerEdge` / `Get`, `Comp_Set_IrqActive`, `Comp_Set_IrqInactive`, `Comp_Set_IrqCallback`, `Comp_Set_IrqPriority` / `Get` |
| Lock | `Comp_Set_Lock`, `Comp_Get_LockState` |
| Pins | `Comp_InitInputGpio`, `Comp_InitOutputGpio` |

```c
comp_ModuleVersion_t Comp_Get_ModuleVersion  ( void );

comp_RequestState_t  Comp_Init               ( const comp_Config_t * const compConfig );
comp_RequestState_t  Comp_Deinit             ( comp_PeriphId_t periphId );
void                 Comp_Task               ( void );
comp_RequestState_t  Comp_Get_DefaultConfig  ( comp_Config_t * const compConfig );

comp_RequestState_t  Comp_Set_PeriphActive   ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_PeriphInactive ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Get_PeriphState    ( comp_PeriphId_t periphId, comp_State_t * const compState );

comp_RequestState_t  Comp_Set_InputPlus      ( comp_PeriphId_t periphId, comp_InputPlus_t inputPlus );
comp_RequestState_t  Comp_Set_InputMinus     ( comp_PeriphId_t periphId, comp_InputMinus_t inputMinus );
comp_RequestState_t  Comp_Set_Hysteresis     ( comp_PeriphId_t periphId, comp_Hysteresis_t hysteresis );
comp_RequestState_t  Comp_Set_PowerMode      ( comp_PeriphId_t periphId, comp_PowerMode_t powerMode );
comp_RequestState_t  Comp_Set_Polarity       ( comp_PeriphId_t periphId, comp_Polarity_t polarity );
comp_RequestState_t  Comp_Set_Blanking       ( comp_PeriphId_t periphId, comp_Blanking_t blanking );

comp_RequestState_t  Comp_Get_OutputLevel    ( comp_PeriphId_t periphId, comp_Level_t * const outputLevel );

comp_RequestState_t  Comp_Set_TriggerEdge    ( comp_PeriphId_t periphId, comp_Edge_t edge );
comp_RequestState_t  Comp_Set_IrqActive      ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_IrqInactive    ( comp_PeriphId_t periphId );
comp_RequestState_t  Comp_Set_IrqCallback    ( comp_PeriphId_t periphId, comp_IrqCallback_t * const callback );

comp_RequestState_t  Comp_Set_Lock           ( comp_PeriphId_t periphId );
```

Every function returns `COMP_REQUEST_OK` / `COMP_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `comp_Config_t` | Instance, inputs, hysteresis, power mode, polarity, blanking, window mode, output pin, edge, callback, interrupt priority, lock request |
| `comp_State_t` | `COMP_STATE_DISABLED`, `COMP_STATE_STARTING`, `COMP_STATE_READY`, `COMP_STATE_LOCKED` |
| `comp_Level_t` | `COMP_LEVEL_LOW`, `COMP_LEVEL_HIGH`, `COMP_LEVEL_INVALID` (read during the start-up) |
| `comp_IrqCallback_t` | `void ( comp_PeriphId_t periphId, comp_Level_t outputLevel )`, called on the selected edge |

---

## Usage

### Wake-up from Stop mode when a voltage rises above a threshold

```c
#include "Comp_Port.h"

static void App_ThresholdCrossed( comp_PeriphId_t periphId, comp_Level_t outputLevel )
{
    /* the MCU is awake, the comparator has seen the rising edge */
}

comp_Config_t compConfig;

(void)Comp_Get_DefaultConfig( &compConfig );

compConfig.PeriphId   = COMP_PERIPH_1;
compConfig.InputPlus  = COMP_INPUT_PLUS_COMP1_IO1;           /* the measured voltage            */
compConfig.InputMinus = COMP_INPUT_MINUS_VREFINT_HALF;       /* internal reference / 2, no pin  */
compConfig.Hysteresis = COMP_HYSTERESIS_MEDIUM;
compConfig.PowerMode  = COMP_POWER_ULTRA_LOW_POWER;
compConfig.Edge       = COMP_EDGE_RISING;
compConfig.Callback   = App_ThresholdCrossed;

(void)Comp_Init( &compConfig );
(void)Comp_Set_PeriphActive( COMP_PERIPH_1 );
(void)Comp_Set_IrqActive( COMP_PERIPH_1 );
```

### Overcurrent protection: comparator with the DAC threshold, the output shuts the PWM down

```c
compConfig.InputPlus  = COMP_INPUT_PLUS_COMP1_IO2;           /* shunt voltage                   */
compConfig.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;           /* threshold set by the Dac module */
compConfig.PowerMode  = COMP_POWER_HIGH_SPEED;
compConfig.Blanking   = COMP_BLANKING_TIM1_OC5;              /* ignore the switching spike      */
compConfig.Lock       = COMP_FUNCTION_ACTIVE;                /* cannot be disabled by software  */
(void)Comp_Init( &compConfig );
(void)Comp_Set_PeriphActive( COMP_PERIPH_1 );
/* the Tim module connects the output of the comparator to the break input of the PWM timer */
```

---

## Behaviour and limits

- **Start-up time.** After the enable the comparator needs a time (it depends on the power mode) before the output is valid.
  The module blocks the edge interrupt and reports `COMP_STATE_STARTING` during it; with the ultra-low-power mode the
  start-up and the propagation delay are the longest.
- **Propagation delay and hysteresis.** The speed / consumption mode sets the delay; the hysteresis protects against the
  oscillation of the output around the threshold but shifts the thresholds. The numbers are in the datasheet of the device.
- **Pins.** The input pins must be in the analog mode (the module does it) and the output pin in the alternate function mode.
  The `VREFINT` and the DAC inputs need their sources enabled (the Dac module for the DAC, the internal reference is enabled by the module).
- **Lock.** A locked configuration is removed only by a reset of the MCU.
- **Wake-up.** Only the instances and the power modes that the family allows wake the MCU up from Stop mode; the
  README of the family lists them.
- **Not part of the first version:** the comparator outputs as a trigger of other peripherals beyond the documented
  routing, the calibration of the offset on the families that have it, the dynamic switching of the scaler during the
  operation.

---

## Dependencies

- **Rcc** - peripheral clock and reset
- **Exti** - the interrupt and the wake-up line of the comparator output
- **Nvic** - interrupt
- **Gpio** - input pins in the analog mode, the output pin
- **Dac** - the DAC channel used as a threshold (configured by its own module)
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, EXTI, NVIC and GPIO configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the list of the inputs of every instance and the rejected combinations,
  the order of the writes (configuration, lock, enable), the start-up state machine and the blocked interrupt, the
  polarity of the read level, the window mode pair rules, the rejected changes of an enabled and of a locked
  comparator, the edge callback.
- **Integration tests** (target, probe-rs): the comparison of the DAC threshold with a ramp from the second DAC channel
  (the edge callbacks counted), the hysteresis effect, the wake-up from Stop mode, the break of a timer by the comparator
  output.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `COMP_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Comp_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Comp_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`comp_FunctionState_t`, `comp_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `COMP_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Comp_Port.h`, `Comp_Types.h` | Public interface (identical in all families) |
| `Comp*.c`, `Comp.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Comp_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Comp_Lib )
```

1. Link `Comp_Lib` to your CMake library.
2. Include `Comp_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
