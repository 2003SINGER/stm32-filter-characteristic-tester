# STM32 Filter Characteristic Tester Firmware

Firmware and HMI project for an STM32G474-based filter characteristic tester. The intended workflow is to generate a sine wave, sweep frequency on a logarithmic grid, capture the filter response through ADC + DMA, estimate gain and phase, and render the Bode-style curves and derived metrics on a UART HMI.

## Project status

This was prepared for an embedded-electronics selection task of the UESTC Yingcai Experimental College team.

- The software side was implemented as an engineering attempt, including the sweep, measurement, analysis, and display paths.
- Hardware integration was not completed because of unresolved hardware issues.
- Therefore, the firmware was **not validated on the intended board or filter**, and this repository makes no accuracy, stability, or competition-result claim.
- The author did not enter the team after the selection. This repository is retained as an honest record of the implementation attempt, not as a finished instrument.

Some implementation work was completed with AI assistance. The repository is published after removing AI dialogue, generated task summaries, competition PDFs, build output, and local IDE state; the remaining source is kept so its technical decisions can be inspected.

## What the firmware is designed to do

1. Generate sine samples through `DAC1` under `TIM6` triggering.
2. Sweep from 10 Hz to 100 kHz on a logarithmic frequency grid.
3. Capture ADC samples by DMA after a settling interval.
4. Use correlation with sine/cosine references to estimate the fundamental amplitude and phase.
5. Derive 1 kHz gain/phase, passband gain, -3 dB cutoff frequency, -90 degree characteristic frequency, a Q-like metric, and a coarse filter-type classification.
6. Send measurement state, curve points, and metrics to a UART HMI.

The measurement and analysis code is mainly in:

- `Core/Src/sweep_engine.c`
- `Core/Src/hmi_driver.c`

## Hardware and configuration

- MCU: STM32G474xx
- DAC: DAC1 channel 1
- ADC: ADC1 channel 1, DMA capture
- Timer: TIM6 trigger for DAC/ADC synchronization
- HMI: UART1, 115200 bps
- Display project: `hmi/screen.HMI`

The code assumes external analog conditioning around the DUT: the DAC output and measured filter output need appropriate level shifting before entering the single-supply ADC path. That analog chain was not fully verified, so pin wiring, voltage range, grounding, and calibration must be checked before any hardware use.

## Build

The repository includes the STM32CubeMX configuration and the minimal generated STM32 HAL/CMSIS subset needed by the CMake project.

Prerequisites:

- STM32CubeMX-compatible STM32G4 HAL/CMSIS environment
- GNU Arm Embedded Toolchain
- CMake and Ninja

With STM32CubeCLT tools on `PATH`:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The checked-in `.ioc` file is the authoritative peripheral configuration. Regenerating it in a different CubeMX version can change generated files; inspect the diff before accepting it.

## Repository scope

Included:

- Firmware source and STM32CubeMX configuration
- Minimal HAL/CMSIS dependency subset required by the CMake build
- UART HMI project

Excluded:

- Competition statement and reference PDFs
- AI dialogue and generated task notes
- Build artifacts, IDE configuration, and local paths
- Measurement data, because no complete hardware validation was obtained

## License and third-party material

Original application-level code in `Core/Src/sweep_engine.c`, `Core/Src/hmi_driver.c`, and their headers is released under the MIT License in this repository. STM32CubeMX-generated files and bundled ST/CMSIS materials remain subject to their original notices; see `THIRD_PARTY_NOTICES.md` and the license files under `Drivers/`.
