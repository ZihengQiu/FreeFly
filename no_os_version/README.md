- System
  - CMSIS
    - Core : about Cortex-M core, e.g. NVIC.
    - Device : about MCU, e.g. Registers.
  - FWLIB : Firmware Library
    - Inc
    - Src


# TODOs
- [ ] Update CMSIS version.

# Tips
- stm32f4XX_fmc.h in FWLIB should be excluded because it is not supported by STM32F401.
- must define STM32F40_41xxx but not STM32F401xx in preprocessor.