# FreeFly
## Project Structure
```
- System
  - BSP
  - CMSIS
  - FWLIB
- ucos2
  - Cfg
  - Ports
  - Source
- Application
  - Hardware
  - SystemView
  - main.c
- build
- STM32F401RETx_FLASH.ld
- CMakeLists.txt
- arm-none-eabi-gcc.cmake
- README.md
```
## Notes
- port ucOS-II OS to STM32F401RE.
- adopt Madgwick filter to estimate attitude.
- use cascade PID controller in flight.


## TODOs
- [x] change os_cpu_a.s from armcc to gcc version.
- [x] sensor calibration 
- [x] attitude estimation algorithm
- [x] pid controller 
- [ ] add camera and more...