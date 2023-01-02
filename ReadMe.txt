
1. Generate a STM32 MDK project normally by STM32CubeMX with most updated HAL drivers.
2. Open project in Keil ARM MDK, change project-option-C/C++ -MiscControls to --cpp
3. Create user C++ files in .h and .cpp

This example shows exactly how to do it.

Hardware platform: STM32 F4 Discovery board (STM32F407VGT6) 
Software platform:  Keil ARM MDK V5.05
                            STM32CubeMX V4.9.0

If four LEDs on STM32 F4 Discovery board toggle respectively every 0.5,1,2,4 seconds, then we are done.


