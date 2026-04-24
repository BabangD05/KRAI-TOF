# KRAI-TOF
Current code is in KRAI-TOF branch.
Add the TOF Sensor Library for the programs that need it.

Library insert tutorial :
Insert/Copy to Drivers folder in your project, then go to project > properties > C/C++ General > Paths and Symbols > Add > Workspace > (Search the TOF Sensor Library, then add the core/inc and patform/inc) > Apply and Close > include in main.h

Too read Ranging Data : debug > (right window) live expression > add expression (RangingData) > Run the STM32
