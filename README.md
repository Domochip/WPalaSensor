# WPalaSensor

This project aims to replace the Palazzetti (Fumis) room temperature sensor by an electronic simulator.  
It allows you to push home automation temperature to the stove.  
You can then keep native modulation that reduce power automatically when desired temperature is reached.  

![Fumis Controller](img/presentation.png)

## Compatibility

It appears that Fumis Controller is used by those brands for their stoves : 

* Palazzetti (All)
* Jotul
* TurboFonte
* Godin
* Fonte Flamme
* Invicta

If you have this controller in your stove, it's likely to be compatible.  
![Fumis Controller](img/fumis.png)

## Build your adapter

All files are inside `schematic` subfolder and has been designed with KiCad (free and open source)

### Schematic

![WPalaSensor schematic](img/schematic.png)

### PCB

![WPalaSensor PCB](img/pcb-top.png)![WPalaSensor PCB2](img/pcb-bottom.png)

*We produced a small batch of this adapter for test/debugging and our personal use.
If you are interested, please PM.*

### Print your box

Box project (Fusion 360 & STL) can be found into `box` folder

![WPalaSensor box](img/box.png)

### Code/Compile/Flash

Source code can be compiled using VisualStudioCode/Platformio and flashed onto a D1 Mini  
Or  
Download latest release in Release section
