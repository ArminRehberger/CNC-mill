# CNC-mill
CNC mill with Raspperry GUI and Teensy real time system

Video:
https://www.youtube.com/watch?v=XnDwizPUjZA&t=19s

Implementation of a mechatronic CNC mill.
Raspberry PI is used to operate the mill, a graphical GUI is implemented.
Teensy4.0 is used to control the real time CNC machine.
A G-Code file can be loaded with the GUI, the file will be readed and then transferred to Teensy4.0
(max. 7500 steps). After loading the file, the CNC-program can be executed.
In manual mode of the CNC machine you can do following:
Inche the axis (X, Y, Z) forwards and backwards.
Four digital outputs (four relay outputs) for spindle, coolant, clamp and light can be forced.
The 0-10V PWM signal for the spindle speed can be forced.
The tool length can be automatically measured with a tool length sensor.
The state of the digital inputs are displayed (Reference sensors X, Y, Z, emergency stop and tool
length sensor.
Four bytes, Hexadecimal, are used to transfer the actual positions of the axis from the Teensi to the
Pi (0.0000 to 9999.9999 mm)

Disclaimer

This hardware/software is provided "as is", and you use the hardware/software at your own risk. Under no circumstances shall any author be liable for direct, indirect, special, incidental, or consequential damages resulting from the use, misuse, or inability to use this hardware/software, even if the authors have been advised of the possibility of such damages

