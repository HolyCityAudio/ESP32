Here are some files which can be used to create DSP sound effects using the AI Thinker ESP32-A1S audio/wifi/bluetooth 
development board.  The DSP code is created using the Faust ESP32 script.

Here are some assumptions as to the environment.
- You are using Ubuntu 18.04 or similar.  My development system is actually the Ubuntu subsystem on Windows 10,
which you can install from the Microsoft store.
- You have installed the ESP-IDF environment version 4.0.  See:  https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html#
- The project layout is like a typical ESP-IDF project.
- You have compiled Faust from source.  See: https://github.com/grame-cncm/faust
- You have copied the file faust-build.sh to your ESP32/esp-idf/tools folder and marked it executable.

cd to the desired folder, e.g.<br>
`cd basicFlanger`<br>
then run<br>
`faust-build.sh basicFlanger`<br>
<br>
This will show the following menu:<br>

============================ Main Menu ============================<br>

    1) Edit the basicFlanger.dsp file
    2) Edit main.cpp
    3) Edit basicFlangercpp
    4) Edit any file
    5) Execute idf.py fullclean
    6) Build C++ code only
    7) Compile DSP code only
    8) Build and upload C++ code, then monitor
    9) Monitor
    10) make menuconfig
    11) Save as reference
    12) Exit

===================================================================<br>
Choose a menu selection (1 - 12):<br>

I encourage you to edit the faust-build.sh script to suit your needs.
Currently I use the notepad++ windows program as my text editor.  
If you'd rather use vi, then change the line:

`EDITOR="/mnt/c/Program Files (x86)/Notepad++/notepad++.exe"`<br>
to:<br>
`EDITOR="vi"`

You should also set the COM port and baud rate according to what your board will support.  This is just my quick attempt at setting up a menu that lets me quickly get to most of what I'd like to do.  Compiling the faust (dsp) source to C++ is a separate step as occasionally it is necessary to edit one of the generated files prior to building the final firmware.

Option 11, "Save as reference", makes a "tar" file of the important bits of your project and puts them in ../reference, which you'll have to create if you want to use this.  This function hasn't been heavily tested.
