Here are some files which can be used to create DSP sound effects using the AI Thinker ESP32-A1S audio/wifi/bluetooth 
development board.  The DSP code is created using the Faust ESP32 script.

Here are some assumptions as to the environment.
- You are using Ubuntu 18.04 or similar.  My development system is actually the Ubuntu subsystem on Windows 10,
which you can install from the Microsoft store.
- You have installed the ESP-IDF environment version 4.0.  See:  https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html#
- You have compiled Faust from source.  See: https://github.com/grame-cncm/faust
- You have copied the file faust-build.sh to your ESP32/esp-idf/tools folder and marked it executable.

cd to the desired folder, e.g.<br>
`cd basicFlanger`<br>
then run<br>
`faust-build.sh basicFlanger`<br>
<br>
This will show the following menu:<br>

============================ Main Menu ============================

    1) Edit the basicFlanger.dsp file
    2) Edit main.cpp
    3) Edit basicFlanger.cpp
    4) Edit any file
    5) Execute idf.py fullclean
    6) Build code only
    7) Build and upload code, then monitor
    8) make menuconfig
    9) Save as reference
    10) Exit

===================================================================<br>
Choose a menu selection (1 - 10):<br>

I encourage you to edit the faust-build.sh script to suit your needs.
Currently I use the notepad++ windows program as my text editor.  
If you'd rather use vi, then change the line:

``
