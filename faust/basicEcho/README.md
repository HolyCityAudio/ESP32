## faust basicEcho is a basic echo for the ESP32-A1S audio/wifi/bluetooth dev board.<br>
This patch uses the sdelay function which allows real time delay time changes without doing
pitch bending.  Sort of depends on what you're after.  Also includes a low pass filter in
the feedback loop.  Mix level is fixed at -3 dB.

It is stereo-mono in and out.

Key 1: delay feedback down<br>
Key 2: delay feedback up<br>
Key 3: delay time down<br>
Key 4: delay time up<br>
Key 5: LPF frequency down<br>
Key 6: LPF frequency up<br>

See main.cpp for the button control mapping to parameters in the Faust code.<br>
The button code came from: https://github.com/craftmetrics/esp32-button
