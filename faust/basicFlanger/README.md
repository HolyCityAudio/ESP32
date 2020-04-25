# faust basicFlanger

This is a basic flanger for the ESP32-A1S audio/wifi/bluetooth dev board.<br>
Actually it looks like in this version of the code, I have two flangers, one on either side of a fixed delay with a little feedback.  They are sweeping in opposite directions and have opposite phase shift.  Take blocks out to experiment with the sound.  Also there is a Faust "12AX7" block in there!

It is stereo-mono in and out.

Key 1: flanger delay time down<br>
Key 2: flanger delay time up<br>
Key 3: LFO rate down<br>
Key 4: LFO rate up<br>
Key 5: LFO width down<br>
Key 6: LFO width up<br>

See main.cpp for the button control mapping to parameters in the Faust code.<br>
The button code came from: https://github.com/craftmetrics/esp32-button
