# faust echoFlanger

This is a basic echo with feedback feeding into a single flanger for the ESP32-A1S audio/wifi/bluetooth dev board.<br>
This is set up somewhat like an oil can delay in that the delay time and LFO frequency are inversely related to each other.
The echo block wet output is also set by the feedback control, so as the feedback increases, so does the echo level.<br>
There is smoothing on the echo time changes from the button presses but it's still fairly fast.  This leads to bizarre glitches when you change the LFO Rate/echo time and the feedback is turned up.

It is stereo-mono in and out.

Key 1: delay feedback down<br>
Key 2: delay feedback up<br>
Key 3: LFO rate down<br>
Key 4: LFO rate up<br>
Key 5: LFO width down<br>
Key 6: LFO width up<br>

See main.cpp for the button control mapping to parameters in the Faust code.<br>
The button code came from: https://github.com/craftmetrics/esp32-button
