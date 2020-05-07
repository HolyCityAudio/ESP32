import("stdfaust.lib");
flaDelay = vslider("[03]Delay", 156, 5, 1000, 1) : si.smoo;
flaLFORate = vslider("[04]Rate", 0.25, 0, 3, 0.01) : si.smoo;
flaLFOWidth = vslider("[05]Width", 0.5, 0, 1.0, 0.01) : si.smoo;
flaDepth = vslider("[06]Depth", 0.95, 0, 1.0, 0.01) : si.smoo;
flaFeedback = 0; // vslider("[07]FlgFbk", 0.8, 0, 0.97, 0.01) : si.smoo;
echoDelay = vslider("[08]EchoDelay", 0.20, 0, 1.0, 0.01) : si.smooth(0.9995);
echoLevel = vslider("[09]EchoLvl", 0.8, 0, 1.0, 0.01) : si.smoo;
echoFeedback = vslider("[10]EchoFbk", 0.3, 0, 0.97, 0.01) : si.smoo;
// can't get lowpass filter to fit on ESP32
// echoLPF = vslider("[11]rotary38", 10000, 1000, 15000, 0.1) : si.smoo;
flaLFO = os.lf_triangle(flaLFORate);
flaLFO2 = os.lf_triangle(flaLFORate/2);
flaMod = flaLFOWidth * (flaLFO/2) ;
flaMod2 = flaLFOWidth * (flaLFO2/2) ;

flanger(x, mod) = pf.flanger_mono(512, flaDelay * (1 + (x * mod)), flaDepth, flaFeedback, 1);
flange(lfo) = hgroup("Flange", flanger(1, lfo));
// 
// line below is basic echo with Feedback
// fbkEcho = ( +:  de.fdelay(ba.sec2samp(1), ba.sec2samp(echoDelay)) : flange ) ~ *(echoFeedback);
// can't get the lowpass filter in there on the ESP32 so far
// fbkEcho = ( +:  fi.lowpass(1,echoLPF) : de.delay(ba.sec2samp(1), ba.sec2samp(echoDelay))) ~ (flange : *(echoFeedback));
fbkEcho = ( + : de.delay(ba.sec2samp(1), ba.sec2samp(echoDelay)) : flange(flaMod)) ~ (flange(flaMod2) : *(echoFeedback));

echoOut = _ <: _,fbkEcho * echoLevel : +;
//=============================================
process =  hgroup("Echo flanger", _,_: + : echoOut <: _,_);