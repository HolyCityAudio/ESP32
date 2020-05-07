import("stdfaust.lib");
flaDelay = vslider("[03]Delay", 156, 5, 1000, 1) : si.smoo;
flaLFORate = vslider("[04]Rate", 0.25, 0, 3, 0.01) : si.smoo;
flaLFOWidth = vslider("[05]Width", 0.5, 0, 1.0, 0.01) : si.smoo;
flaDepth = vslider("[06]Depth", 0.95, 0, 1.0, 0.01) : si.smoo;
flaFeedback = vslider("[07]FlgFbk", 0.8, 0, 0.97, 0.01) : si.smoo;
echoDelay = vslider("[08]EchoDelay", 0.20, 0, 2.6, 0.01) : si.smoo;
echoLevel = 0.7; // vslider("[09]EchoLvl", 0.8, 0, 1.0, 0.01) : si.smoo;
echoFeedback = vslider("[10]EchoFbk", 0.8, 0, 0.97, 0.01) : si.smoo;
echoLPF = vslider("[11]rotary38", 10000, 1000, 15000, 0.1) : si.smoo;
flaLFO = os.lf_triangle(flaLFORate);
flaMod = flaLFOWidth * (flaLFO/2) ;
flanger(x) = pf.flanger_mono(512, flaDelay * (1 + (x * flaMod)), flaDepth, flaFeedback, 1);
flange = vgroup("Flange", flanger(1));
fbkEcho = ( + : de.sdelay(ba.sec2samp(1), 1024, ba.sec2samp(echoDelay))) ~* (echoFeedback);
// fbkEcho = ( + : de.sdelay(ba.sec2samp(1), 1024, ba.sec2samp(echoDelay))) * (echoFeedback);
echoOut = _ <: _,fbkEcho : +;
//=============================================
process =  _,_: + : echoOut: flange <: _,_;