import("stdfaust.lib");
echoTime = hslider("[7]echoTime", 0.10, 0.10, 2.5, 0.01) : si.smooth(0.99995);
echoFeedback = hslider("[7]echoFeedback", 0.0, 0, 1.0, 0.01) : si.smoo;
echoLPF = hslider("[7]echoLPF", 2500.0, 1000.0, 10000.0, 0.01) : si.smoo;
//=============================================
process = _,_ : + <: _,( + : de.sdelay(ba.sec2samp(2.5), 1024, ba.sec2samp(echoTime)) : fi.lowpass(2,echoLPF)) ~* (echoFeedback) * 0.5 :> _ <: _,_;
