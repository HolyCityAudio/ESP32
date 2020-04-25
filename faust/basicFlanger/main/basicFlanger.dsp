import("stdfaust.lib");
flaDelay = hslider("[3]Delay", 156, 5, 1000, 1) : si.smoo;
flaFeedback = hslider("[4]Flange Fb", 0.2, 0, 0.97, 0.01) : si.smoo;
flaDepth = hslider("[5]Flange Dep", 0.95, 0, 1.0, 0.01) : si.smoo;
flaLFORate = hslider("[6]Rate", 0.25, 0, 3, 0.01) : si.smoo;
flaLFOWidth = hslider("[7]Width", 0.5, 0, 1.0, 0.01) : si.smoo;
flaLFO = os.lf_triangle(flaLFORate);
flaMod = flaLFOWidth * (flaLFO/2) ;
flanger(x,y) = pf.flanger_mono(512, flaDelay * (1 + (x * flaMod)), flaDepth, flaFeedback, y);
flange = hgroup("Flange", flanger(1,-1) : ef.echo(2.62, 0.32, 0.18) : flanger(-1,1));
// flange = hgroup("Flange", flanger(1) : flanger(-1));
//=============================================
process =  _,_: + :> component("tubes.lib").T1_12AX7 : flange <: _,_;
// process = _,_ :> component("tubes.lib").T1_12AX7 : flange : re.jcrev :> _,_;
// process = _,_ :> re.mono_freeverb(0.5, 0.65, 0.7, 100) <: _,_;