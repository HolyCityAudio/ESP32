import("stdfaust.lib");
flaDelay = 200; // hslider("[3]Delay", 156, 5, 1000, 1) : si.smoo;
flaFeedback = 0.7; // hslider("[4]Flange Fb", 0.2, 0, 0.97, 0.01) : si.smoo;
flaDepth = 1.0;  //hslider("[5]Flange Dep", 0.95, 0, 1.0, 0.01) : si.smoo;
flaLFORate = hslider("[6]Rate", 0.5, 0, 3, 0.01) : si.smoo;
echoTime = 0.5 / flaLFORate : si.smooth(0.99995);
echoFeedback = hslider("[7]echoFeedback", 0.0, 0, 1.0, 0.01) : si.smoo;
flaLFOWidth = hslider("[7]Width", 0.0, 0, 1.0, 0.01) : si.smoo;
flaLFO = os.lf_triangle(flaLFORate);
flaMod = flaLFOWidth * (flaLFO/2) ;
flanger(x,y) = pf.flanger_mono(512, flaDelay * (1 + (x * flaMod)), flaDepth, flaFeedback, y);
//=============================================
// uncomment one of the flange lines below
flange = hgroup("Flange", ef.echo(2.62, echoTime, echoFeedback) : flanger(-1,1));
// flange = hgroup("Flange", flanger(1) : flanger(-1));
// flange = hgroup("Flange", ef.echo(2.62, 0.32, 0.38) : flanger(-1,1));
//=============================================
// uncomment one of the process lines below
// process =  _,_: + :> component("tubes.lib").T2_12AX7 :  flange <: _,_;
// process =  _,_: + :> component("tubes.lib").T2_12AX7 <: flanger(1,-1), flange : _,_;
process =  _,_: + :> component("tubes.lib").T2_12AX7 : flange <: _,_;
// process = _,_ :> flange <: _,_;
