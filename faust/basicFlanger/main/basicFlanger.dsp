import("stdfaust.lib");
flaDelay = hslider("[3]Delay", 156, 5, 1000, 1) : si.smoo;
flaFeedback = hslider("[4]Flange Fb", 0.2, 0, 0.97, 0.01) : si.smoo;
flaDepth = hslider("[5]Flange Dep", 0.95, 0, 1.0, 0.01) : si.smoo;
flaLFORate = hslider("[6]Rate", 0.25, 0, 3, 0.01) : si.smoo;
flaLFOWidth = hslider("[7]Width", 0.5, 0, 1.0, 0.01) : si.smoo;
flaLFO = os.lf_triangle(flaLFORate);
flaMod = flaLFOWidth * (flaLFO/2) ;
flanger(x,y) = pf.flanger_mono(512, flaDelay * (1 + (x * flaMod)), flaDepth, flaFeedback, y);
//=============================================
// uncomment one of the flange lines below
flange = hgroup("Flange", flanger(1,-1) : ef.echo(2.62, 0.32, 0.48) : flanger(-1,1));
// flange = hgroup("Flange", flanger(1) : flanger(-1));
// flange = hgroup("Flange", ef.echo(2.62, 0.32, 0.38) : flanger(-1,1));
//=============================================
// uncomment one of the process lines below
// process =  _,_: + :> component("tubes.lib").T1_12AX7 : flange <: _,_;
// process = _,_ :> component("tubes.lib").T1_12AX7 : flange : re.jcrev :> _,_;
process = _,_ :> flange : re.jcrev :> _,_;
