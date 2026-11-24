#Rescue Op  Gérard Robin  Copyright
#Au sol 0.07 => Agl ft 3.7
# Alt 1 => Agl ft 31.9  (rope on ground/sea)
# Man rescued heigh => 6.56 ft

# The rope's length follows the height above the ground only while a rescue is under way: the timer runs from the moment
# /sim/model/rescue goes to 1 until it goes back (it used to run all the time, twenty times a second).

var AGL_REF = 33.51;
var agl_low = 0;
var rescue_lift = 1;
var VIEW = "Rescued_View";
var yViewNode = props.globals.getNode("sim/current-view/y-offset-m", 1);
var viewNameNode = props.globals.getNode("sim/current-view/name", 1);
var rescue_liftNode = props.globals.getNode("sim/model/rescue-lift", 1);
var rescueNode = props.globals.getNode("sim/model/rescue", 1);
var aglNode = props.globals.getNode("position/altitude-agl-ft", 1);
var solidNode = props.globals.getNode("fdm/jsbsim/environment/terrain-solid", 1);

#print (AGL_REF);
setprop ("/sim/model/rescue-lift",0);
setprop ("/position/altitude-agl-ft",0.01);


var rope_size = func {
	if (rescueNode.getValue() != 1) return;

	var agl = aglNode.getValue();
	var rope = rescue_liftNode.getValue();
	if (solidNode.getValue() == 1) {
#on ground
		agl_low = agl - 1;
	}else{
#in water
		agl_low = agl + 3;
	}

	if ( rope * AGL_REF  > agl_low )  {
		rescue_lift = agl_low / AGL_REF;
		if (rescue_lift < 0) {
			rescue_lift = 0;
		}
		rescue_liftNode.setValue(0);
		rescue_liftNode.setValue(rescue_lift);
	}
	if (viewNameNode.getValue() == VIEW) {
		yViewNode.setValue(1.5 - rescue_liftNode.getValue() * 10);
	}
};

var _timer = maketimer(0.05, rope_size);
_timer.simulatedTime = 1;               # as settimer: it waits while the simulation does

setlistener("/sim/model/rescue", func {
	if (rescueNode.getValue() == 1) {
		if (!_timer.isRunning) _timer.start();
	} else {
		_timer.stop();
	}
}, 1, 0);
