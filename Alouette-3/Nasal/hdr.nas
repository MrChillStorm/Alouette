###########################################################################
##                           HDR detection                               ##
##                                                                       ##
## Thx to : Sergei "Skydive" Solyshko                                    ##
###########################################################################

props.globals.initNode("/sim/rendering/hdr/hdr-enabled", 0, "BOOL");

var al3HdrListener = setlistener("sim/signals/fdm-initialized", func
{
  ###########################################################################
  var hdr = getprop("/sim/rendering/default-compositor") or "";
  var v = split(".", getprop("/sim/version/flightgear") or "0.0");
  var next = num(v[0]) > 2024 or (num(v[0]) == 2024 and size(v) > 1 and num(v[1]) >= 2);   # HDR is its only pipeline
  if (find("HDR", hdr) >= 0 or next) {
    setprop("/sim/rendering/hdr/hdr-enabled", 1);
  } else {
    setprop("/sim/rendering/hdr/hdr-enabled", 0);
  }
  removelistener(al3HdrListener);
}, 0, 0);
