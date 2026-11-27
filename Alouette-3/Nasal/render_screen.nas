# The cockpit screen (fdm/renderviews.py): a click goes round the mirror's camera (render view 0), the tail camera (view 1) and the hoist camera (view 2).
# Only the view on show is drawn. Only with a FlightGear that renders views into textures: the screen is hidden in any other, so nothing calls this there.
var enabled = func(n) { return "/sim/rendering/render-views/view[" ~ n ~ "]/enabled"; }

var cycle = func {
    if (getprop(enabled(1))) {              # tail camera -> hoist camera
        setprop(enabled(1), 0);
        setprop(enabled(2), 1);
    } elsif (getprop(enabled(2))) {         # hoist camera -> mirror
        setprop(enabled(2), 0);
    } else {                                # mirror -> tail camera
        setprop(enabled(1), 1);
    }
}
