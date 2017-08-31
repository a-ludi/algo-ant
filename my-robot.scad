include <include/legs.scad>;

$fn = 20;

leg(sin($t*180)*[-20, -40, 60, 5 - 10*abs(sin($t*360))]);
