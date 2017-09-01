include <include/leg.scad>;
include <include/hip.scad>;

$fa = 30;
$fs = 5;

alpha = 26;

// distance between hip joint: [180mm, 80mm]
// resulting max turn with [-10, 0, 10] bias: 26

hip(leg_part_descriptor_joint)
    leg(sin($t*180)*[-20, -40, 60, 5 - 10*abs(sin($t*360))]);
