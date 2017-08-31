// utility variables
include <MCAD/units.scad>;

inf = 1e200 * 1e200;

// dimensions
eps = 0.1*mm;
board_thickness = 1.5*mm;
joint_axle_screw_diameter = M3;
joint_axle_diameter_in = 3*mm;
joint_axle_diameter_out = joint_axle_diameter_in + 0.45*mm;
servo_horn_radius = 7.5*mm;
// TODO servos will use < 180
servo_joint_transmission_ratio = 120/180;
tendon_insertion_diameter_in = 3*mm;
tendon_insertion_diameter_out = tendon_insertion_diameter_in + 0.45*mm;
clearance_margin = 1*mm;
upper_leg_descriptor = [
    80*mm, // effective_length
    true, // has_servo
    2*11.1*mm, // inner_width
    35*mm, // servo_joint_distance
    30*mm, // start_thickness
    20*mm, // end_thickness
    -30 // turn_bias
];
middle_leg_descriptor = [
    100*mm, // effective_length
    true, // has_servo
    2*(11.1 - 1.7)*mm, // inner_width
    40*mm, // servo_joint_distance
    20*mm, // start_thickness
    15*mm, // end_thickness
    80 // turn_bias
];
lower_leg_descriptor = [
    60*mm, // effective_length
    false, // has_servo
    2*(11.1 - 2*1.7)*mm, // inner_width
    0*mm, // servo_joint_distance
    15*mm, // start_thickness
    5*mm, // end_thickness
    -30 // turn_bias
];

i_ld_effective_length = 0;
i_ld_has_servo = 1;
i_ld_inner_width = 2;
i_ld_servo_joint_distance = 3;
i_ld_start_thickness = 4;
i_ld_end_thickness = 5;
i_ld_turn_bias = 6;



// colors
c_aluminum = "LightGrey";
c_brass = [242, 198, 85]/255;
c_steel = "LightSteelBlue";
c_board = "BurlyWood";
c_servo = "Blue";
c_servo_horn = "LightCyan";
