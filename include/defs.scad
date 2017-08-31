// utility variables
include <MCAD/units.scad>;

inf = 1e200 * 1e200;

// controlling the output
hide_servos = false;

// dimensions
eps = 0.1*mm;
board_thickness = 1.5*mm;
joint_axle_screw_diameter = M3;
joint_axle_diameter_in = 3*mm;
joint_axle_diameter_out = joint_axle_diameter_in + 0.45*mm;
servo_horn_radius = 7.5*mm;
// TODO servos will use < 180
tendon_insertion_diameter_hole = 0.8*mm;
tendon_insertion_diameter_in = 3*mm;
tendon_insertion_diameter_out = tendon_insertion_diameter_in + 0.45*mm;
clearance_margin = 1*mm;
leg_base_inner_width = 2*13.3*mm;
joint_leg_descriptor = [
    37*mm, // effective_length
    true, // has_servo
    leg_base_inner_width + 2*board_thickness + 1*clearance_margin, // inner_width
    "cardan", // joint_type
    27*mm, // servo_joint_distance
    30*mm, // start_thickness
    30*mm, // end_thickness
    0 // turn_bias
];
upper_leg_descriptor = [
    80*mm, // effective_length
    true, // has_servo
    leg_base_inner_width + 0.5*clearance_margin, // inner_width
    "hinge", // joint_type
    35*mm, // servo_joint_distance
    30*mm, // start_thickness
    20*mm, // end_thickness
    -30 // turn_bias
];
middle_leg_descriptor = [
    100*mm, // effective_length
    true, // has_servo
    leg_base_inner_width - 2*board_thickness, // inner_width
    "hinge", // joint_type
    40*mm, // servo_joint_distance
    20*mm, // start_thickness
    15*mm, // end_thickness
    80 // turn_bias
];
lower_leg_descriptor = [
    60*mm, // effective_length
    false, // has_servo
    leg_base_inner_width - 4*board_thickness - 0.5*clearance_margin, // inner_width
    "hinge", // joint_type
    0*mm, // servo_joint_distance
    15*mm, // start_thickness
    5*mm, // end_thickness
    -30 // turn_bias
];

i_ld_effective_length = 0;
i_ld_has_servo = 1;
i_ld_inner_width = 2;
i_ld_joint_type = 3;
i_ld_servo_joint_distance = 4;
i_ld_start_thickness = 5;
i_ld_end_thickness = 6;
i_ld_turn_bias = 7;



// colors
c_aluminum = "LightGrey";
c_brass = [242, 198, 85]/255;
c_steel = "LightSteelBlue";
c_board = "BurlyWood";
c_servo = "Blue";
c_servo_horn = "LightCyan";
