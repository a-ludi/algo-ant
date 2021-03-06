// utility variables
include <MCAD/units.scad>;

// constants
inf = 1e200 * 1e200;
NULL = [0, 0, 0];
PHI = (1 + sqrt(5))/2;
FRONT = X;
BACK = -X;
LEFT = Y;
RIGHT = -Y;
TOP = Z;
BOTTOM = -Z;
DIN_A0 = [841, 1189]*mm;
DIN_A1 = [594, 841]*mm;
DIN_A2 = [420, 594]*mm;
DIN_A3 = [297, 420]*mm;
DIN_A4 = [210, 297]*mm;
DIN_A5 = [148, 210]*mm;
DIN_A6 = [105, 148]*mm;
DIN_A7 = [74, 105]*mm;
DIN_A8 = [52, 74]*mm;
DIN_A9 = [37, 52]*mm;

// controlling the output
view_mode = "assembled"; // assembled, layout, milling_layout
hide_servos = false;
hide_circuits = false;
hide_axles = false;
hide_tendon_insertion = false;
hide_covers = true;

// dimensions
eps = 0.1*mm;
board_thickness = 2*mm;
joint_axle_screw_diameter = M3;
joint_axle_diameter_in = 3*mm;
joint_axle_diameter_out = joint_axle_diameter_in + 0.45*mm;
servo_cable_shaft_diameter = 10*mm;
servo_horn_radius = 7.5*mm;
rpi_width = 56*mm;
rpi_length = 85*mm;
rpi_height = 1.5*mm;
// TODO servos will use < 180
tendon_insertion_diameter_hole = 0.8*mm;
tendon_insertion_diameter_in = 3*mm;
tendon_insertion_diameter_out = tendon_insertion_diameter_in + 0.45*mm;
single_center_link_threshold = 8*board_thickness;
skeleton_frame_thickness = 7*mm;
clearance_margin = 1*mm;
base_board_size = DIN_A4;
layout_margin = 5*mm;
milling_retainer_width = 0.5*mm;

front_hip_descriptor = [
    undef, // mount_diameter
    undef, // inner_height
    180*mm, // width
    80*mm, // length
    40*mm - 5.2*mm, // servo_joint_distance
    false, // has_servo_driver
    FRONT, // capped_end
    false // has_rpi
];
center_hip_descriptor = [
    undef, // mount_diameter
    undef, // inner_height
    180*mm, // width
    80*mm, // length
    40*mm - 5.2*mm, // servo_joint_distance
    true, // has_servo_driver
    false, // capped_end
    false // has_rpi
];
back_hip_descriptor = [
    undef, // mount_diameter
    undef, // inner_height
    180*mm, // width
    80*mm, // length
    40*mm - 5.2*mm, // servo_joint_distance
    false, // has_servo_driver
    BACK, // capped_end
    false // has_rpi
];

/**
 * Length such that the servo horns are approximately centered.
 */
leg_base_inner_width = 2*13.3*mm - 3.5*board_thickness;
leg_part_descriptor_joint = [
    37*mm, // effective_length
    true, // has_servo
    undef, // inner_width
    "cardan", // joint_type
    27*mm, // servo_joint_distance
    30*mm, // start_thickness
    undef, // end_thickness
    0 // turn_bias
];
leg_part_descriptor_upper = [
    80*mm, // effective_length
    true, // has_servo
    undef, // inner_width
    "hinge", // joint_type
    35*mm, // servo_joint_distance
    30*mm, // start_thickness
    undef, // end_thickness
    -30 // turn_bias
];
leg_part_descriptor_middle = [
    100*mm, // effective_length
    true, // has_servo
    undef, // inner_width
    "hinge", // joint_type
    40*mm, // servo_joint_distance
    20*mm, // start_thickness
    undef, // end_thickness
    90 // turn_bias
];
leg_part_descriptor_lower = [
    60*mm, // effective_length
    false, // has_servo
    leg_base_inner_width, // inner_width
    "hinge", // joint_type
    0*mm, // servo_joint_distance
    15*mm, // start_thickness
    6*mm, // end_thickness
    -40 // turn_bias
];

i_hd_mount_diameter = 0;
i_hd_inner_height = 1;
i_hd_width = 2;
i_hd_length = 3;
i_hd_servo_joint_distance = 4;
i_hd_has_servo_driver = 5;
i_hd_capped_end = 6;
i_hd_has_rpi = 7;

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
c_circuit = "DarkOliveGreen";
c_steel = "LightSteelBlue";
c_board = "BurlyWood";
c_glass = [0.8, 0.8, 0.8, 0.3];
c_marker = [1, 1, 0, 0.3];
c_servo = "Blue";
c_servo_horn = "LightCyan";
