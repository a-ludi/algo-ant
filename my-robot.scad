include <src/leg.scad>;
include <src/hip.scad>;
use <include/raspberry-pi/rpi.scad>;


leg_parts = [
    leg_part_descriptor_joint,
    leg_part_descriptor_upper,
    leg_part_descriptor_middle,
    leg_part_descriptor_lower
];

if (view_mode == "assembled") {
    $fa = 10;
    $fs = 1.5;

    // distance between hip joint: [180mm, 80mm]
    // resulting max turn with [-10, 0, 10] bias: 26

    hip_adjecent_leg_part = generate_leg_part_descriptor(leg_parts);

    for (m = [0, 1])
        mirror(m*Y)
            hip(front_hip_descriptor, hip_adjecent_leg_part) {
                leg_part([
                    set_at(leg_part_descriptor_joint, i_ld_turn_bias, 10),
                    leg_part_descriptor_upper,
                    leg_part_descriptor_middle,
                    leg_part_descriptor_lower
                ], [0, 0, 0, 0]);

                hip(center_hip_descriptor, hip_adjecent_leg_part) {
                    leg_part([
                        set_at(leg_part_descriptor_joint, i_ld_turn_bias, 0),
                        leg_part_descriptor_upper,
                        leg_part_descriptor_middle,
                        leg_part_descriptor_lower
                    ], [0, 0, 0, 0]);

                    hip(
                        m == 0
                            ? set_at(back_hip_descriptor, i_hd_has_rpi, true)
                            : back_hip_descriptor,
                        hip_adjecent_leg_part
                    )
                       leg_part([
                            set_at(leg_part_descriptor_joint, i_ld_turn_bias, -10),
                            leg_part_descriptor_upper,
                            leg_part_descriptor_middle,
                            leg_part_descriptor_lower
                        ], [0, 0, 0, 0]);
                }
            }
} else if (view_mode == "layout") {
    $fa = 5;
    $fs = 0.5;

    leg_layout(leg_parts, show_base_board=true, show_bounding_boxes=false);
} else if (view_mode == "milling_layout") {
    $fa = 2.5;
    $fs = 0.25;

    leg_milling_layout(leg_parts);
} else {
    warn("unkown <code><b>view_mode</b></code>; choose one of <code>assembled</code>, <code>layout</code>, <code>milling_layout</code>");
}