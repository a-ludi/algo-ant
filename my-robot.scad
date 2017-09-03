include <src/leg.scad>;
include <src/hip.scad>;
use <include/raspberry-pi/rpi.scad>;

if (view_mode == "assembled") {
    $fa = 10;
    $fs = 1.5;

    // distance between hip joint: [180mm, 80mm]
    // resulting max turn with [-10, 0, 10] bias: 26

    for (m = [0, 1])
        mirror(m*Y)
            hip(front_hip_descriptor, leg_part_descriptor_joint) {
                leg([0, 0, 0, 0]);

                hip(center_hip_descriptor, leg_part_descriptor_joint) {
                    leg([0, 0, 0, 0]);

                    rpi_back_hip_descriptor = set_at(
                        back_hip_descriptor,
                        i_hd_has_rpi,
                        true
                    );

                    hip(
                        m == 0
                            ? rpi_back_hip_descriptor
                            : back_hip_descriptor,
                        leg_part_descriptor_joint
                    )
                       leg([0, 0, 0, 0]);
                }
            }
} else if (view_mode == "layout") {
    $fa = 5;
    $fs = 0.5;

    leg_layout(show_base_board=true, show_bounding_boxes=false);
} else if (view_mode == "milling_layout") {
    $fa = 2.5;
    $fs = 0.25;

    leg_milling_layout();
} else {
    warn("unkown <code><b>view_mode</b></code>; choose one of <code>assembled</code>, <code>layout</code>, <code>milling_layout</code>");
}