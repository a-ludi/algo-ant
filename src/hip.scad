include <defs.scad>;
include <util.scad>;

i_hd_end_thickness = 0;
i_hd_inner_height = 1;
i_hd_start_thickness = 2;
i_hd_effective_length = 3;
i_hd_max_turn = 4;

module hip(adjecent_leg_part) {
    alp_effective_length = adjecent_leg_part[i_ld_effective_length];
    alp_has_servo = adjecent_leg_part[i_ld_has_servo];
    alp_inner_width = adjecent_leg_part[i_ld_inner_width];
    alp_joint_type = adjecent_leg_part[i_ld_joint_type];
    alp_servo_joint_distance = adjecent_leg_part[i_ld_servo_joint_distance];
    alp_start_thickness = adjecent_leg_part[i_ld_start_thickness];
    alp_end_thickness = adjecent_leg_part[i_ld_end_thickness];
    alp_turn_bias = adjecent_leg_part[i_ld_turn_bias];

    hip_descriptor = [
        alp_inner_width, // end_thickness
        alp_start_thickness + board_thickness + clearance_margin, // inner_height
        180*mm, // start_thickness
        80*mm, // effective_length
        26 // max_turn
    ];

    hip_sides(hip_descriptor);

    children();
}

module hip_sides(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    end_thickness = hip_descriptor[i_hd_end_thickness];
    start_thickness = hip_descriptor[i_hd_start_thickness];
    effective_length = hip_descriptor[i_hd_effective_length];
    max_turn = hip_descriptor[i_hd_max_turn];

    if (max_turn < 0)
        warn("choose positive <code><b>max_turn</b></code>");

    alpha = 90 - max_turn;
    r_bridge1 = (end_thickness/2*sin(alpha) - start_thickness/2)/
                (1 - 1/PHI - sin(alpha));
    r_bridge2 = r_bridge1/PHI;
    delta_v = (end_thickness/2 + r_bridge1)*cos(alpha);
    delta_h = (end_thickness/2 + r_bridge1)*sin(alpha);

    if (r_bridge1 < small_hip_bridge_threshold)
        warn("choose larger <code><b>start_thickness</b></code> or smaller <code><b>max_turn</b></code>");
    if (delta_h < r_bridge1)
        warn("choose smaller <code><b>start_thickness</b></code> or smaller <code><b>max_turn</b></code>");
    if (effective_length < delta_v + r_bridge2)
        warn("choose larger <code><b>effective_length</b></code>");

    for (vertical_offset = inner_height/2*[-1, 1]) {
        translate(vertical_offset*Z) {
            // end part
            cylinder(board_thickness, d=end_thickness, center=true, $fn=end_thickness);

            // bridge part
            difference() {
                linear_extrude(board_thickness, center=true)
                    polygon([
                        [0, 0],
                        [delta_h, -delta_v],
                        [-delta_h, -delta_v]
                    ]);

                translate(-delta_v*Y)
                    for (s = [-1, 1])
                        translate(s*delta_h*X)
                            cylinder(board_thickness + eps, r=r_bridge1, center=true);
            }

            difference() {
                linear_extrude(board_thickness, center=true)
                    polygon([
                        [start_thickness/2, -delta_v],
                        [-start_thickness/2, -delta_v],
                        [-start_thickness/2, -(delta_v + r_bridge2)],
                        [-start_thickness/2, -effective_length],
                        [start_thickness/2, -effective_length],
                        [start_thickness/2, -(delta_v + r_bridge2)]
                    ]);

                translate(-delta_v*Y)
                    for (s = [-1, 1])
                        translate(s*start_thickness/2*X)
                            cylinder(board_thickness + eps, r=r_bridge2, center=true);
            }
        }
    }
}

module hip_sides_draft1(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    end_thickness = hip_descriptor[i_hd_end_thickness];
    start_thickness = hip_descriptor[i_hd_start_thickness];
    effective_length = hip_descriptor[i_hd_effective_length];

    gap = board_thickness + clearance_margin;
    r_gap = 3*gap;
    delta_parts = gap + (start_thickness + end_thickness)/2;
    delta_v = (pow(end_thickness/2 + r_gap, 2) - pow(start_thickness/2 + r_gap, 2) + pow(delta_parts, 2))/(2*delta_parts);
    delta_h = sqrt(pow(end_thickness/2 + r_gap, 2) - pow(delta_v, 2));
    r_gap_factor = 1 + 0.1*pow(delta_v, 2)/(pow(delta_h, 2) + pow(delta_v, 2));
    delta_start = sqrt(2)*start_thickness/2;

    for (vertical_offset = inner_height/2*[-1, 1]) {
        translate(vertical_offset*Z) {
            // end part
            cylinder(board_thickness, d=end_thickness, center=true, $fn=end_thickness);

            // start part (draft #1)
            translate(-delta_parts*Y) {
                intersection() {
                    cylinder(board_thickness, d=start_thickness, center=true);
                    translate(delta_start/2*Y)
                        cube([2*delta_start, delta_start, board_thickness + eps], center=true);
                }

                difference() {
                    translate(delta_start/4*Y)
                        cube([2*delta_start, delta_start/2, board_thickness], center=true);

                    for (offset = delta_start*[-1, 1]) {
                        translate([offset, delta_start])
                            cylinder(board_thickness + eps, d=start_thickness, center=true);
                    }
                }
            }

            // gap part
            difference() {
                linear_extrude(board_thickness, center=true)
                    polygon([
                        [0, 0],
                        [delta_h, -delta_v],
                        [0, -delta_parts],
                        [-delta_h, -delta_v]
                    ]);


                for (horizontal_offset = delta_h*[-1, 1]) {
                    translate([horizontal_offset, -delta_v, 0])
                        cylinder(board_thickness + eps, r=r_gap_factor*r_gap, center=true);
                }
            }
        }
    }
}
