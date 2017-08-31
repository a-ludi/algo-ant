include <defs.scad>;
include <util.scad>;

module leg() {
    translate([0, joint_leg_descriptor[i_ld_start_thickness]/2, 0]) {
        joint_leg();

        translate([0, joint_leg_descriptor[i_ld_effective_length], 0]) {
            rotate([-upper_leg_descriptor[i_ld_turn_bias], 0, 0]) {
                upper_leg();

                translate([0, upper_leg_descriptor[i_ld_effective_length], 0])
                    rotate([-middle_leg_descriptor[i_ld_turn_bias], 0, 0]) {
                        middle_leg();

                        translate([0, middle_leg_descriptor[i_ld_effective_length], 0])
                            rotate([-lower_leg_descriptor[i_ld_turn_bias], 0, 0])
                                lower_leg();
                    }
            }
        }
    }
}

module joint_leg() {
    generic_leg(joint_leg_descriptor);
}

module upper_leg() {
    generic_leg(upper_leg_descriptor);
}

module middle_leg() {
    generic_leg(middle_leg_descriptor);
}

module lower_leg() {
    generic_leg(lower_leg_descriptor);
}


module generic_leg(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    has_servo = leg_descriptor[i_ld_has_servo];
    joint_type = leg_descriptor[i_ld_joint_type];

    translate([0, effective_length/2, 0]) {
        if (has_servo && !hide_servos)
            generic_leg_servo(leg_descriptor);
        generic_leg_skeleton(leg_descriptor);
        if (joint_type == "hinge")
            generic_leg_tendon_insertion_for_hinge(leg_descriptor);
        generic_leg_axle(leg_descriptor);
    }
}

module generic_leg_servo(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    inner_width = leg_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_descriptor[i_ld_servo_joint_distance];

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            sg90_servo();
            translate([0, 0, 15*mm - 5.5*mm])
                servo_horn(2*servo_horn_radius, 4);
        }
}

module generic_leg_skeleton(leg_descriptor) {
    joint_type = leg_descriptor[i_ld_joint_type];

    color(c_board) {
        generic_leg_skeleton_sides(leg_descriptor);
        generic_leg_skeleton_center_link(leg_descriptor);
    }
}

module generic_leg_skeleton_sides(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    has_servo = leg_descriptor[i_ld_has_servo];
    inner_width = leg_descriptor[i_ld_inner_width];
    joint_type = leg_descriptor[i_ld_joint_type];
    start_thickness = leg_descriptor[i_ld_start_thickness];
    end_thickness = leg_descriptor[i_ld_end_thickness];

    for (offset = (inner_width + board_thickness)/2 * [-1, 1]) {
        difference() {
            // basic sides
            union() {
                translate([offset, 0, 0])
                    rotate([90, 0, 90]) {
                        mic_board_with_holes(
                            start_thickness/2,
                            end_thickness/2,
                            effective_length,
                            joint_type == "cardan"
                                ? 0
                                : joint_axle_diameter_out/2,
                            joint_axle_screw_diameter/2,
                            center=true
                        );

                        if (joint_type == "cardan")
                            translate([-(effective_length + start_thickness/2)/2, 0, 0])
                                cube([start_thickness/2, start_thickness, board_thickness], center=true);
                    }
            }

            if (joint_type == "hinge")
                generic_leg_skeleton_hinge_fixation_holes(leg_descriptor);

            if (has_servo)
                generic_leg_servo_cutting(leg_descriptor);
        }
    }
}

module generic_leg_skeleton_hinge_fixation_holes(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    inner_width = leg_descriptor[i_ld_inner_width];
    start_thickness = leg_descriptor[i_ld_start_thickness];
    turn_bias = leg_descriptor[i_ld_turn_bias];

    translate([0, -effective_length/2, 0]) {
        for (vertical_offset = (start_thickness - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
            rotate([turn_bias, 0, 0])
            translate([0, 0, vertical_offset])
                rotate([0, 90, 0])
                    cylinder(
                        h=inner_width + 3*board_thickness,
                        r=joint_axle_diameter_in/2,
                        center=true
                    );
    }
}

module generic_leg_skeleton_center_link(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    has_servo = leg_descriptor[i_ld_has_servo];
    inner_width = leg_descriptor[i_ld_inner_width];
    joint_type = leg_descriptor[i_ld_joint_type];
    start_thickness = leg_descriptor[i_ld_start_thickness];
    end_thickness = leg_descriptor[i_ld_end_thickness];

    link_length = joint_type == "cardan"
        ? effective_length + start_thickness/2 - end_thickness/2 - clearance_margin
        : effective_length - (start_thickness + end_thickness)/2 - 2*clearance_margin;
    smaller_thickness = min(start_thickness, end_thickness);
    offset_set = smaller_thickness < single_center_link_threshold
        ? [0]
        : [-1, 1];

    difference() {
        union() {
            for (offset = (smaller_thickness - board_thickness)/2*offset_set)
                translate([
                    0,
                    effective_length/2 - link_length/2 - end_thickness/2 - clearance_margin,
                    offset
                ]) {
                    cube([inner_width, link_length, board_thickness], center=true);

                    if (joint_type == "cardan")
                        translate([0, -link_length/2, 0])
                            cylinder(board_thickness, d=start_thickness, center=true);
                }
        }

        if (has_servo)
            difference() {
                generic_leg_servo_cutting(leg_descriptor);

                if (joint_type == "cardan")
                    translate([
                        0,
                        effective_length/2 - link_length - end_thickness/2 - clearance_margin,
                        0
                    ])
                        cylinder(
                            start_thickness+2*eps,
                            d=3*joint_axle_diameter_out,
                            center=true
                        );
            };

        if (joint_type == "cardan")
            translate([
                0,
                effective_length/2 - link_length - end_thickness/2 - clearance_margin,
                0
            ])
                cylinder(start_thickness+2*eps, d=joint_axle_diameter_out, center=true);
    }
}

module generic_leg_tendon_insertion_for_hinge(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    inner_width = leg_descriptor[i_ld_inner_width];
    start_thickness = leg_descriptor[i_ld_start_thickness];
    turn_bias = leg_descriptor[i_ld_turn_bias];

    color(c_brass) {
        translate([0, -effective_length/2, 0]) {
            // brass tubes form the insertion
            for (vertical_offset = (start_thickness - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
                rotate([turn_bias, 0, 0])
                translate([0, 0, vertical_offset])
                    rotate([0, 90, 0])
                        tube(
                            h=inner_width,
                            r_out=tendon_insertion_diameter_out/2,
                            r_in=tendon_insertion_diameter_in/2,
                            center=true
                        );
        }
    }
}

module generic_leg_axle(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    inner_width = leg_descriptor[i_ld_inner_width];

    translate([0, effective_length/2, 0]) {
        rotate([0, 90, 0]) {
            // brass tube
            color(c_brass)
                tube(
                    h=inner_width,
                    r_out=joint_axle_diameter_out/2,
                    r_in=joint_axle_diameter_in/2,
                    center=true
                );
            // fixation bolt
            color(c_steel) {
                translate([0, 0, -(inner_width + 2*board_thickness)/2])
                    my_bolt(
                        joint_axle_screw_diameter,
                        inner_width + 2*board_thickness + 1.2*my_flat_nut_thickness(joint_axle_screw_diameter)
                    );
                translate([0, 0, (inner_width + 2*board_thickness)/2])
                    my_flat_nut(joint_axle_screw_diameter);
            }
        }
    }
}

module generic_leg_servo_cutting(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    end_thickness = leg_descriptor[i_ld_end_thickness];
    inner_width = leg_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_descriptor[i_ld_servo_joint_distance];

    servo_horn_arm_width = 4*mm;
    servo_horn_arm_thickness = 1.7*mm;
    servo_body_width = 11.8*mm;
    servo_body_length = 22.2*mm;
    servo_mount_length = 32.2*mm;
    servo_axle_height = 31*mm;
    servo_mount_height = 16*mm;

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            // upper part
            let(
                width = servo_body_width + 2*clearance_margin,
                length = servo_mount_length + 2*clearance_margin,
                height = servo_axle_height - servo_mount_height + 2*clearance_margin,
                overlap = clearance_margin/2 + eps
            ) {
                translate(-[width/2, servo_body_width, 0]) {
                    cube([width, length, height]);
                    translate([-inner_width + overlap, 0, 0])
                        cube([inner_width + overlap, length, inner_width]);
                }
            }

            // horn + tendon
            let(
                r_horn = servo_horn_radius + servo_horn_arm_width/2 + clearance_margin,
                r_insertion = end_thickness/2 + clearance_margin,
                height = servo_horn_arm_thickness + 3*clearance_margin,
                z_shift = servo_axle_height - servo_mount_height - height + 2*clearance_margin
            )
                translate([0, 0, z_shift])
                    hull() {
                        cylinder(height, r=r_horn);
                        translate([0, -servo_joint_distance, 0])
                            cylinder(height, r=r_insertion);
                    }

            // lower part
            let(
                width = servo_body_width + 2*clearance_margin,
                length = servo_body_length + 2*clearance_margin,
                height = servo_mount_height + clearance_margin + eps
            )
                translate(-[
                    width/2,
                    width/2,
                    height - eps
                ]) {
                    cube([width, length, height]);
        }
    }
}
