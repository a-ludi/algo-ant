include <defs.scad>;
include <util.scad>;

module leg(turn) {
    leg_part(leg_part_descriptor_joint, turn[0])
        leg_part(leg_part_descriptor_upper, turn[1])
            leg_part(leg_part_descriptor_middle, turn[2])
                leg_part(leg_part_descriptor_lower, turn[3]);
}

module leg_milling_layout(leg_part_descriptor) {
}

module leg_layout(leg_part_descriptors) {
    color(c_marker)
        translate((-board_thickness + eps)*Z)
            cube([
                DIN_A4[0],
                DIN_A4[1],
                board_thickness
            ]);

    leg_part_layout(leg_part_descriptor_upper);
    leg_part_layout_bounding_box(leg_part_descriptor_upper);
    translate(leg_part_layout_bounding_box_size(leg_part_descriptor_upper)[0]*X) {
        leg_part_layout(leg_part_descriptor_lower);
        leg_part_layout_bounding_box(leg_part_descriptor_lower);
    }

    translate([
        leg_part_layout_bounding_box_size(leg_part_descriptor_middle)[1],
        leg_part_layout_bounding_box_size(leg_part_descriptor_upper)[1],
        0
    ])
        rotate(90*Z) {
            leg_part_layout(leg_part_descriptor_middle);
            leg_part_layout_bounding_box(leg_part_descriptor_middle);
        }

    translate([
        leg_part_layout_bounding_box_size(leg_part_descriptor_upper)[0] + leg_part_layout_bounding_box_size(leg_part_descriptor_joint)[1],
        leg_part_layout_bounding_box_size(leg_part_descriptor_lower)[1],
        0
    ])
        rotate(90*Z) {
            leg_part_layout(leg_part_descriptor_joint);
            leg_part_layout_bounding_box(leg_part_descriptor_joint);
        }
}

module leg_part_layout(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    joint_type = leg_part_descriptor[i_ld_joint_type];

    sides_set = leg_part_skeleton_center_link_sides_set(leg_part_descriptor);

    color(c_board) {
        translate([
            start_thickness/2 + layout_margin/2,
            (effective_length + start_thickness)/2 + layout_margin/2,
            board_thickness/2
        ]) {
            leg_part_skeleton_sides_single(leg_part_descriptor, FRONT);
            translate((start_thickness + layout_margin)*X) {
                rotate(180*Z)
                    leg_part_skeleton_sides_single(leg_part_descriptor, BACK);

                translate([
                    inner_width/2 + start_thickness/2 + layout_margin,
                    joint_type == "cardan"
                        ? start_thickness/2
                        : -(start_thickness + clearance_margin),
                    0
                ])
                    for (i = [0 : len(sides_set) - 1])
                        translate(i*(inner_width + layout_margin)*X)
                            leg_part_skeleton_center_link_single(leg_part_descriptor, sides_set[i]);
            }
        }
    }

    leg_part_layout_bounding_box(leg_part_descriptor);
}

module leg_part_layout_bounding_box(leg_part_descriptor) {
    bounding_box_size = leg_part_layout_bounding_box_size(leg_part_descriptor);

    color(c_marker)
        cube([bounding_box_size[0], bounding_box_size[1], board_thickness/8]);
}

function leg_part_layout_bounding_box_size(leg_part_descriptor) =
    let (
        effective_length = leg_part_descriptor[i_ld_effective_length],
        start_thickness = leg_part_descriptor[i_ld_start_thickness],
        end_thickness = leg_part_descriptor[i_ld_end_thickness],
        inner_width = leg_part_descriptor[i_ld_inner_width],
        sides_set = leg_part_skeleton_center_link_sides_set(leg_part_descriptor),
        width = (2 + len(sides_set))*layout_margin + 2*start_thickness + len(sides_set)*inner_width,
        length = layout_margin + effective_length + start_thickness
    )
        [width, length];

module leg_part(leg_part_descriptor, turn) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    joint_type = leg_part_descriptor[i_ld_joint_type];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];

    rotate_axis = joint_type == "cardan"
        ? Z
        : X;
    inline_shift = joint_type == "cardan"
        ? start_thickness/2
        : 0;

    rotate(-(turn_bias + turn)*rotate_axis) {
        translate(inline_shift*Y) {
            leg_part_content(leg_part_descriptor);

            translate(effective_length*Y)
                children();
        }
    }
}

module leg_part_content(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    has_servo = leg_part_descriptor[i_ld_has_servo];
    joint_type = leg_part_descriptor[i_ld_joint_type];

    translate([0, effective_length/2, 0]) {
        if (has_servo && !hide_servos)
            leg_part_servo(leg_part_descriptor);
        leg_part_skeleton(leg_part_descriptor);
        if (joint_type == "hinge")
            leg_part_tendon_insertion_for_hinge(leg_part_descriptor);
        else if (joint_type == "cardan")
            leg_part_tendon_insertion_for_cardan(leg_part_descriptor);
        leg_part_axle(leg_part_descriptor);
    }
}

module leg_part_servo(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_part_descriptor[i_ld_servo_joint_distance];

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            sg90_servo();
            translate([0, 0, 15*mm - 5.5*mm])
                servo_horn(2*servo_horn_radius, 4);
        }
}

module leg_part_skeleton(leg_part_descriptor) {
    joint_type = leg_part_descriptor[i_ld_joint_type];

    color(c_board) {
        leg_part_skeleton_sides(leg_part_descriptor);
        leg_part_skeleton_center_link(leg_part_descriptor);
    }
}

module leg_part_skeleton_sides(leg_part_descriptor) {
    inner_width = leg_part_descriptor[i_ld_inner_width];

    offset = (inner_width + board_thickness)/2;

    for (side = [FRONT, BACK])
        translate(offset*side)
            rotate(90*Y)
                leg_part_skeleton_sides_single(leg_part_descriptor, side);
}

module leg_part_skeleton_sides_single(leg_part_descriptor, side) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    has_servo = leg_part_descriptor[i_ld_has_servo];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    joint_type = leg_part_descriptor[i_ld_joint_type];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];

    offset = (inner_width + board_thickness)/2;

    rotate(-90*Y)
        translate(-offset*side)
            difference() {
                // basic sides
                union() {
                    translate(offset*side)
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
                    leg_part_skeleton_insertion_fixation_holes(leg_part_descriptor);

                if (has_servo)
                    leg_part_servo_cutting(leg_part_descriptor);
            }
}

module leg_part_skeleton_insertion_fixation_holes(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];

    translate([0, -effective_length/2, 0]) {
        for (vertical_offset = (start_thickness - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
            rotate([turn_bias, 0, 0])
            translate([0, 0, vertical_offset])
                rotate([0, 90, 0])
                    cylinder(
                        h=inner_width + board_thickness,
                        r=tendon_insertion_diameter_out/2,
                        center=true
                    );
    }
}

function leg_part_skeleton_center_link_smaller_thickness(leg_part_descriptor) =
    min(leg_part_descriptor[i_ld_start_thickness], leg_part_descriptor[i_ld_end_thickness]);

function leg_part_skeleton_center_link_sides_set(leg_part_descriptor) =
    leg_part_skeleton_center_link_smaller_thickness(leg_part_descriptor) < single_center_link_threshold
        ? [NULL]
        : [TOP, BOTTOM];

function leg_part_skeleton_center_link_offset(leg_part_descriptor) =
    (leg_part_skeleton_center_link_smaller_thickness(leg_part_descriptor) - board_thickness)/2;

module leg_part_skeleton_center_link(leg_part_descriptor) {
    sides_set = leg_part_skeleton_center_link_sides_set(leg_part_descriptor);
    side_offset = leg_part_skeleton_center_link_offset(leg_part_descriptor);

    for (side = sides_set)
        translate(side_offset*side)
            leg_part_skeleton_center_link_single(leg_part_descriptor, side);
}

module leg_part_skeleton_center_link_single(leg_part_descriptor, side) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    has_servo = leg_part_descriptor[i_ld_has_servo];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    joint_type = leg_part_descriptor[i_ld_joint_type];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];


    link_length = leg_part_skeleton_center_link_length(leg_part_descriptor);
    smaller_thickness = leg_part_skeleton_center_link_smaller_thickness(leg_part_descriptor);
    side_offset = leg_part_skeleton_center_link_offset(leg_part_descriptor);
    inline_offset = effective_length/2 - link_length/2 - end_thickness/2 - clearance_margin;

    translate(-side_offset*side)
        difference() {
            translate(inline_offset*Y + side_offset*side) {
                cube([inner_width, link_length, board_thickness], center=true);

                if (joint_type == "cardan")
                    translate([0, -link_length/2, 0])
                        cylinder(board_thickness, d=inner_width, center=true);
            }

            if (has_servo)
                leg_part_skeleton_center_link_servo_cutting(leg_part_descriptor, link_length);

            if (joint_type == "cardan")
                leg_part_skeleton_center_link_cardan_mod(leg_part_descriptor, link_length);
        }
}

function leg_part_skeleton_center_link_length(leg_part_descriptor) =
    let(
        joint_type = leg_part_descriptor[i_ld_joint_type],
        effective_length = leg_part_descriptor[i_ld_effective_length],
        start_thickness = leg_part_descriptor[i_ld_start_thickness],
        end_thickness = leg_part_descriptor[i_ld_end_thickness]
    )
        joint_type == "cardan"
            ? effective_length + start_thickness/2 - end_thickness/2 - clearance_margin
            : effective_length - (start_thickness + end_thickness)/2 - 2*clearance_margin;

module leg_part_skeleton_center_link_servo_cutting(leg_part_descriptor, link_length) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    joint_type = leg_part_descriptor[i_ld_joint_type];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];

    difference() {
        leg_part_servo_cutting(leg_part_descriptor);

        if (joint_type == "cardan")
            translate([
                0,
                effective_length/2 - link_length - end_thickness/2 - clearance_margin,
                0
            ])
                rotate([0, 0, turn_bias])
                    translate([0, -inner_width/2, 0])
                        cube([
                            2*inner_width,
                            3*joint_axle_diameter_out + inner_width,
                            start_thickness+2*eps
                        ], center=true);
    };
}

module leg_part_skeleton_center_link_cardan_mod(leg_part_descriptor, link_length) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    joint_type = leg_part_descriptor[i_ld_joint_type];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];

    translate([
        0,
        effective_length/2 - link_length - end_thickness/2 - clearance_margin,
        0
    ]) {
        cylinder(start_thickness+2*eps, d=joint_axle_diameter_out, center=true);

        for (vertical_offset = (inner_width - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
            rotate([0, 0, turn_bias])
                translate([vertical_offset, 0, 0])
                    cylinder(
                        h=start_thickness + eps,
                        d=tendon_insertion_diameter_out,
                        center=true
                    );
    }
}

module leg_part_tendon_insertion_for_hinge(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];

    color(c_brass) {
        translate([0, -effective_length/2, 0]) {
            // brass tubes form the insertion
            for (vertical_offset = (start_thickness - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
                rotate([turn_bias, 0, 0])
                translate([0, 0, vertical_offset])
                    rotate([0, 90, 0])
                        tendon_insertion(inner_width + board_thickness);
        }
    }
}

module leg_part_tendon_insertion_for_cardan(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    start_thickness = leg_part_descriptor[i_ld_start_thickness];
    turn_bias = leg_part_descriptor[i_ld_turn_bias];

    color(c_brass) {
        translate([0, -(effective_length + start_thickness)/2, 0]) {
            // brass tubes form the insertion
            for (vertical_offset = (inner_width - tendon_insertion_diameter_out - 2*clearance_margin)/2*[-1, 1])
                rotate([0, 0, turn_bias])
                    translate([vertical_offset, 0, 0])
                        tendon_insertion(start_thickness);
        }
    }
}

module leg_part_axle(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    inner_width = leg_part_descriptor[i_ld_inner_width];

    translate([0, effective_length/2, 0])
        rotate([0, -90, 0])
            joint_axle_with_bolt(inner_width);
}

module leg_part_servo_cutting(leg_part_descriptor) {
    effective_length = leg_part_descriptor[i_ld_effective_length];
    end_thickness = leg_part_descriptor[i_ld_end_thickness];
    inner_width = leg_part_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_part_descriptor[i_ld_servo_joint_distance];

    servo_horn_arm_width = 4*mm;
    servo_horn_arm_thickness = 1.7*mm;
    servo_body_width = 11.8*mm;
    servo_body_length = 22.2*mm;
    servo_mount_length = 32.2*mm;
    servo_axle_height = 31*mm;
    servo_mount_height = 16*mm;

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            // upper part + maintenance
            let(
                width = servo_body_width + 2*clearance_margin,
                length = servo_mount_length + 2*clearance_margin,
                height = servo_axle_height - servo_mount_height + 2*clearance_margin,
                overlap = clearance_margin/2 + eps
            ) {
                translate(-[width/2, servo_body_width, 0]) {
                    cube([width, length, height]);
                    translate([-inner_width + overlap, 0, -eps/2])
                        cube([inner_width + overlap, length, inner_width+eps]);
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
            ) {
                translate(-[
                    width/2,
                    width/2,
                    height - eps
                ]) {
                    cube([width, length, height]);

                    // cable shaft
                    translate([
                        (servo_body_width + 2*clearance_margin)/2,
                        length + servo_cable_shaft_diameter,
                        height - eps - 1.5*board_thickness
                    ])
                        cylinder(2*board_thickness, d=servo_cable_shaft_diameter);
                }
            }
        }
}
