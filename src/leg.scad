include <defs.scad>;
include <util.scad>;

module leg(turn) {
    leg_part(leg_part_descriptor_joint, turn[0])
        leg_part(leg_part_descriptor_upper, turn[1])
            leg_part(leg_part_descriptor_middle, turn[2])
                leg_part(leg_part_descriptor_lower, turn[3]);
}

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
        leg_part_servo_cutting(leg_part_descriptor, for_center_link=true);

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

module leg_part_servo_cutting(leg_part_descriptor, for_center_link=false) {
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

    center_link_correction = for_center_link ? eps : 0;

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            // upper part + maintenance
            let(
                width = servo_body_width + 2*clearance_margin,
                length = servo_mount_length + 2*clearance_margin,
                height = servo_axle_height - servo_mount_height + 2*clearance_margin + center_link_correction,
                overlap = clearance_margin/2 + eps
            ) {
                translate(-[width/2, servo_body_width, center_link_correction]) {
                    cube([width, length, height]);
                    translate([-inner_width + overlap, 0, center_link_correction/2])
                        cube([inner_width + overlap, length, inner_width + center_link_correction]);
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

module leg_milling_layout(leg_part_descriptor) {
    color(c_board) {
        difference() {
            cube([
                base_board_size[0],
                base_board_size[1],
                board_thickness
            ]);

            translate(-eps*Z)
                minkowski() {
                    leg_layout(hull_only=true);
                    cylinder(2*eps, r=layout_margin/4);
                }
        }

        leg_layout(hull_only=false);
        leg_milling_layout_retainers();
    }
}

module leg_milling_layout_retainers() {
    translate([0, 40*mm, 0])
        milling_retainer(80*mm);
    translate([0, 85*mm, 0])
        milling_retainer(80*mm);
    translate([70*mm, 5*mm, 0])
        milling_retainer(65*mm);
    translate([95*mm, 50*mm, 0])
        milling_retainer(40*mm);
    translate([110*mm, 50*mm, 0])
        rotate(90*Z)
            milling_retainer(10*mm);
    translate([125*mm, 50*mm, 0])
        rotate(90*Z)
            milling_retainer(10*mm);
    translate([135*mm, 15*mm, 0])
        milling_retainer(65*mm);
    translate([135*mm, 60*mm, 0])
        milling_retainer(40*mm);
    translate([175*mm, 35*mm, 0])
        milling_retainer(25*mm);

    translate([160*mm, 80*mm, 0])
        rotate(90*Z)
            milling_retainer(40*mm);
    translate([160*mm, 145*mm, 0])
        rotate(90*Z)
            milling_retainer(40*mm);
    translate([197*mm, 85*mm, 0])
        milling_retainer(5*mm);
    translate([197*mm, 105*mm, 0])
        milling_retainer(5*mm);
    translate([197*mm, 132*mm, 0])
        milling_retainer(5*mm);
    translate([197*mm, 168*mm, 0])
        milling_retainer(5*mm);
    translate([197*mm, 203*mm, 0])
        milling_retainer(5*mm);
    translate([175*mm, 190*mm, 0])
        milling_retainer(5*mm);
    translate([175*mm, 210*mm, 0])
        milling_retainer(5*mm);
    translate([75*mm, 97*mm, 0])
        milling_retainer(60*mm);
    translate([20*mm, 125*mm, 0])
        milling_retainer(35*mm);
    translate([20*mm, 135*mm, 0])
        milling_retainer(35*mm);
    translate([75*mm, 135*mm, 0])
        milling_retainer(5*mm);
    translate([20*mm, 180*mm, 0])
        milling_retainer(110*mm);
    translate([20*mm, 200*mm, 0])
        milling_retainer(30*mm);
}

module milling_retainer(length, center=false) {
    translate((center ? board_thickness/2 : 0)*Z)
        cube([length, milling_retainer_width, board_thickness], center=center);
}

module leg_layout(leg_part_descriptors, show_base_board=false, show_bounding_boxes=false, hull_only=false) {
    layout_size = [
        leg_part_layout_bounding_box_size(leg_part_descriptor_upper)[0] +
            leg_part_layout_bounding_box_size(leg_part_descriptor_lower)[0],
        leg_part_layout_bounding_box_size(leg_part_descriptor_lower)[1] +
            leg_part_layout_bounding_box_size(leg_part_descriptor_joint)[0],
    ];

    if (show_base_board)
        color(c_marker)
            translate((-board_thickness + eps)*Z)
                cube([
                    base_board_size[0],
                    base_board_size[1],
                    board_thickness
                ]);

    leg_part_layout(leg_part_descriptor_upper, show_bounding_box=show_bounding_boxes, hull_only=hull_only);
    translate(leg_part_layout_bounding_box_size(leg_part_descriptor_upper)[0]*X)
        leg_part_layout(leg_part_descriptor_lower, show_bounding_box=show_bounding_boxes, hull_only=hull_only);

    translate([
        layout_size[0] - leg_part_layout_bounding_box_size(leg_part_descriptor_joint)[1],
        leg_part_layout_bounding_box_size(leg_part_descriptor_middle)[1] + leg_part_layout_bounding_box_size(leg_part_descriptor_lower)[1],
        0
    ])
        rotate(180*Z)
            leg_part_layout(leg_part_descriptor_middle, show_bounding_box=show_bounding_boxes, hull_only=hull_only);

    translate([
        layout_size[0],
        leg_part_layout_bounding_box_size(leg_part_descriptor_lower)[1],
        0
    ]) {
        rotate(90*Z)
            leg_part_layout(leg_part_descriptor_joint, show_bounding_box=show_bounding_boxes, hull_only=hull_only);
    }

    echo(
        layout_width = layout_size[0],
        layout_height = layout_size[1]
    );
}

module leg_part_layout(leg_part_descriptor, show_bounding_box, hull_only) {
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
            if (hull_only)
                hull() leg_part_skeleton_sides_single(leg_part_descriptor, FRONT);
            else
                leg_part_skeleton_sides_single(leg_part_descriptor, FRONT);

            translate((start_thickness + layout_margin)*X) {
                rotate(180*Z)
                    if (hull_only)
                        hull() leg_part_skeleton_sides_single(leg_part_descriptor, BACK);
                    else
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
                            if (hull_only)
                                hull() leg_part_skeleton_center_link_single(leg_part_descriptor, sides_set[i]);
                            else
                                leg_part_skeleton_center_link_single(leg_part_descriptor, sides_set[i]);

            }
        }
    }

    if (show_bounding_box)
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
