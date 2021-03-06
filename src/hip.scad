include <defs.scad>;
include <util.scad>;

module hip(hip_descriptor, adjecent_leg_part) {
    internal_hip_descriptor = generate_hip_descriptor(hip_descriptor, adjecent_leg_part);

    mount_diameter = internal_hip_descriptor[i_hd_mount_diameter];
    inner_height = internal_hip_descriptor[i_hd_inner_height];
    width = internal_hip_descriptor[i_hd_width];
    length = internal_hip_descriptor[i_hd_length];
    servo_joint_distance = internal_hip_descriptor[i_hd_servo_joint_distance];
    has_servo_driver = internal_hip_descriptor[i_hd_has_servo_driver];
    capped_end = internal_hip_descriptor[i_hd_capped_end];
    has_rpi = internal_hip_descriptor[i_hd_has_rpi];

    translate(length*Y) {
        hip_skeleton(internal_hip_descriptor);

        if (has_servo_driver)
            hip_servo_driver(internal_hip_descriptor);

        if (!hide_servos)
            hip_servo(internal_hip_descriptor);

        hip_axle(internal_hip_descriptor);

        if (has_rpi && !hide_circuits)
            hip_rpi(internal_hip_descriptor);

        // leg
        if ($children > 0)
            children(0);
    }

    // next hip (back)
    if ($children > 1)
        translate((width)*BACK)
            children(1);
}

function generate_hip_descriptor(hip_descriptor, adjecent_leg_part) =
    let(
        alp_inner_width = adjecent_leg_part[i_ld_inner_width],
        alp_end_thickness = adjecent_leg_part[i_ld_end_thickness]
    )
        defaults([
            alp_inner_width + 2*board_thickness, // mount_diameter
            alp_end_thickness + clearance_margin/2, // inner_height
            undef, // width
            undef, // length
            undef, // servo_joint_distance
            undef, // has_servo_driver
            undef, // capped_end
            undef // has_rpi
        ], hip_descriptor);

module hip_rpi(hip_descriptor) {
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    inner_height = hip_descriptor[i_hd_inner_height];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    servo_joint_distance = hip_descriptor[i_hd_servo_joint_distance];
    has_servo_driver = hip_descriptor[i_hd_has_servo_driver];
    capped_end = hip_descriptor[i_hd_capped_end];
    has_rpi = hip_descriptor[i_hd_has_rpi];

    offset_dir = capped_end
        ? -capped_end*X
        : FRONT*X;

    translate([
        -rpi_width/2 + offset_dir*width/4,
        rpi_length/2 - length,
        -inner_height/2 + board_thickness + 5*mm
    ])
        rotate(-90*Z)
            rpi();
}

module hip_skeleton(hip_descriptor) {
    color(c_board) {
        hip_sides(hip_descriptor);
        hip_sides_link(hip_descriptor);

        if (!hide_covers)
            hip_sides_cover(hip_descriptor);
    }
}

module hip_sides(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    has_servo_driver = hip_descriptor[i_hd_has_servo_driver];
    capped_end = hip_descriptor[i_hd_capped_end];

    servo_mount_height = 16*mm;

    capped_end_length = capped_end_length(mount_diameter, servo_mount_height);
    c_width = capped_end
        ? width/2 + capped_end_length
        : width;

    difference() {
        for (vertical_offset = (inner_height + board_thickness)/2*[-1, 1]) {
            translate(vertical_offset*Z) {
                difference() {
                    union() {
                        cylinder(board_thickness, d=mount_diameter, center=true);
                            translate(-length/2*Y + (capped_end ? -(c_width/2 - capped_end_length)*capped_end : NULL))
                                cube([c_width, length, board_thickness], center=true);
                    }

                    cylinder(board_thickness + eps, d=joint_axle_screw_diameter, center=true);
                }
            }
        }

        hip_servo_cutting(hip_descriptor);
        hip_top_cutting(hip_descriptor);
    }
}

module hip_top_cutting(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    capped_end = hip_descriptor[i_hd_capped_end];

    servo_mount_height = 16*mm;

    capped_end_length = capped_end_length(mount_diameter, servo_mount_height);
    c_width = capped_end
        ? width/2 + capped_end_length
        : width;

    translate((inner_height + board_thickness)/2*Z) {
        translate(
            -(length/2 + skeleton_frame_thickness/4)*Y +
            (capped_end
                ? -(c_width/2 - capped_end_length)*capped_end
                : NULL)
        )
            rounded_board([
                c_width - skeleton_frame_thickness,
                length - 1.5*skeleton_frame_thickness,
                board_thickness + eps
            ], r=skeleton_frame_thickness/2, center=true);
    }
}

module hip_sides_cover(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    capped_end = hip_descriptor[i_hd_capped_end];

    servo_mount_height = 16*mm;

    capped_end_length = capped_end_length(mount_diameter, servo_mount_height);
    c_width = capped_end
        ? width/2 + capped_end_length
        : width;

    translate((inner_height + board_thickness)/2*Z) {
        translate(
            -(length/2 + skeleton_frame_thickness/4)*Y +
            (capped_end
                ? -(c_width/2 - capped_end_length)*capped_end
                : NULL) +
            board_thickness*Z
        )
            rounded_board([
                c_width - skeleton_frame_thickness/2,
                length - 1*skeleton_frame_thickness,
                board_thickness + eps
            ], r=3/4*skeleton_frame_thickness, center=true);
            // TODO pegs to fixate cover
    }
}

module hip_sides_link(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    capped_end = hip_descriptor[i_hd_capped_end];

    servo_mount_height = 16*mm;
    servo_body_width = 11.8*mm;

    mount_opening_width = mount_opening_width(mount_diameter);
    mount_opening_servo_cutting = mount_opening_servo_cutting(mount_diameter, servo_mount_height);
    capped_end_length = capped_end_length(mount_diameter, servo_mount_height);
    c_width = width/2 + capped_end_length;

    mirror(capped_end*X > 0 ? X : NULL) {
        // joint side
        for (offset = [0, 1])
            mirror(offset*X)
                translate([
                    mount_opening_width/2 + clearance_margin,
                    -board_thickness,
                    -inner_height/2
                ]) {
                    difference() {
                        cube([
                            offset*X*capped_end
                                ? capped_end_length - mount_opening_width/2 - clearance_margin
                                : (width - mount_opening_width)/2 - clearance_margin,
                            board_thickness,
                            inner_height
                        ]);

                        if (offset > 0)
                            translate([
                                -eps,
                                -eps,
                                (inner_height - servo_body_width - 2*clearance_margin)/2
                            ])
                                cube([
                                    mount_opening_servo_cutting - mount_opening_width/2,
                                    board_thickness + 2*eps,
                                    servo_body_width + 2*clearance_margin
                                ]);
                    }
                }

        // front side
        if (capped_end != FRONT)
            translate([
                (width - board_thickness)/2,
                -(length + board_thickness)/2,
                0
            ]) {
                difference() {
                    cube([
                        board_thickness,
                        length - board_thickness,
                        inner_height
                    ], center=true);

                    let(
                        width = length - board_thickness - 1.5*skeleton_frame_thickness,
                        height = inner_height - 2*skeleton_frame_thickness
                    )
                        translate(-0.25*skeleton_frame_thickness*Y)
                            rotate(90*X + 90*Z)
                                linear_extrude(board_thickness + eps, center=true)
                                    mic_shape(
                                        height/2,
                                        height/2,
                                        width - height,
                                        center=true
                                    );
                }
            }

        // capped side
        if (capped_end)
            translate([
                -(capped_end_length - board_thickness/2),
                -(length + board_thickness)/2,
                0
            ])
                cube([
                    board_thickness,
                    length - board_thickness,
                    inner_height
                ], center=true);
    }
}

function mount_opening_width(mount_diameter) = sqrt(2)*mount_diameter;

function mount_opening_servo_cutting(mount_diameter, servo_mount_height) =
    sqrt(2)*(mount_diameter/2 + servo_mount_height - board_thickness);

function capped_end_length(mount_diameter, servo_mount_height) =
    mount_opening_servo_cutting(mount_diameter, servo_mount_height) +
    skeleton_frame_thickness;

module hip_servo(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    servo_joint_distance = hip_descriptor[i_hd_servo_joint_distance];

    translate(-servo_joint_distance*Y - (inner_height/2)*Z)
        rotate([0, 0, 180]) {
            sg90_servo();
            translate([0, 0, 15*mm - 5.5*mm])
                servo_horn(2*servo_horn_radius, 4);
        }
}

module hip_servo_cutting(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    servo_joint_distance = hip_descriptor[i_hd_servo_joint_distance];

    servo_horn_arm_width = 4*mm;
    servo_horn_arm_thickness = 1.7*mm;
    servo_body_width = 11.8*mm;
    servo_body_length = 22.2*mm;
    servo_mount_length = 32.2*mm;
    servo_axle_height = 31*mm;
    servo_mount_height = 16*mm;

    translate([
        0,
        -servo_joint_distance,
        -inner_height/2 + eps
    ])
        rotate([0, 0, 180]) {
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

module hip_servo_driver(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    capped_end = hip_descriptor[i_hd_capped_end];

    montage_side = FRONT*(capped_end
        ? -capped_end
        : BACK);

    if (!hide_circuits)
        color(c_circuit)
            translate([montage_side*width/4, -length/2, -inner_height/2])
                rotate(90*Z)
                    pca9685_16_channel_pwm_driver();
}

module hip_servo_driver_cutting(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    capped_end = hip_descriptor[i_hd_capped_end];

    servo_driver_length = 62.5*mm;
    servo_driver_width = 25.4*mm;

    montage_side = FRONT*(capped_end
        ? -capped_end
        : BACK);

    translate([montage_side*width/4, -length/2, (board_thickness + eps)/2])
        rotate(90*Z)
            cube([
                servo_driver_length + 2*clearance_margin,
                servo_driver_width + 2*clearance_margin,
                inner_height + board_thickness + eps
            ], center=true);
}

module hip_axle(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];

    rotate(180*X)
        joint_axle_with_bolt(inner_height);
}
