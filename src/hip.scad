include <defs.scad>;
include <util.scad>;

i_hd_mount_diameter = 0;
i_hd_inner_height = 1;
i_hd_width = 2;
i_hd_length = 3;
i_hd_servo_joint_distance = 4;
i_hd_has_servo_driver = 5;
i_hd_capped_end = 6; // TODO make designated end closed and short

module hip(adjecent_leg_part) {
    alp_effective_length = adjecent_leg_part[i_ld_effective_length];
    alp_has_servo = adjecent_leg_part[i_ld_has_servo];
    alp_inner_width = adjecent_leg_part[i_ld_inner_width];
    alp_joint_type = adjecent_leg_part[i_ld_joint_type];
    alp_servo_joint_distance = adjecent_leg_part[i_ld_servo_joint_distance];
    alp_end_thickness = adjecent_leg_part[i_ld_end_thickness];
    alp_start_thickness = adjecent_leg_part[i_ld_start_thickness];
    alp_turn_bias = adjecent_leg_part[i_ld_turn_bias];

    hip_descriptor = [
        alp_inner_width + 2*board_thickness, // mount_diameter
        alp_end_thickness + clearance_margin, // inner_height
        180*mm, // width
        80*mm, // length
        40*mm - 5.2*mm, // servo_joint_distance
        true // has_servo_driver
    ];

    color(c_board) {
        hip_sides(hip_descriptor);
        hip_sides_link(hip_descriptor);
    }

    if (hip_descriptor[i_hd_has_servo_driver])
        hip_servo_driver(hip_descriptor);

    if (!hide_servos)
        hip_servo(hip_descriptor);

    children();
}

module hip_sides(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];
    has_servo_driver = hip_descriptor[i_hd_has_servo_driver];

    difference() {
        for (vertical_offset = (inner_height + board_thickness)/2*[-1, 1]) {
            translate(vertical_offset*Z) {
                difference() {
                    union() {
                        cylinder(board_thickness, d=mount_diameter, center=true);
                        translate(-length/2*Y)
                            cube([width, length, board_thickness], center=true);
                    }

                    cylinder(board_thickness + eps, d=joint_axle_screw_diameter, center=true);
                }
            }
        }

        hip_servo_cutting(hip_descriptor);

        if (has_servo_driver)
            hip_servo_driver_cutting(hip_descriptor);
    }
}

module hip_sides_link(hip_descriptor) {
    inner_height = hip_descriptor[i_hd_inner_height];
    mount_diameter = hip_descriptor[i_hd_mount_diameter];
    width = hip_descriptor[i_hd_width];
    length = hip_descriptor[i_hd_length];

    servo_mount_height = 16*mm;
    servo_body_width = 11.8*mm;

    mount_opening_width = sqrt(2)*mount_diameter;
    mount_opening_servo_cutting = sqrt(2)*(mount_diameter/2 + servo_mount_height - board_thickness);

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
                        (width - mount_opening_width)/2 - clearance_margin,
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
                                mount_opening_servo_cutting - mount_opening_width/2 + clearance_margin + eps,
                                board_thickness + 2*eps,
                                servo_body_width + 2*clearance_margin
                            ]);
                }
            }

    // front side
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
}

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
            // upper part
            let(
                width = 2*servo_horn_radius + servo_horn_arm_width + 2*clearance_margin,
                length = servo_mount_length + 2*clearance_margin,
                height = inner_height + board_thickness + eps
            )
                translate(-[0, servo_body_width/2 - servo_body_length/2, 0])
                    cylinder(height, d=sqrt(pow(width, 2) + pow(length, 2)));

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

    montage_side = capped_end
        ? -capped_end :
        1;

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

    montage_side = capped_end
        ? -capped_end :
        1;

    translate([montage_side*width/4, -length/2, (board_thickness + eps)/2])
        rotate(90*Z)
            cube([
                servo_driver_length + 2*clearance_margin,
                servo_driver_width + 2*clearance_margin,
                inner_height + board_thickness + eps
            ], center=true);
}
