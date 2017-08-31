include <defs.scad>;
include <util.scad>;

module leg() {
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
    inner_width = leg_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_descriptor[i_ld_servo_joint_distance];
    start_thickness = leg_descriptor[i_ld_start_thickness];
    end_thickness = leg_descriptor[i_ld_end_thickness];
    turn_bias = leg_descriptor[i_ld_turn_bias];

    translate([0, effective_length/2, 0]) {
        // servo
        if (has_servo)
            generic_leg_servo(leg_descriptor);

        // skeleton
        color(c_board) {
            // sides
            for (offset = (inner_width + board_thickness)/2 * [-1, 1]) {
                difference() {
                    translate([offset, 0, 0])
                        rotate([90, 0, 90])
                            mic_board_with_holes(
                                start_thickness/2,
                                end_thickness/2,
                                effective_length,
                                joint_axle_diameter_out/2,
                                joint_axle_screw_diameter/2,
                                center=true
                            );
                    translate([0, -effective_length/2, 0]) {
                        for (vertical_offset = servo_joint_transmission_ratio*servo_horn_radius*[-1, 1])
                            rotate([turn_bias, 0, 0])
                            translate([0, 0, vertical_offset])
                                rotate([0, 90, 0])
                                    cylinder(
                                        h=inner_width + 3*board_thickness,
                                        r=joint_axle_diameter_out/2,
                                        center=true
                                    );
                    }
                }
            }

            // center link
            difference() {
                let(
                    link_length = effective_length - (start_thickness + end_thickness)/2 - 2*clearance_margin
                ) {
                    translate([0, effective_length/2 - link_length/2 - end_thickness/2 - clearance_margin, 0])
                        cube([inner_width, link_length, board_thickness], center=true);
                }
                if (has_servo)
                    generic_leg_servo_cutting(leg_descriptor);
            }
        }

        // tendon insertion
        color(c_brass) {
            translate([0, -effective_length/2, 0]) {
                for (vertical_offset = servo_joint_transmission_ratio*servo_horn_radius*[-1, 1])
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

        // axle
        translate([0, effective_length/2, 0]) {
            rotate([0, 90, 0]) {
                color(c_brass)
                    tube(
                        h=inner_width,
                        r_out=joint_axle_diameter_out/2,
                        r_in=joint_axle_diameter_in/2,
                        center=true
                    );
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

module generic_leg_servo_cutting(leg_descriptor) {
    effective_length = leg_descriptor[i_ld_effective_length];
    inner_width = leg_descriptor[i_ld_inner_width];
    servo_joint_distance = leg_descriptor[i_ld_servo_joint_distance];

    servo_body_width = 11.8*mm;
    servo_mount_length = 32.2*mm;
    servo_axle_height = 31*mm;
    servo_mount_height = 16*mm;

    translate([-inner_width/2, effective_length/2 - servo_joint_distance, 0])
        rotate([180, 0, 0]) rotate([0, 90, 0]) {
            translate(-[servo_body_width/2 + clearance_margin, servo_body_width, 0])
                cube([servo_body_width + 2*clearance_margin, servo_mount_length + 2*clearance_margin, servo_axle_height - servo_mount_height + eps]);
        }
}