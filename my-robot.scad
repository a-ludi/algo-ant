include <src/leg.scad>;
include <src/hip.scad>;
use <include/raspberry-pi/rpi.scad>;

 $fa = 10;
$fs = 1.5;

alpha = 26;

// distance between hip joint: [180mm, 80mm]
// resulting max turn with [-10, 0, 10] bias: 26

for (m = [0, 1])
    mirror(m*Y)
        hip(front_hip_descriptor, leg_part_descriptor_joint) {
            cube(1);
            hip(center_hip_descriptor, leg_part_descriptor_joint) {
                cube(1);

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
                );
            }
        }
