include <defs.scad>;
use <MCAD/metric_fastners.scad>;

module servo_horn(d, n_arms) {
    arm_thickness = 1.7*mm;
    arm_width = 4*mm;
    hole_diameter = 0.8*mm;
    mount_diameter = 8.5*mm;
    mount_height = 5.5*mm;

    color(c_servo_horn) {
        cylinder(mount_height, d=mount_diameter);
        rotate_copy([0, 0, 360/n_arms], n_arms) {
            translate([-arm_width/2, 0, mount_height - arm_thickness]) {
                difference() {
                    union() {
                        cube([arm_width, d/2, arm_thickness]);
                        translate([arm_width/2, d/2, 0])
                            cylinder(arm_thickness, d=arm_width);
                    }
                    translate([arm_width/2, d/2, -arm_thickness/2])
                        cylinder(arm_thickness*2, d=hole_diameter);
                }
            }
        }
    }
}

module servo_horn_cutting(d, epsilon=2*mm) {
    arm_width = 4*mm;
    mount_height = 5.5*mm;

    cylinder(mount_height + epsilon, d=d + arm_width + 2*epsilon);
}

module hs_5055mg_servo() {
    body_width = 11.6*mm;
    body_length = 22.8*mm;
    gearbox_height = 24*mm;
    mount_height = 16.7*mm;
    mount_thickness = 2.2*mm;
    body_height = (mount_height + mount_thickness + gearbox_height)/2;
    mount_hole_distance = 29*mm;
    mount_hole_diameter = 3*mm;
    mount_length = mount_hole_distance + 2*mount_hole_diameter;
    axle_height = mount_height + 11.1*mm;
    axle_diameter = 3.2*mm;

    color(c_servo)
    translate([-body_width/2, -body_width/2, -mount_height]) {
        // body
        cube([body_width, body_length, body_height]);
        // mount plates
        difference() {
            translate([0*mm, (body_length - mount_length)/2, mount_height])
                rounded_board([body_width, mount_length, mount_thickness], mount_hole_diameter/2);
            translate([body_width/2, (body_length - mount_hole_distance)/2, mount_height - mount_thickness/2]) {
                cylinder(2*mount_thickness, d=mount_hole_diameter);
                translate([0*mm, mount_hole_distance, 0*mm])
                    cylinder(2*mount_thickness, d=mount_hole_diameter);
            }
        }
        translate([body_width/2, body_width/2, 0*mm]) {
            cylinder(gearbox_height, d=body_width);
            translate([0*mm, body_width/2, 0*mm]) cylinder(gearbox_height, d=body_width/2);
            cylinder(axle_height, d=axle_diameter);
        }
    }
}

module sg90_servo() {
    body_width = 11.8*mm;
    body_length = 22.2*mm;
    body_height = 22.2*mm;
    gearbox_height = 25*mm;
    mount_height = 16*mm;
    mount_length = 32.2*mm;
    mount_thickness = 2.2*mm;
    axle_height = 31*mm;
    axle_diameter = 4*mm;

    color(c_servo)
        translate([-body_width/2, -body_width/2, -mount_height]) {
            // body
            cube([body_width, body_length, body_height]);

            // mount
            translate([0*mm, (body_length - mount_length)/2, mount_height]) cube([body_width, mount_length, mount_thickness]);

            // gearbox + axle
            translate([body_width/2, body_width/2, 0*mm]) {
                cylinder(gearbox_height, d=body_width);
                translate([0*mm, body_width/2, 0*mm]) cylinder(gearbox_height, d=body_width/2);
                cylinder(axle_height, d=axle_diameter);
            }
        }
}

module sg90_servo_cutting(epsilon=1*mm) {
    body_width = 11.8*mm;
    body_length = 22.2*mm;
    body_height = 22.2*mm;
    gearbox_height = 25*mm;
    mount_height = 16*mm;
    mount_length = 32.2*mm;
    mount_thickness = 2.2*mm;
    axle_height = 31*mm;
    axle_diameter = 4*mm;

    translate([-(body_width + 2*epsilon)/2, -(body_width + 2*epsilon)/2, -(mount_height + epsilon)]) {
        cube([body_width + 2*epsilon, body_length + 2*epsilon, gearbox_height + 2*epsilon]);
        translate([0*mm, (body_length - mount_length)/2, mount_height + epsilon])
            cube([body_width + 2*epsilon, mount_length + 2*epsilon, mount_thickness + epsilon]);
    }
}

module rotate_copy(rotation, n) {
    for (i = [0 : n - 1])
        rotate(i*rotation)
            children();
}

/**
 * Create a shape like a microphone, ie two circles of (different) sizes
 * connected by their shared tangents.
 */
module mic_shape(r1, r2, d, center=false) {
    hull() {
        translate([center ? -d/2 : 0, 0, 0]) {
            circle(r1);
            translate([d, 0, 0]) circle(r2);
        }
    }
}

module mic_board_with_holes(r1_board, r2_board, d, r1_hole, r2_hole, thickness=board_thickness, center=false) {
    translate((center ? 1 : 0)*[-d/2, 0, -thickness/2])
        difference() {
            linear_extrude(thickness)
                mic_shape(r1_board, r2_board, d);

            translate([0, 0, -thickness/2])
                linear_extrude(2*thickness) {
                    circle(r1_hole);
                    translate([d, 0, 0]) circle(r2_hole);
                }
        }
}

module rounded_board_with_holes(size, r_corner, d_hole, center=false) {
    x = size[0];
    y = size[1];
    z = size[2] ? size[2] : board_thickness;
    r_corner = min(r_corner, min(x/2, y/2));
    d_hole = len(d_hole) == undef
        ? [d_hole, d_hole, d_hole, d_hole]
        : d_hole;

    translate((center ? 1 : 0) * [-x/2, -y/2, -z/2])
        difference() {
            rounded_board(size, r_corner);

            for (i = [0, 1])
                for (j = [0, 1])
                    translate([
                        i*x + (1 - 2*i) * r_corner,
                        j*y + (1 - 2*j) * r_corner,
                        -z/2
                    ])
                        cylinder(2*z, d=d_hole[i + 2*j]);
        }
}

/**
 * Create a board with rounded corners.
 */
module rounded_board(size, r=inf, center=false) {
    x = size[0];
    y = size[1];
    z = size[2] ? size[2] : board_thickness;
    r = min(r, min(x/2, y/2));

    translate((center ? 1 : 0) * [-x/2, -y/2, -z/2])
        union() {
            translate([0, r, 0])
                cube([x, y - 2*r, z]);
            translate([r, 0, 0])
                cube([x - 2*r, y, z]);

            translate([r,r,0])
                cylinder(z, r=r);
            translate([x-r,r,0])
                cylinder(z, r=r);
            translate([r,y-r,0])
                cylinder(z, r=r);
            translate([x-r,y-r,0])
                cylinder(z, r=r);
        }
}

module tube(h, r_out=false, r_in=false, center=false) {
    eps = 0.001;

    difference() {
        cylinder(h, r=r_out, center=center);

        if (center)
            cylinder(h + 2*eps, r=r_in, center=true);
        else
            translate([0, 0, -eps])
                cylinder(h + 2*eps, r=r_in, center=false);
    }
}

function my_bolt_head_thickness(d) = 0.7*d;

module my_bolt(d, l) {
    translate([0, 0, -my_bolt_head_thickness(d)])
        bolt(d, l + my_bolt_head_thickness(d));
}


function my_flat_nut_thickness(d) = 0.8*d;

module my_flat_nut(d) {
    translate([0, 0, my_flat_nut_thickness(d)])
        rotate([0, 180, 0])
            flat_nut(d);
}

module warn(text) {
    echo(str("<span style=\"color: red\">warning: ", text, "</span>"));
}
