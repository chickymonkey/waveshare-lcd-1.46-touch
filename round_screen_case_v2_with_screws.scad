
// -------- Parameters --------
screen_d        = 45;
board_d         = 42.5;        // nominal PCB round portion
board_d_usb     = 42.2;     // larger round on USB side only
usb_chord_w     = 12;        // width of the wider section (use 23.54 if that's your PCB)

// Orient the USB side (direction the USB bulge faces)
usb_yaw_deg     = 180;       // 0:+X, 90:+Y, 180:-X, 270:-Y

// External USB-C opening
usb_port_w          = usb_chord_w; // opening width
usb_port_h          = 7;           // opening height
usb_port_from_top   = 8;           // distance from TOP to opening center


// Clearances
cavity_clear        = 0.8;     // global PCB clearance
usb_relief_clear    = 0.8;     // base clearance on USB-side widened cavity
usb_relief_extra    = 0.7;     // extra radial clearance on USB relief

// Extra coverage on the USB relief
usb_angle_extra_deg = 2;       // extra degrees on each side of the sector
usb_relief_h_boost  = 0.8;     // extend USB relief up towards the top (mm)

// Total thickness budget
board_thickness = 12.3;

wall            = 2;
bottom          = 1.2;

// Screen lip
lip_thickness   = 1.2;
lip_overlap     = 0.15;

// Screws
screw_d         = 2.3;      // M2 clearance
screw_offset    = 19.09;    // from center

$fn = 120;

// -------- Derived --------
outer_d     = screen_d + wall*2;
inner_d     = board_d + cavity_clear;
height      = board_thickness + bottom + lip_thickness;

// USB relief geometry (sector of a larger circle)
usb_r               = (board_d_usb + usb_relief_clear + usb_relief_extra +2)/2;
usb_chord_half      = usb_chord_w/2;
usb_half_ang        = asin( usb_chord_half / usb_r ); // radians
usb_half_ang_deg    = usb_half_ang * 180 / PI;        // degrees
usb_half_ang_deg_ex = usb_half_ang_deg + usb_angle_extra_deg;

// Heights
cavity_h        = board_thickness + 0.1;         // main cavity height
usb_cavity_h    = cavity_h + usb_relief_h_boost; // USB relief extends higher

// -------- 2D helpers --------

// Wedge polygon centered at origin, opening along +X by default
module wedge_2d(ang_deg=20, radius=100) {
    polygon(points=[
        [0,0],
        [radius*cos(-ang_deg), radius*sin(-ang_deg)],
        [radius*cos( ang_deg), radius*sin( ang_deg)]
    ]);
}

// Union of:
//  - main inner circle (clearance on all sides)
//  - a sector of a larger circle on the USB side only

module inner_cavity_2d(usb_face_deg=0) {
    union() {
        // main round cavity
        circle(d=inner_d);

        // USB-side widened sector, rotated to face usb_face_deg
        rotate([0,0,usb_face_deg])
        intersection() {
            circle(d=2*usb_r);
            wedge_2d(ang_deg=usb_half_ang_deg_ex, radius=outer_d);
        }
    }
}


// -------- Model --------
difference() {
    // Outer body
    cylinder(d=outer_d, h=height);

    // Unified inner cavity (2D union) extruded
    translate([0,0,bottom]) {
        // main cavity portion
        linear_extrude(height=cavity_h)
            inner_cavity_2d(usb_face_deg=usb_yaw_deg);

        // extend only the USB sector upward if requested
        if (usb_relief_h_boost > 0)
            translate([0,0,cavity_h])
                linear_extrude(height=usb_relief_h_boost)
                    rotate(usb_yaw_deg)
                    intersection() {
                        circle(d=2*usb_r);
                        wedge_2d(ang_deg=usb_half_ang_deg_ex, radius=outer_d);
                    }
    }

    // Screen opening
    translate([0,0,height - lip_thickness])
        cylinder(d=screen_d - 2*lip_overlap, h=lip_thickness + 0.2);

    
// --- USB-C port opening
// Clamp Z so we avoid the solid top lip and don’t go too low
usb_port_center_z = max(
    bottom + usb_port_h/2 + 0.2,
    min(height - lip_thickness - 0.2, height - usb_port_from_top)
);

rotate([0,0,360])
translate([outer_d/2, 0, usb_port_center_z])
    cube([outer_d, usb_port_w, usb_port_h], center=true);


    // M2 screw holes
    for (angle = [39.6, 180, 320.4]) {
        rotate([0,0,angle])
            translate([screw_offset, 0, -1])
                cylinder(d=screw_d, h=height);
    }
}
