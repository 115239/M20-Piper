// M20 Pro -> Piper adapter: v7 fit to the existing USD assembly.
// Original filename retained for downstream use. All dimensions are millimetres.
// Install translation relative to M20 base_link: [-5, 0, 80] mm.
// Piper stays at [0, 0, 88.8] mm. Do not scale the exported mesh.

plate_x=250; plate_y=175; corner_r=3;
base_z=5;
pad_x=104; pad_y=115; pad_corner_r=9;
mount_height=8.8;                  // 80 + 8.8 = existing arm bottom 88.8
pad_extra_z=mount_height-base_z;
cable_hole_d=50;

slot_length=160; slot_width=25; slot_y=62.5;
side_hole_x=85; side_hole_w=55; side_hole_h=35; side_hole_r=5;
m20_columns=7; m20_pitch_x=40; m20_row_spacing_y=165; m20_clearance_d=4.6;
piper_pattern=70; piper_clearance_d=5.6;

// Optional fastener recesses. Disabled until the real screw/nut stack is known.
// Keeping through holes avoids thinning the printed load path on both sides.
piper_counterbore_d=10; piper_counterbore_depth=0;
piper_nut_flat=8.5; piper_nut_depth=0;
minimum_web=3;

outer_frame_width=8;             // Upward perimeter reinforcement, no bottom frame.
rib_width=14; rib_inner_y=47; rib_outer_y=82;
x_rib_inner=pad_x/2-1; x_rib_outer=117;
rail_relief_start_y=74;          // Wider than the model rails, beginning near Y=79.25.
rail_surface_z=85;              // Highest chassis lip in base_link coordinates.
installation_z=80;
rail_clearance=0.2;             // Geometric clearance; not a printing calibration.
rail_relief_z=rail_surface_z-installation_z+rail_clearance;
$fn=80;

assert(mount_height>base_z);
assert(mount_height-rail_relief_z>=minimum_web);
assert(mount_height-piper_counterbore_depth-piper_nut_depth>=minimum_web);
assert(rail_relief_start_y>pad_y/2);

module rounded_prism(x,y,z,r) {
    linear_extrude(height=z)
        offset(r=r) square([x-2*r,y-2*r],center=true);
}
module capsule_x(length,width,height) {
    hull() for(x=[-(length-width)/2,(length-width)/2])
        translate([x,0,0]) cylinder(d=width,h=height);
}
module perimeter_frame() {
    difference() {
        rounded_prism(plate_x,plate_y,mount_height,corner_r);
        translate([0,0,-1]) rounded_prism(plate_x-2*outer_frame_width,
            plate_y-2*outer_frame_width,mount_height+2,corner_r);
    }
}
module slotted_deck() {
    difference() {
        union() {
            rounded_prism(plate_x,plate_y,base_z,corner_r);
            translate([0,0,base_z]) rounded_prism(pad_x,pad_y,pad_extra_z,pad_corner_r);
        }
        for(y=[-slot_y,slot_y])
            translate([0,y,-1]) capsule_x(slot_length,slot_width,mount_height+2);
    }
}
module y_rib(xc,sign_y) {
    translate([xc-rib_width/2, sign_y>0 ? rib_inner_y : -rib_outer_y, 0])
        cube([rib_width,rib_outer_y-rib_inner_y,mount_height]);
}
module x_rib(yc,dir) {
    y0=yc-rib_width/2; y1=yc+rib_width/2;
    xi=dir*x_rib_inner; xo=dir*x_rib_outer;
    polyhedron(
        points=[[xi,y0,base_z],[xi,y1,base_z],[xo,y0,base_z],
                [xo,y1,base_z],[xi,y0,mount_height],[xi,y1,mount_height]],
        faces=[[0,2,3,1],[0,1,5,4],[2,4,5,3],[0,4,2],[1,3,5]]);
}

// White is also bound as a material in the generated USD; STL has no standard color.
color([0.95,0.95,0.95])
difference() {
    union() {
        slotted_deck();
        perimeter_frame();
        for(o=[-35,35]) {
            y_rib(o,1); y_rib(o,-1);
            x_rib(o,1); x_rib(o,-1);
        }
    }
    // Underside steps clear both 85 mm chassis lips; 3.6 mm side-band material remains.
    for(sign_y=[-1,1])
        translate([-plate_x/2-1, sign_y>0 ? rail_relief_start_y : -plate_y/2-1, -1])
            cube([plate_x+2,plate_y/2-rail_relief_start_y+1,rail_relief_z+1]);
    for(y=[-m20_row_spacing_y/2,m20_row_spacing_y/2])
        for(i=[0:m20_columns-1])
            translate([-(m20_columns-1)*m20_pitch_x/2+i*m20_pitch_x,y,-1])
                cylinder(d=m20_clearance_d,h=mount_height+2);
    for(x=[-piper_pattern/2,piper_pattern/2])
        for(y=[-piper_pattern/2,piper_pattern/2]) {
            translate([x,y,-1]) cylinder(d=piper_clearance_d,h=mount_height+2);
            if(piper_counterbore_depth>0)
                translate([x,y,mount_height-piper_counterbore_depth])
                    cylinder(d=piper_counterbore_d,h=piper_counterbore_depth+1);
            if(piper_nut_depth>0)
                translate([x,y,-1])
                    cylinder(d=piper_nut_flat/cos(30),h=piper_nut_depth+1,$fn=6);
        }
    translate([0,0,-1]) cylinder(d=cable_hole_d,h=mount_height+2);
    for(x=[-side_hole_x,side_hole_x])
        translate([x,0,-1]) rounded_prism(side_hole_w,side_hole_h,mount_height+2,side_hole_r);
}
