// Basic Model of Intel(R) RealSense(TM) R200 Depth Camera
//   also includes cutting plan for basic mounting
// Developed b0y: Michael McCool
// Copyright 2016 Intel Corporation
// License: CC-BY.  See LICENSE.md

// All units in mm unless noted otherwise.

sm_base = 10;  // "smoothness" of curves; larger->smoother
mount_hole_sm = sm_base;
m3_hole_radius = 3/2;

// CAMERA PARAMETERS

// basic overall size of camera
r200_x = 129;
r200_y = 8.2;
r200_z = 19.4;

// size and location of slot in camera
r200_sx = 60;
r200_sz = 4;
r200_six = 1.4;
r200_siz = 1.4;

// CAMERA MOUNT PARAMETERS
// size of mounting slot in frame
r200_mount_slot_x = 4; // width of slot
r200_mount_slot_y = 2; // height of slot
r200_mount_slot_ey = 1; // slot notch depth (behind camera)
r200_mount_slot_ix = 5; // offset of notch from end of camera

r200_mount_hole_r = m3_hole_radius;
r200_mount_hole_ix = r200_six + r200_mount_hole_r;
r200_mount_hole_iz = r200_siz + r200_sz/2;
r200_mount_hole_dx = r200_sx - r200_mount_hole_r - r200_six;
r200_mount_hole_sm = mount_hole_sm;

// thickness of magnetic mounting plate and tape
r200_plate_thick = 1.4;

// defaults 
r200_mount_hole_p = 0.5;  // relative location of inner mounting hole
r200_mount_hole_t = 0.2;  // tolerance of mounting hole
r200_mount_slot_t = 0.0;  // tolerance of mounting slot

// R200 Camera Model (3D)
module r200_camera() {
  color([0.1,0.1,0.7,0.9])
  translate([-r200_x/2,-r200_y-r200_plate_thick,0]) {
    difference() {
      cube([r200_x,r200_y,r200_z]);
      translate([r200_six,-0.05,r200_siz])
        cube([r200_sx,r200_y+0.1,r200_sz]);
    }
  }
}

// R200 Camera Mount Holes (2D)
module r200_camera_mounting_holes(
  hole_p = r200_mount_hole_p,
  hole_t = r200_mount_hole_t,
  slot_t = r200_mount_slot_t
) {
  // holes for bolts
  translate([-r200_x/2+r200_mount_hole_ix+hole_t,r200_mount_hole_iz])
    circle(r=r200_mount_hole_r+hole_t,$fn=mount_hole_sm);
  translate([-r200_x/2+r200_mount_hole_ix+hole_p*r200_mount_hole_dx+hole_t,r200_mount_hole_iz])
    circle(r=r200_mount_hole_r+hole_t,$fn=mount_hole_sm);
  
  // slots for cable ties
  translate([r200_x/2-r200_mount_slot_ix-r200_mount_slot_x-slot_t,-r200_mount_slot_y-slot_t])
    square([r200_mount_slot_x+2*slot_t,r200_mount_slot_y+r200_mount_slot_ey+2*slot_t]);
  translate([r200_x/2-r200_mount_slot_ix-r200_mount_slot_x,r200_z-r200_mount_slot_ey])
    square([r200_mount_slot_x+2*slot_t,r200_mount_slot_y+r200_mount_slot_ey+2*slot_t]);
}

// VISUALIZATION
r200_camera();
translate([0,3,0])
  rotate([90,0,0])
    linear_extrude(3)
      r200_camera_mounting_holes();