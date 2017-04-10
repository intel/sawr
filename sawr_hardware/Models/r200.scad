// Basic Model of Intel(R) RealSense(TM) R200 Depth Camera
//   also includes cutting plan for basic mounting
// Developed by: Michael McCool
// Copyright 2017 Intel Corporation
// License: CC-BY-4.0.  See LICENSE.md
include <tols.scad>
include <smooth.scad>
include <bolt_params.scad>
include <r200_params.scad>

// All units in mm unless noted otherwise.

mount_hole_sm = 4*sm_base;

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
  
  // slots for cable tie
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
