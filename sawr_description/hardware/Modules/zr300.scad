// Basic Model of Intel(R) RealSense(TM) ZR300 Depth Camera
//   also includes cutting plan for basic mounting
// Developed by: Michael McCool
// Copyright 2016 Intel Corporation
// License: CC-BY.  See LICENSE.md
include <tols.scad>
include <smooth.scad>
include <bolt_params.scad>
include <zr300_params.scad>
include <r200_params.scad>

// All units in mm unless noted otherwise.

mount_hole_sm = 4*sm_base;

// ZR300 Camera Model (3D)
module zr300_camera() {
  color([0.1,0.1,0.7,0.9])
  translate([-zr300_x/2,-zr300_y-zr300_plate_thick,0]) {
    cube([zr300_x,zr300_y,zr300_z]);
    difference() {
      hull() {
        translate([zr300_x/2-zr300_base_x2/2,0,0])
          cube([zr300_base_x2,zr300_y,0.1]);
        translate([zr300_x/2-zr300_base_x1/2,0,-zr300_base_z])
          cube([zr300_base_x1,zr300_y,0.1]);
      }
      translate([0,zr300_y/2,-zr300_base_z-0.1]) {
        translate([zr300_x/2-zr300_base_h/2,0,0])
          cylinder(r=zr300_base_r,h=zr300_base_z,$fn=mount_hole_sm);
        translate([zr300_x/2+zr300_base_h/2,0,0])
          cylinder(r=zr300_base_r,h=zr300_base_z,$fn=mount_hole_sm);
      }
    }
  }
}

// ZR300 Camera Mount Slot (2D)
module zr300_camera_mounting_holes(
  hole_t = zr300_mount_hole_t,
  slot_t = zr300_mount_slot_t
) {  
  // slots for cable tie
  translate([-r200_x/2+zr300_mount_slot_ix+zr300_mount_slot_x+slot_t,
             -r200_mount_slot_y-slot_t])
    square([zr300_mount_slot_x+2*slot_t,zr300_mount_slot_y+zr300_mount_slot_ey+2*slot_t]);
  translate([-r200_x/2+zr300_mount_slot_ix+zr300_mount_slot_x,
             r200_z-zr300_mount_slot_ey])
    square([zr300_mount_slot_x+2*slot_t,zr300_mount_slot_y+zr300_mount_slot_ey+2*slot_t]);

}

// VISUALIZATION
zr300_camera();
translate([0,zr300_y+3.5,0])
  rotate([90,0,0])
    linear_extrude(3)
      zr300_camera_mounting_holes();
