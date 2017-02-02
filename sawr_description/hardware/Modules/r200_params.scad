// Basic Model of Intel(R) RealSense(TM) R200 Depth Camera
//   also includes cutting plan for basic mounting
// Developed b0y: Michael McCool
// Copyright 2016 Intel Corporation
// License: CC-BY.  See LICENSE.md

// All units in mm unless noted otherwise.

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
r200_mount_hole_sm = 4*sm_base;

// thickness of magnetic mounting plate and tape
r200_plate_thick = 1.4;

// defaults 
r200_mount_hole_p = 0.5;  // relative location of inner mounting hole
r200_mount_hole_t = 0.2;  // tolerance of mounting hole
r200_mount_slot_t = 0.0;  // tolerance of mounting slot
