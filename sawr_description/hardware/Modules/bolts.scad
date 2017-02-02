// Nuts, Bolts, Spacers, Standoffs, Washers, etc.
//  For visualization only.  
//  Models are missing important details (like threads).
// Developed by: Michael McCool
// Copyright 2016 Intel Corporation
// License: CC-BY.  See LICENSE.md
include <tols.scad>
include <smooth.scad>
include <bolt_params.scad>

// Cap head bolt, optional shoulder
module bolt(
  shaft_radius = bolt_shaft_radius,
  shaft_length = bolt_shaft_length,
  shoulder_length = 0,
  thread_radius = bolt_thread_radius,
  cap_radius = bolt_cap_radius,
  cap_height = bolt_cap_height,
  socket_size = bolt_socket_size,
  socket_radius = bolt_socket_radius,
  shoulder_sm = bolt_shoulder_sm,
  thread_sm = bolt_thread_sm,
  cap_sm = bolt_cap_sm
) {
  color(bolt_color) {
    translate([0,0,-shaft_length])
      union() {
        cylinder(r=thread_radius,h=shaft_length+eps,$fn=thread_sm);
        if (shoulder_length > 0) {
          translate([0,0,shaft_length-shoulder_length])
            cylinder(r=shaft_radius,h=shoulder_length+eps,$fn=shoulder_sm);
        }
      }
      difference() {
        cylinder(r=cap_radius,h=cap_height,$fn=cap_sm);
        translate([0,0,0.2*cap_height])
          cylinder(r=socket_radius,h=cap_height,$fn=6);
      }
  }
}

// Standard sized bolts
module m2_bolt(length = 10, shoulder = 0, thread_radius = m2_thread_radius) {
  bolt(
    shaft_radius = m2_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m2_cap_radius,
    cap_height = m2_cap_height,
    socket_size = m2_socket_size,
    socket_radius = m2_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}
module m2_5_bolt(length = 10, shoulder = 0, thread_radius = m2_5_thread_radius) {
  bolt(
    shaft_radius = m2_5_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m2_5_cap_radius,
    cap_height = m2_5_cap_height,
    socket_size = m2_5_socket_size,
    socket_radius = m2_5_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}
module m3_bolt(length = 10, shoulder = 0, thread_radius = m3_thread_radius) {
  bolt(
    shaft_radius = m3_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m3_cap_radius,
    cap_height = m3_cap_height,
    socket_size = m3_socket_size,
    socket_radius = m3_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}
module m4_bolt(length = 10, shoulder = 0, thread_radius = m4_thread_radius) {
  bolt(
    shaft_radius = m4_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m4_cap_radius,
    cap_height = m4_cap_height,
    socket_size = m4_socket_size,
    socket_radius = m4_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}
module m5_bolt(length = 10, shoulder = 0, thread_radius = m5_thread_radius) {
  bolt(
    shaft_radius = m5_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m5_cap_radius,
    cap_height = m5_cap_height,
    socket_size = m5_socket_size,
    socket_radius = m5_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}
module m6_bolt(length = 10, shoulder = 0, thread_radius = m6_thread_radius) {
  bolt(
    shaft_radius = m6_shaft_radius,
    shaft_length = length,
    shoulder_length = shoulder,
    thread_radius = thread_radius,
    cap_radius = m6_cap_radius,
    cap_height = m6_cap_height,
    socket_size = m6_socket_size,
    socket_radius = m6_socket_radius,
    shoulder_sm = bolt_shoulder_sm,
    thread_sm = bolt_thread_sm,
    cap_sm = bolt_cap_sm
  );
}

// Nut
module nut(
  thread_radius = bolt_thread_radius,
  size = nut_size,
  radius = nut_radius,
  height = nut_height,
  thread_sm = bolt_thread_sm
) {
  color(nut_color) {
    difference() {
      cylinder(r=radius,h=height,$fn=6);
      translate([0,0,-height/4])
        cylinder(r=thread_radius,h=2*height,$fn=thread_sm);
    }
  }
}

// Standard sized nuts
module m2_nut(height = m2_nut_height) {
  nut(
    thread_radius = m2_thread_radius,
    size = m2_nut_size,
    radius = m2_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}
module m2_5_nut(height = m2_5_nut_height) {
  nut(
    thread_radius = m2_5_thread_radius,
    size = m2_5_nut_size,
    radius = m2_5_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}
module m3_nut(height = m3_nut_height) {
  nut(
    thread_radius = m3_thread_radius,
    size = m3_nut_size,
    radius = m3_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}
module m4_nut(height = m4_nut_height) {
  nut(
    thread_radius = m4_thread_radius,
    size = m4_nut_size,
    radius = m4_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}
module m5_nut(height = m5_nut_height) {
  nut(
    thread_radius = m5_thread_radius,
    size = m5_nut_size,
    radius = m5_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}
module m6_nut(height = m6_nut_height) {
  nut(
    thread_radius = m6_thread_radius,
    size = m6_nut_size,
    radius = m6_nut_radius,
    height = height,
    thread_sm = bolt_thread_sm
  );
}

// Locknut
module locknut(
  thread_radius = bolt_thread_radius,
  size = nut_size,
  radius = nut_radius,
  height = nut_height,
  face_height = locknut_face_height,
  thread_sm = bolt_thread_sm
) {
  color(nut_color) {
    difference() {
      intersection() {
        cylinder(r=radius,h=height,$fn=6);
        cylinder(r2=0.9*size/2,r1=1.3*radius,h=height,$fn=bolt_cap_sm);
      }
      translate([0,0,-height/4])
        cylinder(r=thread_radius,h=2*height,$fn=thread_sm);
    }
  }
}

// Standard sized locknuts
module m2_locknut() {
  locknut(
    thread_radius = m2_thread_radius,
    size = m2_nut_size,
    radius = m2_nut_radius,
    height = m2_locknut_height,
    face_height = m2_nut_height,
    thread_sm = bolt_thread_sm
  );
}
module m2_5_locknut() {
  locknut(
    thread_radius = m2_5_thread_radius,
    size = m2_5_nut_size,
    radius = m2_5_nut_radius,
    height = m2_5_locknut_height,
    face_height = m2_5_nut_height,
    thread_sm = bolt_thread_sm
  );
}
module m3_locknut() {
  locknut(
    thread_radius = m3_thread_radius,
    size = m3_nut_size,
    radius = m3_nut_radius,
    height = m3_locknut_height,
    face_height = m3_nut_height,
    thread_sm = bolt_thread_sm
  );
}
module m4_locknut() {
  locknut(
    thread_radius = m4_thread_radius,
    size = m4_nut_size,
    radius = m4_nut_radius,
    height = m4_locknut_height,
    face_height = m4_nut_height,
    thread_sm = bolt_thread_sm
  );
}
module m5_locknut() {
  locknut(
    thread_radius = m5_thread_radius,
    size = m5_nut_size,
    radius = m5_nut_radius,
    height = m5_locknut_height,
    face_height = m5_nut_height,
    thread_sm = bolt_thread_sm
  );
}
module m6_locknut() {
  locknut(
    thread_radius = m6_thread_radius,
    size = m6_nut_size,
    radius = m6_nut_radius,
    height = m6_locknut_height,
    face_height = m6_nut_height,
    thread_sm = bolt_thread_sm
  );
}

module spacer(
  radius = spacer_hole_radius,
  outer_radius = spacer_outer_radius,
  height = spacer_height,
  outer_sm = spacer_outer_sm,
  sm = spacer_hole_sm
) {
  color(spacer_color)
    difference() {
      cylinder(r=outer_radius,h=height,$fn=outer_sm);
      translate([0,0,-height/4]) 
        cylinder(r=radius,h=2*height,$fn=sm);
    }
}
module m2_spacer(height=m2_spacer_height) {
  spacer(
    radius = m2_spacer_hole_radius,
    outer_radius = m2_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}
module m2_5_spacer(height=m2_5_spacer_height) {
  spacer(
    radius = m2_5_spacer_hole_radius,
    outer_radius = m2_5_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}
module m3_spacer(height=m3_spacer_height) {
  spacer(
    radius = m3_spacer_hole_radius,
    outer_radius = m3_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}
module m4_spacer(height=m4_spacer_height) {
  spacer(
    radius = m4_spacer_hole_radius,
    outer_radius = m4_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}
module m5_spacer(height=m5_spacer_height) {
  spacer(
    radius = m5_spacer_hole_radius,
    outer_radius = m5_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}
module m6_spacer(height=m6_spacer_height) {
  spacer(
    radius = m6_spacer_hole_radius,
    outer_radius = m6_spacer_outer_radius,
    height = height,
    outer_sm = spacer_hole_sm,
    sm = spacer_outer_sm
  );
}

// TESTING
translate([0,0,0]) {
  translate([0,0,-11-m2_nut_height]) 
    rotate([0,0,30]) 
      m2_nut();
  translate([0,0,-11-1-m2_nut_height]) 
    rotate([180,0,30]) 
      m2_locknut();
  translate([0,0,-11-2-m2_nut_height-m2_locknut_height]) 
    rotate([180,0,30]) 
      m2_nut(height=6);
  translate([0,0,-11-2-6-1-m2_nut_height-m2_locknut_height]) 
    rotate([180,0,30]) 
      m2_spacer(height=6);
  m2_bolt();
translate([m2_cap_radius+m2_5_cap_radius+2,0,0]) {
  translate([0,0,-11-m2_5_nut_height]) 
    rotate([0,0,30]) 
      m2_5_nut();
  translate([0,0,-11-1-m2_5_nut_height]) 
    rotate([180,0,30]) 
      m2_5_locknut();
  translate([0,0,-11-2-m2_5_nut_height-m2_5_locknut_height]) 
    rotate([180,0,30]) 
      m2_5_nut(height=8);
  translate([0,0,-11-2-8-1-m2_5_nut_height-m2_5_locknut_height]) 
    rotate([180,0,30]) 
      m2_5_spacer(height=8);
  m2_5_bolt();
translate([m2_5_cap_radius+m3_cap_radius+2,0,0]) {
  translate([0,0,-11-m3_nut_height]) 
    rotate([0,0,30]) 
      m3_nut();
  translate([0,0,-11-1-m3_nut_height]) 
    rotate([180,0,30]) 
      m3_locknut();
  translate([0,0,-11-2-m3_nut_height-m3_locknut_height]) 
    rotate([180,0,30]) 
      m3_nut(height=10);
  translate([0,0,-11-2-10-1-m3_nut_height-m3_locknut_height]) 
    rotate([180,0,30]) 
      m3_spacer(height=10);
  m3_bolt();
translate([m3_cap_radius+m4_cap_radius+2,0,0]) {
  translate([0,0,-11-m4_nut_height]) 
    rotate([0,0,30]) 
      m4_nut();
  translate([0,0,-11-1-m4_nut_height]) 
    rotate([180,0,30]) 
      m4_locknut();
  translate([0,0,-11-2-m4_nut_height-m4_locknut_height]) 
    rotate([180,0,30]) 
      m4_nut(height=12);
  translate([0,0,-11-2-12-1-m4_nut_height-m4_locknut_height]) 
    rotate([180,0,30]) 
      m4_spacer(height=12);
  m4_bolt();
translate([m4_cap_radius+m5_cap_radius+2,0,0]) {
  translate([0,0,-11-m5_nut_height]) 
    rotate([0,0,30]) 
      m5_nut();
  translate([0,0,-11-1-m5_nut_height]) 
    rotate([180,0,30]) 
      m5_locknut();
  translate([0,0,-11-2-m5_nut_height-m5_locknut_height]) 
    rotate([180,0,30]) 
      m5_nut(height=14);
  translate([0,0,-11-2-14-1-m5_nut_height-m5_locknut_height]) 
    rotate([180,0,30]) 
      m4_spacer(height=14);
  m5_bolt();
translate([m5_cap_radius+m6_cap_radius+2,0,0]) {
  translate([0,0,-11-m6_nut_height]) 
    rotate([0,0,30]) 
      m6_nut();
  translate([0,0,-11-1-m6_nut_height]) 
    rotate([180,0,30]) 
      m6_locknut();
  translate([0,0,-11-2-m6_nut_height-m6_locknut_height]) 
    rotate([180,0,30]) 
      m6_nut(height=16);
  translate([0,0,-11-2-16-1-m6_nut_height-m6_locknut_height]) 
    rotate([180,0,30]) 
      m6_spacer(height=16);
  m6_bolt();
}}}}}}
