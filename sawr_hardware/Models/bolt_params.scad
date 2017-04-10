// Nuts, Bolts, Spacers, Standoffs, Washers, etc.
//  For visualization only.  
//  Models are missing important details (like threads).
// Developed by: Michael McCool
// Copyright 2017 Intel Corporation
// License: CC-BY-4.0.  See LICENSE.md

// normally include the following externally, so comment out here
//include <tols.scad>
//include <smooth.scad>

// convenience function
function hex_size_to_radius(size) = size / (2 * cos(30));

// Smoothness

// Default colors
bolt_color = [0.5,0.5,0.7,0.9];
nut_color = [0.5,0.6,0.6,0.9];
spacer_color = [0.5,0.5,0.5,0.9];

// Radius of *unthreaded* part of the shaft (exact only for shoulder)
m2_shaft_radius   = 2/2;
m2_5_shaft_radius = 2.5/2;
m3_shaft_radius   = 3/2;
m4_shaft_radius   = 4/2;
m5_shaft_radius   = 5/2;
m6_shaft_radius   = 6/2;

// Threaded part of the shaft is slightly narrower
m2_thread_radius   = 1.86/2;
m2_5_thread_radius = 2.36/2;
m3_thread_radius   = 2.86/2;
m4_thread_radius   = 3.82/2;
m5_thread_radius   = 4.82/2;
m6_thread_radius   = 5.82/2;

// Cap radius
m2_cap_radius   = 4/2;
m2_5_cap_radius = 4.7/2;
m3_cap_radius   = 5.7/2;
m4_cap_radius   = 7.22/2;
m5_cap_radius   = 8.72/2;
m6_cap_radius   = 10.22/2;

// Cap height
m2_cap_height   = 2*m2_shaft_radius;
m2_5_cap_height = 2*m2_5_shaft_radius;
m3_cap_height   = 2*m3_shaft_radius;
m4_cap_height   = 2*m4_shaft_radius;
m5_cap_height   = 2*m5_shaft_radius;
m6_cap_height   = 2*m6_shaft_radius;

// Socket size (from hex face to hex face)
m2_socket_size   = 1.58;
m2_5_socket_size = 2.08;
m3_socket_size   = 2.58;
m4_socket_size   = 3.08;
m5_socket_size   = 4.1;
m6_socket_size   = 5.14;

// Socket radius (center to hex vertex)
m2_socket_radius   = hex_size_to_radius(m2_socket_size);
m2_5_socket_radius = hex_size_to_radius(m2_5_socket_size);
m3_socket_radius   = hex_size_to_radius(m3_socket_size);
m4_socket_radius   = hex_size_to_radius(m4_socket_size);
m5_socket_radius   = hex_size_to_radius(m5_socket_size);
m6_socket_radius   = hex_size_to_radius(m6_socket_size);

// Nut size (from hex face to hex face)
m2_nut_size   = 4;
m2_5_nut_size = 5;
m3_nut_size   = 5.5;
m4_nut_size   = 7;
m5_nut_size   = 8;
m6_nut_size   = 10;

// Nut radius (from center to hex vertex)
m2_nut_radius   = hex_size_to_radius(m2_nut_size);
m2_5_nut_radius = hex_size_to_radius(m2_5_nut_size);
m3_nut_radius   = hex_size_to_radius(m3_nut_size);
m4_nut_radius   = hex_size_to_radius(m4_nut_size);
m5_nut_radius   = hex_size_to_radius(m5_nut_size);
m6_nut_radius   = hex_size_to_radius(m6_nut_size);

// Nut height (face height for lock nuts)
m2_nut_height   = 1.6;
m2_5_nut_height = 2;
m3_nut_height   = 2.4;
m4_nut_height   = 3.2;
m5_nut_height   = 4;
m6_nut_height   = 5;

// Locknut height
m2_locknut_height   = 2.5;
m2_5_locknut_height = 3.8;
m3_locknut_height   = 4;
m4_locknut_height   = 5;
m5_locknut_height   = 5;
m6_locknut_height   = 6;

// Radius to use when cutting a hole for a given bolt size
m2_hole_radius   = m2_shaft_radius + tol/2 + cut_t;
m2_5_hole_radius = m2_5_shaft_radius + tol/2 + cut_t;
m3_hole_radius   = m3_shaft_radius + tol/2 + cut_t;
m4_hole_radius   = m4_shaft_radius + tol/2 + cut_t;
m5_hole_radius   = m5_shaft_radius + tol/2 + cut_t;
m6_hole_radius   = m6_shaft_radius + tol/2 + cut_t;

// "Standard bolt" defaults - for M3 x 10
bolt_shaft_length     = 10;
bolt_shouldere_length = 0;
bolt_shaft_radius     = m3_shaft_radius;
bolt_thread_radius    = m3_thread_radius;
bolt_shaft_sm         = 2*sm_base;
bolt_cap_radius       = m3_cap_radius;
bolt_cap_height       = m3_cap_height;
bolt_socket_size      = m3_socket_size;
bolt_socket_radius    = m3_socket_radius;

bolt_cap_sm           = 4*sm_base;
bolt_shoulder_sm      = 4*sm_base;
bolt_thread_sm        = 2*sm_base;
bolt_hole_sm          = 2*sm_base;

// "Standard nut" defaults - for M3
nut_height = m3_nut_height;
nut_size   = m3_nut_size;
nut_radius = m3_nut_radius;

// "Standard locknut" defaults - for M3
locknut_face_height = m3_nut_height;
locknut_height      = m3_locknut_height;
locknut_size        = m3_nut_size;
locknut_radius      = m3_nut_radius;

// Spacers
m2_spacer_height       = 6;
m2_spacer_hole_radius  = m2_shaft_radius + 2*tol;
m2_spacer_outer_radius = m2_nut_radius;
m2_5_spacer_height       = 6;
m2_5_spacer_hole_radius  = m2_5_shaft_radius + 2*tol;
m2_5_spacer_outer_radius = m2_5_nut_radius;
m3_spacer_height       = 6;
m3_spacer_hole_radius  = m3_shaft_radius + 2*tol;
m3_spacer_outer_radius = m3_nut_radius;
m4_spacer_height       = 6;
m4_spacer_hole_radius  = m4_shaft_radius + 2*tol;
m4_spacer_outer_radius = m4_nut_radius;
m5_spacer_height       = 6;
m5_spacer_hole_radius  = m5_shaft_radius + 2*tol;
m5_spacer_outer_radius = m5_nut_radius;
m6_spacer_height       = 6;
m6_spacer_hole_radius  = m6_shaft_radius + 2*tol;
m6_spacer_outer_radius = m6_nut_radius;

// Spacer defaults
spacer_height       = 6;
spacer_hole_radius  = m3_hole_radius + tol;
spacer_hole_sm      = 2*sm_base;
spacer_outer_radius = spacer_hole_radius + 2;
spacer_outer_sm     = 4*sm_base; 

