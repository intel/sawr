// Simple Autonomous Wheeled Robot (SAWR) Frame Design
// Developed by: Michael McCool
// Copyright 2016 Intel Corporation
// License: CC-BY.  See LICENSE.md

// All units in mm unless noted otherwise.

printed = 0;  // set to 1 if 3D printing, 0 if laser cutting
sm_base = 5;  // "smoothness" of curves; larger->smoother
              // set to 5 during development, to 100 for laser-cutting output
              
// Material selection
using_pom = false;  // Use of POM/Delrin/Duracon/Acetal allows for certain
                     // features like a cantilevered suspension... but this is
                     // off by default to target acrylic.
                    
// Use external 3D models in visualization
use_external_models = false; // off by default

// TOLERANCES 

// tolerance around cuts; lasers remove a tiny slice (making holes
// very slightly larger) but 3D printers add material (making holes
// smaller... and typically with more "spread").
laser_cut_t = -0.05;  // typical 0.1mm cut width
printed_cut_t = 0.1;  // depends on printer; might need to be up to 0.3
cut_t = printed ? printed_cut_t : laser_cut_t; 

// PLATE THICKNESSES
// If possible, adjust these to fit actual plates used.
plate_thickness = 3;
wheel_plate_thickness = 2;

// Unfortunately acrylic sheet thickness can vary a lot, and we 
// have to account for that in the design, eg for tab/slot joints.
// If you want tight joints, actually measure the plates first... 
// keeping in mind that a single plate can vary in thickness across
// its surface!  But the design works fine if you use the worst case,
// and the following value should be good for all but the worst plates.
plate_thickness_tol_percent = 10;
plate_thickness_tol = (plate_thickness_tol_percent/100)
                    * plate_thickness; 
wheel_plate_thickness_tol = (plate_thickness_tol_percent/100)
                          * wheel_plate_thickness;

// MAIN FORM PARAMETERS
dihedral = 10; // angle between wheels, 0 is vertical
base_radius = 200/2; // radius of basic platform

// other boolean selections
use_arm = false; // enable arm on upper platform (WIP)
use_up = true; // use UP Board
use_tc = false; // use standard Joule carrier (TuChuck)
use_gum = false; // use Gumstix board
use_up_holes = true; // include mounting holes for UP/Gumstix
use_tc_holes = true; // include mounting holes for TC
use_imu_holes = true; // include mounting holes for IMU
use_front_casters = true; // make this true if you want the OPTION of front casters
use_tread = true; // to better grip o-ring
use_tread_on_rim = false; // may be useful on deep carpet; increases cut time
use_tread_on_middle = false; // normally unnecessary; increases cut time
use_friction = false; // use friction drive, otherwise use belt
use_trans = true; // use transmission to improve mechanical advantage
use_relief = true; // use curved notch on T-slots to avoid stress
use_flanged_bearing = false; // use if possible, wheel is more stable
use_hub_bolts = true; // for more hub stabilty, esp. if not using flanged bearing
use_shoulder_bolt = true;  // use 6mm x 40mm + 10mm x M5 shoulder bolt for axle
use_cable_holes = true && !use_arm; // cable holes only if no arm
use_power_holes = !use_tc; // include mounting holes for power
use_rear_suspension = using_pom; // put rear caster on cantilevered suspension
use_front_suspension = using_pom; // put front casters on cantilevered suspension

// Enable individual external models.  
use_up_model = use_up && use_external_models; // use detailed model of up board
use_tc_model = use_tc && use_external_models; // use detailed model of tuchuck carrier
use_gum_model = use_gum && use_external_models; // use detailed model of gumstix carrier
use_servo_model = use_external_models;  // use detailed model of Dynamixel servo
use_bracket_model = use_external_models;  // use detailed model of servo bracket
use_power_model = !use_tc && use_external_models; // use detailed model of power module
use_imu_model = use_external_models; // use detailed model of imu

// Various visualization options.  Affects only rendering, not output model.
use_speaker = false; // affects only visualization
use_tire = true; // affects only visualization
use_lipo_battery = true;  // for visualization of max battery size
show_bolts = true; // show bolts as part of visualization
show_nuts = true;  // show nuts as part of visualization
show_tire = true;  // show tire as part of visualization
show_front_casters = false;  // show front casters in visualization
show_up_board = use_up && true;  // up board in visualization
show_tc_board = use_tc && true;  // tuchuck board in visualization
show_gum_board = use_gum && true; // gumstix board in visualization
show_power = show_up_board && true; // power in visualization (don't need with tc)

// TIRE PARAMETERS 

// NOTE: if you use a transmission, a 5.7mm (3/16") x 125mm (4.92") o-ring
// recommended, but do not change these dimensions
tire_wd = 5.7;
tire_id = 114.6; // of wheel, but o-ring itself is 125mm

// figure out if we should use a thick center plate in the wheel
middle_plate_thickness = 
  (tire_wd > 3*wheel_plate_thickness) 
  ? plate_thickness
  : wheel_plate_thickness;
// figure out if we should ALSO use thick plates in outer plates
outer_plate_thickness =
  (tire_wd > 2*wheel_plate_thickness + middle_plate_thickness)
  ? plate_thickness
  : wheel_plate_thickness;

// additional derived tire parameters
tire_r = tire_wd/2;
tire_R = tire_id/2 + tire_r;
tire_a = 30;
tire_e = 0.5; // "extra" radius to "stretch" tires for holding in place
tire_i = 2; // amount to inset inner wheel slice
tire_od = tire_id + 2*tire_wd + 1.5*tire_e;

// default trim value for friction drive (find exact value by experimentation)
friction_trim = 0.3;

// offset tower when using friction drive to optimize mechanical advantage
// TODO: needs a lot of tweaks and testing to work correctly
// friction_offset = 5;

// use for visualizing cross-section of tire and wheel for fitting
tire_cutout = false;

// wheel_radius = 112/2;
wheel_radius = tire_R - sin(tire_a)*tire_r;

// vertical shift of wheels (0 is flush with caster)
wheel_protrude = 0;

// move motors and wheels away from body to make extra clearance for nuts
clear_pad = 2;

// adjust until tires just touch ground (use orthogonal side view) when
// above value is zero (no protrusion)
wheel_oz = 2.45;    // for 5.7mm P-125 o-rings

motor_oz = wheel_oz - wheel_protrude;

// OTHER PARAMETERS
sm = 10 * sm_base;
eps = 0.0001;
tol = 0.1;
hole_sm = 2 * sm_base;
bolt_sm = sm_base;
torus_sm_R = 10 * sm_base;
torus_sm_r = 2 * sm_base;
tread_sm = sm_base;

wheel_hole_offset = 16/2;
wheel_tread_r = 1;
wheel_tread_n = 80;

m2_hole_radius = (2+tol)/2 + cut_t;
m2_5_hole_radius= (2.5+tol)/2 + cut_t;
m2_6_hole_radius= (2.6+tol)/2 + cut_t;
m3_hole_radius= (3+tol)/2 + cut_t;
//m5_hole_radius= (5+tol)/2 + cut_t;
//m6_hole_radius= (6+tol)/2 + cut_t;

// Robotis Dynamixel MX-12W servo
servo_h = 32;
servo_w = 32;
servo_l = 50;
servo_l2 = 36;
servo_h_xo = servo_w/2;
servo_h_yo = servo_h/2;
servo_h_r = 25/2;
servo_h_rr = 23/2;
servo_h_h = 5;
servo_b_h = 3;
servo_ww = 20;
servo_bw = 3;
servo_ll = 32+tol;

bracket_h = 5.5;
bracket_hole_spacing = 8;
bracket_hole_r = m2_hole_radius;
bracket_hole_sm = hole_sm;
bracket_base_x = 4*bracket_hole_spacing + 2;
bracket_base_y = 3*bracket_hole_spacing + 1;

wheel_clamp_n = 8;
wheel_clamp_offset = 0.9*wheel_radius;
wheel_cutout_R1 = 0.3*wheel_radius;
wheel_cutout_R2 = 0.7*wheel_radius;
wheel_cutout_scale = 0.1;
wheel_cutout_r1 = wheel_cutout_scale*wheel_cutout_R1;
wheel_cutout_r2 = 2.5*wheel_cutout_scale*wheel_cutout_R2;
wheel_cutout_sm = 3 * sm_base;
wheel_buttress_sm = 10 * sm_base;

// 7.4V battery... but does not work with MX-12W's :(
battery_radius = 27.5/2;
battery_length = 135;

// LiPo battery max size (use a 3S)
battery_x = 45;
battery_y = 140;
battery_z = 35;
battery_h_r = 2;

battery_offset_y = base_radius 
                 - battery_length
                 - 40;
battery_pad = 3;
battery_pad_lower = bracket_h/2 + 1;

battery_offset_x = 0;

battery_offset_z = -battery_radius;

battery_base_x = 2*battery_radius;

motor_offset_x = battery_radius + battery_pad;
motor_offset_z = -battery_radius;

base_pad = plate_thickness;
//base_r1 = battery_length + battery_offset_y;
//base_r2 = 2*battery_radius;
base_rm = base_radius - 10;
base_r = base_rm + 10;
base_sm = 10 * sm_base;
base_xm = battery_radius;
base_xh = base_xm
        + (battery_radius + battery_pad + servo_h + servo_b_h)
        / cos(dihedral)
        + battery_radius*sin(dihedral)
        + base_pad;
base_x = 2*base_xh;
base_y = 2*base_r;
base_offset_z = -2*battery_radius-battery_pad_lower-plate_thickness;

tower_offset_y = servo_l2 + bracket_h + plate_thickness;
            // + (use_trans && use_friction ? -friction_offset : 0);

up_board_standoff_h = 15;
up_board_standoff_sm = 6;
up_board_x = 85;
up_board_y = 56;
up_board_z = 20;
up_board_hole_r = m2_6_hole_radius;
up_board_hole_sm = hole_sm;
up_board_hole_ix = up_board_hole_r + 2.1;
up_board_hole_iy = up_board_hole_r + 2.1;
up_board_hole_dx = up_board_x - 2*up_board_hole_ix - 20.1;
up_board_hole_dy = up_board_y - 2*up_board_hole_iy;
up_board_ox = 0;
up_board_oy = tower_offset_y + up_board_standoff_h;
up_board_oz = 54;
up_board_standoff_r = up_board_hole_r;
up_board_standoff_R = up_board_hole_r+1;

gum_board_x = 80;
gum_board_y = 85;
gum_board_z = 20;
gum_board_ox = 0;
gum_board_oy = tower_offset_y + up_board_standoff_h;
gum_board_oz = 52;

// Note: the numbers in the online "documentation" for these are wrong!
// power_x = 48.641;  From docs... WRONG!!!
power_x = 50.6;  // measured
power_y = 20;
//power_z = 43.307;   From docs... WRONG!!!
power_z = 46.55;  // measured

power_standoff_h = 6;
power_standoff_sm = 2*sm_base;  
power_ox = 0;
power_oy = servo_l2 + bracket_h - power_standoff_h;
power_oz = tower_offset_y + 1;

power_hole_r = m3_hole_radius + 0.2;
power_hole_ix = 3.5;
power_hole_iy = 3.5;
power_hole_dx = power_x - 2*power_hole_ix;
power_hole_dy = power_z - 2*power_hole_iy;
power_hole_sm = hole_sm;

power_standoff_r = power_hole_r;
power_standoff_R = power_standoff_r + 1.5;

camera_x = 129;
camera_y = 8.2;
camera_z = 19.4;

camera_ox = 0;
camera_oy = servo_l2 + bracket_h;
camera_oz = up_board_oz + 25;

camera_sx = 60;
camera_sz = 4;
camera_six = 1.4;
camera_siz = 1.4;

camera_hole_r = m3_hole_radius;
camera_hole_ix = camera_six + camera_hole_r;
camera_hole_iz = camera_siz + camera_sz/2;
camera_hole_dx = camera_sx - camera_hole_r - camera_six;
camera_hole_sm = hole_sm;

camera_slot_x = 4;
camera_slot_y = 2;
camera_slot_ey = 1;
camera_slot_ix = 5;

tower_x = base_x + 10;

tower_oy = -2*battery_radius - plate_thickness - battery_pad_lower;
tower_cx = battery_radius;
tower_cy = -battery_radius;
tower_y = camera_oz + camera_y - base_offset_z + battery_pad_lower;
tower_r = 2.5*plate_thickness + 3 + clear_pad;
tower_rr = tower_r; // - 1.5*plate_thickness - 3;
tower_sm = 3 * sm_base;
tower_tab_s = 2*plate_thickness;
tower_tab_w = base_x/2 - 2*battery_radius - battery_pad 
            - base_pad - 2*tower_tab_s;
tower_tab_ox = 2*battery_radius + battery_pad 
             + base_pad + tower_tab_s;

// top_r = base_rm + 2*plate_thickness;
top_r = base_r;
top_h = 50;
top_sm = 10*sm_base;
top_xh = base_xm
        + (battery_radius + battery_pad + base_pad + servo_h + servo_b_h);
top_x = 2*(top_xh - sin(dihedral)*(top_h - battery_radius - plate_thickness));
      //+ 2*plate_thickness;
top_y = 2*top_r - tower_offset_y;
top_offset_y = tower_offset_y;
top_offset_z = base_offset_z + plate_thickness + top_h;

top_tab_s = 2*plate_thickness;
top_tab_w = top_x/2 - 2*battery_radius - battery_pad 
            - base_pad - top_tab_s;
top_tab_ox = 2*battery_radius + battery_pad 
             + base_pad;

speaker_r1 = 96/2;
speaker_r2 = 80/2;
speaker_r3_1 = 100/2;
speaker_r3_2 = 120/2;
speaker_h1 = 4;
speaker_h2 = 8;
speaker_c = 4;
speaker_r = 500+speaker_c;
speaker_h3 = 20-speaker_c;
speaker_sm = 10 * sm_base;
speaker_oy = -48;

// pololu 1" ball caster with plastic rollers
// https://www.pololu.com/product/2691
caster_t = 2;
caster_oyy = 6 + caster_t;
caster_r = 25.4/2; // 1" ball, eg pololu plastic ball caster
caster_R1 = 34/2; // plastic housing radius, base
caster_R2_1 = 32/2; // plastic housing, top
caster_R2_2 = caster_r + 1; // plastic housing, top
caster_h1 = 5; // height of base
caster_h = 2.54; // additional height of housing
caster_h2 = caster_h + 1.1*caster_r;
caster_H = 2*caster_r + caster_h; // overall height of housing
caster_sm = 10 * sm_base;
caster_or = base_radius - caster_R1;
caster_bolt_R = 14/2; // distance between mounting holes
caster_bolt_r = m3_hole_radius;  // radius of mounting holes (3.2mm)
//caster_ox = base_x/2 - caster_R;
caster_a = atan((base_radius-caster_R1)/(base_x/2));
caster_ox = cos(caster_a)*caster_or;
caster_oy = sin(caster_a)*caster_or - caster_oyy;
caster_sl = 50;

// standoff defaults
spacer_h = top_h;
spacer_hole_r = m3_hole_radius;
spacer_R = 6.5/2; // M3
spacer_sm = 6; // actually do want hexagons

//spacer_ox = caster_ox + cos(30)*caster_bolt_R;
//spacer_oy = caster_oy - sin(30)*caster_bolt_R;

spacer_ox = battery_pad + 3*base_pad + 2*battery_radius;
spacer_oy = caster_oy + 2.5*caster_bolt_R + caster_oyy;

cable_hole_sm = hole_sm;

tower_cable_r = 5;
tower_cable_ox = up_board_y/2 + tower_cable_r;
tower_cable_h = 4*tower_cable_r;
tower_cable_s = 10;
tower_cable_oz = top_offset_z 
               + plate_thickness + plate_thickness_tol 
               + tower_cable_h/2 + tower_cable_r + tower_cable_s;
               
top_cable_r = 5;
top_cable_ox = up_board_y/2 + top_cable_r - 5;
top_cable_h = 4*top_cable_r;
top_cable_s = 10;
top_cable_oy = top_offset_y
             - plate_thickness - plate_thickness_tol 
             - top_cable_h/2 - top_cable_r - top_cable_s;
             
battery_strap_r = 3/2;
battery_strap_ox = 2*battery_radius + battery_strap_r;
battery_strap_h = 2*battery_strap_r + 15;
battery_strap_s_1 = 10;
battery_strap_s_2 = battery_offset_y + battery_length - battery_strap_h;
battery_strap_oy = top_offset_y
                   - plate_thickness - plate_thickness_tol 
                   - battery_strap_h/2 - battery_strap_r;
battery_strap_oy_1 = battery_strap_oy - battery_strap_s_1;
battery_strap_oy_2 = battery_strap_oy - battery_strap_s_2;

switch_x = 11;
switch_y = 29;
switch_t = 4;
switch_ox = top_x/2 - switch_x/2 - switch_t;
switch_oy = base_y/2 - 41;

T_slot_sm = hole_sm;
T_slot_relief_r = 1.5/2; // radius of relief slot (if used)

// McMaster-Carr bearing: http://www.mcmaster.com/#7804k105/=1359lzz
// Works with M5 "axle"
// bearing_id = 5;
// bearing_r = bearing_id/2;
// bearing_od = 9;
// bearing_R = bearing_od/2;
// bearing_h = 3;  
// bearing_sm = 4*sm_base;
// bearing_lip = 1;

// Powers bearing: PJ-BB12660ZZ
// Works with M6 "axle"
bearing_id = 6;
bearing_r = bearing_id/2;
bearing_od = 12;
bearing_R = bearing_od/2;
bearing_h = 4;  
bearing_sm = 4*sm_base;
bearing_lip = 1;

// if flanged bearing used, give flange height
// this is for http://www.mcmaster.com/#7804k142/=136cs4d, which otherwise
// has the same dimensions as the PJ-BB12660ZZ
bearing_f = use_flanged_bearing ? 0.8 : 0;
bearing_fR = 13.6/2;

// How much to extend servo axle for transmission
motor_trans_offset = 2;
motor_trans_offset_t = use_trans ? -motor_trans_offset*plate_thickness : 0;

// Axle specs.  Need to match bearing choice, above.
axle_spacer_hh = ((use_flanged_bearing) 
                  ? bearing_f
                  : (bearing_h-wheel_plate_thickness-outer_plate_thickness));
axle_spacer_h = motor_trans_offset*plate_thickness
              + (servo_h_h-plate_thickness)
              - axle_spacer_hh;
axle_size = bearing_id;
axle_r = axle_size/2 + cut_t + tol;
axle_inner_r = axle_r; // 40mm shoulder penetrates hole 2mm
axle_shoulder_length = 40;  // shoulder part
axle_bolt_size = (use_shoulder_bolt) ? 5 : 6;
axle_bolt_r = axle_bolt_size/2;
axle_cap_size = 10;
axle_cap_r = axle_cap_size/2;
axle_cap_h = 4;
axle_length = 50; // want bolt with 12 mm or so unthreaded, tend to be long 
axle_tighten = 3;
axle_sm = 2*bolt_sm;

// Additional servo bracket parameters
bracket_t = 3;
bracket_s = 1;
bracket_tt = 0.5;
bracket_cs = 1;
bracket_cr = bracket_hole_r + 1 + cut_t;
bracket_rr = (bracket_base_y - 2*bracket_hole_spacing)/2;
bracket_sm = 5*base_sm;
bracket_hh = bracket_h + 0.8;
bracket_tweak_x = 0.5;
bracket_tweak_y = 0.25;

// Driver wheel for servo; when using o-ring belt transmission
driver_tread_r = wheel_tread_r;
driver_radius = wheel_hole_offset + m2_hole_radius;
driver_tread_n = 16;
driver_center_r = 6/2;

// Motor mount (for transmission)
mount_r = servo_w/2+2;
mount_R = 1.0*mount_r;
mount_rr_1 = 1;
mount_rr_2 = 5;
mount_a_2 = 20;
mount_sm = 10*sm_base;
mount_he = plate_thickness*cos(dihedral);
mount_h = (top_h + 2*plate_thickness)/cos(dihedral) + mount_he/2;
mount_B = -servo_h-motor_trans_offset_t;
mount_C = battery_radius+battery_pad+base_pad+servo_b_h+clear_pad;
mount_D = mount_C-mount_B;
mount_Q = mount_D*sin(dihedral) + mount_he/2;

wheel_bearing_offset = use_flanged_bearing
                     ? (outer_plate_thickness + wheel_plate_thickness 
                        - bearing_h 
                        + bearing_f)
                     : 0;
wheel_bolt_offset = use_flanged_bearing
                  ? bearing_f
                  : bearing_h-wheel_plate_thickness-outer_plate_thickness
                    +(use_shoulder_bolt ? 1 : 0);
//axle_spacer_h_2 = 15;
axle_spacer_h_2 = 14; // because 15mm not available at McMaster-Carr...
rim_e = 0.6*tire_r;

// Friction drive radius (basic offset, plus trim found by experimentation)
friction_r = 2*servo_l2+2*bracket_h+plate_thickness 
           - tire_R -tire_r + tire_e
           + friction_trim;
friction_rim_r = 2*servo_l2+2*bracket_h+plate_thickness
           - wheel_radius - rim_e 
           - 0.5;
friction_spacer_r = friction_r - 2;
friction_tread_n = 2*driver_tread_n;

// Specs for IMU: based on the LSM9DS1 board from Sparkfun:
// https://www.sparkfun.com/products/13284
imu_w = 23; // 23.45; 
imu_h = 23; // 23.45; 
imu_d = 3;
imu_sw = 17.8; 
imu_sh = 10.2; 
imu_hole_r = 3.35/2; // m3_hole_radius;
imu_hole_sm = hole_sm;
imu_standoff_h = 6;
imu_standoff_sm = 2*sm_base;  
imu_standoff_r = imu_hole_r;
imu_standoff_R = imu_standoff_r + 1.5;

// Arm (WIP)
arm_servo_offset_x = 0;
arm_servo_offset_y = 4.46;
arm_servo_offset_z = top_offset_z+servo_l2+plate_thickness+bracket_h;
arm_servo_offset_dx = servo_l2 - 6.5;
arm_servo_offset_dy = 8;

// rough model of Jabra 510+ USB speakerphone, useful for voice recog
module speaker() {
  color([0.2,0.2,0.2,0.5]) {
    cylinder(r=speaker_r1,h=speaker_h1,$fn=speaker_sm);
    translate([0,0,speaker_h1])
      cylinder(r=speaker_r2,h=speaker_h2,$fn=speaker_sm);
    translate([0,0,speaker_h1+speaker_h2])
      cylinder(r1=speaker_r3_1,r2=speaker_r3_2,h=speaker_h3,$fn=speaker_sm);
    translate([0,0,speaker_h1+speaker_h2+speaker_h3]) { 
      intersection() {
        cylinder(r=speaker_r3_2,h=speaker_c,$fn=speaker_sm);
        translate([0,0,-speaker_r+speaker_c])
          sphere(r=speaker_r,$fn=5*speaker_sm);
      }
    } 
  }
}

// bolt (default M3 x 10)
module bolt(size=3,length=10,sm=bolt_sm) {
  if (show_bolts) {
    color([0.5,0.5,0.7,0.8]) {
      translate([0,0,-length])
        cylinder(r=0.45*size,h=length,$fn=sm);
      difference() {
        cylinder(r=0.8*size,h=size,$fn=2*sm);
        translate([0,0,0.2*size])
          cylinder(r=(size-0.5)/2,h=size,$fn=6);
      }
    }
  }
}
// shoulder bolt (used for axle)
module shoulder_bolt(size=axle_size,cap_d=axle_cap_size,cap_h=axle_cap_h,shoulder_length=axle_shoulder_length,length=axle_length,bolt_size=axle_bolt_size,sm=axle_sm) {
  if (show_bolts) {
    color([0.5,0.5,0.7,0.8]) {
      translate([0,0,-shoulder_length])
        cylinder(r=0.49*size,h=shoulder_length,$fn=sm);
      translate([0,0,-length])
        cylinder(r=0.45*bolt_size,h=length,$fn=sm);
      difference() {
        cylinder(r=cap_d/2,h=cap_h,$fn=2*sm);
        translate([0,0,0.5*cap_h])
          cylinder(r=(cap_d/2)/2,h=cap_h,$fn=6);
      }
    }
  }
}

// a locknut (default M3)
module locknut(size=3) {
  if (show_nuts) {
    color([0.5,0.5,0.7,0.8]) 
    difference() {
      intersection() {
        cylinder(r=size,h=1.3*size,$fn=6);
        cylinder(r1=1.2*size,r2=0.7*size,h=1.5*size,$fn=2*bolt_sm);
      }
      translate([0,0,-size])
        cylinder(r=0.5*size,h=4*size,$fn=bolt_sm);
    }
  }
}

// slot for mounting panels at 90 degrees in a tab using bolt and locknut
module T_slot(size=3+cut_t,length=6.5+cut_t,
              nut_size=5.5+cut_t,nut_height=3+cut_t,
              thick=plate_thickness,pad=0,
              ext=1) {
  // for bolt
  translate([-size/2,-ext])
    square([size,length+pad+ext]);
  // for nut
  translate([-nut_size/2,thick+pad])
    square([nut_size,nut_height]);
  translate([0,thick+nut_height+pad])
    circle(r=nut_size/2,$fn=T_slot_sm);
  // to avoid stress concentration, round out sharp interior corners
  if (use_relief) {
    translate([0,thick+pad+T_slot_relief_r]) {
      translate([nut_size/2,0])
        circle(r=T_slot_relief_r,$fn=T_slot_sm);
      translate([-nut_size/2,0])
        circle(r=T_slot_relief_r,$fn=T_slot_sm);
    }
  }
}
module caster() {
  rotate([0,180,0]) {
    color([1.0,1.0,1.0,0.5])
      translate([0,0,caster_r+caster_h]) 
        sphere(r=caster_r,$fn=caster_sm);
    color([0.2,0.2,0.2,0.7]) {
      cylinder(r=caster_R1,h=caster_h1,$fn=caster_sm);
      cylinder(r1=caster_R2_1,r2=caster_R2_2,h=caster_h2,$fn=caster_sm);
    }
  }
}
module caster_holes() {
  rotate([0,0,60])
    translate([0,caster_bolt_R]) 
      circle(r=caster_bolt_r,$fn=hole_sm);
  rotate([0,0,-60])
    translate([0,caster_bolt_R]) 
      circle(r=caster_bolt_r,$fn=hole_sm);
  rotate([0,0,180])
    translate([0,caster_bolt_R]) 
      circle(r=caster_bolt_r,$fn=hole_sm);
}
module caster_bolts() {
  rotate([0,0,60])
    translate([0,caster_bolt_R,0]) 
      bolt(size=3,length=8);
  rotate([0,0,-60])
    translate([0,caster_bolt_R,0]) 
      bolt(size=3,length=8);
  rotate([0,0,180])
    translate([0,caster_bolt_R,0]) 
      bolt(size=3,length=8);
}
module caster_nuts() {
  rotate([0,0,60])
    translate([0,caster_bolt_R,0]) 
      locknut(size=3);
  rotate([0,0,-60])
    translate([0,caster_bolt_R,0]) 
      locknut(size=3);
  rotate([0,0,180])
    translate([0,caster_bolt_R,0]) 
      locknut(size=3);
}
module caster_suspension(flip=0,cutout=0) {
  difference() {
    difference() {
      union() {
        circle(r=caster_R1+caster_t,$fn=caster_sm);
        if (cutout) {
          translate([flip ? -caster_R1-caster_t : 0,-base_y])
            square([caster_R1+caster_t,base_y]);
        }
      }
      circle(r=caster_R1,$fn=caster_sm);
    }
    translate([-caster_R1-caster_t,0])
      square([2*(caster_R1+2*caster_t),2*(caster_R1+2*caster_t)]);
  }
  translate([caster_R1,0])
      square([caster_t,caster_sl]);
  translate([caster_R1+caster_t/2,caster_sl])
      circle(r=caster_t/2,$fn=caster_sm); 
  translate([-caster_R1-caster_t,0])
      square([caster_t,caster_sl]);
  translate([-caster_R1-caster_t/2,caster_sl])
      circle(r=caster_t/2,$fn=caster_sm); 
}
module switch_hole(t=cut_t) {
  translate([-switch_x/2-t,-switch_y/2-t])
    square([switch_x+2*t,switch_y+2*t]);
}
module spacer_hole(r=spacer_hole_r,sm=hole_sm) {
  circle(r=r,$fn=sm);
}
module spacer(r=spacer_hole_r,R=spacer_R,h=spacer_h,SM=spacer_sm,sm=hole_sm) {
  color([0.5,0.5,0.5,0.9])
    difference() {
      cylinder(r=R,h=h,$fn=SM);
      translate([0,0,-h]) cylinder(r=r,h=3*h,$fn=sm);
    }
}
// MX-12W is same form factor as AX12
module servo(a=0) {
  color([0.3,0.3,0.5,0.5]) 
    translate([0,0,-servo_h/2-servo_h_h])
      scale(10) 
        rotate([0,0,a])
          import("External/servo.stl",convexity=5);
}
module bracket_holes(n=3,m=3,omit_center=true,trans=false) {
  for (i=[0:n-1]) {
    for (j=[0:m-1]) {
      if (!omit_center || i == 0 || i == n-1 || j == 0 || j == m-1) {
        if (!trans || i == 0 || i == n-1) {
          translate([i*bracket_hole_spacing,j*bracket_hole_spacing])
            circle(r=bracket_hole_r,$fn=bracket_hole_sm);
        }
      }
    }
  }
}
module bracket() {
  if (!use_bracket_model || printed) {
    //color([0.5,0.4,0.4,0.2]) import("External/F3.stl",convexity=5);
    difference() {
      // outer shell
      hull() {
        translate([-bracket_base_y/2,0,-bracket_base_x/2])
          cube([bracket_base_y,bracket_t,bracket_base_x]);
        intersection() {
          translate([-bracket_base_y/2,
                     bracket_t-cut_t,
                     -servo_w/2-bracket_t])
            cube([bracket_base_y,
                  bracket_h+bracket_tt,
                  servo_w+2*bracket_t]);
          union() {
            translate([bracket_hole_spacing,
                       bracket_h,
                       -servo_w/2-bracket_t])
              cylinder(r=bracket_rr,
                       h=servo_w+2*bracket_t,
                       $fn=bracket_sm);
            translate([-bracket_hole_spacing,
                       bracket_h,
                       -servo_w/2-bracket_t])
              cylinder(r=bracket_rr,
                       h=servo_w+2*bracket_t,
                       $fn=bracket_sm);
          }
        }
       }
       // servo mount surface
       translate([0,bracket_t-cut_t,0]) hull() {
        translate([-bracket_base_y/2-eps,
                   0,
                   -servo_w/2+bracket_s-cut_t])
          cube([bracket_base_y+2*eps,
                bracket_s,
                servo_w-2*bracket_s+2*cut_t]);
        translate([-bracket_base_y/2-bracket_t,
                   bracket_s,
                   -servo_w/2-cut_t])
          cube([bracket_base_y+2*bracket_t,
                bracket_h,
                servo_w+2*cut_t]);
       }
       // bolt holes
       translate([bracket_hole_spacing,bracket_h,-bracket_base_y])
          cylinder(r=bracket_hole_r,
                   h=2*bracket_base_y,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,bracket_h,-bracket_base_y])
          cylinder(r=bracket_hole_r,
                   h=2*bracket_base_y,
                   $fn=bracket_hole_sm);
       // countersinks for bolt heads
       translate([bracket_hole_spacing,
                  bracket_h,
                  servo_w/2+bracket_t-bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,
                  bracket_h,
                  servo_w/2+bracket_t-bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([bracket_hole_spacing,
                  bracket_h,
                  -3*servo_w/2-bracket_t+bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
       translate([-bracket_hole_spacing,
                  bracket_h,
                  -3*servo_w/2-bracket_t+bracket_cs])
          cylinder(r=bracket_cr,
                   h=servo_w,
                   $fn=bracket_hole_sm);
     }
  } else {
    import("External/F3.stl",convexity=5);
  }
}
module end_bracket(a=0) {
  color([0.5,0.3,0.3,0.5]) 
  rotate([0,0,a-90])
    translate([0,servo_l2+bracket_h,-servo_h/2-servo_h_h]) 
      rotate([0,0,180])
        bracket();
}
module side_bracket(a=0,s=0,n=0) {
  color([0.5,0.3,0.3,0.5]) 
  rotate([0,0,a])
  translate([8+n*8+6.75,0]) 
    rotate([0,0,s])
    translate([0,servo_w/2+servo_h_h-2,-servo_h/2-servo_h_h])
      rotate([0,0,180])
        bracket();
}
tc_board_x = 85;
tc_board_y = 70;
tc_board_z = up_board_z/2;
tc_board_hox = 4;
tc_board_hoy = 18.5;
tc_board_hx1 = 0;
tc_board_hy1 = 0;
tc_board_hx2 = 0;
tc_board_hy2 = 47.5;
tc_board_hx3 = 77;
tc_board_hy3 = -8;
tc_board_hx4 = 77;
tc_board_hy4 = 47.5;
tc_board_hole_r = m2_hole_radius;
tc_board_hole_sm = up_board_hole_sm;
tc_board_ox = up_board_ox;
tc_board_oy = up_board_oy+5;
tc_board_oz = up_board_oz-7;
tc_board_standoff_h = 20;
tc_board_standoff_r = tc_board_hole_r + tol;
tc_board_standoff_R = tc_board_standoff_r + 1;
tc_board_standoff_sm = 6;
tc_fan_standoff_h = 20;
tc_fan_standoff_r = tc_board_hole_r + tol;
tc_fan_standoff_R = tc_board_standoff_r + 1;
tc_fan_standoff_sm = 6;
tc_fan_ox = tc_board_ox;
tc_fan_oy = tc_board_oy+2+tc_fan_standoff_h;
tc_fan_oz = tc_board_oz;
module tc_board_holes(flip=false) {
  if (flip) {
    translate([-tc_board_x/2+tc_board_hox+tc_board_hx1,
               -tc_board_y/2+tc_board_hoy+tc_board_hy3])
      circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
  } else {
    translate([-tc_board_x/2+tc_board_hox+tc_board_hx1,
               -tc_board_y/2+tc_board_hoy+tc_board_hy1])
      circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
  }
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx2,
             -tc_board_y/2+tc_board_hoy+tc_board_hy2])
    circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
  if (flip) {
    translate([-tc_board_x/2+tc_board_hox+tc_board_hx3,
               -tc_board_y/2+tc_board_hoy+tc_board_hy1])
      circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
  } else {
     translate([-tc_board_x/2+tc_board_hox+tc_board_hx3,
               -tc_board_y/2+tc_board_hoy+tc_board_hy3])
      circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
  }
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx4,
             -tc_board_y/2+tc_board_hoy+tc_board_hy4])
    circle(r=tc_board_hole_r,$fn=tc_board_hole_sm);
}
module tc_board() {
  color([0.3,0.3,0.5,0.5]) 
    if (0 && use_tc_model) {
      scale(1000)
        import("External/TC.stl",convexity=5);
    } else {
      difference() {
        translate([-tc_board_x/2,-tc_board_y/2,0])
          cube([tc_board_x,tc_board_y,tc_board_z]);
        translate([0,0,-tc_board_z/2])
          linear_extrude(2*tc_board_z)
            tc_board_holes();
      }
    }
}
module tc_board_spacers() {
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx1,
             -tc_board_y/2+tc_board_hoy+tc_board_hy1,
             -tc_board_standoff_h])
    spacer(r=tc_board_standoff_r,R=tc_board_standoff_R,
           h=tc_board_standoff_h,$fn=tc_board_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx2,
             -tc_board_y/2+tc_board_hoy+tc_board_hy2,
             -tc_board_standoff_h])
    spacer(r=tc_board_standoff_r,R=tc_board_standoff_R,
           h=tc_board_standoff_h,$fn=tc_board_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx3,
             -tc_board_y/2+tc_board_hoy+tc_board_hy3,
             -tc_board_standoff_h])
    spacer(r=tc_board_standoff_r,R=tc_board_standoff_R,
           h=tc_board_standoff_h,$fn=tc_board_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx4,
             -tc_board_y/2+tc_board_hoy+tc_board_hy4,
             -tc_board_standoff_h])
    spacer(r=tc_board_standoff_r,R=tc_board_standoff_R,
           h=tc_board_standoff_h,$fn=tc_board_standoff_sm);
}
tc_fan_corner_r = 4;
tc_fan_r = 58/2; // 60mm fan
tc_fan_w = 60;
tc_fan_ww = 50;
tc_fan_bo = (tc_fan_w - tc_fan_ww)/2;
tc_fan_br = m3_hole_radius;
tc_fan_sm = sm;
module tc_fan_slice() {
  difference() {
    translate([-tc_board_x/2,-tc_board_y/2])
      hull() {
        translate([tc_fan_corner_r,tc_fan_corner_r])
          circle(r=tc_fan_corner_r,$fn=tc_fan_sm);
        translate([tc_board_x-tc_fan_corner_r,tc_fan_corner_r])
          circle(r=tc_fan_corner_r,$fn=tc_fan_sm);
        translate([tc_fan_corner_r,tc_board_y-tc_fan_corner_r])
          circle(r=tc_fan_corner_r,$fn=tc_fan_sm);
        translate([tc_board_x-tc_fan_corner_r,tc_board_y-tc_fan_corner_r])
          circle(r=tc_fan_corner_r,$fn=tc_fan_sm);
      }
    // hole for airflow
    translate([0,0])
      circle(r=tc_fan_r,$fn=tc_fan_sm);
    // mounting bolt holes for fan
    translate([-tc_fan_ww/2,-tc_fan_ww/2])
      circle(r=tc_fan_br,$fn=tc_fan_sm);
    translate([tc_fan_ww/2,-tc_fan_ww/2])
      circle(r=tc_fan_br,$fn=tc_fan_sm);
    translate([-tc_fan_ww/2,tc_fan_ww/2])
      circle(r=tc_fan_br,$fn=tc_fan_sm);
    translate([tc_fan_ww/2,tc_fan_ww/2])
      circle(r=tc_fan_br,$fn=tc_fan_sm);
    // holes to mount plate to tc
    tc_board_holes();
  }
}
module tc_fan_plate() {
  linear_extrude(wheel_plate_thickness)
    tc_fan_slice();
}
module tc_fan_spacers() {
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx1,
             -tc_board_y/2+tc_board_hoy+tc_board_hy1,
             -tc_fan_standoff_h])
    spacer(r=tc_fan_standoff_r,R=tc_fan_standoff_R,
           h=tc_fan_standoff_h,$fn=tc_fan_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx2,
             -tc_board_y/2+tc_board_hoy+tc_board_hy2,
             -tc_fan_standoff_h])
    spacer(r=tc_fan_standoff_r,R=tc_fan_standoff_R,
           h=tc_fan_standoff_h,$fn=tc_fan_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx3,
             -tc_board_y/2+tc_board_hoy+tc_board_hy3,
             -tc_fan_standoff_h])
    spacer(r=tc_fan_standoff_r,R=tc_fan_standoff_R,
           h=tc_fan_standoff_h,$fn=tc_fan_standoff_sm);
  translate([-tc_board_x/2+tc_board_hox+tc_board_hx4,
             -tc_board_y/2+tc_board_hoy+tc_board_hy4,
             -tc_fan_standoff_h])
    spacer(r=tc_fan_standoff_r,R=tc_fan_standoff_R,
           h=tc_fan_standoff_h,$fn=tc_fan_standoff_sm);
}
module up_board() {
  color([0.3,0.3,0.5,0.5]) 
    if (use_up_model) {
      scale(1000)
        import("External/UP.stl",convexity=5);
    } else {
      translate([-up_board_x/2,-up_board_y/2,0])
        cube([up_board_x,up_board_y,up_board_z]);
    }
}
module up_board_holes() {
  translate([-up_board_x/2+up_board_hole_ix,
             -up_board_y/2+up_board_hole_iy])
    circle(r=up_board_hole_r,$fn=up_board_hole_sm);
  translate([-up_board_x/2+up_board_hole_ix+up_board_hole_dx,
             -up_board_y/2+up_board_hole_iy])
    circle(r=up_board_hole_r,$fn=up_board_hole_sm);
  translate([-up_board_x/2+up_board_hole_ix+up_board_hole_dx,
             -up_board_y/2+up_board_hole_iy+up_board_hole_dy])
    circle(r=up_board_hole_r,$fn=up_board_hole_sm);
  translate([-up_board_x/2+up_board_hole_ix,
             -up_board_y/2+up_board_hole_iy+up_board_hole_dy])
    circle(r=up_board_hole_r,$fn=up_board_hole_sm);
}
module up_board_spacers() {
  translate([-up_board_x/2+up_board_hole_ix,
             -up_board_y/2+up_board_hole_iy,
             -up_board_standoff_h])
    spacer(r=up_board_standoff_r,R=up_board_standoff_R,
           h=up_board_standoff_h,$fn=up_board_standoff_sm);
  translate([-up_board_x/2+up_board_hole_ix+up_board_hole_dx,
             -up_board_y/2+up_board_hole_iy,
             -up_board_standoff_h])
    spacer(r=up_board_standoff_r,R=up_board_standoff_R,
           h=up_board_standoff_h,$fn=up_board_standoff_sm);
  translate([-up_board_x/2+up_board_hole_ix+up_board_hole_dx,
             -up_board_y/2+up_board_hole_iy+up_board_hole_dy,
             -up_board_standoff_h])
    spacer(r=up_board_standoff_r,R=up_board_standoff_R,
           h=up_board_standoff_h,$fn=up_board_standoff_sm);
  translate([-up_board_x/2+up_board_hole_ix,
             -up_board_y/2+up_board_hole_iy+up_board_hole_dy,
             -up_board_standoff_h])
    spacer(r=up_board_standoff_r,R=up_board_standoff_R,
           h=up_board_standoff_h,$fn=up_board_standoff_sm);
}
gum_scale = 0.1;
gum_rotate = -90;
module gum_board() {
  color([0.3,0.3,0.5,0.5]) 
    if (use_gum_model) {
      translate([-gum_board_x/2,gum_board_y/2,0])
        scale(gum_scale)
          rotate([0,0,gum_rotate])
            import("External/Gum.stl",convexity=5);
    } else {
      translate([-gum_board_x/2,-gum_board_y/2,0])
        cube([gum_board_x,gum_board_y,gum_board_z]);
    }
}
module wheel_cutout_holes() {
  for (i=[0:wheel_clamp_n-1]) {
    rotate((i+0.5)*360/wheel_clamp_n)
      hull() {
        translate([wheel_cutout_R1,0,0])
          circle(r=wheel_cutout_r1,$fn=wheel_cutout_sm);
        translate([wheel_cutout_R2,0,0])
          circle(r=wheel_cutout_r2,$fn=wheel_cutout_sm);
      }
  } 
}
module wheel_clamp_bolt_holes() {
  for (i = [0:wheel_clamp_n-1]) {
    rotate(i*360/wheel_clamp_n)
      translate([wheel_clamp_offset,0,0]) 
        circle(r=m3_hole_radius,$fn=hole_sm);
  }
}
module wheel_hub_bolt_holes(r) {
    for (i = [0:3]) {
    rotate(i*360/4)
      translate([r,0,0]) 
        circle(r=m3_hole_radius,$fn=hole_sm);
  }
}
module inner_wheel_slice(wr=wheel_radius+tire_e,hr=bearing_R+cut_t) {
  difference() {
    circle(r=wr,$fn=sm);
    if (use_trans) {
      circle(r=hr,$fn=bearing_sm);
      // hub bolt holes
      wheel_hub_bolt_holes((bearing_R+wheel_cutout_R1)/2);
    } else {
      // bolt holes
      for (i = [0:3]) {
        rotate(i*90)
          translate([wheel_hole_offset,0,0]) 
            circle(r=m2_hole_radius,$fn=hole_sm);
      }
    }
    // clamp holes
    wheel_clamp_bolt_holes();
    // tread (optional)
    if (use_tread) {
      for (i=[0:wheel_tread_n-1]) {
        rotate(i*360/wheel_tread_n) 
          translate([wr,0,0])
            circle(r=wheel_tread_r,$fn=tread_sm);
      }
    }
    // cutouts (to reduce inertia)
    wheel_cutout_holes();
  }
}
module middle_wheel_slice(wr=wheel_radius-tire_i,use_tread=use_tread_on_middle) {
  if (use_trans) {
    inner_wheel_slice(wr=wr,hr=bearing_R-bearing_lip,use_tread=use_tread);
  } else {
    difference() {
      circle(r=wr,$fn=sm);
      // clamp holes
      wheel_clamp_bolt_holes();
      // tread (optional)
      if (use_tread_on_middle) {
        for (i=[0:wheel_tread_n-1]) {
          rotate(i*360/wheel_tread_n) 
            translate([wr,0,0])
              circle(r=wheel_tread_r,$fn=tread_sm);
        }
      }
      // cutout (to reduce inertia)
      difference() {
        circle(r=wheel_cutout_R2+1.1*wheel_cutout_r2,$fn=4*wheel_cutout_sm);
        // clamp buttresses
        for (i = [0:wheel_clamp_n-1]) {
          rotate(i*360/wheel_clamp_n)
            hull() {
              translate([wheel_clamp_offset,0,0]) 
                circle(r=1.5*(m3_hole_radius+3),
                       $fn=wheel_buttress_sm);
              translate([2*wheel_clamp_offset,0,0]) 
                circle(r=1.5*8*(m3_hole_radius+3),
                       $fn=wheel_buttress_sm);
            }
        }
      }
    }
  }
}
module half_middle_wheel_slice() {
  intersection() {
    rotate((360/wheel_clamp_n)/2)
      middle_wheel_slice();
    translate([-2*wheel_radius,-laser_t/2])
      square([4*wheel_radius,4*wheel_radius]);
  }
}
module quarter_middle_wheel_slice() {
  intersection() {
    rotate((360/wheel_clamp_n)/2)
      middle_wheel_slice();
    translate([-laser_t/2,-laser_t/2])
      square([4*wheel_radius,4*wheel_radius]);
  }
}
module outer_wheel_slice(wr=wheel_radius+tire_e,use_tread=use_tread) {
  if (use_trans) {
    inner_wheel_slice(wr=wr);
  } else {
    difference() {
      circle(r=wr,$fn=sm);
      // clamp holes
      for (i = [0:wheel_clamp_n-1]) {
        rotate(i*360/wheel_clamp_n)
          translate([wheel_clamp_offset,0]) 
            circle(r=m3_hole_radius,$fn=hole_sm);
      }
      // tread
      if (use_tread) {
        for (i=[0:wheel_tread_n-1]) {
          rotate(i*360/wheel_tread_n) 
            translate([wr,0])
              circle(r=wheel_tread_r,$fn=tread_sm);
        }
      }
      // cutout (to reduce inertia)
      difference() {
        circle(r=wheel_cutout_R2+1.2*wheel_cutout_r2,$fn=4*wheel_cutout_sm);
        // clamp buttresses
        for (i = [0:wheel_clamp_n-1]) {
          rotate(i*360/wheel_clamp_n)
            hull() {
              translate([wheel_clamp_offset,0]) 
                circle(r=m3_hole_radius+3,
                       $fn=wheel_buttress_sm);
              translate([2*wheel_clamp_offset,0]) 
                circle(r=8*(m3_hole_radius+3),
                       $fn=wheel_buttress_sm);
            }
        }
      }
    }
  }
}

module rim_wheel_slice(wr=wheel_radius+rim_e) {
  if (use_trans) {
    inner_wheel_slice(wr=wr,use_tread=use_tread_on_rim);
  } else {
    difference() {
      circle(r=wr,$fn=sm);
      // clamp holes
      for (i = [0:wheel_clamp_n-1]) {
        rotate(i*360/wheel_clamp_n)
          translate([wheel_clamp_offset,0]) 
            circle(r=m3_hole_radius,$fn=hole_sm);
      }
      // tread
      if (use_tread_on_rim) {
        for (i=[0:wheel_tread_n-1]) {
          rotate(i*360/wheel_tread_n) 
            translate([wr,0])
              circle(r=wheel_tread_r,$fn=tread_sm);
        }
      }
      // cutout (to reduce inertia)
      difference() {
        circle(r=wheel_cutout_R2+1.2*wheel_cutout_r2,$fn=4*wheel_cutout_sm);
        // clamp buttresses
        for (i = [0:wheel_clamp_n-1]) {
          rotate(i*360/wheel_clamp_n)
            hull() {
              translate([wheel_clamp_offset,0]) 
                circle(r=m3_hole_radius+3,
                       $fn=wheel_buttress_sm);
              translate([2*wheel_clamp_offset,0]) 
                circle(r=8*(m3_hole_radius+3),
                       $fn=wheel_buttress_sm);
            }
        }
      }
    }
  }
}
module half_outer_wheel_slice() {
  intersection() {
    rotate((360/wheel_clamp_n)/2)
      outer_wheel_slice();
    translate([-2*wheel_radius,-laser_t/2])
      square([4*wheel_radius,4*wheel_radius]);
  }
}
module quarter_outer_wheel_slice() {
  intersection() {
    rotate((360/wheel_clamp_n)/2)
      outer_wheel_slice();
    translate([-laser_t/2,-laser_t/2])
      square([4*wheel_radius,4*wheel_radius]);
  }
}
module inner_wheel_plate() {
  linear_extrude(outer_plate_thickness)
    inner_wheel_slice();
}
module middle_wheel_plate() {
  linear_extrude(middle_plate_thickness)
    middle_wheel_slice();
}
module innermiddle_wheel_plate() {
  union() {
    inner_wheel_plate();  
    translate([0,0,wheel_plate_thickness-cut_t])
      middle_wheel_plate(); 
  }
}
module outer_wheel_plate() {
  linear_extrude(outer_plate_thickness)
    outer_wheel_slice();
}
module rim_wheel_plate() {
  linear_extrude(wheel_plate_thickness)
    rim_wheel_slice();
}

module wheel() {
  rim_wheel_plate();
  translate([0,0,wheel_plate_thickness]) {
    inner_wheel_plate();
  }
  if (use_trans) {
    translate([0,0,
               wheel_plate_thickness+outer_plate_thickness-bearing_h
               -wheel_bearing_offset])
      bearing();
    translate([0,0,
               -axle_spacer_h
               -axle_spacer_hh])
      spacer(r=bearing_r+tol,R=bearing_r+1,
             h=axle_spacer_h,sm=bearing_sm,SM=bearing_sm);
    translate([0,0,
               -axle_spacer_h
               -plate_thickness
               -axle_spacer_h_2
               -axle_spacer_hh])
      spacer(r=bearing_r+tol,R=bearing_r+1,
             h=axle_spacer_h_2,sm=bearing_sm,SM=bearing_sm);
  }
  translate([0,0,
             wheel_plate_thickness+outer_plate_thickness]) {
    middle_wheel_plate();
  }
  translate([0,0,
             wheel_plate_thickness+outer_plate_thickness+middle_plate_thickness]) {
    outer_wheel_plate();
    if (use_trans) {
      translate([0,0,bearing_h+wheel_bearing_offset])
        rotate([0,180,0])
          bearing();
      translate([0,0,bearing_h+wheel_bolt_offset])
        if (use_shoulder_bolt) {
          shoulder_bolt();
        } else {
          bolt(size=axle_size,length=axle_length,sm=axle_sm);
        }
      translate([0,0,
                 -axle_spacer_h
                 -axle_spacer_h_2
                 -plate_thickness
                 +wheel_plate_thickness+outer_plate_thickness-bearing_h
                 -2*wheel_plate_thickness
                 -2*outer_plate_thickness
                 -middle_plate_thickness
                 +wheel_bolt_offset])
        rotate([180,0,0]) 
          if (use_shoulder_bolt) {
            locknut(size=axle_bolt_size);
          } else {
            locknut(size=axle_size);
          }
    }
  }
  translate([0,0,
             wheel_plate_thickness+2*outer_plate_thickness+middle_plate_thickness]) {
    rim_wheel_plate();
  }
  // bolts holding plates together
  for (i = [0:wheel_clamp_n-1]) {
    rotate(i*360/wheel_clamp_n) {
      translate([wheel_clamp_offset,0,
                 2*wheel_plate_thickness
                +2*outer_plate_thickness
                +middle_plate_thickness]) 
        bolt(size=3,length=15);
      translate([wheel_clamp_offset,0,-tol]) 
        rotate([180,0,0]) 
          locknut(size=3);
    }
    for (i = [0:3]) {
      rotate(i*360/4) {
        translate([(bearing_R+wheel_cutout_R1)/2,0,
                    2*wheel_plate_thickness
                   +2*outer_plate_thickness
                   +middle_plate_thickness]) 
          bolt(size=3,length=15);
        translate([(bearing_R+wheel_cutout_R1)/2,0,-tol]) 
          rotate([180,0,0]) 
            locknut(size=3);
      }
    }
  }
}
module driver_slice(
  dr = (use_friction ? friction_r : driver_radius+tire_e),
  tread_n = (use_friction ? friction_tread_n : driver_tread_n)
) {
  difference() {
    circle(r=dr,$fn=sm);
    // bolt holes
    for (i = [0:3]) {
      rotate(i*90)
        translate([wheel_hole_offset,0]) 
          circle(r=m2_hole_radius,$fn=hole_sm);
    }
    // center hole
    circle(r=driver_center_r,$fn=hole_sm);
    // tread (optional)
    if (use_tread) {
      for (i=[0:tread_n-1]) {
        rotate(i*360/tread_n) 
          translate([dr,0])
            circle(r=driver_tread_r,$fn=tread_sm);
      }
    }
  }
}
module driver_plate() {
  linear_extrude(outer_plate_thickness)
    driver_slice();
}
module driver_spacer_slice(
  wr = (use_friction ? friction_spacer_r : driver_radius-m2_hole_radius),
  offset_r = (use_friction ? 0 : 2*m2_hole_radius)
) {
  difference() {
    circle(r=wr,$fn=sm);
    // bolt holes
    for (i = [0:3]) {
      rotate(i*90)
        hull() {
          translate([wheel_hole_offset,0]) 
            circle(r=m2_hole_radius,$fn=hole_sm);
          translate([wheel_hole_offset+offset_r,0]) 
            circle(r=m2_hole_radius,$fn=hole_sm);
        }
    }
    // center hole
    circle(r=driver_center_r,$fn=hole_sm);
  }
}
module driver_rim_slice(
  wr = (use_friction ? friction_rim_r : driver_radius+1.5*tire_r),
  offset_r = 0
) {
  driver_spacer_slice(wr=wr,offset_r=offset_r);
}
module driver_spacer_plate(
  wr = (use_friction ? friction_spacer_r : driver_radius-m2_hole_radius),
  thickness = plate_thickness,
  offset_r = (use_friction ? 0 : 2*m2_hole_radius)
) {
  linear_extrude(thickness)
    driver_spacer_slice(wr=wr,offset_r=offset_r);
}
module driver_rim_plate(
  wr = (use_friction ? friction_rim_r : driver_radius+1.5*tire_r),
  offset_r = 0
) {
  linear_extrude(wheel_plate_thickness)
    driver_rim_slice(wr=wr,offset_r=offset_r);
}
module driver_mount_slice(wr=driver_radius+2,offset_r=0) {
  driver_spacer_slice(wr=wr,offset_r=offset_r);
}
module driver_mount_plate(thickness=plate_thickness) {
  linear_extrude(thickness)
    driver_mount_slice(wr=driver_radius+2,offset_r=0);
}
module driver() {
  for (i = [0:motor_trans_offset-1]) {
    translate([0,0,i*plate_thickness])
      driver_mount_plate();
  }
  translate([0,0,motor_trans_offset*plate_thickness]) {
    driver_rim_plate();
    translate([0,0,
              wheel_plate_thickness])
      driver_plate();
    translate([0,0,
              wheel_plate_thickness+outer_plate_thickness])
      driver_spacer_plate(thickness=middle_plate_thickness);
    translate([0,0,
              wheel_plate_thickness+outer_plate_thickness+middle_plate_thickness])
      driver_plate();
    translate([0,0,
              wheel_plate_thickness+2*outer_plate_thickness+middle_plate_thickness])
      driver_rim_plate();
  }
  for (i = [0:3]) {
    rotate(i*360/4)
      translate([wheel_hole_offset,0,
                 motor_trans_offset*plate_thickness
                 +2*wheel_plate_thickness
                 +2*outer_plate_thickness
                 +middle_plate_thickness]) 
          bolt(size=2,length=20);
  }
}
// helper function; wish this was built in...
module torus(R=10,r=1,sm_R=torus_sm_R,sm_r=torus_sm_r) {
  rotate_extrude(convexity=4,$fn=sm_R)
    translate([R,0,0]) 
      circle(r=r,$fn=sm_r);
}
module tire() {
  if (show_tire) {
   color([0.5,0.5,0.5,0.9])
    if (use_trans && !use_friction) {
      minkowski() {
        difference() {
          hull() {
            translate([0,0,-eps/2])
              cylinder(r=tire_R+eps,h=eps,$fn=torus_sm_R);
            translate([0,2*servo_l2+2*bracket_h+plate_thickness,-eps/2])
              cylinder(r=driver_radius+tire_r+eps,h=eps,$fn=torus_sm_R);
          }
          hull() {
            translate([0,0,-eps])
              cylinder(r=tire_R-eps,h=2*eps,$fn=torus_sm_R);
            translate([0,2*servo_l2+2*bracket_h+plate_thickness,-eps])
              cylinder(r=driver_radius+tire_r-eps,h=2*eps,$fn=torus_sm_R);
          }
        }
        sphere(r=tire_r,$fn=torus_sm_r);
      }
    } else {
       torus(R=tire_R,r=tire_r);
    }
  }
}
module bearing() {
  color([0.2,0.2,0.9,0.8])
  difference() {
    union() {
      cylinder(r=bearing_R,h=bearing_h,$fn=bearing_sm);
      if (use_flanged_bearing) cylinder(r=bearing_fR,h=bearing_f,$fn=bearing_sm);
    }
    translate([0,0,-bearing_h]) 
      cylinder(r=bearing_r,h=3*bearing_h,$fn=bearing_sm);
  } 
}
module battery() {
  color([0.3,0.5,0.3,0.5])
    if (use_lipo_battery) {
      translate([-battery_x/2,0,-battery_radius-battery_pad])
        cube([battery_x,battery_y,battery_z]);
    } else {
      hull() {
        translate([battery_radius,0,0])
          rotate([-90,0,0]) 
            cylinder(r=battery_radius,h=battery_length);
        translate([-battery_radius,0,0])
          rotate([-90,0,0]) 
            cylinder(r=battery_radius,h=battery_length);
      }
    }
}
module camera() {
  color([0.1,0.1,0.7,0.5])
  translate([-camera_x/2,0,0]) {
    difference() {
      cube([camera_x,camera_y,camera_z]);
      translate([camera_six,-0.05,camera_siz])
        cube([camera_sx,camera_y+0.1,camera_sz]);
    }
  }
}
module camera_holes() {
  // holes for bolts
  translate([-camera_x/2+camera_hole_ix,camera_hole_iz])
    circle(r=camera_hole_r,$fn=camera_hole_sm);
  translate([-camera_x/2+camera_hole_ix+camera_hole_dx/2,camera_hole_iz])
    circle(r=camera_hole_r,$fn=camera_hole_sm);
  //translate([-camera_x/2+camera_hole_ix+camera_hole_dx,camera_hole_iz])
  //  circle(r=camera_hole_r,$fn=camera_hole_sm);
  
  // slots for ties
  translate([camera_x/2-camera_slot_ix-camera_slot_x,-camera_slot_y])
    square([camera_slot_x,camera_slot_y+camera_slot_ey]);
  translate([camera_x/2-camera_slot_ix-camera_slot_x,camera_z-camera_slot_ey])
    square([camera_slot_x,camera_slot_y+camera_slot_ey]);
}
module power() {
  color([0.1,0.7,0.1,0.5])
    if (use_power_model) {
      translate([0.5,1.5,0])
        rotate([0,-90,0])
          import("External/DC.stl",convexity=5);
    } else {
      translate([-power_x/2,0,-power_z/2])
        cube([power_x,power_y,power_z]);
    }
}
module power_holes() {
  translate([-power_x/2+power_hole_ix,
             -power_z/2+power_hole_iy])
    circle(r=power_hole_r,$fn=power_hole_sm);
  translate([-power_x/2+power_hole_ix+power_hole_dx,
             -power_z/2+power_hole_iy])
    circle(r=power_hole_r,$fn=power_hole_sm);
  translate([-power_x/2+power_hole_ix+power_hole_dx,
             -power_z/2+power_hole_iy+power_hole_dy])
    circle(r=power_hole_r,$fn=power_hole_sm);
  translate([-power_x/2+power_hole_ix,
             -power_z/2+power_hole_iy+power_hole_dy])
    circle(r=power_hole_r,$fn=power_hole_sm);
}
module power_spacers() {
  translate([-power_x/2+power_hole_ix,
             -power_z/2+power_hole_iy,
             0])
    spacer(r=power_standoff_r,R=power_standoff_R,
           h=power_standoff_h,$fn=power_standoff_sm);
  translate([-power_x/2+power_hole_ix+power_hole_dx,
             -power_z/2+power_hole_iy,
             0])
    spacer(r=power_standoff_r,R=power_standoff_R,
           h=power_standoff_h,$fn=power_standoff_sm);
  translate([-power_x/2+power_hole_ix+power_hole_dx,
             -power_z/2+power_hole_iy+power_hole_dy,
             0])
    spacer(r=power_standoff_r,R=power_standoff_R,
           h=power_standoff_h,$fn=power_standoff_sm);
  translate([-power_x/2+power_hole_ix,
             -power_z/2+power_hole_iy+power_hole_dy,
             0])
    spacer(r=power_standoff_r,R=power_standoff_R,
           h=power_standoff_h,$fn=power_standoff_sm);
}
// IMU
module imu_holes() {
  translate([imu_sw/2,-imu_sh/2])
    circle(r=imu_hole_r,$fn=imu_hole_sm);
  translate([-imu_sw/2,-imu_sh/2])
    circle(r=imu_hole_r,$fn=imu_hole_sm);
  translate([imu_sw/2,imu_sh/2])
    circle(r=imu_hole_r,$fn=imu_hole_sm);
  translate([-imu_sw/2,imu_sh/2])
    circle(r=imu_hole_r,$fn=imu_hole_sm);
}
module imu_base() {
  translate([imu_w/2,-imu_h/2,0]) {
    if (use_imu_model) {
      rotate([0,0,90])
        import("External/SparkFun-LSM9DS1-Breakout.stl",convexity=5);
    } else {
      difference() {
        translate([-imu_w,0,0])
          cube([imu_w,imu_h,imu_d]);
        translate([-imu_w/2,imu_h/2,-imu_h/2])
          linear_extrude(2*imu_h)
            imu_holes();
      }
    }
  }
}
module imu_spacers() {
  translate([imu_sw/2,-imu_sh/2,0])
    spacer(r=imu_standoff_r,R=imu_standoff_R,
           h=imu_standoff_h,$fn=imu_standoff_sm);
  translate([-imu_sw/2,-imu_sh/2,0])
    spacer(r=imu_standoff_r,R=imu_standoff_R,
           h=imu_standoff_h,$fn=imu_standoff_sm);
  translate([imu_sw/2,imu_sh/2,0])
    spacer(r=imu_standoff_r,R=imu_standoff_R,
           h=imu_standoff_h,$fn=imu_standoff_sm);
  translate([-imu_sw/2,imu_sh/2,0])
    spacer(r=imu_standoff_r,R=imu_standoff_R,
           h=imu_standoff_h,$fn=imu_standoff_sm);
}
module imu() {
  color([0,0.5,0.2,0.8])
    translate([0,0,imu_standoff_h])
      imu_base();
  imu_spacers();
}
//mount_tab = 10;
mount_tab_w = servo_l2+bracket_h;
mount_tab_he = 2*plate_thickness*sin(dihedral);
mount_tab_h0 = plate_thickness;
mount_tab_h = mount_tab_h0 + mount_tab_he;
mount_tie_offset_z = 3;
mount_tie_offset_y = max(mount_rr_1,mount_rr_2)-mount_R+2;
mount_tie_w = 4;
mount_tie_h = 2;
mount_h0 = tower_cy+motor_oz-base_offset_z-mount_h+mount_Q;
module mount_blank_slice(
  pt=plate_thickness_tol,
  vo=0,
  trim=0,
  axle_r=axle_r
) {
  // basic shape
  difference() {
    union() {
      hull() {
        translate([vo,2*servo_l2+2*bracket_h+plate_thickness])
          circle(r=mount_r,$fn=mount_sm);
        translate([vo,0])
          circle(r=mount_R,$fn=mount_sm);
      }
      translate([mount_h0,0])
        difference() {
          union() {
            translate([trim,-mount_R])
              square([vo-mount_h0-2*trim,mount_tab_w+mount_R-pt]);
            translate([trim+mount_h+mount_h0,-sin(mount_a_2)*mount_R])
              square([-mount_h0-2*trim,mount_tab_w+sin(mount_a_2)*mount_R-pt]);
          }
          // interlocking tabs for top and bottom plates
          translate([0,mount_tab_w/3-mount_R/2])
            square([mount_tab_h,mount_tab_w/3]);
          translate([mount_h-mount_tab_h+eps,mount_tab_w/3])
            square([mount_tab_h,mount_tab_w/3]);
        }
    }
    // avoid sharp corners on wheel mounts
    translate([vo,0]) {
      hull() {
        rotate(-mount_a_2) {
          translate([mount_R+mount_rr_2,0]) {
            circle(r=mount_rr_2,$fn=mount_sm);
            rotate(mount_a_2)
              translate([mount_h,0])
                circle(r=mount_rr_2,$fn=mount_sm);
          }
        }
      }
    }
    // axle hole
    translate([vo,0])
      circle(r=axle_r,$fn=axle_sm);
  }
}
mount_tie_offset_y_2 = mount_tie_offset_y - 7;
inner_mount_t_r_1 = 10;
inner_mount_t_r_2 = 5;
module inner_mount_slice(pt=plate_thickness_tol,vo=inner_mount_vo,trim=1) {
  difference() {
    mount_blank_slice(pt=pt,vo=vo,trim=trim,axle_r=axle_inner_r);
    // remove servo mount area
    translate([-mount_h,mount_tab_w-pt])
      square([2*mount_h,3*servo_ll]);
    // holes for cable ties to secure side connection
    translate([mount_h0,0]) {
      translate([mount_tab_h+mount_tie_offset_z,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
    }
    translate([mount_h0,0]) {
      translate([mount_tab_h+mount_tie_offset_z,
                 mount_tab_w+mount_tie_offset_y_2])
        square([mount_tie_h,mount_tie_w]);
      translate([mount_h-mount_tie_offset_z-mount_tie_h-mount_tab_h,
                 2*mount_tab_w/3+mount_tie_offset_y+mount_tie_w/2])
        square([mount_tie_h,mount_tie_w]);
    }
    // cutouts to avoid interference with T-slot nuts
    translate([mount_h0+mount_tab_h,mount_tab_w])
      circle(r=inner_mount_t_r_1,$fn=mount_sm);
    translate([mount_h0+mount_tab_h-2*inner_mount_t_r_1,
               mount_tab_w-inner_mount_t_r_1])
      square([2*inner_mount_t_r_1,2*inner_mount_t_r_1]);
    
    translate([mount_h+mount_h0-mount_tab_h,mount_tab_w])
      circle(r=inner_mount_t_r_2,$fn=mount_sm);
    translate([mount_h+mount_h0-inner_mount_t_r_2,mount_tab_w-inner_mount_t_r_2])
      square([2*inner_mount_t_r_2,2*inner_mount_t_r_2]);
  }
  // tab to anchor against tower panel
  translate([vo-mount_h/12,mount_tab_w-pt-tol])
    square([mount_h/6,plate_thickness+tol-pt]);

}
module mount_slice(pt=plate_thickness_tol,trim=1) {
  if (use_trans) {
    difference() {
      // basic shape
      mount_blank_slice(pt=pt,trim=trim);
      // servo horn and body
      translate([0,2*servo_l2+2*bracket_h+plate_thickness]) {
          circle(r=servo_h_rr,$fn=mount_sm);
          translate([-servo_ww/2,-servo_ll])
            square([servo_ww,servo_ll]);
      }
      // bracket and tab
      hull() {
        translate([-servo_ww/2,
                   2*servo_l2+2*bracket_h+plate_thickness-servo_ll+tol])
            square([servo_ww,eps]);
        translate([-servo_ww/2-servo_bw,
                   servo_l2+bracket_h-plate_thickness_tol])
            square([servo_ww+2*servo_bw,
                    bracket_hh+plate_thickness+plate_thickness_tol]);
      }
      // mounting holes
      for (i = [0:3]) {
        translate([bracket_hole_spacing+servo_h_h+bracket_tweak_x,
                   servo_l2+2*bracket_h+plate_thickness
                   +servo_h_h+bracket_tweak_y
                   +i*bracket_hole_spacing]) 
          circle(r=m2_hole_radius,$fn=hole_sm);
        translate([-bracket_hole_spacing-servo_h_h-bracket_tweak_x,
                   servo_l2+2*bracket_h+plate_thickness
                   +servo_h_h+bracket_tweak_y
                   +i*bracket_hole_spacing]) 
          circle(r=m2_hole_radius,$fn=hole_sm);
      }
      // holes for cable ties to secure side connection
      translate([mount_h0,0]) {
        translate([mount_tab_h+mount_tie_offset_z,
                   mount_tie_offset_y])
          square([mount_tie_h,mount_tie_w]);
        translate([mount_h-mount_tie_offset_z-mount_tie_h-mount_tab_h,
                   mount_tie_offset_y+(1-sin(mount_a_2))*mount_R])
          square([mount_tie_h,mount_tie_w]);
      }
      translate([mount_h0,0]) {
        translate([mount_tab_h+mount_tie_offset_z,
                   mount_tab_w+mount_tie_offset_y])
          square([mount_tie_h,mount_tie_w]);
        translate([mount_h-mount_tie_offset_z-mount_tie_h-mount_tab_h,
                   mount_tab_w+mount_tie_offset_y])
          square([mount_tie_h,mount_tie_w]);
      }
    }
  }
}
module mount_plate(e=0,pt=plate_thickness_tol,trim=1) {
  linear_extrude(plate_thickness+e)
    mount_slice(pt=pt,trim=trim);
} 
module inner_mount_plate(e=0,pt=plate_thickness_tol,vo=inner_mount_vo,trim=1) {
  linear_extrude(plate_thickness+e)
    inner_mount_slice(pt=pt,vo=vo,trim=trim);
} 
inner_mount_offset = axle_spacer_h_2+plate_thickness;
inner_mount_vo = sin(dihedral)*inner_mount_offset;
module motor_unit(a=90,p=0) {
  rotate([0,90,0])
    translate([0,0,servo_h_h+servo_h]) {
      difference() {
        union() {
          if (use_tire) {
            translate([0,0,
                        wheel_plate_thickness
                       +outer_plate_thickness
                       +middle_plate_thickness/2]) 
              tire();
          }
          //color([0.7,0.6,0.1,0.9])
            wheel();
        }
        // following is for visualizing tire mounts
        if (tire_cutout) {
          translate([0,0,-wheel_radius])
            cube([2*wheel_radius,2*wheel_radius,2*wheel_radius]);
        }
      }
      if (use_trans) {
        translate([0,
                   2*servo_l2+2*bracket_h+plate_thickness,
                   motor_trans_offset_t])         
          rotate([0,0,180]){
            servo(a);
            if (!printed) end_bracket(a);
            driver();
          }
        translate([0,0,(p?plate_thickness:0)+motor_trans_offset_t-servo_h_h]) {
          color([0.6,0.4,0.2,0.9])
            rotate([0,p?180:0,0]) {
               mount_plate(trim=1);
               translate([-inner_mount_vo,0,
                          p?inner_mount_offset:-inner_mount_offset])
                 inner_mount_plate(vo=inner_mount_vo,trim=1);
            }
        }
      } else {
        servo(a);
        if (!printed) end_bracket(a);
      }
   }
}
module motor_base() {
  translate([tower_cx,0,tower_cy+motor_oz])
    rotate([0,-dihedral,0])
      translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
         motor_unit(a=90,p=0);
  translate([-tower_cx,0,tower_cy+motor_oz])
    rotate([0,180+dihedral,0])
      translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
        motor_unit(a=90,p=1);
}
module base_slice() {
  difference() {
    intersection() {
      circle(r=base_r,$fn=base_sm);
      translate([-base_x/2,-base_y/2,0])
        square([base_x,base_y]);
    }
    // mounting holes and suspension slot for back caster
    translate([0,caster_or-caster_oyy])
      rotate([0,0,180]) {
        caster_holes();
        if (use_rear_suspension) 
          caster_suspension(flip=0,cutout=0);
      }
    // mounting holes and suspension slots for front casters (optional)
    if (use_front_casters) {
      translate([0,-caster_oy]) {
        translate([caster_ox,0]) {
          caster_holes();
          if (use_front_suspension) 
            caster_suspension(flip=0,cutout=1);
        }
        translate([-caster_ox,0]) {
          caster_holes();
          if (use_front_suspension)
            caster_suspension(flip=1,cutout=1);
        }
      }
    }
    // spacer holes
    translate([0,-spacer_oy]) {
      translate([spacer_ox,0])
        spacer_hole();
      translate([-spacer_ox,0])
        spacer_hole();
    }
    // holes for cable ties to secure side panel
    if (use_trans) {
      translate([base_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z,
                 mount_tie_offset_y+(1-sin(mount_a_2))*mount_R])
        square([mount_tie_h,mount_tie_w]);
      translate([-base_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z,
                 mount_tie_offset_y+(1-sin(mount_a_2))*mount_R])
        square([mount_tie_h,mount_tie_w]);
      
      translate([base_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z,
                 mount_tab_w+mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([-base_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z,
                 mount_tab_w+mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
    }
    // holes for cable ties to secure inner side panel
    if (use_trans) {
      translate([base_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 2*mount_tab_w/3+mount_tie_offset_y+mount_tie_w/2])
        square([mount_tie_h,mount_tie_w]);
      translate([-base_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 2*mount_tab_w/3+mount_tie_offset_y+mount_tie_w/2])
        square([mount_tie_h,mount_tie_w]);
      
      translate([base_x/2-mount_tie_h/2+2*plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 2*mount_tab_w/3+mount_tie_offset_y+mount_tie_w/2])
        square([mount_tie_h,mount_tie_w]);
      translate([-base_x/2-mount_tie_h/2-2*plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 2*mount_tab_w/3+mount_tie_offset_y+mount_tie_w/2])
        square([mount_tie_h,mount_tie_w]);
    }
    // battery strap cutouts
    hull() {
      translate([battery_strap_ox,battery_strap_oy_1-battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
      translate([battery_strap_ox,battery_strap_oy_1+battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
    }
    hull() {
      translate([-battery_strap_ox,battery_strap_oy_1-battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
      translate([-battery_strap_ox,battery_strap_oy_1+battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
    }
    hull() {
      translate([battery_strap_ox,battery_strap_oy_2-battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
      translate([battery_strap_ox,battery_strap_oy_2+battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
    }
    hull() {
      translate([-battery_strap_ox,battery_strap_oy_2-battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
      translate([-battery_strap_ox,battery_strap_oy_2+battery_strap_h/2])
        circle(r=battery_strap_r,$fn=hole_sm);
    }
    // tower plate slots
    translate([0,tower_offset_y-plate_thickness]) {
      translate([tower_tab_ox-cut_t,-cut_t]) 
        square([tower_tab_w+2*cut_t,
                plate_thickness+plate_thickness_tol+2*cut_t]);
      translate([-tower_tab_ox-tower_tab_w-cut_t,-cut_t]) 
        square([tower_tab_w+2*cut_t,
                plate_thickness+plate_thickness_tol+2*cut_t]);
    }
  }
}
module base_plate() {
  linear_extrude(plate_thickness)
    base_slice();
}
module tower_slice(trans=false) {
  difference() { 
    // base shape
    union() { 
      hull() {
        translate([-camera_x/2,camera_oz+camera_z])
          square([camera_x,0.001]);
        translate([-tower_x/2,tower_oy+plate_thickness])
          square([tower_x,plate_thickness]);
      }
      // base plate tabs
      translate([0,tower_oy]) {
        translate([tower_tab_ox,plate_thickness_tol]) 
          square([tower_tab_w,plate_thickness+0.1]);
        translate([-tower_tab_ox-tower_tab_w,plate_thickness_tol]) 
          square([tower_tab_w,plate_thickness+0.1]);
      }
    }
    // wheel cutouts
    difference() {
      hull() {
        translate([-tower_cx-wheel_radius-clear_pad,
                   tower_cy+motor_oz])
          rotate(-dihedral) 
            translate([-battery_radius-battery_pad-base_pad
                       -servo_b_h-servo_h-servo_h_h
                       -1.5*plate_thickness,
                       wheel_radius])
                circle(r=tower_r,$fn=tower_sm);
        translate([-tower_cx-clear_pad,tower_cy+motor_oz])
          rotate(-dihedral) 
            translate([-battery_radius-battery_pad-base_pad
                       -servo_b_h-servo_h-servo_h_h
                       -1.5*plate_thickness,
                       wheel_radius])
              circle(r=tower_r,$fn=tower_sm);
        translate([-tower_cx-clear_pad,tower_cy+motor_oz])
          rotate(-dihedral) 
            translate([-battery_radius-battery_pad-base_pad
                       -servo_b_h-servo_h-servo_h_h
                       -1.5*plate_thickness,
                       -wheel_radius])
              circle(r=tower_r,$fn=tower_sm);

      }
      // servo bracket support (useful for 3D printing...)
      if (printed) {
        translate([-tower_cx,tower_cy+motor_oz])
          rotate(-dihedral) 
            translate([-battery_radius-battery_pad-base_pad
                       -servo_b_h-clear_pad,
                       0])
              translate([(bracket_base_x-servo_w)/2-bracket_base_x,
                         -bracket_base_y/2])
                square([bracket_base_x,bracket_base_y]);
      }
    }
    difference() {
      hull() {
        translate([tower_cx+wheel_radius+clear_pad,
                   tower_cy+motor_oz])
          rotate(dihedral) 
            translate([battery_radius+battery_pad+base_pad
                       +servo_b_h+servo_h+servo_h_h
                       +1.5*plate_thickness,
                       wheel_radius])
              circle(r=tower_r,$fn=tower_sm);
        translate([tower_cx+clear_pad,tower_cy+motor_oz])
          rotate(dihedral) 
            translate([battery_radius+battery_pad+base_pad
                       +servo_b_h+servo_h+servo_h_h
                       +1.5*plate_thickness,
                       wheel_radius])
            circle(r=tower_r,$fn=tower_sm);
        translate([tower_cx+clear_pad,tower_cy+motor_oz])
          rotate(dihedral) 
            translate([battery_radius+battery_pad+base_pad
                       +servo_b_h+servo_h+servo_h_h
                       +1.5*plate_thickness,
                       -wheel_radius])
              circle(r=tower_r,$fn=tower_sm);
      }
      // servo bracket support (only if 3D printing...)
      if (printed) {
        translate([tower_cx,tower_cy+motor_oz])
          rotate(dihedral) 
            translate([battery_radius+battery_pad+base_pad
                       +servo_b_h+clear_pad,
                       0])
              translate([-(bracket_base_x-servo_w)/2,
                         -bracket_base_y/2])
                square([bracket_base_x,bracket_base_y]);
      }
    }
    // battery cutout
    hull() {
      translate([-battery_radius,-battery_radius])
        circle(r=battery_radius+battery_pad,$fn=tower_sm);
      translate([battery_radius,-battery_radius])
        circle(r=battery_radius+battery_pad,$fn=tower_sm);
      translate([-battery_radius,-5*battery_radius])
        circle(r=battery_radius+battery_pad,$fn=tower_sm);
      translate([battery_radius,-5*battery_radius])
        circle(r=battery_radius+battery_pad,$fn=tower_sm);
    }
    // extra cutout for extra-tall LiPo packs
    hull() {
      translate([-battery_x/2,tower_oy+battery_z+plate_thickness+battery_pad])
        circle(r=2,$fn=tower_sm);
      translate([battery_x/2,tower_oy+battery_z+plate_thickness+battery_pad])
        circle(r=2,$fn=tower_sm);
      translate([-battery_x/2,tower_oy])
        circle(r=2,$fn=tower_sm);
      translate([battery_x/2,tower_oy])
        circle(r=2,$fn=tower_sm);
    }
    // cable-management cutouts
    if (use_cable_holes) {
      hull() {
        translate([tower_cable_ox,tower_cable_oz-tower_cable_h/2])
          circle(r=tower_cable_r,$fn=cable_hole_sm);
        translate([tower_cable_ox,tower_cable_oz+tower_cable_h/2])
          circle(r=tower_cable_r,$fn=cable_hole_sm);
      }
      hull() {
        translate([-tower_cable_ox,tower_cable_oz-tower_cable_h/2])
          circle(r=tower_cable_r,$fn=cable_hole_sm);
        translate([-tower_cable_ox,tower_cable_oz+tower_cable_h/2])
          circle(r=tower_cable_r,$fn=cable_hole_sm);
      }
      // slots for ties for cable cutouts
      hull() {
        translate([tower_cable_ox+tower_cable_r+3*m3_hole_radius,
                   tower_cable_oz-tower_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
        translate([tower_cable_ox+tower_cable_r+3*m3_hole_radius,
                   tower_cable_oz+tower_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
      }
      hull() {
        translate([-tower_cable_ox-tower_cable_r-3*m3_hole_radius,
                   tower_cable_oz-tower_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
        translate([-tower_cable_ox-tower_cable_r-3*m3_hole_radius,
                   tower_cable_oz+tower_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
      }
    }
    // mounting holes for servo brackets
    if (!printed) {
      translate([tower_cx,tower_cy+motor_oz])
        rotate(dihedral) 
          translate([battery_radius+battery_pad+base_pad
                     +bracket_hole_spacing
                     +servo_b_h+clear_pad
                     +motor_trans_offset_t,
                     -bracket_hole_spacing])
            bracket_holes(trans=trans);
      translate([-tower_cx,tower_cy+motor_oz])
        rotate(-dihedral) 
          translate([-battery_radius-battery_pad-base_pad
                     -3*bracket_hole_spacing
                     -servo_b_h-clear_pad
                     -motor_trans_offset_t,
                     -bracket_hole_spacing])
            bracket_holes(trans=trans);
    }
    // mounting holes for up board
    if (use_up_holes) {
      translate([up_board_ox,up_board_oz])
        rotate(90)
          up_board_holes();
    }
    // mounting holes for tc board
    if (use_tc_holes) {
      translate([tc_board_ox,tc_board_oz])
        rotate(180)
          tc_board_holes(flip=true);
    }
    // mounting holes for power board
    if (use_power_holes) {
      translate([power_ox,power_oz])
        power_holes();
    }
    // mounting holes for camera
    translate([0,camera_oz])
      camera_holes();
    // top plate slots
    translate([-cut_t,top_offset_z-cut_t]) {
      translate([top_tab_ox,0]) 
        square([top_tab_w+2*cut_t,
                plate_thickness+plate_thickness_tol+2*cut_t]);
      translate([-top_tab_w/2,0]) 
        square([top_tab_w+2*cut_t,
                plate_thickness+plate_thickness_tol+2*cut_t]);
      translate([-top_tab_ox-top_tab_w,0]) 
        square([top_tab_w+2*cut_t,
                plate_thickness+plate_thickness_tol+2*cut_t]);
    }
    // base plate T-slots
    translate([tower_tab_w/2,tower_oy-eps]) {
      translate([tower_tab_ox,plate_thickness_tol]) 
          T_slot();
      translate([-tower_tab_ox-tower_tab_w,plate_thickness_tol]) 
          T_slot();
    }
  }
}
module tower_plate() {
  linear_extrude(plate_thickness)
    tower_slice();
  if (printed) {
    // integral servo brackets
    translate([tower_cx,tower_cy+motor_oz,plate_thickness-eps])
      rotate([0,0,dihedral]) 
        translate([battery_radius+battery_pad+base_pad
                   +servo_b_h+clear_pad
                   +servo_w/2,
                   0])
          rotate([0,90,0])
            rotate([0,0,90])
            bracket();
    translate([-tower_cx,tower_cy+motor_oz,plate_thickness-eps])
      rotate([0,0,-dihedral]) 
        translate([-battery_radius-battery_pad-base_pad
                   -servo_b_h-clear_pad
                   -servo_w/2,
                   0])
          rotate([0,90,0])
            rotate([0,0,90])
              bracket();
  }
}
module top_slice() {
  difference() {
    union() {
      intersection() {
        circle(r=top_r,$fn=top_sm);
        translate([-top_x/2,-top_y+top_offset_y])
          square([top_x,top_y-plate_thickness]);
      }
      // tower plate tabs
      translate([0,top_offset_y-plate_thickness-plate_thickness_tol]) {
        translate([top_tab_ox,0]) 
          square([top_tab_w,plate_thickness+0.1]);
        translate([-top_tab_w/2,0]) 
          square([top_tab_w,plate_thickness+0.1]);
        translate([-top_tab_ox-top_tab_w,0]) 
          square([top_tab_w,plate_thickness+0.1]);
      }
      // power switch plates
      translate([0,-switch_oy]) {
        translate([switch_ox,0])
          switch_hole(t=switch_t);
        translate([-switch_ox,0])
          switch_hole(t=switch_t);
    }
    }
    // holes for cable ties to secure side panels
    if (use_trans) {
      translate([top_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([top_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z,
                 mount_tab_w+mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z,
                 mount_tab_w+mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
    }
    // holes for cable ties to secure inner side panels
    if (use_trans) {
      translate([top_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      
      translate([top_x/2-mount_tie_h/2+2*plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2-2*plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 mount_tie_offset_y])
        square([mount_tie_h,mount_tie_w]);
      
      translate([top_x/2-mount_tie_h/2+2*plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 mount_tab_w+mount_tie_offset_y_2])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2-2*plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 mount_tab_w+mount_tie_offset_y_2])
        square([mount_tie_h,mount_tie_w]);
        
      translate([top_x/2-mount_tie_h/2-plate_thickness-mount_tie_offset_z
                 -inner_mount_offset,
                 mount_tab_w+mount_tie_offset_y_2])
        square([mount_tie_h,mount_tie_w]);
      translate([-top_x/2-mount_tie_h/2+plate_thickness+mount_tie_offset_z
                 +inner_mount_offset,
                 mount_tab_w+mount_tie_offset_y_2])
        square([mount_tie_h,mount_tie_w]);
    }
    // spacer holes
    translate([0,-spacer_oy]) {
      translate([spacer_ox,0])
        spacer_hole();
      translate([-spacer_ox,0])
        spacer_hole();
    }
    // power switch holes
      translate([0,-switch_oy]) {
        translate([switch_ox,0])
          switch_hole();
        translate([-switch_ox,0])
          switch_hole();
    }
    // cable-management cutouts
    if (use_cable_holes) {
      hull() {
        translate([top_cable_ox,top_cable_oy-top_cable_h/2])
          circle(r=top_cable_r,$fn=cable_hole_sm);
        translate([top_cable_ox,top_cable_oy+top_cable_h/2])
          circle(r=top_cable_r,$fn=cable_hole_sm);
      }
      hull() {
        translate([-top_cable_ox,top_cable_oy-top_cable_h/2])
          circle(r=top_cable_r,$fn=cable_hole_sm);
        translate([-top_cable_ox,top_cable_oy+top_cable_h/2])
          circle(r=top_cable_r,$fn=cable_hole_sm);
      }
      // slots for ties for cable cutouts
      hull() {
        translate([top_cable_ox-top_cable_r-3*m3_hole_radius,
                   top_cable_oy-top_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
        translate([top_cable_ox-top_cable_r-3*m3_hole_radius,
                   top_cable_oy+top_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
      }
      hull() {
        translate([-top_cable_ox+top_cable_r+3*m3_hole_radius,
                   top_cable_oy-top_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
        translate([-top_cable_ox+top_cable_r+3*m3_hole_radius,
                   top_cable_oy+top_cable_h/2])
          circle(r=m3_hole_radius,$fn=cable_hole_sm);
      }
    }
    // tower plate T-slots
    translate([top_tab_w/2,top_offset_y]) {
      translate([top_tab_ox,0]) 
        rotate([0,0,180]) 
          T_slot();
      translate([-top_tab_ox-top_tab_w,0]) 
        rotate([0,0,180]) 
          T_slot();
    }
    // IMU mounting holes
    if (use_imu_holes) {
      imu_holes();
    }
  }
}
module top_plate() {
  color([0.7,0.7,0.3,0.9])
    linear_extrude(plate_thickness)
      top_slice();
}
module mount_tower_slots_slice() {
  projection() rotate([-90,0,0]) {
    intersection() {
      translate([0,tower_offset_y,0])
        rotate([90,0,0])
          tower_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  mount_plate(e=10);
    }
    intersection() {
      translate([0,tower_offset_y,0])
        rotate([90,0,0])
          tower_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([0,0,-10-plate_thickness])
                      mount_plate(e=10);
    }
    intersection() {
      translate([0,tower_offset_y,0])
        rotate([90,0,0])
          tower_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  translate([-inner_mount_vo+tol,0,
                             -inner_mount_offset-plate_thickness_tol])
                    inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
    intersection() {
      translate([0,tower_offset_y,0])
        rotate([90,0,0])
          tower_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([-inner_mount_vo+tol,0,
                               inner_mount_offset-plate_thickness])
                      inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
  }
}
module tower_slice_trans() {
  difference() {
    tower_slice(trans=true);
    offset(eps) mount_tower_slots_slice();
  }
}
module tower_plate_trans() {
  linear_extrude(plate_thickness)
    tower_slice_trans();
}
module mount_top_slots_slice() {
  projection() {
    intersection() {
      translate([0,0,top_offset_z])
        top_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  mount_plate(e=10,pt=-tol);
    }
    intersection() {
      translate([0,0,top_offset_z])
        top_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,plate_thickness+servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([0,0,-10])
                      mount_plate(e=10,pt=-tol);
    }
    intersection() {
      translate([0,0,top_offset_z])
        top_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  translate([-inner_mount_vo+tol,0,
                             -inner_mount_offset-plate_thickness_tol])
                    inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
    intersection() {
      translate([0,0,top_offset_z])
        top_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,plate_thickness+servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([-inner_mount_vo+tol,0,
                               inner_mount_offset])
                      inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
  }
}
module top_slice_trans() {
  difference() {
    top_slice();
    offset(eps) mount_top_slots_slice();
  }
}
module top_plate_trans() {
  linear_extrude(plate_thickness)
    top_slice_trans();
}
module mount_base_slots_slice() {
  projection() {
    intersection() {
      translate([0,0,base_offset_z])
        base_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  mount_plate(e=10,pt=-tol);
    }
    intersection() {
      translate([0,0,base_offset_z])
        base_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([0,0,-10-plate_thickness])
                      mount_plate(e=10,pt=-tol);
    }
    intersection() {
      translate([0,0,base_offset_z])
        base_plate();
      translate([tower_cx,0,tower_cy+motor_oz])
        rotate([0,-dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                color([0.6,0.4,0.2,0.9])
                  translate([-inner_mount_vo-1.5*tol,0,
                             -inner_mount_offset-plate_thickness_tol])
                    inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
    intersection() {
      translate([0,0,base_offset_z])
        base_plate();
      translate([-tower_cx,0,tower_cy+motor_oz])
        rotate([0,180+dihedral,0])
          translate([battery_radius+battery_pad+base_pad+servo_b_h+clear_pad,0,0])
            rotate([0,90,0])
              translate([0,0,servo_h+motor_trans_offset_t]) 
                rotate([0,180,0])
                  color([0.6,0.4,0.2,0.9])
                    translate([-inner_mount_vo-1.5*tol,0,
                               inner_mount_offset-plate_thickness])
                      inner_mount_plate(e=plate_thickness_tol,vo=inner_mount_vo);
    }
  }
}
module base_slice_trans() {
  difference() {
    base_slice();
    offset(eps) mount_base_slots_slice();
  }
}
module base_plate_trans() {
  linear_extrude(plate_thickness)
    base_slice_trans();
}
module assembly() {
  // power regulator
  if (show_power) {
    translate([power_ox,power_oy,power_oz])
      rotate([0,0,180]) {
        power();
        rotate([90,0,0]) 
          power_spacers();
      }
  }
  // 3d camera
  translate([camera_ox,camera_oy-camera_y,camera_oz])
    camera();
  // up board
  if (show_up_board) {
    translate([up_board_ox,up_board_oy,up_board_oz])
      rotate([0,0,180])
        rotate([90,0,0])
          rotate([0,0,90]) {
            up_board();
            up_board_spacers();
          } 
  } 
  // tc board and fan plate
  if (show_tc_board) {
    translate([tc_board_ox,tc_board_oy,tc_board_oz])
      rotate([0,0,180])
        rotate([90,0,0])
          rotate([0,0,180]) {
            color([0.2,0.2,0.2,0.4]) tc_board();
            tc_board_spacers();
          } 
    translate([tc_fan_ox,tc_fan_oy,tc_fan_oz])
      rotate([0,0,180])
        rotate([90,0,0])
          rotate([0,0,180]) {
            tc_fan_plate();
            tc_fan_spacers();
          } 
  } 
  // gumstix board
  if (show_gum_board) {
    translate([gum_board_ox,gum_board_oy,gum_board_oz])
      rotate([0,0,180])
        rotate([90,0,0])
          rotate([0,0,90])
            gum_board();
    translate([up_board_ox,up_board_oy,up_board_oz])
      rotate([0,0,180])
        rotate([90,0,0])
          rotate([0,0,90])
            up_board_spacers();
  } 
  // base plate   
  color([0.5,0.5,0.3,0.6])
  translate([0,0,base_offset_z])
    if (use_trans) {
      base_plate_trans();
    } else {
      base_plate();
    }
  // top plate
  translate([0,0,top_offset_z])
    if (use_trans) {
      top_plate_trans();
    } else {
      top_plate();
    }
  // imu
  translate([0,0,top_offset_z])
    rotate([0,180,0])
      imu();
  // motors and wheels
  motor_base();
  // battery
  translate([battery_offset_x,battery_offset_y,battery_offset_z]) 
    battery();
  // tower plate
  color([0.6,0.6,0.3,0.6])
  translate([0,tower_offset_y,0])
    rotate([90,0,0])
      if (use_trans) {
        tower_plate_trans();
      } else {
        tower_plate();
      }
  // back caster
  translate([0,caster_or-caster_oyy,base_offset_z]) {
    caster();
    translate([0,0,-1]) rotate([0,180,60]) caster_bolts();
    translate([0,0,plate_thickness+tol]) rotate([0,0,60]) caster_nuts();
  }
  // front casters (optional)
  if (use_front_casters && show_front_casters) {
    translate([0,-caster_oy,base_offset_z]) {
      translate([caster_ox,0,0]) {
        caster();
        translate([0,0,-1]) rotate([0,180,0]) caster_bolts();
        translate([0,0,plate_thickness+tol]) caster_nuts();
      }
      translate([-caster_ox,0,0]) {
        caster();
        translate([0,0,-1]) rotate([0,180,0]) caster_bolts();
        translate([0,0,plate_thickness+tol]) caster_nuts();
      }
    }
  }
  // spacers and bolts
  translate([0,-spacer_oy,base_offset_z+plate_thickness]) {
    translate([spacer_ox,0,0]) {
      spacer();
      translate([0,0,spacer_h+plate_thickness])
        bolt(size=3,length=10);
      translate([0,0,-plate_thickness])
        rotate([0,180,0])
          bolt(size=3,length=10);
    }
    translate([-spacer_ox,0,0]) {
      spacer();
      translate([0,0,spacer_h+plate_thickness])
        bolt(size=3,length=10);
      translate([0,0,-plate_thickness])
        rotate([0,180,0])
          bolt(size=3,length=10);
    }
  }
  // speaker (optional)
  if (use_speaker) {
    translate([0,speaker_oy,top_offset_z+plate_thickness])
      speaker();
  }
  // arm motors
  if (use_arm) {
    translate([arm_servo_offset_x,arm_servo_offset_y,arm_servo_offset_z]) {
      rotate([90,180,0]) {
        servo(90);
        if (!printed) end_bracket(90);
      }
    }
    translate([arm_servo_offset_x+arm_servo_offset_dx,
               arm_servo_offset_y-arm_servo_offset_dy,
               arm_servo_offset_z]) {
      rotate([90,180,0]) {
        servo(90);
        if (!printed) end_bracket(90);
      }
    }
    translate([arm_servo_offset_x-arm_servo_offset_dx,
               arm_servo_offset_y-arm_servo_offset_dy,
               arm_servo_offset_z]) {
      rotate([90,180,0]) {
        servo(90);
        if (!printed) end_bracket(90);
      }
    }
  }
}

// PARTS TO LASER CUT
// Uncomment each of the following, render using CGAL (F6), export as DXF,
// read into inkscape, select and merge control points, place on sheet
// for cutting, then follow laser-cutter workflow (exporting as PDF 
// recommended, units do not seem to be preserved when exporting SVG,
// at least when using AI for cutting)

// Recommended practice: cut interior lines holes first, then perimeter
// of parts.   This can usually be accomplished by coloring the lines
// differently.  If you cut the outside first, the part will be loose and
// may shift, and then the interior holes will not be cut accurately.
// Unfortunately the coloring conventions vary by laser cutter, so
// consult with the cutter manual or the service provider.

//inner_wheel_slice();  // x2
//middle_wheel_slice(); // x2
//outer_wheel_slice();  // x2
//rim_wheel_slice();    // x2 is using transmission
//base_slice();         // x1 if not using transmission
//base_slice_trans();   // x1 if using transmission
//tower_slice();        // x1 if not using transmission
//tower_slice_trans();  // x1 if using transmission
//top_slice();          // x1 if not using transmission
//top_slice_trans();    // x1 if using transmission
//mount_slice(trim=1);        // x2 if using transmission
//inner_mount_slice(trim=1);  // x2 if using transmission
//driver_slice();       // x4
//driver_mount_slice(); // x4
//driver_spacer_slice();// x2
//driver_rim_slice();   // x4
//tc_fan_slice();       // x1 if using TC

// Alternative laser-cut parts
// half and quarter wheel slices... makes layout of sheets easier
// Can't use if using transmission, unfortunately

//half_middle_wheel_slice(); // x4, instead of 2x middle_wheel_slice
//half_outer_wheel_slice(); // x4, instead of 2x outer_wheel_slice
// OR
//quarter_middle_wheel_slice(); // x8, instead of 2x middle_wheel_slice
//quarter_outer_wheel_slice(); // x8, instead of 2x outer_wheel_slice

// PARTS TO 3D PRINT
// These should be generated with "printing = 1" at top of file
// Note that parts have not yet been fully optimized for 3D printing,
// but they should work.   Probably.

// Render with CGAL (F6), then export as STL, then run through
// 3D printer slicer
 
//innermiddle_wheel_plate(); // x2
//outer_wheel_plate();  // x2
//base_plate();         // x1
//tower_plate();        // x1
//top_plate();          // x1
//mount_plate();        // x2
//driver_plate();       // x4
//driver_spacer_plate();  // x2
//driver_mount_plate();
//driver_rim_plate();   // x4
//tc_fan_plate();

// ASSEMBLY VISUALIZATION
// Note: full CGAL compile will fail because UP board is not a 
// solid model; use just "fast" visualization
translate([0,0,-base_offset_z+caster_H]) assembly();

// INTERSECTION TESTING
//mount_tower_slots_slice();
//tower_slice_trans();
//tower_plate_trans();
//mount_top_slots_slice();
//top_slice_trans();
//top_plate_trans();
//mount_base_slots_slice();
//base_slice_trans();
//base_plate_trans();
//mount_blank_slice();
//inner_mount_slice();

// TESTING AND SUBASSEMBLY VISUALIZATION
// Uncomment these one by one to look at a single part or subassembly
//imu();
//shoulder_bolt();
//wheel();
//driver();
//torus();
//tire();
//servo();
//end_bracket();
//side_bracket();
//up_board();
//bracket_holes();
//up_board_holes();
//caster();
//translate([0,0,-1]) rotate([0,180,0]) caster_bolts();
//translate([0,0,plate_thickness]) caster_nuts();
//spacer();
//rotate([180,0,0]) locknut();
//translate([0,0,3]) bolt();
//power();
//rotate([90,0,0]) power_spacers();
//translate([-servo_h-motor_trans_offset_t,0,0]) motor_unit(a=90,p=0);
//translate([servo_h+motor_trans_offset_t,0,0]) rotate([0,180,0]) motor_unit(a=90,p=1);
//motor_base();
//base_plate();
//bracket();
//driver_slice();
//driver_plate();
//driver_spacer_slice();
//driver_spacer_plate();
//bearing();
//tc_board();

// REPORT
echo("WHEEL DIAMETER:");
echo(tire_od);
echo("MECHANICAL ADVANTAGE:");
if (use_friction) {
  echo(tire_od/friction_r);
} else {
  echo((wheel_radius + tire_e)/(driver_radius + tire_e));
}
