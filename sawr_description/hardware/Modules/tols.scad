
printed = 0;

// TOLERANCES 
tol = 0.1;
eps = 0.0001;

// tolerance around cuts; lasers remove a tiny slice (making holes
// very slightly larger) but 3D printers *add* material (making holes
// smaller... and typically with more "spread").
laser_cut_t = -0.05;  // typical 0.1mm cut width
printed_cut_t = 0.1;  // depends on printer; might need to be up to 0.3

// generic value depends on setting of "printed" variable
cut_t = printed ? printed_cut_t : laser_cut_t; 