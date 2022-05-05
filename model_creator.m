clear
clc
% Thrust Calculations

prop_l = convlength(9, "in", "m");
prop_p = convlength(4.3, "in", "m");

T_coeff = 0.35*1.225*((pi*prop_l^2)/(4)) * (prop_p^2/3600);

% state variables at steady state
pitch = 0.3;
yaw = 0;
pitch_rate = 0;
yaw_rate = 0;

% Model constant values
g = 9.81;
A_mod = 0.07443187;
B_mod = 0.064735;
kvc = 0.0324319;
kvf = 0.01;
Jv = 0.03409233;
l_m = 0.2675;
l_t = 0.2675;
Jh = 0.03409233*cos(pitch)*cos(pitch);
khf = 0.04;

beta_t = (T_coeff)/112;
beta_m = (T_coeff)/112;

Wm = sqrt( (g*cos(pitch)*(A_mod - B_mod))/(T_coeff*l_m - (beta_t*beta_m)/(T_coeff*l_t*cos(pitch))) );
Wt = sqrt( (beta_m*Wm^2)/(T_coeff*l_t*cos(pitch)) );

Fm_Wm = T_coeff*Wm^2;
Ft_Wt = T_coeff*Wt^2;








A = [0 0 1 0;
    0 0 0 1;
    (-g*sin(pitch)*(A_mod-B_mod) - (kvc*yaw_rate^2*cos(2*pitch) ) )/(Jv) 0 -(kvf/Jv) 0;
    -(Ft_Wt*l_t*sin(pitch))/(Jh) 0 0 -(khf/Jh)];


B = [0 0;
    0 0;
    2*l_m*(T_coeff*Wm) -2*beta_t*Wt;
    -2*beta_m*Wm 2*l_t*(T_coeff*Wt)];

C = [1 0 0 0; 0 1 0 0];

D = zeros(2);

sys_ct = ss(A,B,C,D)

sys_dt = c2d(sys_ct, 0.01)

save("discrete.mat", "sys_dt")