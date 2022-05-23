clc
close all

%% DD
syms r L th wr wl t x y real

M = [r/2 r/2
    0 0 
    -r/L r/L];

wheels = [wl wr]';

trans = [cos(th*t) -sin(th*t) 0
    sin(th*t) cos(th*t) 0
    0 0 1];

vels_dd = trans*M*wheels;
int_dd = int(vels_dd, 't');

S = solve([x z]' == int_dd([1 3]), [wl wr]);
simplify(S.wl)
simplify(S.wr)

clear r L th wr wl

%% Tricycle
syms ws r alpha th L real

M = [r*cos(alpha)
    0
    r/L*sin(alpha)];

wheels = ws;

trans = [cos(th) -sin(th) 0
    sin(th) cos(th) 0
    0 0 1];

vels_tri = trans*M*wheels
int_tri = int(vels_tri, 't')

solve([x z]' == int_tri([1 3]), [ws, alpha])

clear ws r alpha th L

%% Omni
syms r L th w1 w2 w3 real

M = [0 r/sqrt(3) -r/sqrt(3)
    -2*r/3 r/3 r/3
    r/(3*L) r/(3*L) r/(3*L)];

wheels = [w1 w2 w3]';

trans = [cos(th) -sin(th) 0
    sin(th) cos(th) 0
    0 0 1];

vels_omni = trans*M*wheels
int_omni = int(vels_omni, 't')

S = solve([x y z]' == int_omni,  [w1 w2 w3]);
simplify(S.w1)
simplify(S.w2)
simplify(S.w3)

clear r L th w1 w2 w3
