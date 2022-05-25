clc
close all
clear all

DD = false;
TRI = false;
OMNI = true;

%% DD
if(DD)

    syms theta r L w_l w_r X TH omega real

    % Forward
    trans = [cos(theta) -sin(theta) 0
        sin(theta) cos(theta) 0
        0 0 1];

    M = [r/2 r/2
        0 0 
        -r/L r/L];

    vels = trans * M * [w_l w_r]';

    % Inverse
    V1 = subs(V,theta, omega*t);
    P = int(V1,t);
    
    P1 = subs(P,omega,theta/t);

    S = solve([X TH]' == P1([1 3]), [w_r, w_l], 'ReturnConditions', true);
    wr = simplify(S.w_r)
    wl = simplify(S.w_l)

end

%% Tricycle
if(TRI)


end

%% Omni
if(OMNI)

    syms theta r L w1 w2 w3 real

    % Forward
    trans = [cos(theta) -sin(theta) 0
        sin(theta) cos(theta) 0
        0 0 1];

    M_inv = (1/r) * [0 -1 L
        sqrt(3)/2 1/2 L
        sqrt(3)/2 -1/2 -L];

    M = simplify(inv(M_inv));

    vels = trans * (M * [w1 w2 w3]');

    % Inverse
    syms omega t real

    vels1 = subs(vels, theta, omega*t);
    pos = int(vels1, t)

%     S = solve([X Y TH]' == pos, [w1 w2 w3], 'ReturnConditions', true);
%     w1 = simplify(S.w1)
%     w2 = simplify(S.w2)
%     w3 = simplify(S.w3)

end
