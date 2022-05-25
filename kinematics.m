clc
close all

DD = false;
TRI = false;
OMNI = true;

%% DD
if(DD)

    syms theta r L w_l w_r X TH real

    % Forward
    trans = [cos(theta) -sin(theta) 0
        sin(theta) cos(theta) 0
        0 0 1];

    M = [r/2 r/2
        0 0 
        -r/L r/L];

    V = trans * M * [w_l w_r]';

    % Inverse
    V1 = subs(V,theta, omega*t);
    P = int(V1,t);
    
    P1 = subs(P,omega,theta/t);

    solve([X TH]' == P1([1 3]), [w_r, w_l], 'ReturnConditions', true)


end

%% Tricycle
if(TRI)


end

%% Omni
if(OMNI)

    syms x_dot y_dot theta_dot w1 w2 w3 R L X Y TH t real

    M_inv = (1/R) * [0 -1 L
        sqrt(3)/2 1/2 L
        sqrt(3)/2 -1/2 -L];

    M = simplify(inv(M_inv));

    vels = M * [w1 w2 w3]';

    pos = int(vels, t);

    S = solve([X Y TH]' == pos, [w1 w2 w3], 'ReturnConditions', true);

end
