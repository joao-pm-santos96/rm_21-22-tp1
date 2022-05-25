clc
close all

DD = true;
TRI = false;
OMNI = false;

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

    syms theta w_t r_t alpha L t omega X TH real

    % Forward
    trans = [cos(theta) -sin(theta) 0
        sin(theta) cos(theta) 0
        0 0 1];

    M = [w_t * r_t
        0
        w_t * r_t * tan(alpha)/L];

    V = trans * M;

    % Inverse
    V1 = subs(V,theta, omega*t);
    P = int(V1,t);
    
    %P1 = subs(P,omega,w_t*r_t*tan(alpha)/L);
    P1 = subs(P,omega,theta/t);

    solve([X TH]' == P1([1 3]), [w_t, alpha], 'ReturnConditions', true)

end

%% Omni
if(OMNI)


end
