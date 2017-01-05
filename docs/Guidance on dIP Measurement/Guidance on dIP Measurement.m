clear all;
coordinate1 = zeros(50,4); %generate a zero matrix of n*4, the first line is the cameraDist, the next three lines are the coordinate of xyz
shift_length = 200; %set the start of cameraDist parameter
for screen_d = (1+shift_length):1:240;  %set the end of cameraDist, determine the length of coordinate1. this is manually controlled

    screen_l = 318;
    screen_w = 198;  % the parameter of camera, the size of imaging plane

    xli = 126;
    yli = 102;  %in the following note, we need to note these data
    xri = 185;  %note the size of imaging plane
    yri = 99;   %note the distance of LR and MO
    xmi = 156;  %note a set of data
    ymi = 146;  %note the actual position of this experiment

    P = [screen_l/2, screen_w/2, screen_d];
    Ri = [xri, yri, 0];
    Li = [xli, yli, 0];
    Mi = [xmi, ymi, 0]; % the coordinate of P Ri Li Mi

    P_Ri = Ri - P;
    P_Li = Li - P;
    P_Mi = Mi - P;
    vector_n = cross(P_Ri, P_Li); % calculate the normal vector of plane P_Ri_Li

    za = (vector_n(1)*vector_n(3)*(screen_l/2-xmi) + vector_n(2)*vector_n(3)*(screen_w/2-ymi) + vector_n(3)^2*screen_d)/(vector_n(1)^2 + vector_n(2)^2 + vector_n(3)^2);
    xa = vector_n(1)/vector_n(3)*za + xmi;
    ya = vector_n(2)/vector_n(3)*za + ymi;
    P_A = [xa-screen_l/2, ya-screen_w/2, za-screen_d]; % point A is in plane P_Ri_Li, and line A_Mi is vertival to plane P_Ri_Li

    cosa = dot(P_Ri,P_Li)/(norm(P_Ri)*norm(P_Li));
    angle_a = acos(cosa);
    cosb = dot(P_Ri,P_A)/(norm(P_Ri)*norm(P_A));
    angle_b = acos(cosb);
    sinr = dot(P_Mi,vector_n)/(norm(P_Mi)*norm(vector_n));
    angle_r = asin(sinr);   % calculate the three angles

    d_LR = 50;      %the distance between L and R
    d_MO = 36;      %the distance between M and origin of coordinate

    yR = d_LR/2;
    xC = yR/tan(angle_a);
    r = yR/sin(angle_a);    %the radium of the first round
    xB = xC - r*cos(2*angle_b-angle_a);
    yB = -r*sin(2*angle_b-angle_a);

    B2 = xB*xB + yB*yB;     %define three parameter to help calculate
    n = tan(angle_r)*tan(angle_r);  %they are not useful in later steps
    v = -2*xC*xB + B2;

    c0 = -(d_MO)^2*B2 +n*v^2;
    c1 = (d_MO)^2*xB*2 + 4*n*xC*v;
    c2 = B2 - (d_MO)^2 -2*n*v +4*n*xC^2;
    c3 = -(4*n*xC + 2*xB);
    c4 = n + 1; 

    p1 = [c4, c3, c2, c1, c0];  %calculate the position of M
    xM = roots(p1); %these formulas are based on my own calculation, the formulas the essay provides is not correct

    max = -d_MO;    %to choose the biggest root, the biggest root means the correct root
    for i=1:1:4
        if (imag(xM(i)) == 0) && (xM(i)>max)
            max = xM(i);
        end
    end

    %use a straight line and a circle intersection, calculate P
    k_BM = (-yB)/(max-xB);
    d0 = xC^2 + (k_BM*max)^2 - r^2;
    d1 = -2*(xC + k_BM^2*max);
    d2 = k_BM^2 + 1;
    p2 = [d2, d1, d0];  %get the two P point
    xP_temp = roots(p2);
    if (xP_temp(2)-xB) < (1e-5) %one is B point, the other is P point
        xP = xP_temp(1);        %remove the solutions of B
    else
        xP = xP_temp(2);
    end
    yP = k_BM * ( xP-max );     %get point P in the middle coordinate system

%turn an angle, get the position of P in actual coordinate system
    angle_Coordinate = asin(max/d_MO);
    coordinate1(screen_d - shift_length,1) = screen_d;
    coordinate1(screen_d - shift_length,4) = xP * cos(angle_Coordinate);
    coordinate1(screen_d - shift_length,2) = xP * sin(angle_Coordinate);
    coordinate1(screen_d - shift_length,3) = yP;
end