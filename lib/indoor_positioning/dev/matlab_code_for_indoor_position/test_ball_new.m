clear all;

angle_a = 40*pi/180;    %define the three angle
angle_b = 25*pi/180;
angle_r = 30*pi/180;
d_LR = 20;      %the distance between L and R
d_MO = 15;      %the distance between M and origin of coordinate
 
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
xM = roots(p1);

i=0;
%use a straight line and a circle intersection, calculate P
for i=1:1:4
    k_BM(i) = (-yB)/(xM(i)-xB);
    d0 = xC^2 + (k_BM(i)*xM(i))^2 - r^2;
    d1 = -2*(xC + k_BM(i)^2*xM(i));
    d2 = k_BM(i)^2 + 1;
    p2 = [d2, d1, d0];  %get the two P point
    xP_temp = roots(p2);
    if (xP_temp(2)-xB) < (1e-5) %one is B point, the other is P point
        xP(i) = xP_temp(1);     %remove the solutions of B
    else
        xP(i) = xP_temp(2);
    end
    yP(i) = k_BM(i) * ( xP(i)-xM(i) );
end

%turn an angle, get the position of P in actual coordinate system
angle_Coordinate = arcsin(xM/d_MO);
xP_actual = -xP * cos(angle_Coordinate);
zP_actual = -xP * sin(angle_Coordinate);
yP_actual = yP;
