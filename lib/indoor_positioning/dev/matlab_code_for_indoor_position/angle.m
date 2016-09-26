clear all;

l = 1024
w = 768
d = 500  % camera parameters l w d
xr = 500
yr = 600
xl = 800
yl = 400
xm = 600
ym = 200 % Ri Li Mi coordinates

P = [l/2,w/2,d]
Ri = [xr,yr,0]
Li = [xl,yl,0]
Mi = [xm,ym,0] % P Ri Li Mi coordinates

P_Ri = P-Ri
P_Li = P-Li
P_Mi = P-Mi
n = cross(P_Ri,P_Li) % P_Ri_Li plane normal vector

za = n(1)*n(3)*(l/2-xm)+n(2)*n(3)*(w/2-ym)+n(3)^2*d
xa = n(1)/n(3)*za+xm
ya = n(2)/n(3)*za+ym
P_A = [xa-l/2,ya-w/2,za-d] % A is a point on plane P_Ri_Li£¬A_Mi is perpendicular to plane P_Ri_Li

cosa = dot(P_Ri,P_Li)/(norm(P_Ri)*norm(P_Li))
cosb = dot(P_Ri,P_A)/(norm(P_Ri)*norm(P_A))
sinr = dot(P_Mi,n)/(norm(P_Mi)*norm(n)) % cosine and sine value of the a,b,r angles
