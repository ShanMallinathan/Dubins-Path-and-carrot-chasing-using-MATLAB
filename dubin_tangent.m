%Function to get the entry and exit coordinates of RSR path
function [c1, c2, te, tx, phi_en, phi_ex, es, ef, path_length, config] = dubin_tangent(c1, c2, s, f, flag1, flag2)

r = 5;      %radius
si = 0;
%config gives details about the type of path 
% -3 -> LSL; -1 -> LSR; +1 -> RSL; +3 -> RSR
config = 2 * flag1 + flag2; 

phi = atan2(c2(2) -c1(2), c2(1) - c1(1));   %tc angle
c = norm(c2 - c1);      %center distance

if (config^2 ==1) %only for LSR and RSL
    si = asin(2 * r / c);   %angle between tc and line connecting outer circle
end

%entry and exit angles
phi_en = phi + si + flag1 * pi/2;
phi_ex = phi + si + flag2 * pi/2;
%included angles
te = [(c1(1) + r * cos(phi_en )), (c1(2) + r * sin(phi_en))];
tx = [(c2(1) + r * cos(phi_ex)), (c2(2) + r * sin(phi_ex))];

%calculating the path length
es = calculate_sector_angle(c1, s, te, flag1);
ef = calculate_sector_angle(c2, tx, f, flag2);
d1 = r * es;
d2 = norm(tx - te);
d3 = r *ef;
path_length = d1 +d2 + d3;
