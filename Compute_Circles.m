%Program to calculate the centres of the entry and exit circles

function[S_CCW, S_CW, F_CCW, F_CW] = Compute_Circles(x1, h1, x2, h2)

r = 5; %Turining radius

%Entry Circles
S_CCW = [x1(1) + r * cos(h1 + pi/2), x1(2) + r * sin(h1 +pi/2)];
S_CW = [x1(1) - r * cos(h1 + pi/2), x1(2) - r * sin(h1 +pi/2)];
%Exit Circles
F_CCW = [x2(1) + r * cos(h2 + pi/2), x2(2) + r * sin(h2 +pi/2)];
F_CW = [x2(1) - r * cos(h2 + pi/2), x2(2) - r * sin(h2 +pi/2)];

end
