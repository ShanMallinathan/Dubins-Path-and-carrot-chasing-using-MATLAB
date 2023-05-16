%Program to compute Dubin's path and carrot chasing
%A code by Shanthianth Mallinathan

%Initialising the waypoints
W_pos = [0, 0; 
         0, 10; 
         60, 60; 
         80, 120; 
         150, 70; 
         100, 30; 
         50, 0];
W_head = [0, 0, 45, 30, -90, -120, -180] * pi/180;
r = 5; %radius



%Te and Tx for RSR
%Defining flags
%flag1 = +1/-1 -> initial circle clockwise/Anticlockwise
%flag2 = +1/-1 -> final circle clockwise/Anticlockwise
%pass the value of flag as the last 2 arguments for dubin_tangent function.
%i.e. -1, -1 -> LSL; 1, 1 -> RSR; -1, 1 -> LSR; 1, -1 -> RSL

for i = 1:(length(W_head)-1)

    %Calling Compute_Circles to get circle centres
    [s_ccw, s_cw, f_ccw, f_cw] = Compute_Circles(W_pos(i, :), W_head(i), W_pos(i+1, :), W_head(i+1));
    %computing centre distances
    c_lsl = norm(s_ccw - f_ccw);
    c_lsr = norm(s_ccw - f_cw);
    c_rsl = norm(s_cw - f_ccw);
    c_rsr = norm(s_cw - f_cw);
    path_length = [1 1 1 1] * 10^(2);    %Initialising large value for path length
    [c1(1, :), c2(1, :), te(1, :), tx(1, :), phi_en(1), phi_ex(1), es(1), ef(1), path_length(1), config(1)] = dubin_tangent(s_ccw, f_ccw , W_pos(i, :), W_pos(i+1, :), -1, -1);  %LSL
    [c1(2, :), c2(2, :), te(2, :), tx(2, :), phi_en(2), phi_ex(2), es(2), ef(2), path_length(2), config(2)] = dubin_tangent(s_cw, f_cw , W_pos(i, :), W_pos(i+1, :), 1, 1);    %RSR
    if (c_lsr >= 2 * r)
        [c1(3, :), c2(3, :), te(3, :), tx(3, :), phi_en(3), phi_ex(3), es(3), ef(3), path_length(3), config(3)] = dubin_tangent(s_ccw, f_cw , W_pos(i, :), W_pos(i+1, :), -1, 1);   %LSR
    end
    if (c_rsl >= 2 * r)
        [c1(4, :), c2(4, :), te(4, :), tx(4, :), phi_en(4), phi_ex(4), es(4), ef(4), path_length(4), config(4)] = dubin_tangent(s_cw, f_ccw , W_pos(i, :), W_pos(i+1, :), 1, -1);   %RSL
    end
    [min_length, min_index] = min(path_length);

    %absorbing only the shortest path
    C1(i, :) = c1(min_index, :);
    C2(i, :) = c2(min_index, :);
    Te(i, :) = te(min_index, :);
    Tx(i, :) = tx(min_index, :);
    Phi_en(i) = phi_en(min_index);
    Phi_ex(i) = phi_ex(min_index);
    Es(i) = es(min_index);
    Ef(i) = ef(min_index);
    Path_length(i) = path_length(min_index);
    Config(i) = config(min_index); % To grt the dubin configuration
end

plotter(C1, C2, W_pos, Te, Tx, Es, Ef);

pos = [0; 0]; %initial Position
head = 0;     %initial heading
 
%output the results
disp("Dubins Path by Shanthinath Mallinathan");
for i=1:(length(W_head)-1)
    fprintf('\n\nWay point: %d\n', i);
    fprintf('Initial Waypoint: %.2f %.2f\n', W_pos(i, :));
    fprintf('Final Waypoint: %.2f %.2f\n', W_pos(i+1, :));
    fprintf('Initial Circle: %.2f %.2f\n', C1(i, :));
    fprintf('Final Circle: %.2f %.2f\n', C2(i, :));
    fprintf('Entry Angle: %.2f deg\n', Phi_en(i)*180/pi);
    fprintf('Exit Angle: %.2f deg\n', Phi_ex(i)*180/pi);
    fprintf('Exit from initial circle: %.2f %.2f\n', Te(i, :));
    fprintf('Entry to final circle: %.2f %.2f\n', Tx(i, :));
end
fprintf("\n\nThe total path length is: %.2f m\n", sum(Path_length));
flag1 = 0
flag2 = 0
%Carrot chasing implimentation
for i = 1:(length(W_head)-1)
    i
    if(Config(i) == -3) %LSL
        flag1 = -1;
        flag2 = -1;
    elseif(Config(i) == -1) %LSR
        flag1 = -1;
        flag2 = 1;
    elseif(Config(i) == 1) %RSL
        flag1 = 1;
        flag2 = -1;
    elseif(Config(i) == 3) %RSR
        flag1 = 1;
        flag2 = 1;
    end
   
    flag1
    flag2
   
    [pos, head] = carrot_c(C1(i, :), pos, head, Te(i, :), flag1);
    [pos, head] = carrot_c(C2(i, :), pos, head,  Tx(i, :), flag2);
    [pos, head] = carrot_c(C2(i, :), pos, head,  W_pos(i+1, :), flag2);

end