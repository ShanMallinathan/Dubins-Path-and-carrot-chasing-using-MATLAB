%calculate the included angle of 2 vector v1 and v2 of ciecle c
%flag = -1 for Left and 1 for Right (flag1/2 to be passed here)

function [theta] = calculate_sector_angle(c, v1, v2, flag)

t1 = atan2((v1(2) - c(2)), (v1(1) - c(1)));
t2 = atan2((v2(2) - c(2)), (v2(1) - c(1)));
theta = t2 - t1;

if (flag == -1 && theta  < 0) %For CCW
    theta = 2 * pi + theta;
elseif(flag == 1 && theta > 0) %for CW
    theta = 2 * pi - theta;
elseif(flag == 1 && theta < 0)
    theta = -1 * theta;
end