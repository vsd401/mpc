function [angle] = pipi(angle) % the unit of angle is in rad, but in this case, you dont need to use it ;
 
if (angle > pi)
    angle =  angle - 2*pi;
elseif (angle < -pi)
    angle = angle + 2*pi;
else
    angle = angle;
end
end