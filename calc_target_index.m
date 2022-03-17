function [ind, error] = calc_target_index(x,y, cx,cy,cyaw)% find my location, and lateral error
N =  length(cx);
Distance = zeros(N,1);
for i = 1:N
Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);
end
[value, location]= min(Distance);
ind = location
 
dx1 =  cx(ind) - x;
dy1 =  cy(ind) - y ;
% angle = pipi(cyaw(ind)-atan(dy1/dx1));
% heading = cyaw(ind)*180/pi;
    if y<cy(ind) 
        error = -value;
    else
        error = value;
    end
% error = value;
end