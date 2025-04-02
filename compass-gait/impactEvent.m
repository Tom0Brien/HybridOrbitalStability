function [value, isterminal, direction] = impactEvent(t,x,sys)
    % States
    q     = x(1:2);
    dq    = x(3:4);

    % x/y position of swing foot
    x_pos = sys.x(x);
    y_pos = sys.y(x);

    % y velocity of swing foot
    ydot = sys.l*(dq(1)*-sin(q(1)+sys.psi) + dq(2)*sin(q(2)+sys.psi));
    
    % only terminate if foot moving down and greater than 0.1m from origin to
    % prevent scuffing
    direction = -1;
    value = y_pos;
    if ydot < 0 && x_pos > 0.1
        isterminal = 1;
    else
       isterminal = 0;
    end
end