function [ c, ceq ] = dynamics_constraint( x )
global gridN
gridN = 200;
m1=1;
m2=0.3;
l=0.5;
g=9.81;
umax=20;
dmax=2;
d=1;
T=2;

    
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    delta_time = 0.03; 
    % Get the states / inputs out of the vector
    q1=x(1,:);
    q2=x(2,:);
    q1dot=x(3,:);
    q2dot=x(4,:);
    u=x(5,:);
    for i=1:gridN
        q1_ddot(1,i)=(l*m2*sin(q2(i))*(q2dot(i)^2)+u(i)+m2*g*cos(q2(i))*sin(q2(i)))/(m1+m2*(1-(cos(q2(i)).^2)));
        q2_ddot(1,i)=-(l*m2*cos(q2(i))*sin(q2(i))*(q2dot(i)^2)+u(i)*cos(q2(i))+(m1+m2)*g*sin(q2(i)))/(l*m1+l*m2*(1-(cos(q2((i)).^2))));
    end
    x_dot=[q1dot;q2dot;q1_ddot;q2_ddot];
    
    % Constrain initial position and velocity to be zero
    ceq = [q1(1); q2(1);q1dot(1);q2dot(1);u(1)];
    for i = 1 : length(u) - 1
        % The state at the beginning of the time interval
        x_i = [q1(i); q2(i);q1dot(i);q2dot(i)];
        % What the state should be at the start of the next time interval
        x_n =  [q1(i+1); q2(i+1);q1dot(i+1);q2dot(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = [q1dot(i); q2dot(i);q1_ddot(i); q2_ddot(i)];
        % The time derivative of the state at the end of the time interval
        xdot_n = [q1dot(i+1); q2dot(i+1);q1_ddot(i+1); q2_ddot(i+1)];
        
        
        % The end state of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; q1(end)-d ; q2(end)-pi ; q1dot(end);q2dot(end);u(end)];
end