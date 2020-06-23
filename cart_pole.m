global gridN
gridN = 200;
m1=1;
m2=0.3;
l=0.5;
g=9.81;
umax=20;
dmax=2;
d=3;
T=2;
x_guess=zeros(5,gridN);


 for k=1:gridN

x_guess(:,k)=((k)/gridN).*[d,pi,0,0,0];
 end
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero, leave initial velocities unbounded
lb = [-dmax*ones(1,gridN);-Inf*ones(1,gridN);-Inf*ones(1,gridN);-Inf*ones(1,gridN);-umax*ones(1,gridN)];

ub = [dmax*ones(1,gridN);Inf*ones(1,gridN);Inf*ones(1,gridN);Inf*ones(1,gridN);umax*ones(1,gridN)];

% Solve for the best simulation time + control input
options = optimset('MaxFunEvals',10000,'useParallel','Always');
[optimal,cost] = fmincon(@obj_fun, x_guess, A, b, Aeq, Beq, lb, ub, ...
                  @dynamics_constraint,options);
[optimal,cost] = fmincon(@obj_fun, optimal, A, b, Aeq, Beq, lb, ub, ...
                  @dynamics_constraint,options);
[optimal,cost] = fmincon(@obj_fun, optimal, A, b, Aeq, Beq, lb, ub, ...
                  @dynamics_constraint,options);
[optimal,cost] = fmincon(@obj_fun, optimal, A, b, Aeq, Beq, lb, ub, ...
                  @dynamics_constraint,options);
% time=linspace(0,2,200);             
% axis([-4 4 -4 4] )
% plot(time,optimal(5,:))
% 
% 

for k=1:200
      if k == 1
          pause(5);
      end
%      drawcartpend_bw(optimal(k,:),m2,m1,2);
       drawcartpend(optimal(:,k),m2,m1,2);
 end
 
 