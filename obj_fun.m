function[cost]=obj_fun(x)
fun=x(5,:).^2;
npoint=200;
j=1:npoint;
cost=trapz(j,fun);
end