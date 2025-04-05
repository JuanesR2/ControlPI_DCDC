
A = [0    0;
     0 -0.3];

B = [0 -1;
     1  0];

d = [1; 
     0];

u_ref = 0.6;
x_ref = - (A+ u_ref*B)\d

kp = 1;
ki = 10; 

cvx_begin

variable P(2,2) symmetric
variable Q(2,2) symmetric
P - 0.01*eye(2) == semidefinite(2);
Q == P*A +A'*P;
-Q - 0.00*eye(2) == semidefinite(2);
P*B + B'*P == 0;


cvx_end

y_d = @(x) (x-x_ref)'*P*B*x_ref;

u_control = @(x_e) -kp*y_d(x_e(1:2)) - ki*x_e(3);

convertidor = @(t,x_e) [A*x_e(1:2) + u_control(x_e)*B*x_e(1:2) + d;
                        y_d(x_e(1:2))] ;



[tode, xode] = ode45(convertidor, [0,20],[0;0;0]);


plot(tode,xode)
grid on
