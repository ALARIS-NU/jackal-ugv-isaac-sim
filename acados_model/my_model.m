function model_struct=my_model()

% Jackal robot model
model_name = 'Wheelchair';

r_wheel = 0.10; % radius of main wheels 0.1m
r_base = 0.28;  % wheelchair presented as 2 spheres, one at the center between two main wheels
B = 0.3765; % distance between the wheels
alpha = 3.6; %1.8; % handpicked parameter
r_obst = 0.07; % radius of obstacle in the costmap

% we define constrains as spheres located in grid.
% default local costmap is 5x5m with resolution of 0.1m which is 2601 point
% but on practice it is rearly observed more than 30% of a map as a
% occupied points. (we can test hipotesis of 10%, or select closest only.)
% Thus we will define only as a constraints 260 points.
% By default the coordinate of the constraint point will be outside of the
% costmap (-100, -100). and only first n points will be defined based on
% the actual costmap data.

% States
x = casadi.SX.sym('x');y = casadi.SX.sym('y');theta = casadi.SX.sym('theta');

sym_x = vertcat(x, y, theta);

x_dot = casadi.SX.sym('x_dot');y_dot = casadi.SX.sym('y_dot');theta_dot = casadi.SX.sym('theta_dot');
sym_xdot = vertcat(x_dot, y_dot, theta_dot);

% Controls
phi_dot_1 = casadi.SX.sym('phi_dot_1');phi_dot_2 = casadi.SX.sym('phi_dot_2');
sym_u = vertcat(phi_dot_1, phi_dot_2);
            
% Parameters (External data)
local_costmal = casadi.SX.sym('local_costmal',[2*260,1]);
sym_p = vertcat(local_costmal);

% in robot frame:
% expr_f_expl = [0, 0;...
%                0.5*r_wheel, 0.5*r_wheel;...
%                1*r_wheel/(alpha*B), -1*r_wheel/(alpha*B)]*sym_u; % simple integrator

% in local frame u = [omega_right; omega_left]
expr_f_expl = [0.5*r_wheel*cos(theta), 0.5*r_wheel*cos(theta);...
               0.5*r_wheel*sin(theta), 0.5*r_wheel*sin(theta);...
               1*r_wheel/(alpha*B), -1*r_wheel/(alpha*B)]*sym_u; % simple integrator
expr_f_impl = expr_f_expl - sym_xdot;
%% Forward kinematics

F_cart_1 = [x; y];

%% Constraints
h_expression_array = [];
h_lower_bounds = [];
h_upper_bounds = [];
%%

for k=1:260
    temp1 = (F_cart_1(:,1)-[local_costmal((k-1)*2+1);local_costmal((k-1)*2+2)])'*...
        (F_cart_1(:,1)-[local_costmal((k-1)*2+1);local_costmal((k-1)*2+2)]);
    h_expression_array = vertcat(h_expression_array, -temp1+(r_base+r_obst)*(r_base+r_obst));
    h_lower_bounds(end+1) = -10e6;
    h_upper_bounds(end+1) = 0.0;
end

soft_indices = [1:length(h_expression_array)];  % set index of slack constraints

%% save to model
model_struct = struct();
model_struct.states = sym_x;
model_struct.controls = sym_u;
model_struct.sym_xdot = sym_xdot;
model_struct.parameters = sym_p;
model_struct.expr_f_expl = expr_f_expl;
model_struct.expr_f_impl = expr_f_impl;
model_struct.position_function = F_cart_1;
model_struct.constraints = h_expression_array;
model_struct.constraints_upper_bounds = h_upper_bounds;
model_struct.constraints_lower_bounds = h_lower_bounds;
model_struct.soft_constraints_indices = soft_indices;
model_struct.model_name = model_name;

end
