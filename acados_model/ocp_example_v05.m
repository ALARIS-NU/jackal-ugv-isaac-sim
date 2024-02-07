%% test of native matlab interface
clear all

model_struct = my_model();

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
    error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

%%
T = 0.50; % horizon length
N = 10; % Number of steps

%%

nx = length(model_struct.states);
nu = length(model_struct.controls);
% number of outputs is the concatenation of x and u
ny = nx + nu;
ny_e = nx;

% The linear cost contributions is defined through Vx, Vu and Vz
Vx = zeros(ny, nx);
Vx_e = zeros(ny_e, nx);
Vu = zeros(ny, nu);

Vx(1:nx,:) = eye(nx);
Vx_e(1:nx,:) = eye(nx);
Vu(nx+1:end,:) = eye(nu);


Wu = 1e3*eye(nu);
Wx = 1e2*eye(nx);
Wx(3,3)= 1000000.0;%3e-1;
W = blkdiag(Wx, Wu);
W_e = Wx*1000;
W_e(3,3)= 1e8;


Jbx = eye(nx);
lbx =  [-10 -10 -6*pi];
ubx = [10 10 6*pi];

Jbx_e = eye(nx);  
lbx_e = [-10 -10 -6*pi];
ubx_e = [10 10 6*pi];


Jbu = eye(nu);
lbu = [-4.6 -4.6];
ubu = [4.6 4.6];

x0 = zeros(3,1);

%% acados ocp model
ocp_model = acados_ocp_model();

ocp_model.set('T', T);
ocp_model.set('name', model_struct.model_name);

% symbolics
ocp_model.set('sym_x', model_struct.states);
ocp_model.set('sym_u', model_struct.controls);
ocp_model.set('sym_xdot', model_struct.controls);
% ocp_model.set('sym_z', sym_z);
ocp_model.set('sym_p', model_struct.parameters);

% cost
ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vu', Vu);
%ocp_model.set('cost_Vz', Vz);
%ocp_model.set('cost_y_ref', y_ref);        % value to change for every .solve()

ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', Vx_e);
%ocp_model.set('cost_y_ref_e', y_ref_e);   % value to change for every .solve()

ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);


% dynamics
is_impl = false;
if is_impl
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model_struct.expr_f_impl);
else
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model_struct.expr_f_expl);
end

% constraints
ocp_model.set('constr_type', 'bgh');
ocp_model.set('constr_x0', x0);        % value to change for every .solve()

ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);

ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);

ocp_model.set('constr_Jbx_e', Jbx_e);
ocp_model.set('constr_lbx_e', lbx_e);
ocp_model.set('constr_ubx_e', ubx_e);


ocp_model.set('constr_expr_h', model_struct.constraints);
ocp_model.set('constr_lh', model_struct.constraints_lower_bounds);
ocp_model.set('constr_uh', model_struct.constraints_upper_bounds);
nh = length(model_struct.constraints);

% Configure constraint slack variables
if ~isempty(model_struct.soft_constraints_indices)
    nsh = length(model_struct.soft_constraints_indices);
    Jsh = zeros(nh, nsh);
    for i=1:nsh
        Jsh(model_struct.soft_constraints_indices(i),i) = 1;
    end
    ocp_model.set('constr_Jsh', Jsh);
    % Set cost on slack
    % L1 slack (linear term)
    ocp_model.set('cost_zl', 0 * ones(nsh,1));
    ocp_model.set('cost_zu', 0 * ones(nsh,1));
    % L2 slack (squared term)
    ocp_model.set('cost_Zl', 0.0 * ones(nsh,nsh));
    ocp_model.set('cost_Zu', 10000 * ones(nsh,nsh));
end

%ocp_model.set('cost_y_ref', 20*ones(ny,1));
%ocp_model.set('cost_y_ref_e', 1000*ones(ny_e,1));

%ocp_model.set('cost_y_ref', [10, 10, 5.0, 0.000, 0.000]');
%ocp_model.set('cost_y_ref_e', [1000, 1000, 500]');

%%
ocp_opts = acados_ocp_opts();
% Code generation
ocp_opts.set('compile_interface', 'auto');
ocp_opts.set('codgen_model', 'true');
ocp_opts.set('compile_model', 'true');
ocp_opts.set('output_dir', 'build');
% Shooting nodes
ocp_opts.set('param_scheme_N',N); % Can be set wariable steps
% Integrator
if is_impl
    ocp_opts.set('sim_method', 'irk');
else
    ocp_opts.set('sim_method', 'erk');  % ’irk’ ’erk’ , ’irk’ , ’irk_gnsf’
end
ocp_opts.set('sim_method_num_stages', 4); % RK4
ocp_opts.set('sim_method_num_steps', 1);
ocp_opts.set('sim_method_newton_iter', 3);
% NLP solver
ocp_opts.set('nlp_solver', 'sqp_rti');
if (strcmp(ocp_opts.opts_struct.nlp_solver, 'sqp'))
    ocp_opts.set('nlp_solver_max_iter', 40);
    ocp_opts.set('nlp_solver_tol_stat', 1e-4);
    ocp_opts.set('nlp_solver_tol_eq', 1e-4);
    ocp_opts.set('nlp_solver_tol_ineq', 1e-4);
    ocp_opts.set('nlp_solver_tol_comp', 1e-4);
elseif (strcmp(ocp_opts.opts_struct.nlp_solver, 'sqp_rti'))
    ocp_opts.set('rti_phase',0); % RTI phase: (1) preparation, (2) feedback, (0) both
end
ocp_opts.set('nlp_solver_ext_qp_res', 1);
ocp_opts.set('nlp_solver_step_length', 1.0); % fixed step length in SQP algorithm
% QP solver
ocp_opts.set('qp_solver', 'full_condensing_hpipm');
if (strcmp(ocp_opts.opts_struct.qp_solver, 'full_condensing_hpipm'))
    %ocp_opts.set('qp_solver_cond_N', N);
    ocp_opts.set('qp_solver_cond_ric_alg', 1); % 0: dont factorize hessian in the condensing; 1: factorize
    ocp_opts.set('qp_solver_ric_alg', 1); % HPIPM specific
    ocp_opts.set('qp_solver_warm_start', 1);
    % ocp_opts.set('warm_start_first_qp', 0); % warm start even in first SQP iteration: (0) no, (1) yes
end
% globalization
% ocp_opts.set('globalization', 'fixed_step'); % ’fixed_step’ , ’merit_backtracking’
% ocp_opts.set('alpha_min', 0.05);
% ocp_opts.set('alpha_reduction', 0.7);
% Hessian approximation
ocp_opts.set('nlp_solver_exact_hessian', 'false');
ocp_opts.set('regularize_method', 'no_regularize');
ocp_opts.set('levenberg_marquardt', 1e-5); % in case of a singular hessian, setting this > 0 can help convergence
if (strcmp(ocp_opts.opts_struct.nlp_solver_exact_hessian, 'true'))
    ocp_opts.set('exact_hess_dyn', 1); % compute and use hessian in dynamics, only if ’nlp_solver_exact_hessian’ = ’true’
    ocp_opts.set('exact_hess_cost', 1);
    ocp_opts.set('exact_hess_constr', 1);
end
ocp_opts.set('print_level', 0);
%%
tic
ocp = acados_ocp(ocp_model, ocp_opts);
toc
%%
ocp.generate_c_code()

%% acados ocp
disp('Start simulation')

% Simulate
dt = T / N;
Tf = 20.00;  % maximum simulation time[s]
Nsim = round(Tf / dt);

% initialize data structs
simX = zeros(Nsim, nx);
simU = zeros(Nsim, nu);



%x0 = [0.0, 0.0, +pi/2]';
x0 = [0.0, 0.0, pi/4]';
x0_initial = x0;
goal = [-3.0, +1.6, -1.7];

local_costmap = -100*ones(260*2,1);

% set trajectory initialization
init_x=zeros(nx,N+1);
for i=1:N+1
    init_x(:,i)=x0;
end
ocp.set('init_x', init_x);
ocp.set('init_u', zeros(nu, N));
ocp.set('init_pi', zeros(nx, N));

fun = casadi.Function('fun',...
    {model_struct.states, model_struct.controls, model_struct.parameters},...
    {model_struct.constraints});

elapsed = [];
% simulate
for i = 1:Nsim
    % update initial state
    ocp.set('constr_x0', x0);    
    ocp.set('constr_lbx', x0, 0);
    ocp.set('constr_ubx', x0, 0);
%     ocp.set('constr_lbx', goal, N);
%     ocp.set('constr_ubx', goal, N);
    % update reference
    for j = 0:(N-1)
        %yref = [goal, zeros(1,2)];
        yref = [(goal'-x0)/N*(1+j); zeros(2,1)];
        ocp.set('cost_y_ref', yref, j);
        ocp.set('p',local_costmap,j);
    end
    yref_N = goal;   
    ocp.set('cost_y_ref_e', yref_N);

    % solve ocp
    t = tic();
    for k=1:10
        ocp.solve();
    end
    status = ocp.get('status'); % 0 - success
    if status ~= 0
        % borrowed from acados/utils/types.h
        %statuses = {
        %    0: 'ACADOS_SUCCESS',
        %    1: 'ACADOS_FAILURE',
        %    2: 'ACADOS_MAXITER',
        %    3: 'ACADOS_MINSTEP',
        %    4: 'ACADOS_QP_FAILURE',
        %    5: 'ACADOS_READY'
        %error(sprintf('acados returned status %d in closed loop iteration %d. Exiting.', status, i));
%         enum hpipm_status
% 	{
% 	SUCCESS, // found solution satisfying accuracy tolerance
% 	MAX_ITER, // maximum iteration number reached
% 	MIN_STEP, // minimum step length reached
% 	NAN_SOL, // NaN in solution detected
% 	INCONS_EQ, // unconsistent equality constraints
% 	}
        disp(sprintf('acados returned status %d in closed loop iteration %d. Exiting.', status, i));
    end
    %ocp.print('stat')
    elapsed_now = toc(t);
    elapsed(end+1) = elapsed_now;
    %disp(ocp.get('x', 0))
    result = fun(ocp.get('x', 0),ocp.get('u', 0),local_costmap);
    disp(find(full(result>0)))

    % get solution
    if 1
        x0 = ocp.get('x', 0);
        u0 = ocp.get('u', 0);
        for j = 1:nx
            simX(i, j) = x0(j);
        end
        for j = 1:nu
            simU(i, j) = u0(j);
        end
        %result(4);
        %if status==4
        %    human_pose = human_pose_B;
        %end
        % update initial condition
        x0 = ocp.get('x', 1);
    end
end
figure; plot(simX); title('pose')
figure; plot(simU); title('speed')
disp('mean solution time')
disp(mean(elapsed))
%%
figure;
fun_my_cart = casadi.Function('fun_my_cart',{model_struct.states},{model_struct.position_function});

end_effector_path = [];
for i=1:length(simX)
    trajectory = full(fun_my_cart(simX(i, :)));
    %end_effector_path(end+1,:) = [trajectory(1,1),trajectory(2,1),trajectory(1,2),trajectory(2,2)];
    end_effector_path(end+1,:) = [trajectory(1),trajectory(2)];
end


cart_initial = full(fun_my_cart(x0_initial));
viscircles([cart_initial(:,1)'], [0.28], 'color', 'b')
xlim([-5 5])
ylim([-5 5])
hold on
cart_final = full(fun_my_cart(goal));
viscircles([cart_final(:,1)'], [0.28], 'color', 'r')


% plot(end_effector_path(:,1),end_effector_path(:,2))
% hold on;viscircles([0.331503,   0.467319], 0.0500,'Color','b')

plot(end_effector_path(:,1),end_effector_path(:,2))
%plot(end_effector_path(:,3),end_effector_path(:,4))

axis equal
xlabel('x')
ylabel('y')
