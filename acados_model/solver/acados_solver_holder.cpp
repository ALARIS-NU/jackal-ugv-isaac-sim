// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_Wheelchair.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     WHEELCHAIR_NX
#define NZ     WHEELCHAIR_NZ
#define NU     WHEELCHAIR_NU
#define NP     WHEELCHAIR_NP
#define NBX    WHEELCHAIR_NBX
#define NBX0   WHEELCHAIR_NBX0
#define NBU    WHEELCHAIR_NBU
#define NSBX   WHEELCHAIR_NSBX
#define NSBU   WHEELCHAIR_NSBU
#define NSH    WHEELCHAIR_NSH
#define NSG    WHEELCHAIR_NSG
#define NSPHI  WHEELCHAIR_NSPHI
#define NSHN   WHEELCHAIR_NSHN
#define NSGN   WHEELCHAIR_NSGN
#define NSPHIN WHEELCHAIR_NSPHIN
#define NSBXN  WHEELCHAIR_NSBXN
#define NS     WHEELCHAIR_NS
#define NSN    WHEELCHAIR_NSN
#define NG     WHEELCHAIR_NG
#define NBXN   WHEELCHAIR_NBXN
#define NGN    WHEELCHAIR_NGN
#define NY0    WHEELCHAIR_NY0
#define NY     WHEELCHAIR_NY
#define NYN    WHEELCHAIR_NYN
#define NH     WHEELCHAIR_NH
#define NPHI   WHEELCHAIR_NPHI
#define NHN    WHEELCHAIR_NHN
#define NPHIN  WHEELCHAIR_NPHIN
#define NR     WHEELCHAIR_NR

class my_NMPC_solver {
private:
  int num_steps;
  // global data
  Wheelchair_solver_capsule * acados_ocp_capsule;
public:
  my_NMPC_solver(int n) {
    num_steps = n; // set number of real-time iterations
    Wheelchair_solver_capsule * my_acados_ocp_capsule = Wheelchair_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = WHEELCHAIR_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = Wheelchair_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("Wheelchair_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    
  }
  int solve_my_mpc(double current_robot_position[3], double costmap_data[520], double current_robot_goal[3], double tracking_goal[30], double results[8], double trajectory[33]) {
    int status = -1;
    int N = WHEELCHAIR_N;

    ocp_nlp_config *nlp_config = Wheelchair_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = Wheelchair_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = Wheelchair_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = Wheelchair_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = Wheelchair_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = Wheelchair_acados_get_nlp_opts(acados_ocp_capsule);

    
    // initial condition
    int idxbx0[NBX0];   // indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    for (int i=0;i<NBX0;i++) idxbx0[i]=i;

    int idxbx[NBX];   // indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    for (int i=0;i<NBX;i++) idxbx[i]=i;

    int idxbu[NBU];
    for (int i=0;i<NBU;i++) idxbu[i]=i;

    int idxsh[NSH];
    for (int i=0;i<NSH;i++) idxsh[i]=i; // set stick constraint as slack
    double lh[NH];
    double uh[NH];
    for (int i=0;i<NH;i++) lh[i]=-10e6;
    for (int i=0;i<NH;i++) uh[i]=0.0;

    //printf("*********\n\n %i %i %i\n*********\n\n", NSH, NSHN, NH);

    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i=0;i<3;i++) lbx0[i] = current_robot_position[i];
    for (int i=0;i<3;i++) ubx0[i] = current_robot_position[i];


    // Set x0
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    //printf("********* where is nlp *********\n\n");
    double pi=3.1415926;
    double lbx[3] = {-10.0, -10.0, -6*pi};
    double ubx[3] = {+10.0, +10.0, 6*pi};

    double lbu[2] = {-4.6, -4.6};
    double ubu[2] = {4.6, 4.6};

    double Vx[NY*NX] = {0.0};
    for (int i=0;i<NX;i++) Vx[NY*i+i] = 1.0;

    double Vx_e[NYN*NX] = {0.0};
    for (int i=0;i<NX;i++) Vx[NYN*i+i] = 1.0;

    double Vu[NY*NU] = {0.0};
    for (int i=0;i<NU;i++) Vu[NY*i+i] = 1.0;


    double W[(NU+NX)*(NU+NX)]= {0.0};
    double WN[(NX)*(NX)] = {0.0};

    //double weights_diag[NX] = {10.0, 10.0, 1000.0}; 
    double weights_diag[NX] = {100.0, 100.0, 5000.0}; 
    //double weights_diag[NX] = {1000.0, 1000.0, 1.0}; 
    for (int ii = 0; ii < (NX); ii++) W[ii+ii*(NU+NX)] = weights_diag[ii];
    for (int ii = NX; ii < (NU+NX); ii++) W[ii+ii*(NU+NX)] = 1000;

    for (int ii = 0; ii < (NX); ii++) WN[ii+ii*(NX)] = weights_diag[ii]*1000.0;
    WN[NX-1+(NX-1)*(NX)] = 1000.0;//100000000.0;

    double zl[NSH]={0.0};
    double zu[NSH]={0.0};

    double Zl[NSH] = {0.0};
    double Zu[NSH] = {0.0};
    for (int i=0;i<NSH;i++) Zl[i] = 0.0;
    for (int i=0;i<NSH;i++) Zu[i] = 10000.0;

    double y_ref[NY];
    for (int i=0;i<3;i++) y_ref[i] = current_robot_goal[i];
    for (int i=0;i<2;i++) y_ref[i+3] = 0.0;
    double y_ref_N[NYN];
    for (int i=0;i<NYN;i++) y_ref_N[i] = current_robot_goal[i];

    // Update constraints
    for (int ii = 0; ii < N; ii++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubu", ubu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubx", ubx);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", lh);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", uh);
        //ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxsh", idxsh);
    }
    // Set x0
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    for (int ii=0;ii<N;ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", W);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);
    

    // Set goal
    for (int ii=0;ii<N;ii++) {
        //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zl", zl);
        //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zu", zu);
        //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zl", Zl);
        //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zu", Zu);
        //for (int i=0;i<3;i++) y_ref[i] = tracking_goal[ii*5+i]; // Check if OK???
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", y_ref);
        //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", y_ref_N);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", y_ref_N);

    // initialization for state values
    //double x_init[NX] = {0.0};
    //for (int j=0;j<NX;j++) x_init[j]=current_robot_position[j];
    double x_init[NX*(N+1)]={0.0};
    for (int i=0; i<=N;i++) for (int j=0;j<NX;j++) x_init[i*NX+j]=current_robot_position[j];

    // initial value for control input
    double u0[NU] = {0.0};
    // set parameters
    double p[NP] = {0.0};
    for (int i = 0; i < 520; i++) p[i] = costmap_data[i];

    for (int ii = 0; ii <= N; ii++)
    {
        Wheelchair_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

    // prepare evaluation
    int NTIMINGS = num_steps; // set number of real-time iterations
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)] = {0.0};
    double utraj[NU * N] = {0.0};

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    // solve ocp in loop
    int rti_phase = 0;
    double ocp_cost = 0.0;
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i <= nlp_dims->N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init); // Check if OK??
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = Wheelchair_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        exec_time = exec_time + elapsed_time;
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
        ocp_nlp_eval_cost(nlp_solver,nlp_in,nlp_out); 
        ocp_nlp_get(nlp_config, nlp_solver,"cost_value", &ocp_cost);
        //printf("solver SQP step %d, status %d times, cost %f", ii, status, ocp_cost);
        if (ocp_cost<0.001) break;
        /*if (status!=4) {
	    for (int ii = 0; ii <= nlp_dims->N; ii++)
		ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
	    for (int ii = 0; ii < nlp_dims->N; ii++)
		ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
        } */
          /*else {
	    idxsh[1]=0; // set stick constraint as slack
	    idxsh[2]=1; // set stick constraint as slack
	    idxsh[3]=2; // set stick constraint as slack
            // Update constraints
            for (int ii = 0; ii < N; ii++)
                 ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxsh", idxsh);
        }*/
    }

    /* print solution and statistics */
    
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
    
    /*
    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    */
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    //printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("Wheelchair_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("Wheelchair_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
    
    //***************************************
    //double results[8];
    results[7] = 0.0; // NaN marker
    for (int i=0;i<4;i++) {
        if (utraj[i]!=utraj[i]) results[7] = 0.0;
        else results[i] = utraj[i];
    }
    results[4] = sqp_iter; results[5] = exec_time*1000; results[6] = kkt_norm_inf;
    //***************************************
    for (int i=0;i<33;i++) trajectory[i] = xtraj[i];


    //Wheelchair_acados_print_stats(acados_ocp_capsule);

    //printf("\nSolver info:\n");
    //printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  cost %f \n",
    //       sqp_iter, NTIMINGS, exec_time*1000, ocp_cost);



    return status;
  }  // end of 'solve' function

  int reset_solver(){
    int status = -1;
    // free solver
    status = Wheelchair_acados_free(acados_ocp_capsule);
    if (status) {
        printf("Wheelchair_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = Wheelchair_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("Wheelchair_acados_free_capsule() returned status %d. \n", status);
    }
    return status;
  }


};
