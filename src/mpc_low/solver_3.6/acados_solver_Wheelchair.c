/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "Wheelchair_model/Wheelchair_model.h"
#include "Wheelchair_constraints/Wheelchair_constraints.h"




#include "acados_solver_Wheelchair.h"

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
// #define N      WHEELCHAIR_N
#define NH     WHEELCHAIR_NH
#define NPHI   WHEELCHAIR_NPHI
#define NHN    WHEELCHAIR_NHN
#define NPHIN  WHEELCHAIR_NPHIN
#define NR     WHEELCHAIR_NR


// ** solver data **

Wheelchair_solver_capsule * Wheelchair_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(Wheelchair_solver_capsule));
    Wheelchair_solver_capsule *capsule = (Wheelchair_solver_capsule *) capsule_mem;

    return capsule;
}


int Wheelchair_acados_free_capsule(Wheelchair_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int Wheelchair_acados_create(Wheelchair_solver_capsule* capsule)
{
    int N_shooting_intervals = WHEELCHAIR_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return Wheelchair_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int Wheelchair_acados_update_time_steps(Wheelchair_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "Wheelchair_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for Wheelchair_acados_create: step 1
 */
void Wheelchair_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/
    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
    nlp_solver_plan->regularization = NO_REGULARIZE;
}


/**
 * Internal function for Wheelchair_acados_create: step 2
 */
ocp_nlp_dims* Wheelchair_acados_create_2_create_and_set_dimensions(Wheelchair_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 3;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);
return nlp_dims;
}


/**
 * Internal function for Wheelchair_acados_create: step 3
 */
void Wheelchair_acados_create_3_create_and_set_functions(Wheelchair_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 520); \
    }while(false)


    // constraints.constr_type == "BGH" and dims.nh > 0
    capsule->nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac[i], Wheelchair_constr_h_fun_jac_uxt_zt);
    }
    capsule->nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun[i], Wheelchair_constr_h_fun);
    }
    



    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], Wheelchair_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], Wheelchair_expl_ode_fun);
    }



#undef MAP_CASADI_FNC
}


/**
 * Internal function for Wheelchair_acados_create: step 4
 */
void Wheelchair_acados_create_4_set_default_parameters(Wheelchair_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        Wheelchair_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for Wheelchair_acados_create: step 5
 */
void Wheelchair_acados_create_5_set_nlp_in(Wheelchair_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps
    

    if (new_time_steps) {
        Wheelchair_acados_update_time_steps(capsule, N, new_time_steps);
    } else {// all time_steps are identical
        double time_step = 0.05;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);
   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 100;
    W_0[1+(NY0) * 1] = 100;
    W_0[2+(NY0) * 2] = 1000000;
    W_0[3+(NY0) * 3] = 1000;
    W_0[4+(NY0) * 4] = 1000;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 100;
    W[1+(NY) * 1] = 100;
    W[2+(NY) * 2] = 1000000;
    W[3+(NY) * 3] = 1000;
    W[4+(NY) * 4] = 1000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 100000;
    W_e[1+(NYN) * 1] = 100000;
    W_e[2+(NYN) * 2] = 100000000;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[3+(NY0) * 0] = 1;
    Vu_0[4+(NY0) * 1] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu[3+(NY) * 0] = 1;
    Vu[4+(NY) * 1] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);
    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:
    Zu[0] = 10000;
    Zu[1] = 10000;
    Zu[2] = 10000;
    Zu[3] = 10000;
    Zu[4] = 10000;
    Zu[5] = 10000;
    Zu[6] = 10000;
    Zu[7] = 10000;
    Zu[8] = 10000;
    Zu[9] = 10000;
    Zu[10] = 10000;
    Zu[11] = 10000;
    Zu[12] = 10000;
    Zu[13] = 10000;
    Zu[14] = 10000;
    Zu[15] = 10000;
    Zu[16] = 10000;
    Zu[17] = 10000;
    Zu[18] = 10000;
    Zu[19] = 10000;
    Zu[20] = 10000;
    Zu[21] = 10000;
    Zu[22] = 10000;
    Zu[23] = 10000;
    Zu[24] = 10000;
    Zu[25] = 10000;
    Zu[26] = 10000;
    Zu[27] = 10000;
    Zu[28] = 10000;
    Zu[29] = 10000;
    Zu[30] = 10000;
    Zu[31] = 10000;
    Zu[32] = 10000;
    Zu[33] = 10000;
    Zu[34] = 10000;
    Zu[35] = 10000;
    Zu[36] = 10000;
    Zu[37] = 10000;
    Zu[38] = 10000;
    Zu[39] = 10000;
    Zu[40] = 10000;
    Zu[41] = 10000;
    Zu[42] = 10000;
    Zu[43] = 10000;
    Zu[44] = 10000;
    Zu[45] = 10000;
    Zu[46] = 10000;
    Zu[47] = 10000;
    Zu[48] = 10000;
    Zu[49] = 10000;
    Zu[50] = 10000;
    Zu[51] = 10000;
    Zu[52] = 10000;
    Zu[53] = 10000;
    Zu[54] = 10000;
    Zu[55] = 10000;
    Zu[56] = 10000;
    Zu[57] = 10000;
    Zu[58] = 10000;
    Zu[59] = 10000;
    Zu[60] = 10000;
    Zu[61] = 10000;
    Zu[62] = 10000;
    Zu[63] = 10000;
    Zu[64] = 10000;
    Zu[65] = 10000;
    Zu[66] = 10000;
    Zu[67] = 10000;
    Zu[68] = 10000;
    Zu[69] = 10000;
    Zu[70] = 10000;
    Zu[71] = 10000;
    Zu[72] = 10000;
    Zu[73] = 10000;
    Zu[74] = 10000;
    Zu[75] = 10000;
    Zu[76] = 10000;
    Zu[77] = 10000;
    Zu[78] = 10000;
    Zu[79] = 10000;
    Zu[80] = 10000;
    Zu[81] = 10000;
    Zu[82] = 10000;
    Zu[83] = 10000;
    Zu[84] = 10000;
    Zu[85] = 10000;
    Zu[86] = 10000;
    Zu[87] = 10000;
    Zu[88] = 10000;
    Zu[89] = 10000;
    Zu[90] = 10000;
    Zu[91] = 10000;
    Zu[92] = 10000;
    Zu[93] = 10000;
    Zu[94] = 10000;
    Zu[95] = 10000;
    Zu[96] = 10000;
    Zu[97] = 10000;
    Zu[98] = 10000;
    Zu[99] = 10000;
    Zu[100] = 10000;
    Zu[101] = 10000;
    Zu[102] = 10000;
    Zu[103] = 10000;
    Zu[104] = 10000;
    Zu[105] = 10000;
    Zu[106] = 10000;
    Zu[107] = 10000;
    Zu[108] = 10000;
    Zu[109] = 10000;
    Zu[110] = 10000;
    Zu[111] = 10000;
    Zu[112] = 10000;
    Zu[113] = 10000;
    Zu[114] = 10000;
    Zu[115] = 10000;
    Zu[116] = 10000;
    Zu[117] = 10000;
    Zu[118] = 10000;
    Zu[119] = 10000;
    Zu[120] = 10000;
    Zu[121] = 10000;
    Zu[122] = 10000;
    Zu[123] = 10000;
    Zu[124] = 10000;
    Zu[125] = 10000;
    Zu[126] = 10000;
    Zu[127] = 10000;
    Zu[128] = 10000;
    Zu[129] = 10000;
    Zu[130] = 10000;
    Zu[131] = 10000;
    Zu[132] = 10000;
    Zu[133] = 10000;
    Zu[134] = 10000;
    Zu[135] = 10000;
    Zu[136] = 10000;
    Zu[137] = 10000;
    Zu[138] = 10000;
    Zu[139] = 10000;
    Zu[140] = 10000;
    Zu[141] = 10000;
    Zu[142] = 10000;
    Zu[143] = 10000;
    Zu[144] = 10000;
    Zu[145] = 10000;
    Zu[146] = 10000;
    Zu[147] = 10000;
    Zu[148] = 10000;
    Zu[149] = 10000;
    Zu[150] = 10000;
    Zu[151] = 10000;
    Zu[152] = 10000;
    Zu[153] = 10000;
    Zu[154] = 10000;
    Zu[155] = 10000;
    Zu[156] = 10000;
    Zu[157] = 10000;
    Zu[158] = 10000;
    Zu[159] = 10000;
    Zu[160] = 10000;
    Zu[161] = 10000;
    Zu[162] = 10000;
    Zu[163] = 10000;
    Zu[164] = 10000;
    Zu[165] = 10000;
    Zu[166] = 10000;
    Zu[167] = 10000;
    Zu[168] = 10000;
    Zu[169] = 10000;
    Zu[170] = 10000;
    Zu[171] = 10000;
    Zu[172] = 10000;
    Zu[173] = 10000;
    Zu[174] = 10000;
    Zu[175] = 10000;
    Zu[176] = 10000;
    Zu[177] = 10000;
    Zu[178] = 10000;
    Zu[179] = 10000;
    Zu[180] = 10000;
    Zu[181] = 10000;
    Zu[182] = 10000;
    Zu[183] = 10000;
    Zu[184] = 10000;
    Zu[185] = 10000;
    Zu[186] = 10000;
    Zu[187] = 10000;
    Zu[188] = 10000;
    Zu[189] = 10000;
    Zu[190] = 10000;
    Zu[191] = 10000;
    Zu[192] = 10000;
    Zu[193] = 10000;
    Zu[194] = 10000;
    Zu[195] = 10000;
    Zu[196] = 10000;
    Zu[197] = 10000;
    Zu[198] = 10000;
    Zu[199] = 10000;
    Zu[200] = 10000;
    Zu[201] = 10000;
    Zu[202] = 10000;
    Zu[203] = 10000;
    Zu[204] = 10000;
    Zu[205] = 10000;
    Zu[206] = 10000;
    Zu[207] = 10000;
    Zu[208] = 10000;
    Zu[209] = 10000;
    Zu[210] = 10000;
    Zu[211] = 10000;
    Zu[212] = 10000;
    Zu[213] = 10000;
    Zu[214] = 10000;
    Zu[215] = 10000;
    Zu[216] = 10000;
    Zu[217] = 10000;
    Zu[218] = 10000;
    Zu[219] = 10000;
    Zu[220] = 10000;
    Zu[221] = 10000;
    Zu[222] = 10000;
    Zu[223] = 10000;
    Zu[224] = 10000;
    Zu[225] = 10000;
    Zu[226] = 10000;
    Zu[227] = 10000;
    Zu[228] = 10000;
    Zu[229] = 10000;
    Zu[230] = 10000;
    Zu[231] = 10000;
    Zu[232] = 10000;
    Zu[233] = 10000;
    Zu[234] = 10000;
    Zu[235] = 10000;
    Zu[236] = 10000;
    Zu[237] = 10000;
    Zu[238] = 10000;
    Zu[239] = 10000;
    Zu[240] = 10000;
    Zu[241] = 10000;
    Zu[242] = 10000;
    Zu[243] = 10000;
    Zu[244] = 10000;
    Zu[245] = 10000;
    Zu[246] = 10000;
    Zu[247] = 10000;
    Zu[248] = 10000;
    Zu[249] = 10000;
    Zu[250] = 10000;
    Zu[251] = 10000;
    Zu[252] = 10000;
    Zu[253] = 10000;
    Zu[254] = 10000;
    Zu[255] = 10000;
    Zu[256] = 10000;
    Zu[257] = 10000;
    Zu[258] = 10000;
    Zu[259] = 10000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(3 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -4.6;
    ubu[0] = 4.6;
    lbu[1] = -4.6;
    ubu[1] = 4.6;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);




    // set up soft bounds for nonlinear constraints
    int* idxsh = malloc(NSH * sizeof(int));
    
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    idxsh[4] = 4;
    idxsh[5] = 5;
    idxsh[6] = 6;
    idxsh[7] = 7;
    idxsh[8] = 8;
    idxsh[9] = 9;
    idxsh[10] = 10;
    idxsh[11] = 11;
    idxsh[12] = 12;
    idxsh[13] = 13;
    idxsh[14] = 14;
    idxsh[15] = 15;
    idxsh[16] = 16;
    idxsh[17] = 17;
    idxsh[18] = 18;
    idxsh[19] = 19;
    idxsh[20] = 20;
    idxsh[21] = 21;
    idxsh[22] = 22;
    idxsh[23] = 23;
    idxsh[24] = 24;
    idxsh[25] = 25;
    idxsh[26] = 26;
    idxsh[27] = 27;
    idxsh[28] = 28;
    idxsh[29] = 29;
    idxsh[30] = 30;
    idxsh[31] = 31;
    idxsh[32] = 32;
    idxsh[33] = 33;
    idxsh[34] = 34;
    idxsh[35] = 35;
    idxsh[36] = 36;
    idxsh[37] = 37;
    idxsh[38] = 38;
    idxsh[39] = 39;
    idxsh[40] = 40;
    idxsh[41] = 41;
    idxsh[42] = 42;
    idxsh[43] = 43;
    idxsh[44] = 44;
    idxsh[45] = 45;
    idxsh[46] = 46;
    idxsh[47] = 47;
    idxsh[48] = 48;
    idxsh[49] = 49;
    idxsh[50] = 50;
    idxsh[51] = 51;
    idxsh[52] = 52;
    idxsh[53] = 53;
    idxsh[54] = 54;
    idxsh[55] = 55;
    idxsh[56] = 56;
    idxsh[57] = 57;
    idxsh[58] = 58;
    idxsh[59] = 59;
    idxsh[60] = 60;
    idxsh[61] = 61;
    idxsh[62] = 62;
    idxsh[63] = 63;
    idxsh[64] = 64;
    idxsh[65] = 65;
    idxsh[66] = 66;
    idxsh[67] = 67;
    idxsh[68] = 68;
    idxsh[69] = 69;
    idxsh[70] = 70;
    idxsh[71] = 71;
    idxsh[72] = 72;
    idxsh[73] = 73;
    idxsh[74] = 74;
    idxsh[75] = 75;
    idxsh[76] = 76;
    idxsh[77] = 77;
    idxsh[78] = 78;
    idxsh[79] = 79;
    idxsh[80] = 80;
    idxsh[81] = 81;
    idxsh[82] = 82;
    idxsh[83] = 83;
    idxsh[84] = 84;
    idxsh[85] = 85;
    idxsh[86] = 86;
    idxsh[87] = 87;
    idxsh[88] = 88;
    idxsh[89] = 89;
    idxsh[90] = 90;
    idxsh[91] = 91;
    idxsh[92] = 92;
    idxsh[93] = 93;
    idxsh[94] = 94;
    idxsh[95] = 95;
    idxsh[96] = 96;
    idxsh[97] = 97;
    idxsh[98] = 98;
    idxsh[99] = 99;
    idxsh[100] = 100;
    idxsh[101] = 101;
    idxsh[102] = 102;
    idxsh[103] = 103;
    idxsh[104] = 104;
    idxsh[105] = 105;
    idxsh[106] = 106;
    idxsh[107] = 107;
    idxsh[108] = 108;
    idxsh[109] = 109;
    idxsh[110] = 110;
    idxsh[111] = 111;
    idxsh[112] = 112;
    idxsh[113] = 113;
    idxsh[114] = 114;
    idxsh[115] = 115;
    idxsh[116] = 116;
    idxsh[117] = 117;
    idxsh[118] = 118;
    idxsh[119] = 119;
    idxsh[120] = 120;
    idxsh[121] = 121;
    idxsh[122] = 122;
    idxsh[123] = 123;
    idxsh[124] = 124;
    idxsh[125] = 125;
    idxsh[126] = 126;
    idxsh[127] = 127;
    idxsh[128] = 128;
    idxsh[129] = 129;
    idxsh[130] = 130;
    idxsh[131] = 131;
    idxsh[132] = 132;
    idxsh[133] = 133;
    idxsh[134] = 134;
    idxsh[135] = 135;
    idxsh[136] = 136;
    idxsh[137] = 137;
    idxsh[138] = 138;
    idxsh[139] = 139;
    idxsh[140] = 140;
    idxsh[141] = 141;
    idxsh[142] = 142;
    idxsh[143] = 143;
    idxsh[144] = 144;
    idxsh[145] = 145;
    idxsh[146] = 146;
    idxsh[147] = 147;
    idxsh[148] = 148;
    idxsh[149] = 149;
    idxsh[150] = 150;
    idxsh[151] = 151;
    idxsh[152] = 152;
    idxsh[153] = 153;
    idxsh[154] = 154;
    idxsh[155] = 155;
    idxsh[156] = 156;
    idxsh[157] = 157;
    idxsh[158] = 158;
    idxsh[159] = 159;
    idxsh[160] = 160;
    idxsh[161] = 161;
    idxsh[162] = 162;
    idxsh[163] = 163;
    idxsh[164] = 164;
    idxsh[165] = 165;
    idxsh[166] = 166;
    idxsh[167] = 167;
    idxsh[168] = 168;
    idxsh[169] = 169;
    idxsh[170] = 170;
    idxsh[171] = 171;
    idxsh[172] = 172;
    idxsh[173] = 173;
    idxsh[174] = 174;
    idxsh[175] = 175;
    idxsh[176] = 176;
    idxsh[177] = 177;
    idxsh[178] = 178;
    idxsh[179] = 179;
    idxsh[180] = 180;
    idxsh[181] = 181;
    idxsh[182] = 182;
    idxsh[183] = 183;
    idxsh[184] = 184;
    idxsh[185] = 185;
    idxsh[186] = 186;
    idxsh[187] = 187;
    idxsh[188] = 188;
    idxsh[189] = 189;
    idxsh[190] = 190;
    idxsh[191] = 191;
    idxsh[192] = 192;
    idxsh[193] = 193;
    idxsh[194] = 194;
    idxsh[195] = 195;
    idxsh[196] = 196;
    idxsh[197] = 197;
    idxsh[198] = 198;
    idxsh[199] = 199;
    idxsh[200] = 200;
    idxsh[201] = 201;
    idxsh[202] = 202;
    idxsh[203] = 203;
    idxsh[204] = 204;
    idxsh[205] = 205;
    idxsh[206] = 206;
    idxsh[207] = 207;
    idxsh[208] = 208;
    idxsh[209] = 209;
    idxsh[210] = 210;
    idxsh[211] = 211;
    idxsh[212] = 212;
    idxsh[213] = 213;
    idxsh[214] = 214;
    idxsh[215] = 215;
    idxsh[216] = 216;
    idxsh[217] = 217;
    idxsh[218] = 218;
    idxsh[219] = 219;
    idxsh[220] = 220;
    idxsh[221] = 221;
    idxsh[222] = 222;
    idxsh[223] = 223;
    idxsh[224] = 224;
    idxsh[225] = 225;
    idxsh[226] = 226;
    idxsh[227] = 227;
    idxsh[228] = 228;
    idxsh[229] = 229;
    idxsh[230] = 230;
    idxsh[231] = 231;
    idxsh[232] = 232;
    idxsh[233] = 233;
    idxsh[234] = 234;
    idxsh[235] = 235;
    idxsh[236] = 236;
    idxsh[237] = 237;
    idxsh[238] = 238;
    idxsh[239] = 239;
    idxsh[240] = 240;
    idxsh[241] = 241;
    idxsh[242] = 242;
    idxsh[243] = 243;
    idxsh[244] = 244;
    idxsh[245] = 245;
    idxsh[246] = 246;
    idxsh[247] = 247;
    idxsh[248] = 248;
    idxsh[249] = 249;
    idxsh[250] = 250;
    idxsh[251] = 251;
    idxsh[252] = 252;
    idxsh[253] = 253;
    idxsh[254] = 254;
    idxsh[255] = 255;
    idxsh[256] = 256;
    idxsh[257] = 257;
    idxsh[258] = 258;
    idxsh[259] = 259;
    double* lush = calloc(2*NSH, sizeof(double));
    double* lsh = lush;
    double* ush = lush + NSH;
    

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ush", ush);
    }
    free(idxsh);
    free(lush);




    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 0;
    idxbx[1] = 1;
    idxbx[2] = 2;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -10;
    ubx[0] = 10;
    lbx[1] = -10;
    ubx[1] = 10;
    lbx[2] = -18.84955592;
    ubx[2] = 18.84955592;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);




    // set up nonlinear constraints for stage 0 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;

    
    lh[0] = -10000000;
    lh[1] = -10000000;
    lh[2] = -10000000;
    lh[3] = -10000000;
    lh[4] = -10000000;
    lh[5] = -10000000;
    lh[6] = -10000000;
    lh[7] = -10000000;
    lh[8] = -10000000;
    lh[9] = -10000000;
    lh[10] = -10000000;
    lh[11] = -10000000;
    lh[12] = -10000000;
    lh[13] = -10000000;
    lh[14] = -10000000;
    lh[15] = -10000000;
    lh[16] = -10000000;
    lh[17] = -10000000;
    lh[18] = -10000000;
    lh[19] = -10000000;
    lh[20] = -10000000;
    lh[21] = -10000000;
    lh[22] = -10000000;
    lh[23] = -10000000;
    lh[24] = -10000000;
    lh[25] = -10000000;
    lh[26] = -10000000;
    lh[27] = -10000000;
    lh[28] = -10000000;
    lh[29] = -10000000;
    lh[30] = -10000000;
    lh[31] = -10000000;
    lh[32] = -10000000;
    lh[33] = -10000000;
    lh[34] = -10000000;
    lh[35] = -10000000;
    lh[36] = -10000000;
    lh[37] = -10000000;
    lh[38] = -10000000;
    lh[39] = -10000000;
    lh[40] = -10000000;
    lh[41] = -10000000;
    lh[42] = -10000000;
    lh[43] = -10000000;
    lh[44] = -10000000;
    lh[45] = -10000000;
    lh[46] = -10000000;
    lh[47] = -10000000;
    lh[48] = -10000000;
    lh[49] = -10000000;
    lh[50] = -10000000;
    lh[51] = -10000000;
    lh[52] = -10000000;
    lh[53] = -10000000;
    lh[54] = -10000000;
    lh[55] = -10000000;
    lh[56] = -10000000;
    lh[57] = -10000000;
    lh[58] = -10000000;
    lh[59] = -10000000;
    lh[60] = -10000000;
    lh[61] = -10000000;
    lh[62] = -10000000;
    lh[63] = -10000000;
    lh[64] = -10000000;
    lh[65] = -10000000;
    lh[66] = -10000000;
    lh[67] = -10000000;
    lh[68] = -10000000;
    lh[69] = -10000000;
    lh[70] = -10000000;
    lh[71] = -10000000;
    lh[72] = -10000000;
    lh[73] = -10000000;
    lh[74] = -10000000;
    lh[75] = -10000000;
    lh[76] = -10000000;
    lh[77] = -10000000;
    lh[78] = -10000000;
    lh[79] = -10000000;
    lh[80] = -10000000;
    lh[81] = -10000000;
    lh[82] = -10000000;
    lh[83] = -10000000;
    lh[84] = -10000000;
    lh[85] = -10000000;
    lh[86] = -10000000;
    lh[87] = -10000000;
    lh[88] = -10000000;
    lh[89] = -10000000;
    lh[90] = -10000000;
    lh[91] = -10000000;
    lh[92] = -10000000;
    lh[93] = -10000000;
    lh[94] = -10000000;
    lh[95] = -10000000;
    lh[96] = -10000000;
    lh[97] = -10000000;
    lh[98] = -10000000;
    lh[99] = -10000000;
    lh[100] = -10000000;
    lh[101] = -10000000;
    lh[102] = -10000000;
    lh[103] = -10000000;
    lh[104] = -10000000;
    lh[105] = -10000000;
    lh[106] = -10000000;
    lh[107] = -10000000;
    lh[108] = -10000000;
    lh[109] = -10000000;
    lh[110] = -10000000;
    lh[111] = -10000000;
    lh[112] = -10000000;
    lh[113] = -10000000;
    lh[114] = -10000000;
    lh[115] = -10000000;
    lh[116] = -10000000;
    lh[117] = -10000000;
    lh[118] = -10000000;
    lh[119] = -10000000;
    lh[120] = -10000000;
    lh[121] = -10000000;
    lh[122] = -10000000;
    lh[123] = -10000000;
    lh[124] = -10000000;
    lh[125] = -10000000;
    lh[126] = -10000000;
    lh[127] = -10000000;
    lh[128] = -10000000;
    lh[129] = -10000000;
    lh[130] = -10000000;
    lh[131] = -10000000;
    lh[132] = -10000000;
    lh[133] = -10000000;
    lh[134] = -10000000;
    lh[135] = -10000000;
    lh[136] = -10000000;
    lh[137] = -10000000;
    lh[138] = -10000000;
    lh[139] = -10000000;
    lh[140] = -10000000;
    lh[141] = -10000000;
    lh[142] = -10000000;
    lh[143] = -10000000;
    lh[144] = -10000000;
    lh[145] = -10000000;
    lh[146] = -10000000;
    lh[147] = -10000000;
    lh[148] = -10000000;
    lh[149] = -10000000;
    lh[150] = -10000000;
    lh[151] = -10000000;
    lh[152] = -10000000;
    lh[153] = -10000000;
    lh[154] = -10000000;
    lh[155] = -10000000;
    lh[156] = -10000000;
    lh[157] = -10000000;
    lh[158] = -10000000;
    lh[159] = -10000000;
    lh[160] = -10000000;
    lh[161] = -10000000;
    lh[162] = -10000000;
    lh[163] = -10000000;
    lh[164] = -10000000;
    lh[165] = -10000000;
    lh[166] = -10000000;
    lh[167] = -10000000;
    lh[168] = -10000000;
    lh[169] = -10000000;
    lh[170] = -10000000;
    lh[171] = -10000000;
    lh[172] = -10000000;
    lh[173] = -10000000;
    lh[174] = -10000000;
    lh[175] = -10000000;
    lh[176] = -10000000;
    lh[177] = -10000000;
    lh[178] = -10000000;
    lh[179] = -10000000;
    lh[180] = -10000000;
    lh[181] = -10000000;
    lh[182] = -10000000;
    lh[183] = -10000000;
    lh[184] = -10000000;
    lh[185] = -10000000;
    lh[186] = -10000000;
    lh[187] = -10000000;
    lh[188] = -10000000;
    lh[189] = -10000000;
    lh[190] = -10000000;
    lh[191] = -10000000;
    lh[192] = -10000000;
    lh[193] = -10000000;
    lh[194] = -10000000;
    lh[195] = -10000000;
    lh[196] = -10000000;
    lh[197] = -10000000;
    lh[198] = -10000000;
    lh[199] = -10000000;
    lh[200] = -10000000;
    lh[201] = -10000000;
    lh[202] = -10000000;
    lh[203] = -10000000;
    lh[204] = -10000000;
    lh[205] = -10000000;
    lh[206] = -10000000;
    lh[207] = -10000000;
    lh[208] = -10000000;
    lh[209] = -10000000;
    lh[210] = -10000000;
    lh[211] = -10000000;
    lh[212] = -10000000;
    lh[213] = -10000000;
    lh[214] = -10000000;
    lh[215] = -10000000;
    lh[216] = -10000000;
    lh[217] = -10000000;
    lh[218] = -10000000;
    lh[219] = -10000000;
    lh[220] = -10000000;
    lh[221] = -10000000;
    lh[222] = -10000000;
    lh[223] = -10000000;
    lh[224] = -10000000;
    lh[225] = -10000000;
    lh[226] = -10000000;
    lh[227] = -10000000;
    lh[228] = -10000000;
    lh[229] = -10000000;
    lh[230] = -10000000;
    lh[231] = -10000000;
    lh[232] = -10000000;
    lh[233] = -10000000;
    lh[234] = -10000000;
    lh[235] = -10000000;
    lh[236] = -10000000;
    lh[237] = -10000000;
    lh[238] = -10000000;
    lh[239] = -10000000;
    lh[240] = -10000000;
    lh[241] = -10000000;
    lh[242] = -10000000;
    lh[243] = -10000000;
    lh[244] = -10000000;
    lh[245] = -10000000;
    lh[246] = -10000000;
    lh[247] = -10000000;
    lh[248] = -10000000;
    lh[249] = -10000000;
    lh[250] = -10000000;
    lh[251] = -10000000;
    lh[252] = -10000000;
    lh[253] = -10000000;
    lh[254] = -10000000;
    lh[255] = -10000000;
    lh[256] = -10000000;
    lh[257] = -10000000;
    lh[258] = -10000000;
    lh[259] = -10000000;

    

    for (int i = 0; i < N; i++)
    {
        // nonlinear constraints for stages 0 to N-1
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }
    free(luh);



    /* terminal constraints */

    // set up bounds for last stage
    // x
    int* idxbx_e = malloc(NBXN * sizeof(int));
    
    idxbx_e[0] = 0;
    idxbx_e[1] = 1;
    idxbx_e[2] = 2;
    double* lubx_e = calloc(2*NBXN, sizeof(double));
    double* lbx_e = lubx_e;
    double* ubx_e = lubx_e + NBXN;
    
    lbx_e[0] = -10;
    ubx_e[0] = 10;
    lbx_e[1] = -10;
    ubx_e[1] = 10;
    lbx_e[2] = -18.84955592;
    ubx_e[2] = 18.84955592;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", ubx_e);
    free(idxbx_e);
    free(lubx_e);














}


/**
 * Internal function for Wheelchair_acados_create: step 6
 */
void Wheelchair_acados_create_6_set_opts(Wheelchair_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);


    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int nlp_solver_ext_qp_res = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    int qp_solver_warm_start = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_warm_start", &qp_solver_warm_start);int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    int ext_cost_num_hess = 0;
}


/**
 * Internal function for Wheelchair_acados_create: step 7
 */
void Wheelchair_acados_create_7_set_nlp_out(Wheelchair_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for Wheelchair_acados_create: step 8
 */
//void Wheelchair_acados_create_8_create_solver(Wheelchair_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for Wheelchair_acados_create: step 9
 */
int Wheelchair_acados_create_9_precompute(Wheelchair_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int Wheelchair_acados_create_with_discretization(Wheelchair_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != WHEELCHAIR_N && !new_time_steps) {
        fprintf(stderr, "Wheelchair_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, WHEELCHAIR_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    Wheelchair_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = Wheelchair_acados_create_2_create_and_set_dimensions(capsule);
    Wheelchair_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    Wheelchair_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    Wheelchair_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    Wheelchair_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    Wheelchair_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //Wheelchair_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = Wheelchair_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int Wheelchair_acados_update_qp_solver_cond_N(Wheelchair_solver_capsule* capsule, int qp_solver_cond_N)
{
    printf("\nacados_update_qp_solver_cond_N() failed, since no partial condensing solver is used!\n\n");
    // Todo: what is an adequate behavior here?
    exit(1);
    return -1;
}


int Wheelchair_acados_reset(Wheelchair_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }

    free(buffer);
    return 0;
}




int Wheelchair_acados_update_params(Wheelchair_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 520;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi+stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun+stage, p);
    

        // constraints
    
        capsule->nl_constr_h_fun_jac[stage].set_param(capsule->nl_constr_h_fun_jac+stage, p);
        capsule->nl_constr_h_fun[stage].set_param(capsule->nl_constr_h_fun+stage, p);

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    
    }

    return solver_status;
}


int Wheelchair_acados_update_params_sparse(Wheelchair_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 520;
    if (casadi_np < n_update) {
        printf("Wheelchair_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("Wheelchair_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi+stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun+stage, n_update, idx, p);
    

        // constraints
    
        capsule->nl_constr_h_fun_jac[stage].set_param_sparse(capsule->nl_constr_h_fun_jac+stage, n_update, idx, p);
        capsule->nl_constr_h_fun[stage].set_param_sparse(capsule->nl_constr_h_fun+stage, n_update, idx, p);

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    
    }


    return solver_status;
}

int Wheelchair_acados_solve(Wheelchair_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int Wheelchair_acados_free(Wheelchair_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost

    // constraints
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);

    return 0;
}


void Wheelchair_acados_print_stats(Wheelchair_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1200];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int Wheelchair_acados_custom_update(Wheelchair_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *Wheelchair_acados_get_nlp_in(Wheelchair_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *Wheelchair_acados_get_nlp_out(Wheelchair_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *Wheelchair_acados_get_sens_out(Wheelchair_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *Wheelchair_acados_get_nlp_solver(Wheelchair_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *Wheelchair_acados_get_nlp_config(Wheelchair_solver_capsule* capsule) { return capsule->nlp_config; }
void *Wheelchair_acados_get_nlp_opts(Wheelchair_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *Wheelchair_acados_get_nlp_dims(Wheelchair_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *Wheelchair_acados_get_nlp_plan(Wheelchair_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
