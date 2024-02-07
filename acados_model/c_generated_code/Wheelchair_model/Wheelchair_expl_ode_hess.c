/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Wheelchair_expl_ode_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s2[11] = {3, 2, 0, 3, 6, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[524] = {520, 1, 0, 520, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519};
static const casadi_int casadi_s5[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s6[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

/* Wheelchair_expl_ode_hess:(i0[3],i1[3x3],i2[3x2],i3[3],i4[2],i5[520])->(o0[5],o1[15]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][2] : 0;
  a1=cos(a0);
  a2=5.0000000000000003e-02;
  a3=arg[4]? arg[4][1] : 0;
  a4=arg[3]? arg[3][1] : 0;
  a5=(a3*a4);
  a5=(a2*a5);
  a6=(a1*a5);
  a7=cos(a0);
  a8=arg[4]? arg[4][0] : 0;
  a9=(a8*a4);
  a9=(a2*a9);
  a10=(a7*a9);
  a6=(a6+a10);
  a10=sin(a0);
  a11=arg[3]? arg[3][0] : 0;
  a3=(a3*a11);
  a3=(a2*a3);
  a12=(a10*a3);
  a6=(a6-a12);
  a12=sin(a0);
  a8=(a8*a11);
  a8=(a2*a8);
  a13=(a12*a8);
  a6=(a6-a13);
  if (res[0]!=0) res[0][2]=a6;
  a6=7.3778958241109638e-02;
  a13=arg[3]? arg[3][2] : 0;
  a6=(a6*a13);
  a14=sin(a0);
  a14=(a2*a14);
  a14=(a14*a4);
  a6=(a6+a14);
  a14=cos(a0);
  a14=(a2*a14);
  a14=(a14*a11);
  a6=(a6+a14);
  if (res[0]!=0) res[0][3]=a6;
  a6=-7.3778958241109638e-02;
  a6=(a6*a13);
  a13=sin(a0);
  a13=(a2*a13);
  a13=(a13*a4);
  a6=(a6+a13);
  a13=cos(a0);
  a13=(a2*a13);
  a13=(a13*a11);
  a6=(a6+a13);
  if (res[0]!=0) res[0][4]=a6;
  a6=arg[1]? arg[1][2] : 0;
  a13=sin(a0);
  a14=(a13*a6);
  a14=(a5*a14);
  a15=sin(a0);
  a16=(a15*a6);
  a16=(a9*a16);
  a14=(a14+a16);
  a16=cos(a0);
  a17=(a16*a6);
  a17=(a3*a17);
  a14=(a14+a17);
  a17=cos(a0);
  a18=(a17*a6);
  a18=(a8*a18);
  a14=(a14+a18);
  a18=(a6*a14);
  a18=(-a18);
  if (res[1]!=0) res[1][0]=a18;
  a18=arg[1]? arg[1][5] : 0;
  a19=(a18*a14);
  a19=(-a19);
  if (res[1]!=0) res[1][1]=a19;
  a19=arg[1]? arg[1][8] : 0;
  a20=(a19*a14);
  a20=(-a20);
  if (res[1]!=0) res[1][2]=a20;
  a20=cos(a0);
  a21=(a20*a6);
  a21=(a2*a21);
  a21=(a4*a21);
  a22=sin(a0);
  a23=(a22*a6);
  a23=(a2*a23);
  a23=(a11*a23);
  a21=(a21-a23);
  a23=arg[2]? arg[2][2] : 0;
  a24=(a23*a14);
  a21=(a21-a24);
  if (res[1]!=0) res[1][3]=a21;
  a21=cos(a0);
  a24=(a21*a6);
  a24=(a2*a24);
  a24=(a4*a24);
  a0=sin(a0);
  a6=(a0*a6);
  a6=(a2*a6);
  a6=(a11*a6);
  a24=(a24-a6);
  a6=arg[2]? arg[2][5] : 0;
  a14=(a6*a14);
  a24=(a24-a14);
  if (res[1]!=0) res[1][4]=a24;
  a24=(a13*a18);
  a24=(a5*a24);
  a14=(a15*a18);
  a14=(a9*a14);
  a24=(a24+a14);
  a14=(a16*a18);
  a14=(a3*a14);
  a24=(a24+a14);
  a14=(a17*a18);
  a14=(a8*a14);
  a24=(a24+a14);
  a14=(a18*a24);
  a14=(-a14);
  if (res[1]!=0) res[1][5]=a14;
  a14=(a19*a24);
  a14=(-a14);
  if (res[1]!=0) res[1][6]=a14;
  a14=(a20*a18);
  a14=(a2*a14);
  a14=(a4*a14);
  a25=(a22*a18);
  a25=(a2*a25);
  a25=(a11*a25);
  a14=(a14-a25);
  a25=(a23*a24);
  a14=(a14-a25);
  if (res[1]!=0) res[1][7]=a14;
  a14=(a21*a18);
  a14=(a2*a14);
  a14=(a4*a14);
  a18=(a0*a18);
  a18=(a2*a18);
  a18=(a11*a18);
  a14=(a14-a18);
  a24=(a6*a24);
  a14=(a14-a24);
  if (res[1]!=0) res[1][8]=a14;
  a14=(a13*a19);
  a14=(a5*a14);
  a24=(a15*a19);
  a24=(a9*a24);
  a14=(a14+a24);
  a24=(a16*a19);
  a24=(a3*a24);
  a14=(a14+a24);
  a24=(a17*a19);
  a24=(a8*a24);
  a14=(a14+a24);
  a24=(a19*a14);
  a24=(-a24);
  if (res[1]!=0) res[1][9]=a24;
  a24=(a20*a19);
  a24=(a2*a24);
  a24=(a4*a24);
  a18=(a22*a19);
  a18=(a2*a18);
  a18=(a11*a18);
  a24=(a24-a18);
  a18=(a23*a14);
  a24=(a24-a18);
  if (res[1]!=0) res[1][10]=a24;
  a24=(a21*a19);
  a24=(a2*a24);
  a24=(a4*a24);
  a19=(a0*a19);
  a19=(a2*a19);
  a19=(a11*a19);
  a24=(a24-a19);
  a14=(a6*a14);
  a24=(a24-a14);
  if (res[1]!=0) res[1][11]=a24;
  a24=(a2*a4);
  a7=(a7*a24);
  a24=(a15*a23);
  a24=(a9*a24);
  a7=(a7-a24);
  a24=(a13*a23);
  a24=(a5*a24);
  a7=(a7-a24);
  a24=(a16*a23);
  a24=(a3*a24);
  a7=(a7-a24);
  a24=(a17*a23);
  a24=(a8*a24);
  a14=(a2*a11);
  a12=(a12*a14);
  a24=(a24+a12);
  a7=(a7-a24);
  a24=(a23*a7);
  a20=(a20*a23);
  a20=(a2*a20);
  a20=(a4*a20);
  a22=(a22*a23);
  a22=(a2*a22);
  a22=(a11*a22);
  a20=(a20-a22);
  a24=(a24+a20);
  if (res[1]!=0) res[1][12]=a24;
  a7=(a6*a7);
  a24=(a21*a23);
  a24=(a2*a24);
  a24=(a4*a24);
  a23=(a0*a23);
  a23=(a2*a23);
  a23=(a11*a23);
  a24=(a24-a23);
  a7=(a7+a24);
  if (res[1]!=0) res[1][13]=a7;
  a7=(a2*a4);
  a1=(a1*a7);
  a13=(a13*a6);
  a5=(a5*a13);
  a1=(a1-a5);
  a15=(a15*a6);
  a9=(a9*a15);
  a1=(a1-a9);
  a16=(a16*a6);
  a3=(a3*a16);
  a16=(a2*a11);
  a10=(a10*a16);
  a3=(a3+a10);
  a1=(a1-a3);
  a17=(a17*a6);
  a8=(a8*a17);
  a1=(a1-a8);
  a1=(a6*a1);
  a21=(a21*a6);
  a21=(a2*a21);
  a4=(a4*a21);
  a0=(a0*a6);
  a2=(a2*a0);
  a11=(a11*a2);
  a4=(a4-a11);
  a1=(a1+a4);
  if (res[1]!=0) res[1][14]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int Wheelchair_expl_ode_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Wheelchair_expl_ode_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Wheelchair_expl_ode_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Wheelchair_expl_ode_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Wheelchair_expl_ode_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Wheelchair_expl_ode_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Wheelchair_expl_ode_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void Wheelchair_expl_ode_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Wheelchair_expl_ode_hess_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int Wheelchair_expl_ode_hess_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real Wheelchair_expl_ode_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Wheelchair_expl_ode_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Wheelchair_expl_ode_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Wheelchair_expl_ode_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Wheelchair_expl_ode_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Wheelchair_expl_ode_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif