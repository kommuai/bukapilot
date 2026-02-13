#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3310266499749110268) {
   out_3310266499749110268[0] = delta_x[0] + nom_x[0];
   out_3310266499749110268[1] = delta_x[1] + nom_x[1];
   out_3310266499749110268[2] = delta_x[2] + nom_x[2];
   out_3310266499749110268[3] = delta_x[3] + nom_x[3];
   out_3310266499749110268[4] = delta_x[4] + nom_x[4];
   out_3310266499749110268[5] = delta_x[5] + nom_x[5];
   out_3310266499749110268[6] = delta_x[6] + nom_x[6];
   out_3310266499749110268[7] = delta_x[7] + nom_x[7];
   out_3310266499749110268[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9111078211151530773) {
   out_9111078211151530773[0] = -nom_x[0] + true_x[0];
   out_9111078211151530773[1] = -nom_x[1] + true_x[1];
   out_9111078211151530773[2] = -nom_x[2] + true_x[2];
   out_9111078211151530773[3] = -nom_x[3] + true_x[3];
   out_9111078211151530773[4] = -nom_x[4] + true_x[4];
   out_9111078211151530773[5] = -nom_x[5] + true_x[5];
   out_9111078211151530773[6] = -nom_x[6] + true_x[6];
   out_9111078211151530773[7] = -nom_x[7] + true_x[7];
   out_9111078211151530773[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8318288169772448250) {
   out_8318288169772448250[0] = 1.0;
   out_8318288169772448250[1] = 0.0;
   out_8318288169772448250[2] = 0.0;
   out_8318288169772448250[3] = 0.0;
   out_8318288169772448250[4] = 0.0;
   out_8318288169772448250[5] = 0.0;
   out_8318288169772448250[6] = 0.0;
   out_8318288169772448250[7] = 0.0;
   out_8318288169772448250[8] = 0.0;
   out_8318288169772448250[9] = 0.0;
   out_8318288169772448250[10] = 1.0;
   out_8318288169772448250[11] = 0.0;
   out_8318288169772448250[12] = 0.0;
   out_8318288169772448250[13] = 0.0;
   out_8318288169772448250[14] = 0.0;
   out_8318288169772448250[15] = 0.0;
   out_8318288169772448250[16] = 0.0;
   out_8318288169772448250[17] = 0.0;
   out_8318288169772448250[18] = 0.0;
   out_8318288169772448250[19] = 0.0;
   out_8318288169772448250[20] = 1.0;
   out_8318288169772448250[21] = 0.0;
   out_8318288169772448250[22] = 0.0;
   out_8318288169772448250[23] = 0.0;
   out_8318288169772448250[24] = 0.0;
   out_8318288169772448250[25] = 0.0;
   out_8318288169772448250[26] = 0.0;
   out_8318288169772448250[27] = 0.0;
   out_8318288169772448250[28] = 0.0;
   out_8318288169772448250[29] = 0.0;
   out_8318288169772448250[30] = 1.0;
   out_8318288169772448250[31] = 0.0;
   out_8318288169772448250[32] = 0.0;
   out_8318288169772448250[33] = 0.0;
   out_8318288169772448250[34] = 0.0;
   out_8318288169772448250[35] = 0.0;
   out_8318288169772448250[36] = 0.0;
   out_8318288169772448250[37] = 0.0;
   out_8318288169772448250[38] = 0.0;
   out_8318288169772448250[39] = 0.0;
   out_8318288169772448250[40] = 1.0;
   out_8318288169772448250[41] = 0.0;
   out_8318288169772448250[42] = 0.0;
   out_8318288169772448250[43] = 0.0;
   out_8318288169772448250[44] = 0.0;
   out_8318288169772448250[45] = 0.0;
   out_8318288169772448250[46] = 0.0;
   out_8318288169772448250[47] = 0.0;
   out_8318288169772448250[48] = 0.0;
   out_8318288169772448250[49] = 0.0;
   out_8318288169772448250[50] = 1.0;
   out_8318288169772448250[51] = 0.0;
   out_8318288169772448250[52] = 0.0;
   out_8318288169772448250[53] = 0.0;
   out_8318288169772448250[54] = 0.0;
   out_8318288169772448250[55] = 0.0;
   out_8318288169772448250[56] = 0.0;
   out_8318288169772448250[57] = 0.0;
   out_8318288169772448250[58] = 0.0;
   out_8318288169772448250[59] = 0.0;
   out_8318288169772448250[60] = 1.0;
   out_8318288169772448250[61] = 0.0;
   out_8318288169772448250[62] = 0.0;
   out_8318288169772448250[63] = 0.0;
   out_8318288169772448250[64] = 0.0;
   out_8318288169772448250[65] = 0.0;
   out_8318288169772448250[66] = 0.0;
   out_8318288169772448250[67] = 0.0;
   out_8318288169772448250[68] = 0.0;
   out_8318288169772448250[69] = 0.0;
   out_8318288169772448250[70] = 1.0;
   out_8318288169772448250[71] = 0.0;
   out_8318288169772448250[72] = 0.0;
   out_8318288169772448250[73] = 0.0;
   out_8318288169772448250[74] = 0.0;
   out_8318288169772448250[75] = 0.0;
   out_8318288169772448250[76] = 0.0;
   out_8318288169772448250[77] = 0.0;
   out_8318288169772448250[78] = 0.0;
   out_8318288169772448250[79] = 0.0;
   out_8318288169772448250[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3923135191490236514) {
   out_3923135191490236514[0] = state[0];
   out_3923135191490236514[1] = state[1];
   out_3923135191490236514[2] = state[2];
   out_3923135191490236514[3] = state[3];
   out_3923135191490236514[4] = state[4];
   out_3923135191490236514[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3923135191490236514[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3923135191490236514[7] = state[7];
   out_3923135191490236514[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2349988940558973348) {
   out_2349988940558973348[0] = 1;
   out_2349988940558973348[1] = 0;
   out_2349988940558973348[2] = 0;
   out_2349988940558973348[3] = 0;
   out_2349988940558973348[4] = 0;
   out_2349988940558973348[5] = 0;
   out_2349988940558973348[6] = 0;
   out_2349988940558973348[7] = 0;
   out_2349988940558973348[8] = 0;
   out_2349988940558973348[9] = 0;
   out_2349988940558973348[10] = 1;
   out_2349988940558973348[11] = 0;
   out_2349988940558973348[12] = 0;
   out_2349988940558973348[13] = 0;
   out_2349988940558973348[14] = 0;
   out_2349988940558973348[15] = 0;
   out_2349988940558973348[16] = 0;
   out_2349988940558973348[17] = 0;
   out_2349988940558973348[18] = 0;
   out_2349988940558973348[19] = 0;
   out_2349988940558973348[20] = 1;
   out_2349988940558973348[21] = 0;
   out_2349988940558973348[22] = 0;
   out_2349988940558973348[23] = 0;
   out_2349988940558973348[24] = 0;
   out_2349988940558973348[25] = 0;
   out_2349988940558973348[26] = 0;
   out_2349988940558973348[27] = 0;
   out_2349988940558973348[28] = 0;
   out_2349988940558973348[29] = 0;
   out_2349988940558973348[30] = 1;
   out_2349988940558973348[31] = 0;
   out_2349988940558973348[32] = 0;
   out_2349988940558973348[33] = 0;
   out_2349988940558973348[34] = 0;
   out_2349988940558973348[35] = 0;
   out_2349988940558973348[36] = 0;
   out_2349988940558973348[37] = 0;
   out_2349988940558973348[38] = 0;
   out_2349988940558973348[39] = 0;
   out_2349988940558973348[40] = 1;
   out_2349988940558973348[41] = 0;
   out_2349988940558973348[42] = 0;
   out_2349988940558973348[43] = 0;
   out_2349988940558973348[44] = 0;
   out_2349988940558973348[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2349988940558973348[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2349988940558973348[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2349988940558973348[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2349988940558973348[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2349988940558973348[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2349988940558973348[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2349988940558973348[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2349988940558973348[53] = -9.8000000000000007*dt;
   out_2349988940558973348[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2349988940558973348[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2349988940558973348[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2349988940558973348[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2349988940558973348[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2349988940558973348[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2349988940558973348[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2349988940558973348[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2349988940558973348[62] = 0;
   out_2349988940558973348[63] = 0;
   out_2349988940558973348[64] = 0;
   out_2349988940558973348[65] = 0;
   out_2349988940558973348[66] = 0;
   out_2349988940558973348[67] = 0;
   out_2349988940558973348[68] = 0;
   out_2349988940558973348[69] = 0;
   out_2349988940558973348[70] = 1;
   out_2349988940558973348[71] = 0;
   out_2349988940558973348[72] = 0;
   out_2349988940558973348[73] = 0;
   out_2349988940558973348[74] = 0;
   out_2349988940558973348[75] = 0;
   out_2349988940558973348[76] = 0;
   out_2349988940558973348[77] = 0;
   out_2349988940558973348[78] = 0;
   out_2349988940558973348[79] = 0;
   out_2349988940558973348[80] = 1;
}
void h_25(double *state, double *unused, double *out_3861404861143990202) {
   out_3861404861143990202[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4795307519782956573) {
   out_4795307519782956573[0] = 0;
   out_4795307519782956573[1] = 0;
   out_4795307519782956573[2] = 0;
   out_4795307519782956573[3] = 0;
   out_4795307519782956573[4] = 0;
   out_4795307519782956573[5] = 0;
   out_4795307519782956573[6] = 1;
   out_4795307519782956573[7] = 0;
   out_4795307519782956573[8] = 0;
}
void h_24(double *state, double *unused, double *out_3341134745436053412) {
   out_3341134745436053412[0] = state[4];
   out_3341134745436053412[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2348295926063274966) {
   out_2348295926063274966[0] = 0;
   out_2348295926063274966[1] = 0;
   out_2348295926063274966[2] = 0;
   out_2348295926063274966[3] = 0;
   out_2348295926063274966[4] = 1;
   out_2348295926063274966[5] = 0;
   out_2348295926063274966[6] = 0;
   out_2348295926063274966[7] = 0;
   out_2348295926063274966[8] = 0;
   out_2348295926063274966[9] = 0;
   out_2348295926063274966[10] = 0;
   out_2348295926063274966[11] = 0;
   out_2348295926063274966[12] = 0;
   out_2348295926063274966[13] = 0;
   out_2348295926063274966[14] = 1;
   out_2348295926063274966[15] = 0;
   out_2348295926063274966[16] = 0;
   out_2348295926063274966[17] = 0;
}
void h_30(double *state, double *unused, double *out_236898437135381662) {
   out_236898437135381662[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9123740223798986845) {
   out_9123740223798986845[0] = 0;
   out_9123740223798986845[1] = 0;
   out_9123740223798986845[2] = 0;
   out_9123740223798986845[3] = 0;
   out_9123740223798986845[4] = 1;
   out_9123740223798986845[5] = 0;
   out_9123740223798986845[6] = 0;
   out_9123740223798986845[7] = 0;
   out_9123740223798986845[8] = 0;
}
void h_26(double *state, double *unused, double *out_8745070734871459103) {
   out_8745070734871459103[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8536810838657012797) {
   out_8536810838657012797[0] = 0;
   out_8536810838657012797[1] = 0;
   out_8536810838657012797[2] = 0;
   out_8536810838657012797[3] = 0;
   out_8536810838657012797[4] = 0;
   out_8536810838657012797[5] = 0;
   out_8536810838657012797[6] = 0;
   out_8536810838657012797[7] = 1;
   out_8536810838657012797[8] = 0;
}
void h_27(double *state, double *unused, double *out_8863078128804835847) {
   out_8863078128804835847[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7099409778726621554) {
   out_7099409778726621554[0] = 0;
   out_7099409778726621554[1] = 0;
   out_7099409778726621554[2] = 0;
   out_7099409778726621554[3] = 1;
   out_7099409778726621554[4] = 0;
   out_7099409778726621554[5] = 0;
   out_7099409778726621554[6] = 0;
   out_7099409778726621554[7] = 0;
   out_7099409778726621554[8] = 0;
}
void h_29(double *state, double *unused, double *out_4764774830525463983) {
   out_4764774830525463983[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8812772505596172587) {
   out_8812772505596172587[0] = 0;
   out_8812772505596172587[1] = 1;
   out_8812772505596172587[2] = 0;
   out_8812772505596172587[3] = 0;
   out_8812772505596172587[4] = 0;
   out_8812772505596172587[5] = 0;
   out_8812772505596172587[6] = 0;
   out_8812772505596172587[7] = 0;
   out_8812772505596172587[8] = 0;
}
void h_28(double *state, double *unused, double *out_5826292077360503162) {
   out_5826292077360503162[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4551572551043848455) {
   out_4551572551043848455[0] = 1;
   out_4551572551043848455[1] = 0;
   out_4551572551043848455[2] = 0;
   out_4551572551043848455[3] = 0;
   out_4551572551043848455[4] = 0;
   out_4551572551043848455[5] = 0;
   out_4551572551043848455[6] = 0;
   out_4551572551043848455[7] = 0;
   out_4551572551043848455[8] = 0;
}
void h_31(double *state, double *unused, double *out_3586210798859484313) {
   out_3586210798859484313[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4764661557905996145) {
   out_4764661557905996145[0] = 0;
   out_4764661557905996145[1] = 0;
   out_4764661557905996145[2] = 0;
   out_4764661557905996145[3] = 0;
   out_4764661557905996145[4] = 0;
   out_4764661557905996145[5] = 0;
   out_4764661557905996145[6] = 0;
   out_4764661557905996145[7] = 0;
   out_4764661557905996145[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3310266499749110268) {
  err_fun(nom_x, delta_x, out_3310266499749110268);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9111078211151530773) {
  inv_err_fun(nom_x, true_x, out_9111078211151530773);
}
void car_H_mod_fun(double *state, double *out_8318288169772448250) {
  H_mod_fun(state, out_8318288169772448250);
}
void car_f_fun(double *state, double dt, double *out_3923135191490236514) {
  f_fun(state,  dt, out_3923135191490236514);
}
void car_F_fun(double *state, double dt, double *out_2349988940558973348) {
  F_fun(state,  dt, out_2349988940558973348);
}
void car_h_25(double *state, double *unused, double *out_3861404861143990202) {
  h_25(state, unused, out_3861404861143990202);
}
void car_H_25(double *state, double *unused, double *out_4795307519782956573) {
  H_25(state, unused, out_4795307519782956573);
}
void car_h_24(double *state, double *unused, double *out_3341134745436053412) {
  h_24(state, unused, out_3341134745436053412);
}
void car_H_24(double *state, double *unused, double *out_2348295926063274966) {
  H_24(state, unused, out_2348295926063274966);
}
void car_h_30(double *state, double *unused, double *out_236898437135381662) {
  h_30(state, unused, out_236898437135381662);
}
void car_H_30(double *state, double *unused, double *out_9123740223798986845) {
  H_30(state, unused, out_9123740223798986845);
}
void car_h_26(double *state, double *unused, double *out_8745070734871459103) {
  h_26(state, unused, out_8745070734871459103);
}
void car_H_26(double *state, double *unused, double *out_8536810838657012797) {
  H_26(state, unused, out_8536810838657012797);
}
void car_h_27(double *state, double *unused, double *out_8863078128804835847) {
  h_27(state, unused, out_8863078128804835847);
}
void car_H_27(double *state, double *unused, double *out_7099409778726621554) {
  H_27(state, unused, out_7099409778726621554);
}
void car_h_29(double *state, double *unused, double *out_4764774830525463983) {
  h_29(state, unused, out_4764774830525463983);
}
void car_H_29(double *state, double *unused, double *out_8812772505596172587) {
  H_29(state, unused, out_8812772505596172587);
}
void car_h_28(double *state, double *unused, double *out_5826292077360503162) {
  h_28(state, unused, out_5826292077360503162);
}
void car_H_28(double *state, double *unused, double *out_4551572551043848455) {
  H_28(state, unused, out_4551572551043848455);
}
void car_h_31(double *state, double *unused, double *out_3586210798859484313) {
  h_31(state, unused, out_3586210798859484313);
}
void car_H_31(double *state, double *unused, double *out_4764661557905996145) {
  H_31(state, unused, out_4764661557905996145);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
