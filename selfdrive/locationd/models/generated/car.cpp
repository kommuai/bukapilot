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
void err_fun(double *nom_x, double *delta_x, double *out_500439478528929704) {
   out_500439478528929704[0] = delta_x[0] + nom_x[0];
   out_500439478528929704[1] = delta_x[1] + nom_x[1];
   out_500439478528929704[2] = delta_x[2] + nom_x[2];
   out_500439478528929704[3] = delta_x[3] + nom_x[3];
   out_500439478528929704[4] = delta_x[4] + nom_x[4];
   out_500439478528929704[5] = delta_x[5] + nom_x[5];
   out_500439478528929704[6] = delta_x[6] + nom_x[6];
   out_500439478528929704[7] = delta_x[7] + nom_x[7];
   out_500439478528929704[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8177809488341246547) {
   out_8177809488341246547[0] = -nom_x[0] + true_x[0];
   out_8177809488341246547[1] = -nom_x[1] + true_x[1];
   out_8177809488341246547[2] = -nom_x[2] + true_x[2];
   out_8177809488341246547[3] = -nom_x[3] + true_x[3];
   out_8177809488341246547[4] = -nom_x[4] + true_x[4];
   out_8177809488341246547[5] = -nom_x[5] + true_x[5];
   out_8177809488341246547[6] = -nom_x[6] + true_x[6];
   out_8177809488341246547[7] = -nom_x[7] + true_x[7];
   out_8177809488341246547[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_48951311923778637) {
   out_48951311923778637[0] = 1.0;
   out_48951311923778637[1] = 0.0;
   out_48951311923778637[2] = 0.0;
   out_48951311923778637[3] = 0.0;
   out_48951311923778637[4] = 0.0;
   out_48951311923778637[5] = 0.0;
   out_48951311923778637[6] = 0.0;
   out_48951311923778637[7] = 0.0;
   out_48951311923778637[8] = 0.0;
   out_48951311923778637[9] = 0.0;
   out_48951311923778637[10] = 1.0;
   out_48951311923778637[11] = 0.0;
   out_48951311923778637[12] = 0.0;
   out_48951311923778637[13] = 0.0;
   out_48951311923778637[14] = 0.0;
   out_48951311923778637[15] = 0.0;
   out_48951311923778637[16] = 0.0;
   out_48951311923778637[17] = 0.0;
   out_48951311923778637[18] = 0.0;
   out_48951311923778637[19] = 0.0;
   out_48951311923778637[20] = 1.0;
   out_48951311923778637[21] = 0.0;
   out_48951311923778637[22] = 0.0;
   out_48951311923778637[23] = 0.0;
   out_48951311923778637[24] = 0.0;
   out_48951311923778637[25] = 0.0;
   out_48951311923778637[26] = 0.0;
   out_48951311923778637[27] = 0.0;
   out_48951311923778637[28] = 0.0;
   out_48951311923778637[29] = 0.0;
   out_48951311923778637[30] = 1.0;
   out_48951311923778637[31] = 0.0;
   out_48951311923778637[32] = 0.0;
   out_48951311923778637[33] = 0.0;
   out_48951311923778637[34] = 0.0;
   out_48951311923778637[35] = 0.0;
   out_48951311923778637[36] = 0.0;
   out_48951311923778637[37] = 0.0;
   out_48951311923778637[38] = 0.0;
   out_48951311923778637[39] = 0.0;
   out_48951311923778637[40] = 1.0;
   out_48951311923778637[41] = 0.0;
   out_48951311923778637[42] = 0.0;
   out_48951311923778637[43] = 0.0;
   out_48951311923778637[44] = 0.0;
   out_48951311923778637[45] = 0.0;
   out_48951311923778637[46] = 0.0;
   out_48951311923778637[47] = 0.0;
   out_48951311923778637[48] = 0.0;
   out_48951311923778637[49] = 0.0;
   out_48951311923778637[50] = 1.0;
   out_48951311923778637[51] = 0.0;
   out_48951311923778637[52] = 0.0;
   out_48951311923778637[53] = 0.0;
   out_48951311923778637[54] = 0.0;
   out_48951311923778637[55] = 0.0;
   out_48951311923778637[56] = 0.0;
   out_48951311923778637[57] = 0.0;
   out_48951311923778637[58] = 0.0;
   out_48951311923778637[59] = 0.0;
   out_48951311923778637[60] = 1.0;
   out_48951311923778637[61] = 0.0;
   out_48951311923778637[62] = 0.0;
   out_48951311923778637[63] = 0.0;
   out_48951311923778637[64] = 0.0;
   out_48951311923778637[65] = 0.0;
   out_48951311923778637[66] = 0.0;
   out_48951311923778637[67] = 0.0;
   out_48951311923778637[68] = 0.0;
   out_48951311923778637[69] = 0.0;
   out_48951311923778637[70] = 1.0;
   out_48951311923778637[71] = 0.0;
   out_48951311923778637[72] = 0.0;
   out_48951311923778637[73] = 0.0;
   out_48951311923778637[74] = 0.0;
   out_48951311923778637[75] = 0.0;
   out_48951311923778637[76] = 0.0;
   out_48951311923778637[77] = 0.0;
   out_48951311923778637[78] = 0.0;
   out_48951311923778637[79] = 0.0;
   out_48951311923778637[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4996357532719299431) {
   out_4996357532719299431[0] = state[0];
   out_4996357532719299431[1] = state[1];
   out_4996357532719299431[2] = state[2];
   out_4996357532719299431[3] = state[3];
   out_4996357532719299431[4] = state[4];
   out_4996357532719299431[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4996357532719299431[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4996357532719299431[7] = state[7];
   out_4996357532719299431[8] = state[8];
}
void F_fun(double *state, double dt, double *out_78166648368307141) {
   out_78166648368307141[0] = 1;
   out_78166648368307141[1] = 0;
   out_78166648368307141[2] = 0;
   out_78166648368307141[3] = 0;
   out_78166648368307141[4] = 0;
   out_78166648368307141[5] = 0;
   out_78166648368307141[6] = 0;
   out_78166648368307141[7] = 0;
   out_78166648368307141[8] = 0;
   out_78166648368307141[9] = 0;
   out_78166648368307141[10] = 1;
   out_78166648368307141[11] = 0;
   out_78166648368307141[12] = 0;
   out_78166648368307141[13] = 0;
   out_78166648368307141[14] = 0;
   out_78166648368307141[15] = 0;
   out_78166648368307141[16] = 0;
   out_78166648368307141[17] = 0;
   out_78166648368307141[18] = 0;
   out_78166648368307141[19] = 0;
   out_78166648368307141[20] = 1;
   out_78166648368307141[21] = 0;
   out_78166648368307141[22] = 0;
   out_78166648368307141[23] = 0;
   out_78166648368307141[24] = 0;
   out_78166648368307141[25] = 0;
   out_78166648368307141[26] = 0;
   out_78166648368307141[27] = 0;
   out_78166648368307141[28] = 0;
   out_78166648368307141[29] = 0;
   out_78166648368307141[30] = 1;
   out_78166648368307141[31] = 0;
   out_78166648368307141[32] = 0;
   out_78166648368307141[33] = 0;
   out_78166648368307141[34] = 0;
   out_78166648368307141[35] = 0;
   out_78166648368307141[36] = 0;
   out_78166648368307141[37] = 0;
   out_78166648368307141[38] = 0;
   out_78166648368307141[39] = 0;
   out_78166648368307141[40] = 1;
   out_78166648368307141[41] = 0;
   out_78166648368307141[42] = 0;
   out_78166648368307141[43] = 0;
   out_78166648368307141[44] = 0;
   out_78166648368307141[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_78166648368307141[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_78166648368307141[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_78166648368307141[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_78166648368307141[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_78166648368307141[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_78166648368307141[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_78166648368307141[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_78166648368307141[53] = -9.8100000000000005*dt;
   out_78166648368307141[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_78166648368307141[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_78166648368307141[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_78166648368307141[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_78166648368307141[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_78166648368307141[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_78166648368307141[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_78166648368307141[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_78166648368307141[62] = 0;
   out_78166648368307141[63] = 0;
   out_78166648368307141[64] = 0;
   out_78166648368307141[65] = 0;
   out_78166648368307141[66] = 0;
   out_78166648368307141[67] = 0;
   out_78166648368307141[68] = 0;
   out_78166648368307141[69] = 0;
   out_78166648368307141[70] = 1;
   out_78166648368307141[71] = 0;
   out_78166648368307141[72] = 0;
   out_78166648368307141[73] = 0;
   out_78166648368307141[74] = 0;
   out_78166648368307141[75] = 0;
   out_78166648368307141[76] = 0;
   out_78166648368307141[77] = 0;
   out_78166648368307141[78] = 0;
   out_78166648368307141[79] = 0;
   out_78166648368307141[80] = 1;
}
void h_25(double *state, double *unused, double *out_5046435348237256990) {
   out_5046435348237256990[0] = state[6];
}
void H_25(double *state, double *unused, double *out_9204604535873535742) {
   out_9204604535873535742[0] = 0;
   out_9204604535873535742[1] = 0;
   out_9204604535873535742[2] = 0;
   out_9204604535873535742[3] = 0;
   out_9204604535873535742[4] = 0;
   out_9204604535873535742[5] = 0;
   out_9204604535873535742[6] = 1;
   out_9204604535873535742[7] = 0;
   out_9204604535873535742[8] = 0;
}
void h_24(double *state, double *unused, double *out_892464665149108733) {
   out_892464665149108733[0] = state[4];
   out_892464665149108733[1] = state[5];
}
void H_24(double *state, double *unused, double *out_14074351766820649) {
   out_14074351766820649[0] = 0;
   out_14074351766820649[1] = 0;
   out_14074351766820649[2] = 0;
   out_14074351766820649[3] = 0;
   out_14074351766820649[4] = 1;
   out_14074351766820649[5] = 0;
   out_14074351766820649[6] = 0;
   out_14074351766820649[7] = 0;
   out_14074351766820649[8] = 0;
   out_14074351766820649[9] = 0;
   out_14074351766820649[10] = 0;
   out_14074351766820649[11] = 0;
   out_14074351766820649[12] = 0;
   out_14074351766820649[13] = 0;
   out_14074351766820649[14] = 1;
   out_14074351766820649[15] = 0;
   out_14074351766820649[16] = 0;
   out_14074351766820649[17] = 0;
}
void h_30(double *state, double *unused, double *out_5321629410521762879) {
   out_5321629410521762879[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4676908205745927544) {
   out_4676908205745927544[0] = 0;
   out_4676908205745927544[1] = 0;
   out_4676908205745927544[2] = 0;
   out_4676908205745927544[3] = 0;
   out_4676908205745927544[4] = 1;
   out_4676908205745927544[5] = 0;
   out_4676908205745927544[6] = 0;
   out_4676908205745927544[7] = 0;
   out_4676908205745927544[8] = 0;
}
void h_26(double *state, double *unused, double *out_4718313525845285362) {
   out_4718313525845285362[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5463101216999479518) {
   out_5463101216999479518[0] = 0;
   out_5463101216999479518[1] = 0;
   out_5463101216999479518[2] = 0;
   out_5463101216999479518[3] = 0;
   out_5463101216999479518[4] = 0;
   out_5463101216999479518[5] = 0;
   out_5463101216999479518[6] = 0;
   out_5463101216999479518[7] = 1;
   out_5463101216999479518[8] = 0;
}
void h_27(double *state, double *unused, double *out_4102925495335594555) {
   out_4102925495335594555[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2502144893945502633) {
   out_2502144893945502633[0] = 0;
   out_2502144893945502633[1] = 0;
   out_2502144893945502633[2] = 0;
   out_2502144893945502633[3] = 1;
   out_2502144893945502633[4] = 0;
   out_2502144893945502633[5] = 0;
   out_2502144893945502633[6] = 0;
   out_2502144893945502633[7] = 0;
   out_2502144893945502633[8] = 0;
}
void h_29(double *state, double *unused, double *out_3660339761485077886) {
   out_3660339761485077886[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5187139550060319728) {
   out_5187139550060319728[0] = 0;
   out_5187139550060319728[1] = 1;
   out_5187139550060319728[2] = 0;
   out_5187139550060319728[3] = 0;
   out_5187139550060319728[4] = 0;
   out_5187139550060319728[5] = 0;
   out_5187139550060319728[6] = 0;
   out_5187139550060319728[7] = 0;
   out_5187139550060319728[8] = 0;
}
void h_28(double *state, double *unused, double *out_9104598762996440200) {
   out_9104598762996440200[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7150769821625645979) {
   out_7150769821625645979[0] = 1;
   out_7150769821625645979[1] = 0;
   out_7150769821625645979[2] = 0;
   out_7150769821625645979[3] = 0;
   out_7150769821625645979[4] = 0;
   out_7150769821625645979[5] = 0;
   out_7150769821625645979[6] = 0;
   out_7150769821625645979[7] = 0;
   out_7150769821625645979[8] = 0;
}
void h_31(double *state, double *unused, double *out_4760431248753752742) {
   out_4760431248753752742[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4836893114766128042) {
   out_4836893114766128042[0] = 0;
   out_4836893114766128042[1] = 0;
   out_4836893114766128042[2] = 0;
   out_4836893114766128042[3] = 0;
   out_4836893114766128042[4] = 0;
   out_4836893114766128042[5] = 0;
   out_4836893114766128042[6] = 0;
   out_4836893114766128042[7] = 0;
   out_4836893114766128042[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_500439478528929704) {
  err_fun(nom_x, delta_x, out_500439478528929704);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8177809488341246547) {
  inv_err_fun(nom_x, true_x, out_8177809488341246547);
}
void car_H_mod_fun(double *state, double *out_48951311923778637) {
  H_mod_fun(state, out_48951311923778637);
}
void car_f_fun(double *state, double dt, double *out_4996357532719299431) {
  f_fun(state,  dt, out_4996357532719299431);
}
void car_F_fun(double *state, double dt, double *out_78166648368307141) {
  F_fun(state,  dt, out_78166648368307141);
}
void car_h_25(double *state, double *unused, double *out_5046435348237256990) {
  h_25(state, unused, out_5046435348237256990);
}
void car_H_25(double *state, double *unused, double *out_9204604535873535742) {
  H_25(state, unused, out_9204604535873535742);
}
void car_h_24(double *state, double *unused, double *out_892464665149108733) {
  h_24(state, unused, out_892464665149108733);
}
void car_H_24(double *state, double *unused, double *out_14074351766820649) {
  H_24(state, unused, out_14074351766820649);
}
void car_h_30(double *state, double *unused, double *out_5321629410521762879) {
  h_30(state, unused, out_5321629410521762879);
}
void car_H_30(double *state, double *unused, double *out_4676908205745927544) {
  H_30(state, unused, out_4676908205745927544);
}
void car_h_26(double *state, double *unused, double *out_4718313525845285362) {
  h_26(state, unused, out_4718313525845285362);
}
void car_H_26(double *state, double *unused, double *out_5463101216999479518) {
  H_26(state, unused, out_5463101216999479518);
}
void car_h_27(double *state, double *unused, double *out_4102925495335594555) {
  h_27(state, unused, out_4102925495335594555);
}
void car_H_27(double *state, double *unused, double *out_2502144893945502633) {
  H_27(state, unused, out_2502144893945502633);
}
void car_h_29(double *state, double *unused, double *out_3660339761485077886) {
  h_29(state, unused, out_3660339761485077886);
}
void car_H_29(double *state, double *unused, double *out_5187139550060319728) {
  H_29(state, unused, out_5187139550060319728);
}
void car_h_28(double *state, double *unused, double *out_9104598762996440200) {
  h_28(state, unused, out_9104598762996440200);
}
void car_H_28(double *state, double *unused, double *out_7150769821625645979) {
  H_28(state, unused, out_7150769821625645979);
}
void car_h_31(double *state, double *unused, double *out_4760431248753752742) {
  h_31(state, unused, out_4760431248753752742);
}
void car_H_31(double *state, double *unused, double *out_4836893114766128042) {
  H_31(state, unused, out_4836893114766128042);
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
