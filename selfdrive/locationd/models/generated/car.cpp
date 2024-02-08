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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_122908531796567845) {
   out_122908531796567845[0] = delta_x[0] + nom_x[0];
   out_122908531796567845[1] = delta_x[1] + nom_x[1];
   out_122908531796567845[2] = delta_x[2] + nom_x[2];
   out_122908531796567845[3] = delta_x[3] + nom_x[3];
   out_122908531796567845[4] = delta_x[4] + nom_x[4];
   out_122908531796567845[5] = delta_x[5] + nom_x[5];
   out_122908531796567845[6] = delta_x[6] + nom_x[6];
   out_122908531796567845[7] = delta_x[7] + nom_x[7];
   out_122908531796567845[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2538234936892293981) {
   out_2538234936892293981[0] = -nom_x[0] + true_x[0];
   out_2538234936892293981[1] = -nom_x[1] + true_x[1];
   out_2538234936892293981[2] = -nom_x[2] + true_x[2];
   out_2538234936892293981[3] = -nom_x[3] + true_x[3];
   out_2538234936892293981[4] = -nom_x[4] + true_x[4];
   out_2538234936892293981[5] = -nom_x[5] + true_x[5];
   out_2538234936892293981[6] = -nom_x[6] + true_x[6];
   out_2538234936892293981[7] = -nom_x[7] + true_x[7];
   out_2538234936892293981[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4589170859792487472) {
   out_4589170859792487472[0] = 1.0;
   out_4589170859792487472[1] = 0;
   out_4589170859792487472[2] = 0;
   out_4589170859792487472[3] = 0;
   out_4589170859792487472[4] = 0;
   out_4589170859792487472[5] = 0;
   out_4589170859792487472[6] = 0;
   out_4589170859792487472[7] = 0;
   out_4589170859792487472[8] = 0;
   out_4589170859792487472[9] = 0;
   out_4589170859792487472[10] = 1.0;
   out_4589170859792487472[11] = 0;
   out_4589170859792487472[12] = 0;
   out_4589170859792487472[13] = 0;
   out_4589170859792487472[14] = 0;
   out_4589170859792487472[15] = 0;
   out_4589170859792487472[16] = 0;
   out_4589170859792487472[17] = 0;
   out_4589170859792487472[18] = 0;
   out_4589170859792487472[19] = 0;
   out_4589170859792487472[20] = 1.0;
   out_4589170859792487472[21] = 0;
   out_4589170859792487472[22] = 0;
   out_4589170859792487472[23] = 0;
   out_4589170859792487472[24] = 0;
   out_4589170859792487472[25] = 0;
   out_4589170859792487472[26] = 0;
   out_4589170859792487472[27] = 0;
   out_4589170859792487472[28] = 0;
   out_4589170859792487472[29] = 0;
   out_4589170859792487472[30] = 1.0;
   out_4589170859792487472[31] = 0;
   out_4589170859792487472[32] = 0;
   out_4589170859792487472[33] = 0;
   out_4589170859792487472[34] = 0;
   out_4589170859792487472[35] = 0;
   out_4589170859792487472[36] = 0;
   out_4589170859792487472[37] = 0;
   out_4589170859792487472[38] = 0;
   out_4589170859792487472[39] = 0;
   out_4589170859792487472[40] = 1.0;
   out_4589170859792487472[41] = 0;
   out_4589170859792487472[42] = 0;
   out_4589170859792487472[43] = 0;
   out_4589170859792487472[44] = 0;
   out_4589170859792487472[45] = 0;
   out_4589170859792487472[46] = 0;
   out_4589170859792487472[47] = 0;
   out_4589170859792487472[48] = 0;
   out_4589170859792487472[49] = 0;
   out_4589170859792487472[50] = 1.0;
   out_4589170859792487472[51] = 0;
   out_4589170859792487472[52] = 0;
   out_4589170859792487472[53] = 0;
   out_4589170859792487472[54] = 0;
   out_4589170859792487472[55] = 0;
   out_4589170859792487472[56] = 0;
   out_4589170859792487472[57] = 0;
   out_4589170859792487472[58] = 0;
   out_4589170859792487472[59] = 0;
   out_4589170859792487472[60] = 1.0;
   out_4589170859792487472[61] = 0;
   out_4589170859792487472[62] = 0;
   out_4589170859792487472[63] = 0;
   out_4589170859792487472[64] = 0;
   out_4589170859792487472[65] = 0;
   out_4589170859792487472[66] = 0;
   out_4589170859792487472[67] = 0;
   out_4589170859792487472[68] = 0;
   out_4589170859792487472[69] = 0;
   out_4589170859792487472[70] = 1.0;
   out_4589170859792487472[71] = 0;
   out_4589170859792487472[72] = 0;
   out_4589170859792487472[73] = 0;
   out_4589170859792487472[74] = 0;
   out_4589170859792487472[75] = 0;
   out_4589170859792487472[76] = 0;
   out_4589170859792487472[77] = 0;
   out_4589170859792487472[78] = 0;
   out_4589170859792487472[79] = 0;
   out_4589170859792487472[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2042901932542470359) {
   out_2042901932542470359[0] = state[0];
   out_2042901932542470359[1] = state[1];
   out_2042901932542470359[2] = state[2];
   out_2042901932542470359[3] = state[3];
   out_2042901932542470359[4] = state[4];
   out_2042901932542470359[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2042901932542470359[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2042901932542470359[7] = state[7];
   out_2042901932542470359[8] = state[8];
}
void F_fun(double *state, double dt, double *out_749452283589495020) {
   out_749452283589495020[0] = 1;
   out_749452283589495020[1] = 0;
   out_749452283589495020[2] = 0;
   out_749452283589495020[3] = 0;
   out_749452283589495020[4] = 0;
   out_749452283589495020[5] = 0;
   out_749452283589495020[6] = 0;
   out_749452283589495020[7] = 0;
   out_749452283589495020[8] = 0;
   out_749452283589495020[9] = 0;
   out_749452283589495020[10] = 1;
   out_749452283589495020[11] = 0;
   out_749452283589495020[12] = 0;
   out_749452283589495020[13] = 0;
   out_749452283589495020[14] = 0;
   out_749452283589495020[15] = 0;
   out_749452283589495020[16] = 0;
   out_749452283589495020[17] = 0;
   out_749452283589495020[18] = 0;
   out_749452283589495020[19] = 0;
   out_749452283589495020[20] = 1;
   out_749452283589495020[21] = 0;
   out_749452283589495020[22] = 0;
   out_749452283589495020[23] = 0;
   out_749452283589495020[24] = 0;
   out_749452283589495020[25] = 0;
   out_749452283589495020[26] = 0;
   out_749452283589495020[27] = 0;
   out_749452283589495020[28] = 0;
   out_749452283589495020[29] = 0;
   out_749452283589495020[30] = 1;
   out_749452283589495020[31] = 0;
   out_749452283589495020[32] = 0;
   out_749452283589495020[33] = 0;
   out_749452283589495020[34] = 0;
   out_749452283589495020[35] = 0;
   out_749452283589495020[36] = 0;
   out_749452283589495020[37] = 0;
   out_749452283589495020[38] = 0;
   out_749452283589495020[39] = 0;
   out_749452283589495020[40] = 1;
   out_749452283589495020[41] = 0;
   out_749452283589495020[42] = 0;
   out_749452283589495020[43] = 0;
   out_749452283589495020[44] = 0;
   out_749452283589495020[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_749452283589495020[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_749452283589495020[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_749452283589495020[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_749452283589495020[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_749452283589495020[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_749452283589495020[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_749452283589495020[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_749452283589495020[53] = -9.8000000000000007*dt;
   out_749452283589495020[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_749452283589495020[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_749452283589495020[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_749452283589495020[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_749452283589495020[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_749452283589495020[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_749452283589495020[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_749452283589495020[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_749452283589495020[62] = 0;
   out_749452283589495020[63] = 0;
   out_749452283589495020[64] = 0;
   out_749452283589495020[65] = 0;
   out_749452283589495020[66] = 0;
   out_749452283589495020[67] = 0;
   out_749452283589495020[68] = 0;
   out_749452283589495020[69] = 0;
   out_749452283589495020[70] = 1;
   out_749452283589495020[71] = 0;
   out_749452283589495020[72] = 0;
   out_749452283589495020[73] = 0;
   out_749452283589495020[74] = 0;
   out_749452283589495020[75] = 0;
   out_749452283589495020[76] = 0;
   out_749452283589495020[77] = 0;
   out_749452283589495020[78] = 0;
   out_749452283589495020[79] = 0;
   out_749452283589495020[80] = 1;
}
void h_25(double *state, double *unused, double *out_8910777389406178182) {
   out_8910777389406178182[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4447527938048292635) {
   out_4447527938048292635[0] = 0;
   out_4447527938048292635[1] = 0;
   out_4447527938048292635[2] = 0;
   out_4447527938048292635[3] = 0;
   out_4447527938048292635[4] = 0;
   out_4447527938048292635[5] = 0;
   out_4447527938048292635[6] = 1;
   out_4447527938048292635[7] = 0;
   out_4447527938048292635[8] = 0;
}
void h_24(double *state, double *unused, double *out_6983956359321902164) {
   out_6983956359321902164[0] = state[4];
   out_6983956359321902164[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2274878339042793069) {
   out_2274878339042793069[0] = 0;
   out_2274878339042793069[1] = 0;
   out_2274878339042793069[2] = 0;
   out_2274878339042793069[3] = 0;
   out_2274878339042793069[4] = 1;
   out_2274878339042793069[5] = 0;
   out_2274878339042793069[6] = 0;
   out_2274878339042793069[7] = 0;
   out_2274878339042793069[8] = 0;
   out_2274878339042793069[9] = 0;
   out_2274878339042793069[10] = 0;
   out_2274878339042793069[11] = 0;
   out_2274878339042793069[12] = 0;
   out_2274878339042793069[13] = 0;
   out_2274878339042793069[14] = 1;
   out_2274878339042793069[15] = 0;
   out_2274878339042793069[16] = 0;
   out_2274878339042793069[17] = 0;
}
void h_30(double *state, double *unused, double *out_5437663386024001570) {
   out_5437663386024001570[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4318188990905052565) {
   out_4318188990905052565[0] = 0;
   out_4318188990905052565[1] = 0;
   out_4318188990905052565[2] = 0;
   out_4318188990905052565[3] = 0;
   out_4318188990905052565[4] = 1;
   out_4318188990905052565[5] = 0;
   out_4318188990905052565[6] = 0;
   out_4318188990905052565[7] = 0;
   out_4318188990905052565[8] = 0;
}
void h_26(double *state, double *unused, double *out_435990118363155925) {
   out_435990118363155925[0] = state[7];
}
void H_26(double *state, double *unused, double *out_706024619174236411) {
   out_706024619174236411[0] = 0;
   out_706024619174236411[1] = 0;
   out_706024619174236411[2] = 0;
   out_706024619174236411[3] = 0;
   out_706024619174236411[4] = 0;
   out_706024619174236411[5] = 0;
   out_706024619174236411[6] = 0;
   out_706024619174236411[7] = 1;
   out_706024619174236411[8] = 0;
}
void h_27(double *state, double *unused, double *out_8152927244245359372) {
   out_8152927244245359372[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2143425679104627654) {
   out_2143425679104627654[0] = 0;
   out_2143425679104627654[1] = 0;
   out_2143425679104627654[2] = 0;
   out_2143425679104627654[3] = 1;
   out_2143425679104627654[4] = 0;
   out_2143425679104627654[5] = 0;
   out_2143425679104627654[6] = 0;
   out_2143425679104627654[7] = 0;
   out_2143425679104627654[8] = 0;
}
void h_29(double *state, double *unused, double *out_6040979270700479087) {
   out_6040979270700479087[0] = state[1];
}
void H_29(double *state, double *unused, double *out_430062952235076621) {
   out_430062952235076621[0] = 0;
   out_430062952235076621[1] = 1;
   out_430062952235076621[2] = 0;
   out_430062952235076621[3] = 0;
   out_430062952235076621[4] = 0;
   out_430062952235076621[5] = 0;
   out_430062952235076621[6] = 0;
   out_430062952235076621[7] = 0;
   out_430062952235076621[8] = 0;
}
void h_28(double *state, double *unused, double *out_9096437097147021807) {
   out_9096437097147021807[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2393693223800402872) {
   out_2393693223800402872[0] = 1;
   out_2393693223800402872[1] = 0;
   out_2393693223800402872[2] = 0;
   out_2393693223800402872[3] = 0;
   out_2393693223800402872[4] = 0;
   out_2393693223800402872[5] = 0;
   out_2393693223800402872[6] = 0;
   out_2393693223800402872[7] = 0;
   out_2393693223800402872[8] = 0;
}
void h_31(double *state, double *unused, double *out_5391335255085532576) {
   out_5391335255085532576[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4478173899925253063) {
   out_4478173899925253063[0] = 0;
   out_4478173899925253063[1] = 0;
   out_4478173899925253063[2] = 0;
   out_4478173899925253063[3] = 0;
   out_4478173899925253063[4] = 0;
   out_4478173899925253063[5] = 0;
   out_4478173899925253063[6] = 0;
   out_4478173899925253063[7] = 0;
   out_4478173899925253063[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_122908531796567845) {
  err_fun(nom_x, delta_x, out_122908531796567845);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2538234936892293981) {
  inv_err_fun(nom_x, true_x, out_2538234936892293981);
}
void car_H_mod_fun(double *state, double *out_4589170859792487472) {
  H_mod_fun(state, out_4589170859792487472);
}
void car_f_fun(double *state, double dt, double *out_2042901932542470359) {
  f_fun(state,  dt, out_2042901932542470359);
}
void car_F_fun(double *state, double dt, double *out_749452283589495020) {
  F_fun(state,  dt, out_749452283589495020);
}
void car_h_25(double *state, double *unused, double *out_8910777389406178182) {
  h_25(state, unused, out_8910777389406178182);
}
void car_H_25(double *state, double *unused, double *out_4447527938048292635) {
  H_25(state, unused, out_4447527938048292635);
}
void car_h_24(double *state, double *unused, double *out_6983956359321902164) {
  h_24(state, unused, out_6983956359321902164);
}
void car_H_24(double *state, double *unused, double *out_2274878339042793069) {
  H_24(state, unused, out_2274878339042793069);
}
void car_h_30(double *state, double *unused, double *out_5437663386024001570) {
  h_30(state, unused, out_5437663386024001570);
}
void car_H_30(double *state, double *unused, double *out_4318188990905052565) {
  H_30(state, unused, out_4318188990905052565);
}
void car_h_26(double *state, double *unused, double *out_435990118363155925) {
  h_26(state, unused, out_435990118363155925);
}
void car_H_26(double *state, double *unused, double *out_706024619174236411) {
  H_26(state, unused, out_706024619174236411);
}
void car_h_27(double *state, double *unused, double *out_8152927244245359372) {
  h_27(state, unused, out_8152927244245359372);
}
void car_H_27(double *state, double *unused, double *out_2143425679104627654) {
  H_27(state, unused, out_2143425679104627654);
}
void car_h_29(double *state, double *unused, double *out_6040979270700479087) {
  h_29(state, unused, out_6040979270700479087);
}
void car_H_29(double *state, double *unused, double *out_430062952235076621) {
  H_29(state, unused, out_430062952235076621);
}
void car_h_28(double *state, double *unused, double *out_9096437097147021807) {
  h_28(state, unused, out_9096437097147021807);
}
void car_H_28(double *state, double *unused, double *out_2393693223800402872) {
  H_28(state, unused, out_2393693223800402872);
}
void car_h_31(double *state, double *unused, double *out_5391335255085532576) {
  h_31(state, unused, out_5391335255085532576);
}
void car_H_31(double *state, double *unused, double *out_4478173899925253063) {
  H_31(state, unused, out_4478173899925253063);
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

ekf_init(car);
