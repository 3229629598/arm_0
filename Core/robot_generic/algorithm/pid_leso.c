#include "pid_leso.h"
#include "filter.h"

#define ANGLE_RANGE 8192.0f
#define PI 3.141592653f

#define Z_MAX 30000.0f

#define K_M 0.741f  // N*m/A
#define K_E 13.33f  // rpm/V
#define R_S 1.8f //Ω

void update_leso(leso_para_t* leso, float y,float u);

/**
  * @brief  pid dual loop calculation with LESO
  * @param      pid     the inner and outter loop pid struct containing parameters
  * @param      leso   the leso struct containing parameters
  * @param[in]  err     error value
  * @param[in]  err_out outter loop feedback value
  * @return pid calculation result
  */
float pid_leso_dualloop(pid_struct_t pid[2], leso_para_t* leso, float err, float err_out){
  float pid_in_output, pid_out_output;

  err -= leso->z1 *leso->a_z1;

  pid_in_output = pid_calc(&pid[ANG_LOOP], err , 0);
  pid_in_output -= leso->z2*30/PI;

  pid_out_output = pid_calc(&pid[RPM_LOOP], pid_in_output , err_out);
  pid_out_output -= leso->z3* leso->b1;
  //TODO: iir_filter for output

  update_leso(leso,err*(2.0f*PI/ANGLE_RANGE),pid_out_output);
  return pid_out_output;
}

/**
  * @brief  init leso for gm6020 pid
  * @param  leso   the leso struct containing parameters
  * @param[in] h
  * @param[in] b1 		: by default 0.1 ~ 2
  * @param[in] a_z1		: by defalut 0.01
  * @param[in] _J		: momentum of inertia, kg.m^2
  * @param[in] omega0 	: bandwidth of disturb
  * @return none
  */
void leso_6020_init(leso_para_t* leso, float h,float b1, float a_z1, float _J ,float omega0){
	leso->h = h;
	leso->b1 = b1;
	// (s + w)^2 = s^2 + beta_1 * s + beta_2
	leso->beta1 = 3.0f*omega0;
	leso->beta2 = 3.0f*omega0*omega0;
	leso->beta3 = omega0*omega0*omega0;
	//leso->b0 = b0;
	//b_0 = Km*K/(JR)
	leso->b0 = K_M*(24.0f/30000.0f)/(_J*R_S);
	leso->a0 = -K_M/(_J*R_S)/K_E*(PI/30);
	leso->a_z1 = a_z1;
	
	//leso->z1 = leso->z2 = leso->z3= 0.0f;
}


/**
  * @brief  update leso
  * @param  leso   the leso struct containing parameters
  * @param[in] y : feedback value(current error in rad)
  * @param[in] u : current output value
  * @return none
  */
void update_leso(leso_para_t* leso, float y,float u){
	float e = leso->z1 - y;
	
	leso->z1 += leso->h * (leso->z2 - leso->beta1*e);
	if(leso->z1 > Z_MAX) leso->z1 = Z_MAX; else if(leso->z1 < -Z_MAX) leso->z1 = -Z_MAX;
	
    leso->z2 += leso->h * (leso->z3 - leso->beta2 * e + leso->a0 * leso->z2 + leso->b0 * u);
	if(leso->z2 > Z_MAX) leso->z2 = Z_MAX; else if(leso->z2 < -Z_MAX) leso->z2 = -Z_MAX;
	
    leso->z3 -= leso->h * leso->beta3 * e;
	if(leso->z3 > Z_MAX) leso->z3 = Z_MAX; else if(leso->z3 < -Z_MAX) leso->z3 = -Z_MAX;
}

/**
  * @brief  reset leso
  * @param  leso   the leso struct containing parameters
  * @return none
  */
void reset_leso(leso_para_t* leso){
	leso->z1 = leso->z2 = leso->z3= 0.0f;
}
