/*=====================================================================================
 File name:        PID_REG3.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  The PID controller with anti-windup                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/
#include "pid_reg3.h"

void pid_reg3_calc(PIDREG3 *v)
{	
    // Compute the error
    //v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp*v->Err;
    // Compute the integral output
    v->Ui = v->Ui + v->Ki*(v->Up+v->Up1)*0.5 + (v->Kc*v->SatErr);
    // Compute the derivative output
    //v->Ud = (v->Kd*(v->Up - v->Up1));
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui;
    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;
}

void pid_reg3_reset(PIDREG3 *v){
	v->Err = 0;
	v->Fdb = 0;
	v->Out = 0;
	v->OutPreSat = 0;
	v->Ui = 0;
	v->Up1 = 0;
	v->Ud = 0;
	v->SatErr = 0;
	v->Out = 0;
}

