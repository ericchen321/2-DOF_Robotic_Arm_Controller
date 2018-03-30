/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_v1_modified_v4.h"

/*Constructor (...)*********************************************************
 *    parameter: pointer to input (actual angle in deg)
 *	  parameter: pointer to output (PWM)
 *    parameter: pointer to setpoint (desired angle in deg)
 *    parameter: P Gain
 *	  parameter: I Gain
 *	  parameter: D Gain
 *    parameter: PID Sample Time in millisecond (ms)
 *	  parameter: the flag which indicates a new output should be computed
 *    output:	 void
 ***************************************************************************/
PID::PID(float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd, int* errorClear, int PIDSampleTime)
{
	myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;
	myErrorClear = errorClear;
	pidSampleTime = PIDSampleTime;

    PID::SetTunings(Kp, Ki, Kd);

	iTerm = 0;
	lastError = 0;

	/*
	int i;										// initialize derivArray array
	for (i = 0; i < DERIV_ARRAY_SIZE; i++) {
		derivArray[i] = 0;
	}

	derivCounter = DERIV_ARRAY_SIZE - 1;	 // initialize counter used for indicating whether to use moving mean or not
	*/
}

/* SetTunings(...)*************************************************************
******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd)
{
	if (Kp<0 || Ki<0 || Kd<0) return;

	dispKp = Kp; dispKi = Ki; dispKd = Kd;

	float SampleTimeInSec = (float)pidSampleTime / 1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
}

/* Compute() **********************************************************************
 * computes output based on a bunch of stuff
 * parameter: none
 * output:    a bool which indicates if a new output has been computed or not
 **********************************************************************************/
bool PID::Compute()
{
	float derivAve = 0;
	float input;
	float output;
	float error;
	float dError;
	float pTerm;
	float dTerm;
	int i = 0;

	/*Check if lastError and iTerm should be resetted*/
	if (*myErrorClear == 1) {
		iTerm = 0;
		lastError = 0;
		*myErrorClear = 0;
	}

	/*Compute all the working error variables*/
	input = *myInput;
	error = *mySetpoint - input;
	dError = error - lastError;

	/*Compute the integral term*/
	iTerm = iTerm + (ki * error);

	/*Compute the proportional term*/
	pTerm = kp * error;

	/*Compute the derivative term*/
	/*
	if (derivCounter >= 0)	// if we haven't populated the entire array yet then use dError as derivArray
	{
	dTerm = kd * dError; // use dError for the D term
	derivArray[derivCounter] = dError;
	derivCounter--;
	}
	else		// otherwise we compute average derivArray from the array
	{
	for (i == (DERIV_ARRAY_SIZE - 2); i >= 0; i--) { // remove the oldest derivArray
	derivArray[i + 1] = derivArray[i];
	}
	derivArray[0] = dError;	// move the new dError into the derivArray array

	for (i = 0; i < DERIV_ARRAY_SIZE; i++) {	// compute average derivArray from the array
	derivAve += derivArray[i];
	}
	derivAve = derivAve / DERIV_ARRAY_SIZE;

	dTerm = kd * derivAve; // use derivAve for the D term
	}
	*/
	dTerm = kd * dError;

	/*sums up the three terms to get the output*/
	output = iTerm + pTerm + dTerm;
	if (output > OUT_MAX) output = OUT_MAX;
	else if (output < OUT_MIN) output = OUT_MIN;
	*myOutput = output;

	/*Remember some variables for next time*/
	lastError = error;

	return true;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}

