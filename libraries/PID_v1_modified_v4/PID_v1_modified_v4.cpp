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
PID::PID(float* Input, float* Output, float* Setpoint, int* ComputeFlag,
        float Kp, float Ki, float Kd, float PIDSampleTime)
{
	myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;
	myComputeFlag = ComputeFlag;

    PID::SetTunings(Kp, Ki, Kd);
	pidSampleTime = PIDSampleTime;

	int i;										// initialize velocity array
	for (i = 0; i < VELOCTIY_ARRAY_SIZE; i++) {
		velocity[i] = 0;
	}

	velCounter = VELOCTIY_ARRAY_SIZE - 1;	 // initialize counter used for indicating whether to use moving mean or not

	iTerm = 0;
	lastError = 0;
	if (iTerm > OUT_MAX) iTerm = OUT_MAX;
	else if (iTerm < OUT_MIN) iTerm = OUT_MIN;
}

/* SetTunings(...)*************************************************************
******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd)
{
	if (Kp<0 || Ki<0 || Kd<0) return;

	dispKp = Kp; dispKi = Ki; dispKd = Kd;

	float SampleTimeInSec = ((float)pidSampleTime) / 1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
}

/* Compute() **********************************************************************
 * computes output based on a bunch of stuff
 * parameter: computeFlag, which is asserted by Timer and deasserted by Compute()
 * output:    a bool which indicates if a new output has been computed or not
 **********************************************************************************/
bool PID::Compute()
{
	float velocityAve = 0;
	float input;
	float output;
	float error;
	float dError;
	float pTerm;
	float dTerm;
	int i = 0;

	if (*(myComputeFlag) == 1)
	{
		/*Compute all the working error variables*/
		input = *myInput;
		error = *mySetpoint - input;
		dError = error - lastError;

		/*Compute the integral term*/
		iTerm = iTerm + (ki * error);
		if (iTerm > OUT_MAX) iTerm = OUT_MAX;
		else if (iTerm < OUT_MIN) iTerm = OUT_MIN;

		/*Compute the proportional term*/
		pTerm = kp * error;

		/*Compute the derivative term*/
		if (velCounter >= 0)	// if we haven't populated the entire array yet then use dError as velocity
		{
			dTerm = kd * dError; // use dError for the D term
			velocity[velCounter] = dError;
			velCounter--;
		}
		else		// otherwise we compute average velocity from the array
		{
			for (i == (VELOCTIY_ARRAY_SIZE - 2); i >= 0; i--) { // remove the oldest velocity
				velocity[i + 1] = velocity[i];
			}
			velocity[0] = dError;	// move the new dError into the velocity array

			for (i = 0; i < VELOCTIY_ARRAY_SIZE; i++) {	// compute average velocity from the array
				velocityAve += velocity[i];
			}
			velocityAve = velocityAve / VELOCTIY_ARRAY_SIZE;

			dTerm = kd * velocityAve; // use velocityAve for the D term
		}

		/*sums up the three terms to get the output*/
		output = iTerm + pTerm + dTerm;
		if (output > OUT_MAX) output = OUT_MAX;
		else if (output < OUT_MIN) output = OUT_MIN;
		*myOutput = output;

		/*Remember some variables for next time*/
		lastError = error;

		/*Deassert the compute flag*/
		*(myComputeFlag) = 0;

		return true;
	}
	else 
	{
		return false;
	}
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}

