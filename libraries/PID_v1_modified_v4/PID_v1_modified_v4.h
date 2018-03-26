#ifndef PID_v1_MODIFIED_v4_h
#define PID_v1_MODIFIED_v4_h
#define LIBRARY_VERSION	1.1.1


class PID
{
  public:

  //Constants used in some of the functions below
  #define OUT_MIN -1023
  #define OUT_MAX +1023
  #define VELOCTIY_ARRAY_SIZE 4

  //commonly used functions **************************************************************************
    PID(float*, float*, float*, int*, float, float, float, float);		// * constructor         				

    bool Compute();                       // * performs the PID calculation, takes a flag input which
											 // indicates a new output should be computed


  //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the 
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
   

									
  //Display functions ****************************************************************
	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually 


  private:

	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter
	float pidSampleTime;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	int *myComputeFlag;			// the flag is asserted by Timer every (pidSampleTime)ms and 
								// should be deasserted by Compute()

	float iTerm; 
	float lastError;
	float velocity[VELOCTIY_ARRAY_SIZE];			 // used for computing moving average
	int velCounter;
};
#endif

