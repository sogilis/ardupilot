#ifndef PARAMETERS_STUB_H_
#define PARAMETERS_STUB_H_

#include "ap_int16_stub.h"

class Parameters {
public:
	//Attributes
	bool compass_enabled;
	bool optflow_enabled;
	bool sonar_enabled;
	int8_t telem_delay;
	int16_t sysid_my_gcs;
	RC_Channel rc_1;
	RC_Channel rc_2;
	RC_Channel rc_3;
	RC_Channel rc_4;
	RC_Channel rc_5;
	RC_Channel rc_6;
	RC_Channel rc_7;
	RC_Channel rc_8;
    AP_Int16   log_bitmask;
    //Methods
    Parameters() {
    	this->compass_enabled = true;
    	this->optflow_enabled = true;
    	this->sonar_enabled   = true;
    	this->telem_delay     = 0;
    	this->sysid_my_gcs    = 1;
    }//Constructor
};//Parameters

#endif /* PARAMETERS_STUB_H_ */
