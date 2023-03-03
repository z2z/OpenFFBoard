/*
 * AnalogSource.cpp
 *
 *  Created on: 06.11.2020
 *      Author: Yannick
 */

#include "AnalogSource.h"

#ifdef ANALOGAXES
#include "LocalAnalog.h"
#endif
#ifdef CANANALOG
#include "CanAnalog.h"
#endif
#ifdef ADS111XANALOG
#include "ADS111X.h"
#endif

ClassIdentifier AnalogSource::info = {
	 .name 	= "NONE" ,
	 .id	= CLSID_ANALOG_NONE, //0
	 .visibility = ClassVisibility::hidden
};

// Register possible analog sources (id 0-15)
const std::vector<class_entry<AnalogSource>> AnalogSource::all_analog_sources =
{
#ifdef ANALOGAXES
		add_class<LocalAnalog,AnalogSource>(0),
#endif
#ifdef CANANALOG
		add_class<CanAnalog<8>,AnalogSource>(1),
#endif
#ifdef ADS111XANALOG
		add_class<ADS111X_AnalogSource,AnalogSource>(2),
#endif
};

AnalogSource::AnalogSource() {

}

AnalogSource::~AnalogSource() {

}

std::vector<int32_t>* AnalogSource::getAxes(){
	return &this->buf;
}
