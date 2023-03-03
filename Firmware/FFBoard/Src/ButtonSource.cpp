/*
 * ButtonSource.cpp
 *
 *  Created on: 09.02.2020
 *      Author: Yannick
 */

#include "ButtonSource.h"

#if defined(SPIBUTTONS) || defined(SPIBUTTONS2)
#include "SPIButtons.h"
#endif
#ifdef CANBUTTONS
#include "CanButtons.h"
#endif
#ifdef LOCALBUTTONS
#include "LocalButtons.h"
#endif
#ifdef SHIFTERBUTTONS
#include <ShifterAnalog.h>
#endif
#ifdef PCF8574BUTTONS
#include "PCF8574.h"
#endif


ClassIdentifier ButtonSource::info = {
	 .name 	= "NONE" ,
	 .id	= 0,
	 .visibility = ClassVisibility::hidden
};

// Register possible button sources (id 0-15)
const std::vector<class_entry<ButtonSource>> ButtonSource::all_button_sources =
{
#ifdef LOCALBUTTONS
		add_class<LocalButtons,ButtonSource>(0),
#endif
#ifdef SPIBUTTONS
		add_class<SPI_Buttons_1,ButtonSource>(1),
#endif
#ifdef SPIBUTTONS2
		add_class<SPI_Buttons_2,ButtonSource>(2),
#endif
#ifdef SHIFTERBUTTONS
		add_class<ShifterAnalog,ButtonSource>(3),
#endif
#ifdef PCF8574BUTTONS
		add_class<PCF8574Buttons,ButtonSource>(4),
#endif
#ifdef CANBUTTONS
		add_class<CanButtons,ButtonSource>(5),
#endif
};


ButtonSource::ButtonSource() {

}

ButtonSource::~ButtonSource() {

}


uint16_t ButtonSource::getBtnNum(){
	return this->btnnum;
}

