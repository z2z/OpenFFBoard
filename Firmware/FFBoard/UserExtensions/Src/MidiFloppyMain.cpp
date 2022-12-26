/*
 * MidiMain.cpp
 *
 *  Created on: 23.01.2020
 *      Author: Yannick
 */

#include "target_constants.h"
#ifdef MIDIFLOPPY
#include <MidiFloppyMain.h>

#include "math.h"
#include "ledEffects.h"
#include "USBdevice.h"
#include "cmsis_os2.h"
#include "cpp_target_config.h"

ClassIdentifier MidiFloppyMain::info = {
		 .name = "MIDI FDD (SPI)" ,
		 .id=CLSID_MAIN_MIDI_FLOPPY,
		 .visibility = ClassVisibility::debug
 };

const ClassIdentifier MidiFloppyMain::getInfo(){
	return info;
}


MidiFloppyMain::MidiFloppyMain() : SPIDevice{motor_spi,cs1}{
	// Generate notes
	for(uint8_t i = 0;i<128;i++){
		float f = std::pow(2, (i - 69) / 12.0) * 440.0;
		this->noteToFreq[i] = f;
	}

	// Setup timer
//	extern TIM_HandleTypeDef TIM_USER;
//	this->timer_update = &TIM_USER; // Timer setup with prescaler of sysclock
//	this->timer_update->Instance->ARR = period;
//	this->timer_update->Instance->PSC = (SystemCoreClock / 1000000)-1;
//	this->timer_update->Instance->CR1 = 1;
//	HAL_TIM_Base_Start_IT(this->timer_update);

	// Reconfigure pins if

	// Enable SPI crc mode
	//this->spiConfig.peripheral.Direction = SPI_DIRECTION_1LINE;
	this->spiConfig.peripheral.DataSize = SPI_DATASIZE_8BIT;
	this->spiConfig.peripheral.CLKPolarity = SPI_POLARITY_LOW;
	this->spiConfig.peripheral.CLKPhase = SPI_PHASE_1EDGE;
	//this->spiConfig.peripheral.NSS = SPI_NSS_SOFT;
	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 8 = 10MHz
	this->spiConfig.peripheral.FirstBit = SPI_FIRSTBIT_MSB;
	this->spiConfig.peripheral.TIMode = SPI_TIMODE_DISABLE;
	this->spiConfig.peripheral.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	this->spiConfig.peripheral.CRCPolynomial = 7;

	motor_spi.configurePort(&this->spiConfig.peripheral);



	//CommandHandler::registerCommands();
//	registerCommand("power", MidiMain_commands::power, "Intensity",CMDFLAG_GET|CMDFLAG_SET);
//	registerCommand("range", MidiMain_commands::range, "Range of phase change",CMDFLAG_GET|CMDFLAG_SET);


}

MidiFloppyMain::~MidiFloppyMain() {

}


void MidiFloppyMain::update(){
	osDelay(500); // Slow down main thread
	//play();
}

void MidiFloppyMain::timerElapsed(TIM_HandleTypeDef* htim){
	if(htim == this->timer_update){
		play();
	}
}

void MidiFloppyMain::play(){
	// Take only first channel for now and last note...
//	for(uint8_t chan = 0;chan<channels;chan++){
//
//		if(notes[chan].empty()){
//			if(active[chan]){
//				active[chan] = false;
//				// stop
//
//			}
//		}else{
//			if(!active[chan]){
//				// Send message out
//				active[chan] = true;
//			}
//
//			MidiNote* note = &notes[chan].back();
//			note->frequency = noteToFreq[note->note];
//			float frequency = note->frequency + note->pitchbend;
//
//			float volume = note->volume / 127.0f;
//			// Send note frequency out
//		}
//	}
}

void MidiFloppyMain::sendCommand(midifloppy_spi_cmd& cmd,uint8_t bus){
	memcpy(txbuf,&cmd,packetlength);
	this->spiPort.transmit_DMA(txbuf, packetlength, this);
	// TODO handle multiple CS pins
}

void MidiFloppyMain::sendFrequency(uint8_t adr,float freq,uint8_t bus){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr;
	cmd.cmd = CMD_PLAYFREQ;
	cmd.val1_32 = *reinterpret_cast<uint32_t*>(&freq);
	sendCommand(cmd, bus);
}

void MidiFloppyMain::noteOn(uint8_t chan, uint8_t note,uint8_t velocity){
	// If note already present remove
	noteOff(chan,note,velocity);

	MidiNote midinote;
	midinote.note = note;
	midinote.frequency = noteToFreq[note];
	midinote.volume = velocity;
	notes[chan].push_back(midinote);

	sendFrequency(chan,midinote.frequency,0);
}

void MidiFloppyMain::noteOff(uint8_t chan, uint8_t note,uint8_t velocity){
	for(auto it = notes[chan].begin(); it!=notes[chan].end(); ++it){
		if(it->note == note){
			notes[chan].erase(it);
			break;
		}
	}
	// Stop note or play last note
	if(notes[chan].empty()){
		sendFrequency(chan,0,0);
	}else{
		MidiNote *note = &notes[chan].back();
		sendFrequency(chan,note->frequency*note->pitchbend,0);
	}

}

void MidiFloppyMain::controlChange(uint8_t chan, uint8_t c, uint8_t val){
	if(c == 120 || c == 121 || c == 123){
		notes[chan].clear();
		// Reset
		midifloppy_spi_cmd cmd;
		cmd.adr = chan;
		cmd.cmd = CMD_RESET;
		sendCommand(cmd, 0);
	}
}
void MidiFloppyMain::pitchBend(uint8_t chan, int16_t val){
	float pb = std::pow(2.0f, (((float)val/8192.0f)));
	//printf("PB: %d, %f\n",val,pb);
	for(auto it = notes[chan].begin(); it!=notes[chan].end(); ++it){
		it->pitchbend =  pb;
	}
	MidiNote *note = &notes[chan].back();
	sendFrequency(chan,note->frequency*note->pitchbend,0);
}

CommandStatus MidiFloppyMain::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	CommandStatus result = CommandStatus::OK;
	switch(static_cast<MidiFloppyMain_commands>(cmd.cmdId)){



	default:
		result = CommandStatus::NOT_FOUND;
		break;
	}
	return result;
}


void MidiFloppyMain::usbInit(){
	this->usbdev = std::make_unique<USBdevice>(&usb_devdesc_ffboard_composite,usb_cdc_midi_conf,&usb_ffboard_strings_default);
	usbdev->registerUsb();
}

#endif
