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


//const OutputPin FloppyMain_itf::cs1 = OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin);
//const OutputPin FloppyMain_itf::cs2 = OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin);
//const OutputPin FloppyMain_itf::cs3 = OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin);
//const OutputPin FloppyMain_itf::cs4 = OutputPin(*DRV_GP1_GPIO_Port, DRV_GP1_Pin);
std::array<OutputPin,4> FloppyMain_itf::cspins = {
		OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin)
		,OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin)
		,OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin)
		,OutputPin(*DRV_GP1_GPIO_Port, DRV_GP1_Pin)
};


FloppyMain_itf::FloppyMain_itf() : SPIDevice{motor_spi,OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin)}{


	// Setup timer
//	extern TIM_HandleTypeDef TIM_USER;
//	this->timer_update = &TIM_USER; // Timer setup with prescaler of sysclock
//	this->timer_update->Instance->ARR = period;
//	this->timer_update->Instance->PSC = (SystemCoreClock / 1000000)-1;
//	this->timer_update->Instance->CR1 = 1;
//	HAL_TIM_Base_Start_IT(this->timer_update);

	// Reconfigure pins if
	// MUST enable CRC in ioc file so that USE_SPI_CRC is 1 or define USE_SPI_CRC 1
	/**SPI1 GPIO Configuration
	    PA5     ------> SPI1_SCK
	    PA6     ------> SPI1_MISO
	    PA7     ------> SPI1_MOSI
	    */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure CS pins
	for(auto& pin : cspins){
		pin.configureOutput(GPIO_MODE_OUTPUT_PP, false, GPIO_SPEED_FREQ_LOW);
	}

	// Enable SPI crc half duplex mode
	HAL_SPI_DeInit(&hspi1);
	this->spiConfig.cspol = true;
	this->spiConfig.peripheral.Mode = SPI_MODE_MASTER;
	this->spiConfig.peripheral.Direction = SPI_DIRECTION_1LINE;
	this->spiConfig.peripheral.DataSize = SPI_DATASIZE_8BIT;
	this->spiConfig.peripheral.CLKPolarity = SPI_POLARITY_LOW;
	this->spiConfig.peripheral.CLKPhase = SPI_PHASE_1EDGE;
	this->spiConfig.peripheral.NSS = SPI_NSS_SOFT;
	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 8 = 10MHz, 16 = 5MHz
	this->spiConfig.peripheral.FirstBit = SPI_FIRSTBIT_MSB;
	this->spiConfig.peripheral.TIMode = SPI_TIMODE_DISABLE;
	this->spiConfig.peripheral.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
	this->spiConfig.peripheral.CRCPolynomial = 7;

	motor_spi.configurePort(&this->spiConfig.peripheral);


}

/*
 * Called before the spi transfer starts.
 * Can be used to take the semaphore and set CS pins by default
 */
void FloppyMain_itf::beginSpiTransfer(SPIPort* port){
	assertChipSelect();
	uint32_t cnt = 50;
	while(cnt--){asm("nop");}
}

/*
 * Called after a transfer is finished
 * Gives a semaphore back and resets the CS pin
 */
void FloppyMain_itf::endSpiTransfer(SPIPort* port){
	uint32_t cnt = 100; // 100
	while(cnt--){asm("nop");} // Do nothing before resetting CS
	clearChipSelect();
	port->giveSemaphore();
}


void FloppyMain_itf::sendCommand(midifloppy_spi_cmd& cmd,uint8_t bus){
	this->spiPort.takeSemaphore(); // Take semaphore before writing into the DMA buffer
	memcpy(txbuf,&cmd,packetlength);
	this->spiPort.transmit_DMA(txbuf, packetlength, this);
	// TODO handle multiple CS pins
}

void FloppyMain_itf::resetDrive(uint8_t adr,uint8_t bus){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr;
	cmd.cmd = CMD_RESET;
	sendCommand(cmd, bus);
}

FloppyMain_itf::~FloppyMain_itf() {

}

// MIDI FLOPPY
MidiFloppyMain::MidiFloppyMain() {
	// Generate notes
	for(uint8_t i = 0;i<128;i++){
		float f = std::pow(2, (i - 69) / 12.0) * 440.0;
		this->noteToFreq[i] = f;
	}

}

MidiFloppyMain::~MidiFloppyMain() {

}

void MidiFloppyMain::update(){
	//osDelay(500); // Slow down main thread
	//play();
}



void MidiFloppyMain::sendFrequency(uint8_t adr,float freq,uint8_t bus){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr;
	cmd.cmd = CMD_PLAYFREQ;
	cmd.val1_32 = *((uint32_t*)(&freq));
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
//		midifloppy_spi_cmd cmd;
//		cmd.adr = chan;
//		cmd.cmd = CMD_RESET;
//		sendCommand(cmd, 0);
		sendFrequency(chan,0,0);
	}
}
void MidiFloppyMain::pitchBend(uint8_t chan, int16_t val){
	float pb = std::pow(2.0f, (((float)val/8192.0f)));
	// Apply pitchbend to all notes on channel
	for(auto it = notes[chan].begin(); it!=notes[chan].end(); ++it){
		it->pitchbend =  pb;
	}
	MidiNote *note = &notes[chan].back();
	sendFrequency(chan,note->frequency*note->pitchbend,0);
}

CommandStatus MidiFloppyMain::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	CommandStatus result = CommandStatus::OK;
	switch(static_cast<MidiFloppyMain_commands>(cmd.cmdId)){

	case MidiFloppyMain_commands::drivesPerPort:
		return handleGetSet(cmd, replies, drivesPerPort);

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



/**
 * Only checks the flag pin. reconfigure 3 more pins for exti to get individual counts per port
 */
void FloppyMain_itf::exti(uint16_t GPIO_Pin){
	if(GPIO_Pin == adr0pin){
		if(HAL_GetTick() - lastAdrTime > 1000){
			drivesPerPort = 0;
		}
		lastAdrTime = HAL_GetTick();
		drivesPerPort++;
	}
}



#endif
