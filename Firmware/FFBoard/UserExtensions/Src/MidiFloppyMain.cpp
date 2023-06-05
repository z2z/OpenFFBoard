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


std::array<float,128> MidiNote::noteToFreq = {0};

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
	    PA6     ------> SPI1_MISO or htim13
	    PA7     ------> SPI1_MOSI
	    */
	//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7; // GPIO_PIN_6
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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

void FloppyMain_itf::setSpiSpeed(uint8_t speed){
	speed = clip<uint8_t>(speed, 0, 7);
	switch(speed){
	case 0:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; break;
	case 1:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; break;
	case 2:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; break;
	case 3:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; break;
	case 4:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; break;
	case 5:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; break;
	case 6:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; break;
	case 7:	this->spiConfig.peripheral.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; break;
	}
	spispeed = speed;
	motor_spi.configurePort(&this->spiConfig.peripheral);
}

void FloppyMain_itf::enableExtClkMode(bool enable){
	// external clock mode on MISO pin w. TIM13 (84MHz base freq on F407)
	extclkmode = enable;
	if(enable){
		// Set up timer pin for ext clk
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


		FDDCLKTIM.Instance = TIM13;
		FDDCLKTIM.Init.Prescaler = 0;
		FDDCLKTIM.Init.CounterMode = TIM_COUNTERMODE_UP;
		FDDCLKTIM.Init.Period = 839; // 100khz
		FDDCLKTIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		FDDCLKTIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		HAL_TIM_Base_Init(&FDDCLKTIM);
		HAL_TIM_PWM_Init(&FDDCLKTIM);

		TIM_OC_InitTypeDef sConfigOC = {0};
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = FDDCLKTIM.Init.Period/2; // 50% pwm
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

		HAL_TIM_PWM_ConfigChannel(&FDDCLKTIM, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&FDDCLKTIM, TIM_CHANNEL_1);
		// Start timer with right clock frequency

		uint32_t extClkFreq = 100000; // 100khz
		// Broadcast ext clk command
		midifloppy_spi_cmd cmd;
		cmd.adr = 0xff;
		cmd.cmd = CMD_SET_EXTCLK;
		cmd.val1_32 = extClkFreq;
		sendCommand(cmd, 15);
	}else{
		HAL_TIM_PWM_Stop(&FDDCLKTIM, TIM_CHANNEL_1);
		midifloppy_spi_cmd cmd;
		cmd.adr = 0xff;
		cmd.cmd = CMD_SET_EXTCLK;
		cmd.val1_32 = 0;
		sendCommand(cmd, 15);
	}
}

/*
 * Called before the spi transfer starts.
 * Can be used to take the semaphore and set CS pins by default
 */
void FloppyMain_itf::beginSpiTransfer(SPIPort* port){
	//assertChipSelect();
	for(uint8_t i = 0; i < cspins.size(); i++){
		if(activeBus & (1 << i))
			cspins[i].reset();
	}
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
	//clearChipSelect();
	for(uint8_t i = 0; i < cspins.size(); i++){
		if(activeBus & 1 << i)
			cspins[i].set();
	}
	activeBus = 0;
	port->giveSemaphore();
}


/**
 * Always use this function to send data to the drives
 */
void FloppyMain_itf::sendCommand(midifloppy_spi_cmd& cmd,uint8_t bus){
 	this->spiPort.takeSemaphore(); // Take semaphore before writing into the DMA buffer
	activeBus = bus;

	memcpy(txbuf,&cmd,packetlength);
	this->spiPort.transmit_DMA(txbuf, packetlength, this);
}

void FloppyMain_itf::resetDrive(uint8_t adr,uint8_t bus){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr;
	cmd.cmd = CMD_RESET;
	sendCommand(cmd, bus);
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

FloppyMain_itf::~FloppyMain_itf() {

}

// ---------------------------------------------------------------------------------------------------------------------//

// MIDI FLOPPY
MidiFloppyMain::MidiFloppyMain() {
	// Generate notes
	MidiNote::initFreqTable();
	restoreFlash();

	CommandHandler::registerCommands();
	registerCommand("drives", MidiFloppyMain_commands::drivesPerPort, "Drives per port",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("extclk", MidiFloppyMain_commands::extclk, "Master clock mode",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("mode", MidiFloppyMain_commands::mode, "Channel mode (d1p split4p d4p)",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("enable", MidiFloppyMain_commands::enable, "Set drive enable pins of chan",CMDFLAG_SETADR);
	registerCommand("spispeed", MidiFloppyMain_commands::spispeed, "SPI prescaler (0-7)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("reset", MidiFloppyMain_commands::reset, "Reset drives",CMDFLAG_GET);
//	resetAll();

}

MidiFloppyMain::~MidiFloppyMain() {

}

void MidiFloppyMain::update(){
	osDelay(100);
	if(!initialized)
		initialize();

}

void MidiFloppyMain::initialize(){
	enableExtClkMode(false);
	initialized = true;
}

void MidiFloppyMain::midiTick(){
	if(operationMode != MidiFloppyMain_modes::direct1port){
		// Update channels
		for(uint8_t i = 0;i<channels;i++){
			if(channelUpdateFlag & 1 << i){
				sendNotesForChannel(i);
			}
		}
		channelUpdateFlag = 0;
	}
}

DriveAdr MidiFloppyMain::chanToPortAdr(uint16_t drive){
	return chanToPortAdr(drive % drivesPerPort,drive / drivesPerPort);
}

DriveAdr MidiFloppyMain::chanToPortAdr(uint8_t chan, uint8_t idx){
	// Depending on mode and number of total drives calculate the address and port of the target drive
	DriveAdr drive;
	if(!drivesPerPort){
		return {0,0};
	}
	switch(operationMode){
	case MidiFloppyMain_modes::direct1port:
		default:
			return {chan,1 << idx};

		case MidiFloppyMain_modes::split4port:
		{
			uint16_t channelsPerBlock = channels / 2;
			drive.port = 1 << ((idx / 2) + (2 * ( chan / channelsPerBlock))); // First 8 channels first 2 ports
			drive.adr = (idx * channelsPerBlock + (chan % channelsPerBlock)) % drivesPerPort; // (2 * 8 + 3) % 16. 2 idx per port in first 2 ports
			break;
		}
		case MidiFloppyMain_modes::direct4port:
		{
			uint16_t channelsPerBlock = channels;

			drive.adr = (idx * channelsPerBlock + chan) % drivesPerPort; //
			drive.port = 1 << ( (idx + (1 * ( chan / channelsPerBlock))) % 4); // idx is port if 16 drives per port
			break;
		}

	}
	return drive;
}

void MidiFloppyMain::sendFrequency(DriveAdr drive,float freq){
	sendFrequency(drive.adr,freq,drive.port);
}



void MidiFloppyMain::sendFrequency(uint8_t adr,float freq,uint8_t bus){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr;
	cmd.cmd = CMD_PLAYFREQ;
	cmd.val1_32 = *((uint32_t*)(&freq));
	sendCommand(cmd, bus);
}

/**
 * Efficiently sends notes by combining ports if note and address are the same and only the port differs
 */
void MidiFloppyMain::sendNotes(std::vector<NoteToSend>& notes){

	// Broadcast more efficiently by combining packets if possible
	for(NoteToSend& current : notes){
		if(current.sent)
			continue; // Skip if already marked sent
		for(NoteToSend& other : notes){
			if(&current != &other && current.target.adr == other.target.adr && current.note == other.note){
				// Matches. Mark both as sent and add port mask to first drive and skip others
				other.sent = true;
				current.target.port |= other.target.port;
			}
		}
		current.sent = true;
		// Send here
		sendFrequency(current.target,current.note.getFrequency() * pitchBends[current.note.channel]);
	}
	// Simple method. send one command for each note
//	for(NoteToSend& current : notes){
//		sendFrequency(current.target,current.note.getFrequency() * pitchBends[current.note.channel]);
//	}
}

/**
 * Sends notes for the channel that are currently playing.
 * Useful for updating notes when pitch has changed
 */
void MidiFloppyMain::sendNotesForChannel(uint8_t channel){
	std::vector<MidiNote>& currentNotes = this->notes[channel];
//	uint8_t amount = currentNotes.size();//std::min<uint8_t>(currentNotes.size(), drivesPerChannel);

	std::vector<NoteToSend> newNotes;
	newNotes.reserve(drivesPerChannel);

	uint8_t i = 0;
	// Iterate reverse. Newest note goes to first drive idx. Amount of drives per note depends on volume and total amount of playing notes
	bool done = false;
	for(auto n = currentNotes.rbegin(); n != currentNotes.rend() ; n++){
		MidiNote &note = *n;
		// Minimum 1 drive per note even if volume low
		uint8_t activeIdxForNote = std::min<uint8_t>(drivesPerChannel,std::max(1,(note.volume * drivesPerChannel) / (100 * std::min<uint8_t>(currentNotes.size(), drivesPerChannel))));
		while(activeIdxForNote--){ // Send note to multiple drives
			if(newNotes.size() >= drivesPerChannel){
				done = true;
				break; // Stop if already all drives are full
			}
			NoteToSend noteT;
			noteT.note = note;
			noteT.target = chanToPortAdr(channel, i++);
			newNotes.push_back(noteT);
		}
		if(done)
			break;
	}
	// Fill commands with empty notes first so that unused drives stop playing
	for(uint8_t idx = newNotes.size();idx < drivesPerChannel;idx++){
		NoteToSend noteT;
		noteT.target = chanToPortAdr(channel, idx);
		newNotes.push_back(noteT);
	}
	sendNotes(newNotes);
}

/**
 * In single port mode a note is directly sent to its channel address. Multiple notes are stacked and the last note is playing.
 * In multi port mode the amount of ports playing this note depend on the volume (up to 4) or polyphony.
 */
void MidiFloppyMain::noteOn(uint8_t chan, uint8_t note,uint8_t velocity){
	// If note already present remove?
//	noteOff(chan,note,velocity);

	MidiNote midinote(note, velocity, chan);
	notes[chan].push_back(midinote);

	if(operationMode == MidiFloppyMain_modes::direct1port){
		sendFrequency(chan,midinote.getFrequency() * pitchBends[chan],1); // Send direct
	}else{ // Multi port splitting
//		sendNotesForChannel(chan);
		channelUpdateFlag |= 1 << chan;
	}

}

void MidiFloppyMain::noteOff(uint8_t chan, uint8_t note,uint8_t velocity){


	for(auto it = notes[chan].begin(); it!=notes[chan].end(); ++it){
		if(it->note == note){
			notes[chan].erase(it);
			break;
		}
	}
		// Stop note or play last note
	if(operationMode == MidiFloppyMain_modes::direct1port){
		if(notes[chan].empty()){
			sendFrequency(chan,0,1);
		}else{
			MidiNote *note = &notes[chan].back();
			sendFrequency(chan,note->getFrequency()*pitchBends[chan],1);
		}
	}else{
//		sendNotesForChannel(chan);
		channelUpdateFlag |= 1 << chan;
	}
}

/**
 * Stops all channels and deletes notes
 */
void MidiFloppyMain::resetAll(){
	for(uint8_t i = 0;i < channels;i++){
		resetChannel(i);
	}
}

void MidiFloppyMain::resetChannel(uint8_t chan){
	notes[chan].clear();
	pitchBends[chan] = 1;
//	sendFrequency(chan, 0, 1);
	for(uint8_t idx = 0;idx < drivesPerChannel;idx++){
		DriveAdr target = chanToPortAdr(chan, idx);
		sendFrequency(target,0);
	}
}

void MidiFloppyMain::controlChange(uint8_t chan, uint8_t c, uint8_t val){
	if(c == 120 || c == 121 || c == 123){
		resetChannel(chan);
		// Reset
//		midifloppy_spi_cmd cmd;
//		cmd.adr = chan;
//		cmd.cmd = CMD_RESET;
//		sendCommand(cmd, 0);

	}
}

void MidiFloppyMain::saveFlash(){
	uint16_t val = (uint16_t)this->operationMode & 0x7;
	val |= extclkmode ? 0x8 : 0;
	val |= (spispeed & 0x7) << 4;
	Flash_Write(ADR_MIDIFLOPPY_CONF1, val);
}

void MidiFloppyMain::restoreFlash(){
	uint16_t val;
	if(Flash_Read(ADR_MIDIFLOPPY_CONF1, &val)){
		this->operationMode = (MidiFloppyMain_modes) (val & 0x7); // 3 bit
		this->enableExtClkMode(val & 0x8);
		setSpiSpeed((val >> 4) & 0x7); // 3 bit
	}
}

void MidiFloppyMain::pitchBend(uint8_t chan, int16_t val){
	float pb = std::pow(2.0f, (((float)val/8192.0f)));
	pitchBends[chan] = pb;
	// Must resend all notes again with new PB value
	channelUpdateFlag |= 1 << chan;
	//sendNotesForChannel(chan);

	// Apply pitchbend to all notes on channel
//	for(auto it = notes[chan].begin(); it!=notes[chan].end(); ++it){
//		it->pitchbend =  pb;
//	}

	if(operationMode == MidiFloppyMain_modes::direct1port){
		MidiNote *note = &notes[chan].back();
		sendFrequency(chan,note->getFrequency()*pitchBends[chan],1);
	}else{
		channelUpdateFlag |= 1 << chan;
	}

}


/**
 * Enables or disables a drive
 */
void MidiFloppyMain::enableDrive(DriveAdr adr,bool drive, bool motor){
	midifloppy_spi_cmd cmd;
	cmd.adr = adr.adr;
	cmd.cmd = CMD_SETENABLE;
	cmd.val1 = drive;
	cmd.val2 = motor;
	sendCommand(cmd, adr.port);
}

CommandStatus MidiFloppyMain::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	CommandStatus result = CommandStatus::OK;
	switch(static_cast<MidiFloppyMain_commands>(cmd.cmdId)){

	case MidiFloppyMain_commands::drivesPerPort:
		return handleGetSet(cmd, replies, drivesPerPort);
	case MidiFloppyMain_commands::extclk:
		return handleGetSetFunc(cmd, replies, extclkmode, &FloppyMain_itf::enableExtClkMode, this);
	case MidiFloppyMain_commands::mode:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("0:direct1port\n1:split4port\n2:direct4port");
		}else{
			return handleGetSet(cmd, replies, (uint32_t&)operationMode);
		}
		break;
	case MidiFloppyMain_commands::enable:
	{
		if(cmd.type==CMDtype::setat){
			if(cmd.adr == 255){
				enableDrive({255,0xf}, cmd.val & 1, cmd.val & 2);
			}
			enableDrive(chanToPortAdr(cmd.adr), cmd.val & 1, cmd.val & 2);
		}
		break;
	}
	case MidiFloppyMain_commands::spispeed:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->spispeed);
		}else if(cmd.type == CMDtype::set){
			setSpiSpeed(cmd.val);
		}
		break;
	case MidiFloppyMain_commands::reset:
		if(cmd.type == CMDtype::get){
			resetAll();
			resetDrive(255, 255); // Reset all drives
		}
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
