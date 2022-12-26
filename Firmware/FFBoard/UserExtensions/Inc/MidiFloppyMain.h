/*
 * MidiMain.h
 *
 *  Created on: 23.01.2020
 *      Author: Yannick
 */

// TODO rewrite for tinyusb!
#ifndef MidiMAIN_H_
#define MidiMAIN_H_
#include "target_constants.h"
#ifdef MIDIFLOPPY
#include <FFBoardMain.h>
#include "cppmain.h"
#include "TMC4671.h"
#include "MotorDriver.h"
#include "TimerHandler.h"
#include "vector"
#include "MidiHandler.h"
#include "thread.hpp"
#include "SPI.h"

struct MidiNote{
	uint8_t note = 0;
	uint8_t volume = 0;
	float frequency = 0;
	float pitchbend = 1;
};

typedef struct{
	uint8_t adr;
	uint8_t cmd;
	union{
		struct{
			uint8_t val1;
			uint8_t val2;
			uint8_t val3;
			uint8_t val4;
		};
		struct{
			uint16_t val1_16;
			uint16_t val2_16;
		};
		struct{
			uint32_t val1_32;
		};

	};

} __attribute__((packed)) midifloppy_spi_cmd;

class MidiFloppyMain: public FFBoardMain, public MidiHandler,TimerHandler,SPIDevice {

	enum class MidiFloppyMain_commands : uint32_t{
		reset
	};

public:
	MidiFloppyMain();
	virtual ~MidiFloppyMain();

	TIM_HandleTypeDef* timer_update;

	static ClassIdentifier info;
	const ClassIdentifier getInfo();
	static bool isCreatable() {return true;};
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	void usbInit();
	void update();

	void noteOn(uint8_t chan, uint8_t note,uint8_t velocity);
	void noteOff(uint8_t chan, uint8_t note,uint8_t velocity);
	void controlChange(uint8_t chan, uint8_t c, uint8_t val);
	void pitchBend(uint8_t chan, int16_t val);

	void sendFrequency(uint8_t adr,float freq,uint8_t bus = 0);
	void sendCommand(midifloppy_spi_cmd& cmd,uint8_t bus=0);

	void timerElapsed(TIM_HandleTypeDef* htim);

	virtual std::string getHelpstring(){
		return "Plays MIDI Floppymusic via SPI";
	}

	void play();

private:
	static const uint32_t channels = 16;
	static const uint32_t packetlength = 6;
	uint8_t txbuf[packetlength+1] = {0}; // 6 data bytes + crc

	volatile bool updateflag = false;
	float noteToFreq[128] = {0};
	std::vector<MidiNote> notes[channels];
	bool active[channels] = {false};

	const uint16_t period = 100;//71;	// Microseconds
	float periodf = period / 1000000.0; // seconds

	OutputPin cs1 = OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin);
	OutputPin cs2 = OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin);
	OutputPin cs3 = OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin);


	static const uint8_t ADR_BROADCAST = 0xff;
	static const uint8_t CMDMASK_READ = 0x80;

	static const uint8_t CMD_READ = 0x7f;
	static const uint8_t CMD_REPLY_ADR = 0xDA;
	static const uint8_t CMD_REPLY_SPIERR = 0xDE;

	static const uint8_t CMD_PLAYFREQ = 0x01;
	static const uint8_t CMD_PLAYFREQ_FIX = 0x02;
	static const uint8_t CMD_SETENABLE = 0x03;
	static const uint8_t CMD_RESET = 0x10;
	static const uint8_t CMD_SET_STEPS = 0x11;

	static const uint8_t CMD_DATAMODE = 0x20;
	static const uint8_t CMD_DATASEEK = 0x21; // Seek to track, head and sector
	static const uint8_t CMD_READSECTOR = 0xD5; // Replies a full sector 512B

};

#endif /* MidiMAIN_H_ */
#endif
