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

class FloppyMain_itf: public FFBoardMain,public SPIDevice {

public:

	FloppyMain_itf();
	virtual ~FloppyMain_itf();

	static const OutputPin cs1;
	static const OutputPin cs2;
	static const OutputPin cs3;

	void beginSpiTransfer(SPIPort* port);
	void endSpiTransfer(SPIPort* port);
	void sendCommand(midifloppy_spi_cmd& cmd,uint8_t bus=0);

	virtual void usbInit();

	void resetDrive(uint8_t adr,uint8_t bus=0);

protected:
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
	std::vector<midifloppy_spi_cmd> cmdbuffer;
	static const uint32_t packetlength = 6;
	uint8_t txbuf[packetlength+1] = {0}; // 6 data bytes + crc

};

class MidiFloppyMain: public FloppyMain_itf, public MidiHandler {
	enum class MidiFloppyMain_commands : uint32_t{
		reset
	};


public:
	MidiFloppyMain();
	virtual ~MidiFloppyMain();

	TIM_HandleTypeDef* timer_update;

	static ClassIdentifier info;
	const ClassIdentifier getInfo();


	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	void usbInit();
	void update();

	void noteOn(uint8_t chan, uint8_t note,uint8_t velocity);
	void noteOff(uint8_t chan, uint8_t note,uint8_t velocity);
	void controlChange(uint8_t chan, uint8_t c, uint8_t val);
	void pitchBend(uint8_t chan, int16_t val);

	void sendFrequency(uint8_t adr,float freq,uint8_t bus = 0);

	virtual std::string getHelpstring(){
		return "Plays MIDI Floppymusic via SPI";
	}

	void play();

private:
	static const uint32_t channels = 16;


	volatile bool updateflag = false;
	float noteToFreq[128] = {0};
	std::vector<MidiNote> notes[channels];
	float active[channels] = {false};

	const uint16_t period = 100;//71;	// Microseconds
	float periodf = period / 1000000.0; // seconds


};

// From Adafruit_Floppy project
#define GW_FIRMVER_MAJOR 1
#define GW_FIRMVER_MINOR 0
#define GW_MAXCMD      21
#define GW_HW_MODEL    8
#define GW_HW_SUBMODEL 0
#define GW_USB_SPEED   0  // Full Speed

#define GW_CMD_GETINFO    0
#define GW_CMD_GETINFO_FIRMWARE 0
#define GW_CMD_GETINFO_BANDWIDTH 1
#define GW_CMD_SEEK       2
#define GW_CMD_HEAD       3
#define GW_CMD_SETPARAMS  4
#define GW_CMD_GETPARAMS  5
#define GW_CMD_GETPARAMS_DELAYS 0
#define GW_CMD_MOTOR      6
#define GW_CMD_READFLUX   7
#define GW_CMD_WRITEFLUX   8
#define GW_CMD_GETFLUXSTATUS  9
#define GW_CMD_SELECT    12
#define GW_CMD_DESELECT  13
#define GW_CMD_SETBUSTYPE 14
#define GW_CMD_SETBUSTYPE_IBM 1
#define GW_CMD_SETBUSTYPE_SHUGART 2
#define GW_CMD_SETBUSTYPE_APPLE2 3
#define GW_CMD_SETPIN    15
#define GW_CMD_SETPIN_DENSITY 2
#define GW_CMD_RESET     16
#define GW_CMD_SOURCEBYTES 18
#define GW_CMD_SINKBYTES 19
#define GW_CMD_GETPIN 20
#define GW_CMD_GETPIN_TRACK0 26
#define GW_ACK_OK (byte)0
#define GW_ACK_BADCMD 1
#define GW_ACK_NOINDEX 2
#define GW_ACK_NOTRACK0 3
#define GW_ACK_WRPROT 6
#define GW_ACK_NOUNIT 7
#define GW_ACK_BADPIN 10

class GWFloppyMain: public FloppyMain_itf, public MidiHandler {
	enum class GWFloppyMain_commands : uint32_t{
		reset,readTrack,seek,setdrive
	};


public:
	GWFloppyMain();
	virtual ~GWFloppyMain();

	TIM_HandleTypeDef* timer_update;

	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies);
	void usbInit();
	void update();

	virtual void cdcRcv(char* Buf, uint32_t *Len);

	virtual std::string getHelpstring(){
		return "Greaseweazle to FDD SPI interface";
	}


private:
	std::array<char,32> gwCmdBuf;
	std::array<char,128> reply_buffer;

	uint32_t bandwidth_timer;
	float bytes_per_sec;
	uint32_t transfered_bytes;
	uint32_t captured_pulses;
	uint8_t curAdr = 0;
};

#endif /* MidiMAIN_H_ */
#endif
