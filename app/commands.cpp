
#include "board.h"
#include "console.h"
#include "app.h"
#include "laser4_plus.h"
#include "iface_cc2500.h"
#include "multiprotocol.h"
#include "debug.h"

#ifdef ENABLE_CLI

static char *getOptValue(const char *opt, uint32_t argc, char **argv)
{
    for(uint32_t i = 0; i < argc; i++){
        if(xstrcmp(opt, argv[i]) == 0){
            return argv[i + 1];
        }
    }
    return NULL;
}

static class CmdHelp : public ConsoleCommand {
	Console *console;
public:
    CmdHelp() : ConsoleCommand("help") {}
	void init(void *params) { console = static_cast<Console*>(params); }

	void help(void) {
		console->println("Available commands:");

		for (uint8_t i = 0; i < console->getCmdListSize(); i++) {
				console->printf("\t%s\n", console->getCmdIndexed(i)->getName());
		}
		console->printchar('\n');
	}

	char execute(int argc, char **argv) {
		help();
		return CMD_OK;
	}
}cmdhelp;


class CmdCC25 : public ConsoleCommand {
	Console *console;

public:
    CmdCC25() : ConsoleCommand("cc2500") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->println("usage: cc2500 <[-r, --reset, -rs, status]>");
		console->println("\t-r <0-2E>, Read Register");
		console->println("\t-w <0-2E>, Write Register");
		console->println("\t-rs <30-3D>, Read status register");
		console->println("\t--reset, Reset CC2500 module");
	}

	char reset(void){
		if(CC2500_Reset()){
			console->print("Reset ok\n");
		}else{
			console->print("Reset fail\n");
		}
		return CMD_OK;
	}

	char readRegister(uint8_t reg){
		uint8_t val;
		val = CC2500_ReadReg(reg);
		console->printf("Reg[0x%02x] = %x\n", reg, val);
		return CMD_OK;
	}

	uint32_t dataRate(void){
		uint8_t drate_m = CC2500_ReadReg(CC2500_11_MDMCFG3);
		uint8_t drate_e = CC2500_ReadReg(CC2500_10_MDMCFG4) & 15;
		float rdata = ((256.0 + drate_m) * (1 << drate_e)) / (1 << 28);
		return (uint32_t)(rdata * 26000000); // Fosc
	}

	int8_t rssi(uint8_t rssi_offset){
		uint16_t rssi_dec = CC2500_ReadStatus(CC2500_34_RSSI);
		if(rssi_dec >= 128)
			return (rssi_dec - 256) / 2 - rssi_offset;
		return rssi_dec / 2 - rssi_offset;
	}

	char execute(int argc, char **argv) {
        char *param;
        uint32_t value;

		if(argc == 1 || !xstrcmp(argv[1], "help")){
            help();
			return CMD_OK;
		}

		if(!xstrcmp(argv[1], "status")){
			uint32_t rdata = dataRate();
			console->printf("Data Rate: %dbps\n", rdata);
			uint8_t rssi_offset;
			if(rdata < 10000)
				rssi_offset = 71;
			else if(rdata < 250000)
				rssi_offset = 69;
			else
				rssi_offset = 72;

			console->printf("RSSI: %ddBm\n", rssi(rssi_offset));
			return CMD_OK;
		}

		if(!xstrcmp(argv[1], "reset")){
			return reset();
		}

		if((param = getOptValue("-r", argc, argv)) != NULL){
			if(ha2u(param, &value)){
				return readRegister(value & 255);
			}
		}

		if((param = getOptValue("-rs", argc, argv)) != NULL){
			if(ha2u(param, &value)){
				console->printf("Status [0x%02x] = %x\n", value, CC2500_ReadStatus(value & 255));
				return CMD_OK;
			}
		}

		if((param = getOptValue("-w", argc, argv)) != NULL){
			uint32_t reg, val;
			if(nextHex(&param, &reg)){
				param++; // skip string terminator
				if(nextHex(&param, &val)){
					CC2500_WriteReg(reg, val);
					return CMD_OK;
				}
			}
		}

        return CMD_BAD_PARAM;
	}
}cmdcc25;

class CmdStatus : public ConsoleCommand {
	Console *console;
public:
    CmdStatus() : ConsoleCommand("status") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}

	void systemFlags(void){
        uint32_t flags = multiprotocol_flags_get();

		console->printf(
			"RX               : %d\n"
			"Change protocol  : %d\n"
			"Range            : %d\n"
			"PPM              : %d\n"
			"Bind done        : %d\n",
			!!(flags & FLAG_RX),
            !!(flags & FLAG_CHANGE_PROTOCOL),
            !!(flags & FLAG_RANGE),
            !!(flags & FLAG_PPM),
            !!(flags & FLAG_BIND)
		);
		console->printf(
			"Wait bind        : %d\n"
			"Tx pause         : %d\n"
			"Input signal     : %d\n",
            !!(flags & FLAG_WAIT_BIND),
			!!(flags & FLAG_TX_PAUSE),
            !!(flags & FLAG_INPUT_SIGNAL)
		);
	}
#ifdef ENABLE_PPM
	void channelValues(void){
		for(uint8_t i = 0; i < radio.channel_aux + MAX_AUX_CHANNELS; i++){
        	console->print("CH[%u]:\t%u\n", i, radio.channel_data[i]);
    	}
	}
#endif

	char execute(int argc, char **argv) {
        console->println("\n========================================");
        console->println("            System Status");
		console->println("========================================");

        console->println("\n  System Flags");
        console->println("----------------------------------------");
        systemFlags();

#ifdef ENABLE_BATTERY_MONITOR
        console->println("\n  Battery");
        console->println("----------------------------------------");
        console->printf("Voltage          : %umV\n", batteryGetVoltage());
#endif
#ifdef ENABLE_PPM
        console->println("\n  Channel data");
        console->println("----------------------------------------");
		channelValues();
#endif
        uint8_t aux = appGetCurrentMode();
		console->println("\n  Operating Mode");
        console->println("----------------------------------------");
        console->printf("Mode             : %s\n\n",
            aux == MODE_MULTIPROTOCOL ? "Multiprotocol" : "Game Controller");
        return CMD_OK;
	}
}cmdstatus;

class CmdId : public ConsoleCommand {
	Console *console;
public:
    CmdId() : ConsoleCommand("id") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}

	char execute(int argc, char **argv) {
        console->printf("Current ID: %x\n", multiprotocol_protocol_id_get());
		return CMD_OK;
	}
}cmdid;

class CmdReset : public ConsoleCommand {
	Console *console;
public:
    CmdReset() : ConsoleCommand("reset") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(int argc, char **argv) {NVIC_SystemReset();}
}cmdreset;

class CmdBind : public ConsoleCommand {
	Console *console;
public:
    CmdBind() : ConsoleCommand("bind") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(int argc, char **argv) {
        uint32_t flags = multiprotocol_flags_get();

		if((flags & FLAG_BIND) == 0){
			console->print("Bind already in progress");
		}else{
			appReqModeChange(MODE_MULTIPROTOCOL);
            multiprotocol_flags_set(FLAG_CHANGE_PROTOCOL);
		    multiprotocol_flags_clr(FLAG_BIND);
			if((flags & FLAG_INPUT_SIGNAL) == 0){
				console->print("No input signal!!");
			}
		}
		return CMD_OK;
	}
}cmdbind;

uint16_t ppm_sim_data[] = {3000, 3000, 1950, 3000};
static void ppm_sim(void)
{
	setPpmFlag(ppm_sim_data, 4);
}

class CmdTest : public ConsoleCommand {
	Console *console;
	int32_t tim;
public:
    CmdTest() : ConsoleCommand("test") {}
	void init(void *params) { console = static_cast<Console*>(params); tim = -1;}
	void help(void) {}
	char execute(int argc, char **argv) {
		switch(argv[2][0]){
			case '0': {
                uint16_t *channel_data = multiprotocol_channel_data_get();
				for(uint8_t i = 0; i < MAX_CHN_NUM; i++ ){
					console->printf("\nChannel[%u]: %u", i, channel_data[i]);
				}
				console->printchar('\n');
				break;
            }
			case '1':
                DBG_DUMP_MEM((const uint8_t*)eeprom_data, EEPROM_SIZE);
				break;

			case '2':
				if(tim != -1 ){
					break;
				}
				console->print("Starting ppm simulation");
				tim = startTimer(20, SWTIM_AUTO_RELOAD, ppm_sim);
				break;
			case '3':
				console->print("Stoping ppm simulation");
				stopTimer(tim);
				tim = -1;
				break;
			case '4':
				//uint32_t start = HAL_GetTick();
				/* Some function to test */
				//console->print("Time: %ums\n", HAL_GetTick() - start);
				break;

		}
		return CMD_OK;
	}
}cmdtest;

class CmdMode : public ConsoleCommand {
	Console *console;
public:
    CmdMode() : ConsoleCommand("mode") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(int argc, char **argv) {
		if(argc == 1){
			console->printf("Current mode: %d\n",appGetCurrentMode());
			return CMD_OK;
		}

        int32_t int_value;
		if(ia2i(argv[2], &int_value)){
			appReqModeChange((uint8_t)int_value);
            return CMD_OK;
		}

		return CMD_BAD_PARAM;
	}
}cmdmode;

class CmdEeprom : public ConsoleCommand {
	Console *console;
public:
    CmdEeprom() : ConsoleCommand("eeprom") {}
	void init(void *params) { console = static_cast<Console*>(params); }

	void help(void) {
		console->print("usage: eeprom [dump|load|save|erase|default]");
	}

	void id(void){
		uint32_t *ptr = (uint32_t*)eeprom_data;

		console->printf(
			"ID          \t\t0x%X\n",
			*(ptr + EEPROM_ID_OFFSET)
		);
	}

	void channelRanges(void){
		console->printf(
			"CH MAX      \t\t%d\n"
			"CH MIN      \t\t%d\n"
			"CH switch   \t\t%d\n"
			"PPM MAX     \t\t%d\n"
			"PPM MIN     \t\t%d\n",
			eeprom_data[IDX_CHANNEL_MAX_100],
			eeprom_data[IDX_CHANNEL_MIN_100],
			eeprom_data[IDX_CHANNEL_SWITCH],
			eeprom_data[IDX_PPM_MAX_100],
			eeprom_data[IDX_PPM_MIN_100]
		);
	}

	char execute(int argc, char **argv) {
		if(argc == 1){
			console->println("----------------------------------------");
			channelRanges();
			id();
			console->println("----------------------------------------");
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"help") == 0){
			help();
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"erase") == 0){
			console->printf("Erasing EEPROM: %s\n", EEPROM_Erase() == 0? "Fail": "ok");
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"dump") == 0){
			DBG_DUMP_MEM_LINE((uint8_t*)eeprom_data, EEPROM_SIZE, 0);
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"save") == 0){
			appSaveEEPROM();
			return CMD_OK;
		}

        if(xstrcmp(argv[1],"load") == 0){
			appLoadEEPROM();
			return CMD_OK;
		}

        if(xstrcmp(argv[1],"default") == 0){
            appDefaultEEPROM();
			return CMD_OK;
		}

		return CMD_BAD_PARAM;
	}
}cmdeeprom;

#ifdef ENABLE_BATTERY_MONITOR
class CmdAdc : public ConsoleCommand {
	Console *console;
public:
    CmdAdc() : ConsoleCommand("adc") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->println("usage: adc [ calibrate | -div | -rs ]");
		console->println("\t calibrate  Adc calibration based on internal voltage reference");
        console->println("\t-div <racio> : Battery voltage divider racio");
		console->println("\t-rs <resistor> Sense resistor");
	}

	int readFloatParameter(const char *opt, uint32_t argc, char **argv, uint16_t *dst, void (*func)(float)){
		f2u_u t;
		char *param = getOptValue(opt, argc, argv);
		double d;

		if(param == NULL){
			return CMD_NOT_FOUND;
		}

		if(nextDouble(&param, &d) == 0){
			return CMD_BAD_PARAM;
		}

		t.f = d;
		*dst = (uint16_t)t.u;
		*(dst + 1) = (uint16_t)(t.u>>16);
		func(t.f);

		return CMD_OK;
	}

	void batVoltageCalibration(void){
		console->printf("Bat voltage divider \t%.3f\n", adcGetVdivRacio());
	}

	void adcResolution(void){
		console->printf("Adc resolution  \t%.3fmV/step\n", adcGetResolution());
	}

	void current(void){
		console->printf("Current  \t\t%umA\n", batteryGetCurrent());
	}

	void senseResistor(void){
		console->printf("Sense resistor  \t%.3f Ohm\n", adcGetSenseResistor());
	}

	char execute(int argc, char **argv) {
		if(argc < 2){
			batVoltageCalibration();
			adcResolution();
			current();
			senseResistor();
			return CMD_OK;
		}

		if(xstrcmp(argv[0],"help") == 0){
			help();
			return CMD_OK;
		}

		if(xstrcmp(argv[0],"calibrate") == 0){
			if(adcCalibrate()){
				adcResolution();
			}else{
				console->print("Fail to calibrate adc\n");
			}
			return CMD_OK;
		}

		if(readFloatParameter("-div", argc, argv, &eeprom_data[IDX_BAT_VOLTAGE_DIV], adcSetVdivRacio) == CMD_OK)
			return CMD_OK;
		if(readFloatParameter("-rs", argc, argv, &eeprom_data[IDX_SENSE_RESISTOR], adcSetSenseResistor) == CMD_OK)
			return CMD_OK;
		return CMD_BAD_PARAM;
	}
}cmdadc;
#endif

#ifdef ENABLE_BUZZER
class CmdBuz : public ConsoleCommand {
	Console *console;
public:
    CmdBuz() : ConsoleCommand("buz") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->println("usage: buz <-r|-v|-t>");
		console->println(
			"-v <volume>, set volume\n"
			"-t <freq> <duration>"
		);
	}
	char execute(int argc, char **argv) {
		char *param;

		if(argc < 1){
			help();
			console->printf("Current level %u\n", buzSetLevel(0));
			return CMD_OK;
		}

		if((param = getOptValue("-v", argc, argv)) != NULL){
			int32_t val;
			if(nextInt(&param, &val)){
				console->printf("Current level %u\n", buzSetLevel(val));
				*((uint8_t*)eeprom_data + IDX_BUZ_VOLUME) = val&255;
				return CMD_OK;
			}
		}

		if((param = getOptValue("-t",argc,argv)) != NULL){
			int32_t freq, duration;
			if(nextInt(&param, &freq)){
				param++; // skip string terminator
				if(nextInt(&param, &duration)){
					buzPlayTone(freq, duration);
					return CMD_OK;
				}
			}
			return CMD_OK;
		}

		return CMD_BAD_PARAM;
	}
}cmdbuz;
#endif

#ifdef ENABLE_DFU
#include "dfu_boot.h"

class CmdDfu : public ConsoleCommand{
    Console *console;
public:
    void init(void *params) { console = static_cast<Console*>(params); }
    CmdDfu () : ConsoleCommand("dfu") { }
    char execute(int argc, char **argv){

    	console->print("Entering DFU mode\n");
    	//LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    	//LCD_Update();

		DFU_Enable();
    	NVIC_SystemReset();

    	return CMD_OK;
	}
    void help(void){}
}cmddfu;
#endif

ConsoleCommand *laser4_commands[]{
    &cmdhelp,
    &cmdcc25,
	&cmdreset,
	&cmdid,
	&cmdbind,
	&cmdstatus,
	&cmdtest,
	&cmdmode,
	&cmdeeprom,
#ifdef ENABLE_BATTERY_MONITOR
	&cmdadc,
#endif
#ifdef ENABLE_BUZZER
	&cmdbuz,
#endif
#ifdef ENABLE_DFU
	&cmddfu,
#endif
    NULL
};
#endif