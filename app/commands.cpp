
#include "app.h"
#include "iface_cc2500.h"
#include "multiprotocol.h"

#ifdef ENABLE_CLI

char *getOptValue(const char *opt, uint32_t argc, char **argv){
    for(uint32_t i = 0; i < argc; i++){
        if(xstrcmp(opt, argv[i]) == 0){
            return argv[i+1];
        }
    }
    return NULL;
}

char *skipSpaces(char *str){
	while((*str == ' ' || *str == '\t') && *str != '\0') 
		str++;
	return str;
}

uint32_t strToArray(char *str, char **argv){
uint32_t argc = 0;

    if(str == NULL){
        return 0;
    }

	str = skipSpaces(str);

	if(*str == '\0')
		return 0;

	argv[argc++] = str;

    while(*str != '\0'){
        if(*str == ' '){
			*str = '\0';
			str = skipSpaces(str + 1);
			if(*str != '\0')
				argv[argc++] = str;
        }else{
			str++;
		}
    }

	return argc;
}

class CmdHelp : public ConsoleCommand {
	Console *console;
public:
    CmdHelp() : ConsoleCommand("help") {}	
	void init(void *params) { console = static_cast<Console*>(params); }

	void help(void) {
		console->xputs("Available commands:\n");
		
		for (uint8_t i = 0; i < console->getCmdListSize(); i++) {			
				console->print("\t%s\n", console->getCmdIndexed(i)->getName());
		}
		console->xputchar('\n');
	}

	char execute(void *ptr) {
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
		console->xputs("usage: cc2500 <[-r, --reset, -rs, status]>");	
		console->xputs("\t-r <0-2E>, Read Register");
		console->xputs("\t-w <0-2E>, Write Register");
		console->xputs("\t-rs <30-3D>, Read status register");	
		console->xputs("\t--reset, Reset CC2500 module");
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
		console->print("Reg[0x%02x] = %x\n", reg, val);
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

    
	char execute(void *ptr) {
        char *argv[4], *param;
        uint32_t argc, int_value;

        argc = strToArray((char*)ptr, argv);

		if(argc == 0){
			return CMD_BAD_PARAM;
		}

		if(getOptValue((char*)"help", argc, argv) != NULL){
			help();
			return CMD_OK;
		}

		if(getOptValue((char*)"status", argc, argv) != NULL){
			uint32_t rdata = dataRate();
			console->print("Data Rate: %dbps\n", rdata);
			uint8_t rssi_offset;
			if(rdata < 10000)
				rssi_offset = 71;
			else if(rdata < 250000)
				rssi_offset = 69;
			else
				rssi_offset = 72;

			console->print("RSSI: %ddBm\n", rssi(rssi_offset));
			return CMD_OK;
		}

		if(getOptValue((char*)"--reset", argc, argv) != NULL){
			return reset();
		}

		param = getOptValue((char*)"-r", argc, argv);
		if(param != NULL){
			if(nextHex((char**)&param, &int_value)){
				return readRegister(int_value & 255);
			}
		}

		param = getOptValue((char*)"-rs", argc, argv);
		if(param != NULL){
			if(nextHex((char**)&param, &int_value)){
				console->print("Status [0x%02x] = %x\n", int_value, CC2500_ReadStatus(int_value & 255));
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

	void batteryVoltage(void){
		console->print(
			"Battery voltage: %umV\n",
			batteryGetVoltage()
		);
	}

	void systemFlags(void){
		console->print(
			"RX              [%d]\n"
			"Change protocol [%d]\n"
			"Range           [%d]\n"
			"PPM             [%d]\n"
			"Bind done       [%d]\n",
			IS_RX_FLAG_on,
			IS_CHANGE_PROTOCOL_FLAG_on,
			IS_RANGE_FLAG_on,
			IS_PPM_FLAG_on,
			IS_BIND_DONE
		);
		console->print(	
			"Wait bind       [%d]\n"
			"Tx pause        [%d]\n"
			"Input signal    [%d]\n",			
			IS_WAIT_BIND_on,
			IS_TX_MAIN_PAUSE_on,
			IS_INPUT_SIGNAL_on
		);
	}

	void channelValues(void){
		for(uint8_t i = 0; i < radio.ppm_chan_max + MAX_AUX_CHANNELS; i++){
        	console->print("CH[%u]:\t%u\n", i, radio.channel_data[i]);
    	}
	}

	void mode(void){
		uint8_t aux = appGetCurrentMode();
		console->print("Mode: %s\n", aux == MODE_MULTIPROTOCOL ? "Multiprotocol" : "Game Controller");
	}

	char execute(void *ptr) {
		console->xputs("\n----------------------------------------");
        batteryVoltage();
		console->xputs("----------------------------------------");
        systemFlags();
		console->xputs("----------------------------------------");		
		channelValues();
		console->xputs("----------------------------------------");
		mode();
		console->xputs("----------------------------------------");
        return CMD_OK;        
	}	
}cmdstatus;

class CmdMockPPM : public ConsoleCommand {
	Console *console;    
public:
    CmdMockPPM() : ConsoleCommand("id") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->xputs("usage: id [0|1]\n"
				"\t0,1 : Generate random id\n");
	}

	char execute(void *ptr) {
		char *p = (char*)ptr;
		uint32_t int_value;

		if(xstrcmp(p,"help") == 0){	
			help();
			return CMD_OK;
		}

		nextHex(&p, &int_value);
		if(int_value == 1){
			console->print("Random ID: %x\n", xrand());
		}else{
			console->print("Current ID: %x\n", radio.protocol_id_master);
		}	
		return CMD_OK;		
	}
}cmdid;	

class CmdReset : public ConsoleCommand {
	Console *console;    
public:
    CmdReset() : ConsoleCommand("reset") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(void *ptr) {NVIC_SystemReset();}
}cmdreset;

class CmdBind : public ConsoleCommand {
	Console *console;    
public:
    CmdBind() : ConsoleCommand("bind") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(void *ptr) {
		appReqModeChange(MODE_MULTIPROTOCOL);
		CHANGE_PROTOCOL_FLAG_on;
		BIND_IN_PROGRESS;
		if(IS_INPUT_SIGNAL_off){
			console->xputs("No input signal!!");
		}
		return CMD_OK;
	}
}cmdbind;

uint16_t ppm_sim_data[] = {3000, 3000, 1950, 3000};
void ppm_sim(void){
	setPpmFlag(ppm_sim_data, 4);
}

class CmdTest : public ConsoleCommand {
	Console *console;
	int32_t tim;
public:
    CmdTest() : ConsoleCommand("test") {}
	void init(void *params) { console = static_cast<Console*>(params); tim = -1;}
	void help(void) {}
	char execute(void *ptr) {
		uint32_t test_code;
		
		if(!nextHex((char**)&ptr, &test_code)){
			return CMD_OK;
		}

		switch(test_code){
			case 0:
				for(uint8_t i = 0; i < MAX_CHN_NUM; i++ ){
					console->print("\nChannel[%u]: %u", i, radio.channel_data[i]);
				}
				console->xputchar('\n');
				break;
			case 1:
				dbg_HexDump((uint8_t*)eeprom_data, EEPROM_SIZE);
				break;

			case 2:
				if(tim != -1 ){
					break;
				}
				console->xputs("Starting ppm simulation");
				tim = startTimer(20, SWTIM_AUTO_RELOAD, ppm_sim);
				break;
			case 3:
				console->xputs("Stoping ppm simulation");
				stopTimer(tim);
				tim = -1;
				break;
			case 4:
				uint32_t start = HAL_GetTick();				
				
				console->print("Time: %ums\n", HAL_GetTick() - start);
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
	char execute(void *ptr) {
		if(*(char*)ptr == '\0'){
			console->print("Current mode: %d\n",appGetCurrentMode());
			return CMD_OK;
		}
		uint32_t int_value;
		if(nextHex((char**)&ptr, &int_value)){
			appReqModeChange((uint8_t)int_value);
		}
		return CMD_OK;
	}
}cmdmode;

class CmdEeprom : public ConsoleCommand {
	Console *console;    
public:
    CmdEeprom() : ConsoleCommand("eeprom") {}
	void init(void *params) { console = static_cast<Console*>(params); }

	void help(void) {
		console->xputs("usage: eeprom [dump|save|erase]");
	}

	void id(void){
		console->print(			
			"ID \t	\t\t0x%X\n",
			*(uint32_t*)(eeprom_data+EEPROM_ID_OFFSET));	
	}

	void channelRanges(void){
		console->print(
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

	char execute(void *ptr) {
		char *p = (char*)ptr;

		if(*p == '\0'){
			console->xputs("----------------------------------------");
			channelRanges();
			id();
			console->xputs("----------------------------------------");
			return CMD_OK;
		}

		if(xstrcmp(p,"help") == 0){	
			help();
			return CMD_OK;
		}

		if(xstrcmp(p,"erase") == 0){
			DBG_PRINT("Erasing NV Data: %s\n", NV_Erase() == 0? "Fail": "ok");
			return CMD_OK;
		}
		
		if(xstrcmp(p,"dump") == 0){
			DBG_DUMP_LINE((uint8_t*)eeprom_data, EEPROM_SIZE, 0);			
			return CMD_OK;
		}
		
		if(xstrcmp(p,"save") == 0){			
			appSaveEEPROM();
			return CMD_OK;
		}

		uint32_t int_value;
		if(nextHex((char**)&ptr, &int_value)){
			appReqModeChange((uint8_t)int_value);
		}
		return CMD_OK;
	}
}cmdeeprom;

class CmdAdc : public ConsoleCommand {
	Console *console;    
public:
    CmdAdc() : ConsoleCommand("adc") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->xputs("usage: adc [calibrate|-r]");
		console->xputs("\t calibrate  Adc calibration based on internal voltage reference\n"
						"\t-div <racio> : Battery voltage divider racio\n"
						"\t-rs <resistor> Sense resistor");
	}

	int readFloatParameter(const char *opt, uint32_t argc, char **argv, uint16_t *dst, void (*func)(float)){
		f2u_u t;
		char *param = getOptValue((char*)opt, argc, argv);
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
		console->print("Bat voltage divider \t%.3f\n", adcGetVdivRacio());
	}

	void adcResolution(void){
		console->print("Adc resolution  \t%.3fmV/step\n", adcGetResolution());
	}

	void current(void){
		console->print("Current  \t\t%umA\n", batteryGetCurrent());
	}

	void senseResistor(void){
		console->print("Sense resistor  \t%.3f Ohm\n", adcGetSenseResistor());
	}

	char execute(void *ptr) {
		char *argv[4];
        uint32_t argc;

		argc = strToArray((char*)ptr, argv);
		
		if(argc == 0){
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


class CmdBuz : public ConsoleCommand {
	Console *console;    
public:
    CmdBuz() : ConsoleCommand("buz") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->xputs("usage: buz <-r|-v|-t>");
		console->xputs(
			"-v <volume>, set volume\n"
			"-t <freq> <duration>\n"
		);
	}
	char execute(void *ptr) {
		char *argv[4], *param;
        uint32_t argc;
		argc = strToArray((char*)ptr, argv);

		if(argc == 0){
			help();
			console->print("Current level %u\n", buzSetLevel(0));
			return CMD_OK;
		}

		if((param = getOptValue("-v", argc, argv)) != NULL){
			int32_t val;
			if(nextInt(&param, &val)){
				console->print("Current level %u\n", buzSetLevel(val));
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
	&cmdadc,
	&cmdbuz,
    NULL
};
#endif