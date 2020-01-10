
#include "app.h"
#include "iface_cc2500.h"
#include "multiprotocol.h"

#ifdef ENABLE_CONSOLE
/**
 * Possible commands
 * > bind
 * > select protocol
 * > override channel
 */

char *getOptValue(char *opt, uint32_t argc, char **argv){
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
	void help(void) {}

	char reset(void){
		if(CC2500_Reset()){
			console->print("Reset ok\n");
		}else{
			console->print("Reset fail\n");
		}
		return CMD_OK;
	}

	char readRegister(uint8_t reg){
	//uint8_t val;	
		//val = CC2500_ReadReg(reg);
		//console->print("Reg[0x%02x] = %x\n", reg, val);
		return CMD_OK;
	}
    
	char execute(void *ptr) {
        char *argv[4], *param;
        uint32_t argc, int_value;


        argc = strToArray((char*)ptr, argv);

		if(argc == 0){
			return CMD_BAD_PARAM;
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

        return CMD_BAD_PARAM;        
	}	
}cmdcc25;

class CmdStatus : public ConsoleCommand {
	Console *console;
    
public:
    CmdStatus() : ConsoleCommand("status") {}	
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}   
	char execute(void *ptr) {
        console->print("\n"
			"Rx              [%d]\n"
			"Change protocol [%d]\n"
			"Range           [%d]\n"
			"PPM             [%d]\n"
			"Bind done       [%d]\n"
			"Tx pause        [%d]\n"
			"Input signal    [%d]\n"
			"\n",
			IS_RX_FLAG_on,
			IS_CHANGE_PROTOCOL_FLAG_on,
			IS_RANGE_FLAG_on,
			IS_PPM_FLAG_on,
			IS_BIND_DONE,
			IS_TX_MAIN_PAUSE_on,
			IS_INPUT_SIGNAL_on
		);
        return CMD_OK;        
	}	
}cmdstatus;

class CmdMockPPM : public ConsoleCommand {
	Console *console;    
public:
    CmdMockPPM() : ConsoleCommand("mock-ppm") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(void *ptr) {
		uint32_t int_value;
		if(nextHex((char**)&ptr, &int_value)){
			if(int_value == 1){
				//setTimer(6000, multiprotocol_mock_ppm);
			}else{
				//stopTimer();				
			}
			return CMD_OK;
		}
		return CMD_BAD_PARAM;
	}
}cmdmockppm;	

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
		BIND_BUTTON_FLAG_on;
		BIND_IN_PROGRESS;	
		return CMD_OK;
	}
}cmdbind;

class CmdTest : public ConsoleCommand {
	Console *console;    
public:
    CmdTest() : ConsoleCommand("test") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
	char execute(void *ptr) {
		uint8_t i;		
		for(i = 0; i < MAX_PPM_CHANNELS; i++ ){
			console->print("\nChannel[%u]: %u", i, radio.channel_data[i]);
		}
		console->xputchar('\n');
		return CMD_OK;
	}
}cmdtest;

ConsoleCommand *laser4_commands[]{
    &cmdhelp,
    &cmdcc25,
	&cmdreset,
	&cmdmockppm,
	&cmdbind,
	&cmdstatus,
	&cmdtest,
    NULL
};
#endif