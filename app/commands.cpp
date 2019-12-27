
#include "app.h"
#include "iface_cc2500.h"
#include "radio.h"

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

extern radio_t radio;
class Radio : public ConsoleCommand {
	Console *console;
    
public:
    Radio() : ConsoleCommand("radio") {}	
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}

	void printFlags(void){
		console->print("\n"
			"Rx              [%d]\n"
			"Change protocol [%d]\n"
			"Range           [%d]\n"
			"PPM             [%d]\n"
			"Bind done       [%d]\n"
			"Tx pause        [%d]\n"
			"Input signal    [%d]\n"
			"\n",
			IS_RX_FLAG_on(radio.flags),
			IS_CHANGE_PROTOCOL_FLAG_on(radio.flags),
			IS_RANGE_FLAG_on(radio.flags),
			IS_PPM_FLAG_on(radio.flags),
			IS_BIND_DONE(radio.flags),
			IS_TX_MAIN_PAUSE_on(radio.flags),
			IS_INPUT_SIGNAL_on(radio.flags)
		);
	}
    
	char execute(void *ptr) {
        char *argv[4];
        uint32_t argc;

        argc = strToArray((char*)ptr, argv);

		if(argc == 0){
			printFlags();
			return CMD_OK;
		}

		if(getOptValue((char*)"--bind", argc, argv) != NULL){
			BIND_BUTTON_FLAG_on(radio.flags);
			BIND_IN_PROGRESS(radio.flags);	
			return CMD_OK;
		}
        return CMD_BAD_PARAM;        
	}	
}cmdradio;

ConsoleCommand *laser4_commands[]{
    &cmdhelp,
    &cmdcc25,
	&cmdradio,
    NULL
};
