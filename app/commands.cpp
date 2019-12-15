
#include "app.h"

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
		console->xputs("Available commands:\n\n");
		
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



static uint32_t cmdSpiWrite(uint32_t argc, char **argv){
uint32_t data;
char *value = getOptValue((char*)"-w", argc, argv);

    if(value == NULL){
        return CMD_BAD_PARAM;
    }

    if(nextHex(&value, &data)){
        CC25_CSN_on;
        SPI_Write(data);
        CC25_CSN_off;
        return CMD_OK;
    }

    return CMD_BAD_PARAM;
}

class CmdSpi : public ConsoleCommand {
	Console *console;
    
public:
    CmdSpi() : ConsoleCommand("spi") {}	
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {}
    
	char execute(void *ptr) {
        char *argv[4];
        uint32_t argc;
        argc = strToArray((char*)ptr, argv);
        return cmdSpiWrite(argc, argv);
        
	}	
}cmdspi;

ConsoleCommand *laser4_commands[]{
    &cmdhelp,
    &cmdspi,
    NULL
};
