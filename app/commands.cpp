
#include "board.h"
#include "console.h"
#include "app.h"
#include "laser4_plus.h"
#include "iface_cc2500.h"
#include "multiprotocol.h"

#ifdef ENABLE_CLI

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
        (void) argc;
        (void) argv;

		help();
		return CMD_OK;
	}
}cmdhelp;

#ifdef CC2500_INSTALLED
class CmdCC25 : public ConsoleCommand {
	Console *console;

public:
    CmdCC25() : ConsoleCommand("cc2500") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
		console->println("usage: cc2500 <[rs, rr, wr, status, reset]>");
		console->println("\trr <0-2E>, Read Register");
		console->println("\twr <0-2E>, Write Register");
		console->println("\trs <30-3D>, Read status register");
		console->println("\treset, Reset CC2500 module");
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
        uint32_t value;

		if(argc == 1){
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

		if(!xstrcmp(argv[1], "rr")){
			if(ha2u(argv[2], &value)){
				return readRegister(value & 255);
			}
		}

		if(!xstrcmp(argv[1], "rs")){
			if(ha2u(argv[2], &value)){
				console->printf("Status [0x%02x] = %x\n", value, CC2500_ReadStatus(value & 255));
				return CMD_OK;
			}
		}

		if(!xstrcmp(argv[1], "wr")){
			uint32_t reg, val;
			if(ha2u(argv[2], &reg) && ha2u(argv[3], &val)){
                CC2500_WriteReg(reg, val);
                return CMD_OK;
            }
		}

        return CMD_BAD_PARAM;
	}
}cmdcc25;
#endif

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
			"PPM              : %d\n"
			"Change protocol  : %d\n"
			//"Range            : %d\n"
			"Bind done        : %d\n",
			!!(flags & FLAG_RX),
            !!(flags & FLAG_PPM),
            !!(flags & FLAG_CHANGE_PROTOCOL),
            //!!(flags & FLAG_RANGE),
            !!(flags & FLAG_BIND)
		);

		console->printf(
			"Wait bind        : %d\n"
			//"Tx pause         : %d\n"
			"Input signal     : %d\n",
            !!(flags & FLAG_WAIT_BIND),
			//!!(flags & FLAG_TX_PAUSE),
            !!(flags & FLAG_INPUT_SIGNAL)
		);
	}

	void channelValues(void){
        const uint16_t *channel_data;
        uint8_t nchannels;
        multiprotocol_channel_data_get(&channel_data, &nchannels);
		for(uint8_t i = 0; i < nchannels; i++){
        	console->printf("CH[%u]\t%u\t%u\n", i, ppm_value_get(i), channel_data[i]);
    	}
	}

	char execute(int argc, char **argv) {
        (void) argc;
        (void) argv;

        console->println("\n========================================");
        console->println("            System Status");
		console->println("========================================");
        console->printf("\nVersion: %s\n", VERSION_STRING);
        console->println("\n  System Flags");
        console->println("----------------------------------------");
        systemFlags();
        console->println("\n  Up Time");
        console->println("----------------------------------------");
        uint32_t uptime = appGetUpTime();
        console->printf(" %02u:%02u\n",  uptime/3600, (uptime%3600) / 60);

#ifdef ENABLE_BATTERY_MONITOR
        console->println("\n  Battery");
        console->println("----------------------------------------");
        vires_t batsoc;
        batteryReadVI(&batsoc);
        console->printf("Voltage          : %umV\n", batsoc.vbat);
        console->printf("Current          : %umA\n", batsoc.cur);
        console->printf("Consumed         : %umAh\n", appGetBatConsumed());
#endif

        console->println("\n  Channel data");
        console->println("----------------------------------------");
        console->println("Channel\tPPM\tServo");
		channelValues();
        const char *mode_name;
        switch(appModeGet()){
            case MODE_PPM: mode_name = "TX 35MHz"; break;
            case MODE_SERIAL: mode_name = "Serial"; break;
            case MODE_HID: mode_name = "Game controller"; break;
            case MODE_CC2500: mode_name = "CC2500"; break;
            default: mode_name = "NONE"; break;
        }
		console->println("\n  Operating Mode");
        console->println("----------------------------------------");
        console->printf("Mode             : %s\n\n",mode_name);
        return CMD_OK;
	}
}cmdstatus;

class CmdId : public ConsoleCommand {
	Console *console;
public:
    CmdId() : ConsoleCommand("id") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
        console->println("usage: id");
        console->println("\t prints transmitter identification");
    }

	char execute(int argc, char **argv) {
        (void) argc;
        (void) argv;
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
	char execute(int argc, char **argv) {
        (void) argc;
        (void) argv;
        NVIC_SystemReset();
    }
}cmdreset;

class CmdBind : public ConsoleCommand {
	Console *console;
public:
    CmdBind() : ConsoleCommand("bind") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
        console->println("usage: bind");
        console->println("\tStart bind process");
    }

	char execute(int argc, char **argv) {
        (void) argc;
        (void) argv;

        uint32_t flags = multiprotocol_flags_get();

		if((flags & FLAG_BIND) == 0){
			console->print("Bind already in progress");
		}else{
			appModeRequest(MODE_CC2500);
            multiprotocol_flags_set(FLAG_CHANGE_PROTOCOL);
		    multiprotocol_flags_clr(FLAG_BIND);
			if((flags & FLAG_INPUT_SIGNAL) == 0){
				console->print("No input signal!!");
			}
		}
		return CMD_OK;
	}
}cmdbind;


class CmdPpm : public ConsoleCommand {
    Console *console;
    int32_t ppm_timer_id;
public:
    CmdPpm() : ConsoleCommand("ppm") {}
	void init(void *params) { console = static_cast<Console*>(params); ppm_timer_id = -1;}
	void help(void) {
        console->println("usage: ppm <sim|set>");
        console->println("sim [0|1],\t\tEnable/disable simulation");
        console->println("set <channel> <value>,"
            "\tSet simulated channel value [900;2100]");
        console->println("trim <channel> <min> <max>,"
            "\t Trim stick to ppm");
        console->println("servo <channel> <min> <max>,"
            "\t Servo limits range [750;2250]");
    }

	char execute(int argc, char **argv) {
        int32_t channel;
        int32_t value, min, max;

        if(argc == 1){
            help();
            return CMD_OK;
        }
        if(!xstrcmp(argv[1], "sim")){
            // Only 4 channels are simulated, remaining channels are obtained
            // From encoder and switches
            int32_t sim_enable;
            if(ia2i(argv[2], &sim_enable)){
                if((sim_enable & 1) && (ppm_timer_id == -1)){
                    for(uint8_t ch = 0; ch < PPM_CH_IN_NUM; ch++){
                        ppm_sim_set_channel_data(ch, ((SERVO_MAX - SERVO_MIN) >> 1) + SERVO_MIN);
                    }
                    console->println("Starting ppm simulation");
                    ppm_timer_id = startTimer(20, SWTIM_AUTO_RELOAD, ppm_sim_handler);
                }else{
                    console->println("Stoping ppm simulation");
                    stopTimer(ppm_timer_id);
                    ppm_timer_id = -1;
                }
            }else{
                console->printf("ppm simulation is %s\n", ppm_timer_id < 0 ? "disabled":"enabled");
            }
            return CMD_OK;
        }

        /* Set value of simulated channels */
        if(!xstrcmp(argv[1], "set")){
            if(ia2i(argv[2], &channel)){
                channel &= 3;
                if(ia2i(argv[3], &value)){
                    if(value > SERVO_MAX_125)value = SERVO_MAX_125;
                    if(value < SERVO_MIN_125)value = SERVO_MIN_125;
                    ppm_sim_set_channel_data(channel, value);
                    return CMD_OK;
                }
            }
        }

        /* Trims channel range, command 'eeprom save' must be issued to
        make trim persistent */
        while(!xstrcmp(argv[1], "trim")){
            if(!ia2i(argv[2], &channel)) break;
            if(!ia2i(argv[3], &min)) break;
            if(!ia2i(argv[4], &max)) break;
            if(channel > PPM_CH_IN_NUM) break;
            if(min > PPM_MAX_PERIOD) min = PPM_MAX_PERIOD;
            if(min < PPM_MIN_PERIOD) min = PPM_MIN_PERIOD;
            if(max > PPM_MAX_PERIOD) max = PPM_MAX_PERIOD;
            if(max < PPM_MIN_PERIOD) max = PPM_MIN_PERIOD;

            eeprom->ppm_range[channel].min = min;
            eeprom->ppm_range[channel].max = max;

            return CMD_OK;
        }
        // Change this parameter to other command??
        while(!xstrcmp(argv[1], "servo")){
            if(!ia2i(argv[2], &channel)) break;
            if(!ia2i(argv[3], &min)) break;
            if(!ia2i(argv[4], &max)) break;
            if(channel > MAX_CHN_NUM) break;
            if(min > SERVO_MAX_125) min = SERVO_MAX_125;
            if(min < SERVO_MIN_125) min = SERVO_MIN_125;
            if(max > SERVO_MAX_125) max = SERVO_MAX_125;
            if(max < SERVO_MIN_125) max = SERVO_MIN_125;

            eeprom->ch_range[channel].min = min;
            eeprom->ch_range[channel].max = max;

            return CMD_OK;
        }

		return CMD_BAD_PARAM;
	}
}cmdppm;

class CmdProto : public ConsoleCommand {
	Console *console;
public:
    CmdProto() : ConsoleCommand("proto") {}
	void init(void *params) { console = static_cast<Console*>(params); }
	void help(void) {
        console->println("usage: proto [n]");
        console->println("Protocol selection by 'switches'");
        console->println("proto, 0-14");
    }
	char execute(int argc, char **argv) {
        app_mode_t cur_mode = appModeGet();
		if(argc == 1){
			console->printf("Current: %d\n", cur_mode);
			return CMD_OK;
		}

        int32_t int_value;
		if(ia2i(argv[1], &int_value)){
			appModeRequest((app_mode_t)int_value);
            return CMD_OK;
		}

		return CMD_BAD_PARAM;
	}
}cmdproto;

class CmdEeprom : public ConsoleCommand {
	Console *console;
public:
    CmdEeprom() : ConsoleCommand("eeprom") {}
	void init(void *params) { console = static_cast<Console*>(params); }

	void help(void) {
		console->println("usage: eeprom [dump|load|save|erase|default]");
	}

	void channelRanges(void){
        for(uint8_t i = 0; i < PPM_CH_IN_NUM; i++){
            console->printf(
                "PPM[%u]\tMin: %uus\tMax: %uus\n",
                    i,
                    eeprom->ppm_range[i].min,
                    eeprom->ppm_range[i].max
            );
        }

        for(uint8_t i = 0; i < MAX_CHN_NUM; i++){
            console->printf(
                "CH[%d]\tMin: %dus\tMax: %dus\n",
                i,
                eeprom->ch_range[i].min,
                eeprom->ch_range[i].max
            );
        }
	}

	char execute(int argc, char **argv) {
		if(argc == 1){
			console->println("========================================");
			console->printf("ID\t: 0x%X\n", eeprom->uid);
            f2u_u t;
            t.u = eeprom->vdiv;
            console->printf("Vdiv\t: %.3f\n", t.f);
            t.u = eeprom->rsense;
            console->printf("RSense\t: %.3f\n", t.f);
			channelRanges();
            console->printf("Buz vol\t: %d\n", eeprom->buz_vol);
			console->println("========================================");
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"erase") == 0){
			console->printf("Erasing EEPROM: %s\n", EEPROM_Erase() == 0? "Fail": "ok");
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"dump") == 0){
			console->hexdump((uint8_t*)eeprom, sizeof(meep_t), 16, 1);
			return CMD_OK;
		}

		if(xstrcmp(argv[1],"save") == 0){
			appSaveEEPROM();
			return CMD_OK;
		}

        if(xstrcmp(argv[1],"load") == 0){
			appInitEEPROM();
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
		console->println("usage: adc [ calibrate | div | rs ]");
		console->println("\tcalibrate,  Adc calibration based on internal voltage reference");
        console->println("\tdiv <racio>, Battery voltage divider racio");
		console->println("\trs <resistor>, Sense resistor");
	}

	char execute(int argc, char **argv) {
        f2u_u t;
        double d;

		if(argc < 2){
			console->printf("Bat voltage divider \t%.3f\n", adcGetVdivRacio());
			console->printf("Adc resolution  \t%.3fmV/step\n", adcGetResolution());
			console->printf("Current  \t\t%umA\n", batteryGetCurrent());
			console->printf("Sense resistor  \t%.3f Ohm\n", adcGetSenseResistor());
			return CMD_OK;
		}

		if(!xstrcmp(argv[1],"calibrate")){
			if(adcCalibrate()){
				console->printf("Adc resolution  \t%.3fmV/step\n", adcGetResolution());
			}else{
				console->print("Fail to calibrate adc\n");
			}
			return CMD_OK;
		}

        if(!xstrcmp(argv[1], "div")){
            if(da2d(argv[2], &d)){
		        t.f = d;
                eeprom->vdiv = t.u;
		        adcSetVdivRacio(t.f);
                return CMD_OK;
            }
        }

        if(!xstrcmp(argv[1], "rs")){
            if(da2d(argv[2], &d)){
		        t.f = d;
                eeprom->rsense = t.u;
		        adcSetSenseResistor(t.f);
                return CMD_OK;
            }
        }

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
		console->println("usage: buz <vol|freq>");
		console->println(
			"vol <volume>, set volume\n"
			"freq <f> <d>"
		);
	}
	char execute(int argc, char **argv) {
        int32_t val;

		if(argc < 1){
			console->printf("Current level %u\n", buzSetLevel(0));
			return CMD_OK;
		}

		if(!xstrcmp(argv[1], "vol")){
			if(ia2i(argv[2], &val)){
				console->printf("Current level %u\n", buzSetLevel(val));
				eeprom->buz_vol = val&255;
				return CMD_OK;
			}
		}

		if(!xstrcmp(argv[1], "freq")){
			int32_t freq, duration;
			if(ia2i(argv[2], &freq) && ia2i(argv[3], &duration)){
                buzPlayTone(freq, duration);
                return CMD_OK;
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
        (void)argc;
        (void)argv;

    	console->print("Entering DFU mode\n");

		reboot_into_dfu();
    	NVIC_SystemReset();

    	return CMD_OK;
	}
    void help(void){}
}cmddfu;
#endif

ConsoleCommand *laser4_commands[]{
    &cmdhelp,
#ifdef CC2500_INSTALLED
    &cmdcc25,
#endif
	&cmdreset,
	&cmdid,
	&cmdbind,
	&cmdstatus,
	&cmdppm,
	&cmdproto,
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