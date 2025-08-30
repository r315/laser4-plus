/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(FRSKYD_CC2500_INO) && defined(CC2500_INSTALLED)

#include "FrSkyDVX_Common.h"
#include "iface_cc2500.h"

static void __attribute__((unused)) FRSKYD_RF_init(radio_t *radio, uint8_t bind)
{
    FRSKY_init_cc2500(radio, FRSKYD_cc2500_conf);

    CC2500_WriteReg(CC2500_09_ADDR, bind ? 0x03 : radio->rx_tx_addr[3]);
    CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x05);
    CC2500_Strobe(CC2500_SIDLE);    // Go to idle...
    //
    CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
    CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
    CC2500_Strobe(CC2500_SFRX);
}

static void __attribute__((unused)) FRSKYD_build_bind_packet(radio_t *radio)
{
    //11 03 01 d7 2d 00 00 1e 3c 5b 78 00 00 00 00 00 00 01
    //11 03 01 19 3e 00 02 8e 2f bb 5c 00 00 00 00 00 00 01
    radio->packet[0] = 0x11;
    radio->packet[1] = 0x03;
    radio->packet[2] = 0x01;
    radio->packet[3] = radio->rx_tx_addr[3];
    radio->packet[4] = radio->rx_tx_addr[2];
    uint16_t idx = ((radio->state - FRSKY_BIND) % 10) * 5;
    radio->packet[5] = idx;
    radio->packet[6] = radio->hopping_frequency[idx++];
    radio->packet[7] = radio->hopping_frequency[idx++];
    radio->packet[8] = radio->hopping_frequency[idx++];
    radio->packet[9] = radio->hopping_frequency[idx++];
    radio->packet[10] = radio->hopping_frequency[idx++];
    radio->packet[11] = 0x00;
    radio->packet[12] = 0x00;
    radio->packet[13] = 0x00;
    radio->packet[14] = 0x00;
    radio->packet[15] = 0x00;
    radio->packet[16] = 0x00;
    radio->packet[17] = 0x01;
}

static void __attribute__((unused)) FRSKYD_data_frame(radio_t *radio)
{
    //pachet[4] is telemetry user frame counter(hub)
    //11 d7 2d 22 00 01 c9 c9 ca ca 88 88 ca ca c9 ca 88 88
    //11 57 12 00 00 01 f2 f2 f2 f2 06 06 ca ca ca ca 18 18
    radio->packet[0] = 0x11;             //Length
    radio->packet[1] = radio->rx_tx_addr[3];
    radio->packet[2] = radio->rx_tx_addr[2];
    radio->packet[3] = radio->counter;
#if defined TELEMETRY
    packet[4] = telemetry_counter;
#else
    radio->packet[4] = 0x00;
#endif

    radio->packet[5] = 0x01;
    //
    radio->packet[10] = 0;
    radio->packet[11] = 0;
    radio->packet[16] = 0;
    radio->packet[17] = 0;
    for(uint8_t i = 0; i < 8; i++){
        uint16_t value = convert_channel_frsky(radio->channel_data[i]);

        if(i < 4){
            radio->packet[6+i] = value & 0xff;
            radio->packet[10+(i>>1)] |= ((value >> 8) & 0x0f) << (4 *(i & 0x01));
        }else{
            radio->packet[8+i] = value & 0xff;
            radio->packet[16+((i-4)>>1)] |= ((value >> 8) & 0x0f) << (4 * ((i-4) & 0x01));
        }
    }
}

/**
 * @brief
 * @param radio
 * @return callback interval in us
 */
uint16_t FRSKYD_init(radio_t *radio)
{
    Frsky_init_hop(radio);

    radio->packet_count = 0;

    if(radio->flags & FLAG_BIND){
        radio->state = FRSKY_BIND_DONE;
    }else{
        uint8_t id = CC2500_ReadStatus(CC2500_30_PARTNUM);
        if(id != 0x80){
            DBG_FRSKY_ERR("Invalid cc2500 id: 0x%x", id);
        }else{
            FRSKYD_RF_init(radio, 1);
            radio->state = FRSKY_BIND;
        }
    }

    return CC2500_CALLBACK_INTERVAL;
}

/**
 * @brief Callback from multiportocol to send data
 * @param radio
 * @return Interval for next call in us
 */
uint16_t FRSKYD_callback(radio_t *radio)
{
    if (radio->state < FRSKY_BIND_DONE){
        FRSKYD_build_bind_packet(radio);
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
        CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
        CC2500_Strobe(CC2500_SFRX);
        CC2500_WriteData(radio->packet, radio->packet[0] + 1);

        if(radio->flags & FLAG_BIND){
            radio->state = FRSKY_BIND_DONE;
        }else{
            radio->state++;
        }

        return 9000;
    }

    if (radio->state == FRSKY_BIND_DONE){
        radio->state = FRSKY_DATA2;
        FRSKYD_RF_init(radio, 0);
        radio->counter = 0;
        radio->flags |= FLAG_BIND;
        DBG_FRSKY_INF("Bind done");
    }else{
        if (radio->state == FRSKY_DATA5){
            CC2500_Strobe(CC2500_SRX);//0x34 RX enable
            radio->state = FRSKY_DATA1;
            return 9200;
        }
    }

    radio->counter = (radio->counter + 1) % 188;

    if (radio->state == FRSKY_DATA4){
        //telemetry receive
        CC2500_SetTxRxMode(RX_EN);
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteReg(CC2500_0A_CHANNR, radio->hopping_frequency[radio->counter % 47]);
        CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
        radio->state++;
        return 1300;
    }else{
        if (radio->state == FRSKY_DATA1){
        #ifdef MULTI_SYNC
            telemetry_set_input_sync(9000);
        #endif
            radio->len = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (radio->len && radio->len <= (0x11 + 3)){// 20bytes
                CC2500_ReadData(radio->packet_in, radio->len);                //received telemetry packets
            #if defined(TELEMETRY)
                if(radio->packet_in[len-1] & 0x80){
                    //with valid crc
                    packet_count=0;
                    frsky_check_telemetry(packet_in,len);    //check if valid telemetry packets and buffer them.
                }
            #endif
            }else{
                radio->packet_count++;
                // restart sequence on missed packet - might need count or timeout instead of one missed
                if(radio->packet_count > 100){
                    //~1sec
                    radio->packet_count = 0;
                #if defined TELEMETRY
                    telemetry_link=0;//no link frames
                    packet_in[6]=0;//no user frames.
                #endif
                }
            }
            CC2500_SetTxRxMode(TX_EN);
            Frsky_SetPower(radio);    // Set tx_power
        }

        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteReg(CC2500_0A_CHANNR, radio->hopping_frequency[radio->counter % 47]);

        if ( radio->prev_option != radio->option ){
            CC2500_WriteReg(CC2500_0C_FSCTRL0, radio->option);    // Frequency offset hack
            radio->prev_option = radio->option ;
        }

        CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
        CC2500_Strobe(CC2500_SFRX);
        FRSKYD_data_frame(radio);
        CC2500_WriteData(radio->packet, radio->packet[0] + 1);
        radio->state++;
    }
    return radio->state == FRSKY_DATA4 ? 7500 : 9000;
}
#endif
