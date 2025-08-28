#ifndef APP_H
#define APP_H

// Declare function prototypes
void appLoadEEPROM(void);
void appSaveEEPROM(void);
void appDefaultEEPROM(void);
void appGetAuxChannels(uint16_t *channel_aux, uint8_t *nchannel);

#endif // APP_H