#ifndef APP_H
#define APP_H

// Declare function prototypes
void appInitEEPROM(void);
void appSaveEEPROM(void);
void appDefaultEEPROM(void);
uint8_t appGetAuxChannels(uint16_t *channel_aux);

#endif // APP_H