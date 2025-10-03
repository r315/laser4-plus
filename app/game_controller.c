#include "board.h"
#include "usb_device.h"
#include "game_controller.h"
#include "multiprotocol.h"
#include "math.h"

#ifdef ENABLE_GAME_CONTROLLER
//#define TEST_CONTROLLER

static controller_t laser4;
static uint8_t *channel_map;

#ifdef TEST_CONTROLLER
static float angle = 0;
#endif

RAM_CODE uint16_t USBHID_callback(radio_t *radio)
{

#if defined(TEST_CONTROLLER)
    static uint8_t count = 0;
    angle += 0.1;
#endif

#if !defined(TEST_CONTROLLER)
    uint8_t i;
    uint8_t *data = (uint8_t*)&laser4.pitch; // 1st channel

    for(i = 0; i < REPORT_CHANNELS_SIZE; i++){
        uint16_t val = radio->channel_data[i];

        // Clip servo value
        if(val > SERVO_MAX){ val = SERVO_MAX; }
        if(val < SERVO_MIN){ val = SERVO_MIN; }

        val = map16b(val, SERVO_MIN, SERVO_MAX, LOGICAL_MINIMUM, LOGICAL_MAXIMUM);

        *(data + (channel_map[i] * 2)) = val;
        *(data + 1 + (channel_map[i] * 2)) = val >> 8;
    }

#ifdef ENABLE_AUX_SWITCHES
    laser4.buttons = auxGetSwitches();
#else
    laser4.buttons = 0;
#endif

#else /* TEST_CONTROLLER */
    //float t = angle * 0.15915f; // Normalize t = x/2pi - floor(x/2pi)
    //t = t - (int)t;
    //float sine = 20.785f * (t - 0.0f) * (t - 0.5f) * (t - 1.0f);
    //float cosine = 20.785f * (t + 0.25f) * (t - 0.25f) * (t - 0.75f);

    laser4.roll = (LOGICAL_MAXIMUM/2) + sin(angle) * (LOGICAL_MAXIMUM/2);
    laser4.pitch = (LOGICAL_MAXIMUM/2) + cos(angle) * (LOGICAL_MAXIMUM/2);

    laser4.throttle = laser4.roll;
    laser4.yaw = laser4.pitch;

    if( (count--) == 0){
        uint8_t tmp = (uint8_t)laser4.buttons;
        tmp = (tmp >> 1) | (tmp << 3);
        laser4.buttons = tmp;
        count = 20;
    }

#endif /* TEST_CONTROLLER */
    USB_DEVICE_SendReport((uint8_t*)&laser4, REPORT_SIZE);

    return PPM_TX_CALLBACK_INTERVAL;
}


/**
 *
 * */
uint16_t USBHID_init(radio_t *radio)
{
    (void)radio;

    laser4.roll = LOGICAL_MAXIMUM;
    laser4.pitch = LOGICAL_MAXIMUM/2;
    laser4.throttle = LOGICAL_MINIMUM;
    laser4.yaw = LOGICAL_MAXIMUM/2;
    laser4.aux1 = LOGICAL_MAXIMUM/2;
    laser4.aux2 = LOGICAL_MAXIMUM/2;
    laser4.buttons = 0;
    channel_map = CH_AETR;

    return PPM_TX_CALLBACK_INTERVAL;
}

#endif /* ENABLE_GAME_CONTROLLER */