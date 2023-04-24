/* inside of biped.cpp */

#include "BluetoothSerial.h"

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

    /* loop for bluetooth */

    if (SerialBT.available()) {
        int incoming = SerialBT.read(); //Read what we receive 

        // separate button ID from button value -> button ID is 10, 20, 30, etc, value is 1 or 0
        int button = floor(incoming / 10);
        int value = incoming % 10;

        /* could potentially be simplified to this logic */
        /* In the case a new direction other than our current one is set to true, change biped to that direction */
        if ((button != biped::biped_direction_) && (value == 1))
            biped::biped_direction_ = button;
        else
            biped::biped_direction_ = 0;
        
        // switch (button) {
        // /* left button */
        // case 1:  
        //     if (value) {
        //         biped_direction_ = 1;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // /* forward button */
        // case 2:  
        //     if (value) {
        //         biped_direction_ = 2;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        // /* right button */
        // case 3:  
        //     if (value) {
        //         biped_direction_ = 3;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // /* backward button */
        // case 4:
        // if (value) {
        //         biped_direction_ = 4;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // }
        
    }

    /* inside of global.h */
    /* 0 = stationary, 1 = left, 2 = forward, 3 = right, 4 = backward */
    extern int biped_direction_;

    /* inside of global.cpp */
    int biped_direction_ = 0;