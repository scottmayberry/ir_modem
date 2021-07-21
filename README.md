# Infrared Modem (ir_modem)
## Introduction
The Infrared Modem (ir_modem) is an open source infrared communication board, including software and hardware. The board is designed to fit within a 4in tube for use in underwater communciation in clear water, but can be modified to work in many different environments. The board is equipped with 16 IR leds and 4 IR sensors providing 360 degree communication capability. The current iteration needs a 24V input to drive the IR Leds, but modification is possible to use any voltage level.

## Communication
The board communicates utilizing [Frequency-Shift Keying (FSK)](https://en.wikipedia.org/wiki/Frequency-shift_keying), with the primary (center) frequency determined by the IR sensor selected [(TSOP38238: 38khz, TSOP38230: 30kHz, TSOP38233: 33kHz, TSOP38236: 36kHz, TSOP38240: 40kHz, TSOP38256: 56kHz)](https://www.vishay.com/docs/82491/tsop382.pdf) and the second frequency at 0Hz. Depending on configuration settings and the IR sensor selected, baudrate can be as high as 5600. With multiple IR boards, communication is possible on these 6 frequencies.

## General Hardware
The board leverages the Teensy LC as the onboard microcontroller and uses some of the prebuilt teensy libraries to accomplish FSK.

## Standards
The ir_modem uses the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf)
