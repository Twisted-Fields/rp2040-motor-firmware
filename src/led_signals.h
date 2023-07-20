
#pragma once


/*
 * Simple implementation for the moment. Eventually, this should be built out to run via
 * interrupts or in a thread on the second core - then it would be possible to have
 * blink effects etc without blocking the motor control.
 */


class LEDSignals {
public:
    void init(int brightness = 50);

    void signalInitState(int state);
    void signalErrorState();

    void setPixel(int num, int r, int g, int b);
};
