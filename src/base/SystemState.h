#ifndef SystemState_h
#define SystemState_h

class SystemState
{
public:
    // flag used to trigger a system reboot
    static bool shouldReboot;
    // flag to pause custom application Run during Firmware Update
    static bool pauseCustomApp;
};

#endif