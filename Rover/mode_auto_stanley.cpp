#include "Rover.h"

#if MODE_AUTO_STANLEY_ENABLED

bool ModeAuto::_enter_auto_stanley()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Auto: Using Stanley controller.");
    return true;
}

#endif
