#include "utility.h"

void dprint(HardwareSerial &serial, const char message[], ...)
{
    // Only print if DEBUG is true
    if (DEBUG)
    {
        // Buffer for text
        char temp[500];

        // Write each argument to buffer
        va_list args;
        va_start(args, message);
        vsnprintf(temp, sizeof(temp), message, args);
        va_end(args);

        // Append timestamp to buffer and print to serial
        serial.printf("[%.3f] %s\n", ((float)millis() / 1000), temp);
    }
}
