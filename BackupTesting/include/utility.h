#ifndef UTILITY_H
#define UTILITY_H

#include <HardwareSerial.h>

// Note: the DEBUG precompiler variable must be defined to 1 or 0 in the calling program
#ifndef DEBUG
#define DEBUG 1
#endif

/**
 * @brief Prints the specified debug message to serial.
 *
 * Prints the string passed to @p message using the serial bus defined by @p serial using printf
 * style formatting. Additional arguments after @p message are treated as args just as in printf.
 * Will not output anything if the global define DEBUG is 0. Maximum string length is 500 chars.
 *
 * @param serial Serial interface to use to print the message.
 * @param message Message to print, behaves identically to printf including following arguments.
 */
void dprint(HardwareSerial &serial, const char message[], ...);

#endif
