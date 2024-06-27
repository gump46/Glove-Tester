#ifndef HMI_H
#define HMI_H

#include "NexConfig.h"
#include <Nextion.h>

// Enum Defintions ========================================================================

// Pages
enum class Page
{
    Splash,
    Home,
    Test,
    TestComplete,
    TestFail,
    TestAbort,
    PressFail
};

// Button push events
enum class ButtonPushEvent
{
    HomeTestIncrease,
    HomeTestDecrease,
    HomeSpeedIncrease,
    HomeSpeedDecrease,
    HomeForceIncrease,
    HomeForceDecrease,
    HomeSprayIncrease,
    HomeSprayDecrease,
    HomeStart,
    TestAbort,
    TestCompleteOk,
    TestFailOk,
    PressFailOk
};

// Button release events
enum class ButtonReleaseEvent
{
    HomeTestIncrease,
    HomeTestDecrease,
    HomeSpeedIncrease,
    HomeSpeedDecrease,
    HomeForceIncrease,
    HomeForceDecrease,
    HomeSprayIncrease,
    HomeSprayDecrease,
    HomeStart,
    TestAbort,
    TestCompleteOk,
    TestFailOk,
    PressFailOk
};

// Text fields
enum class TextField
{
    HomeTest,
    HomeSpeed,
    HomeForce,
    HomeSpray,
    TestElapsed,
    TestNum,
    TestCompleteElapsed,
    TestCompleteNum,
    TestFailElapsed,
    TestFailNum
};

// Public Functions ===============================================================================

/**
 * Initializes the Nextion touchscreen. Must be called once before calling any other hmi functions.
 * Also sets up the button release callback function.
 *
 * \param pushFunction The function to execute when a button is released (popped), must take one
 *                     parameter of type ButtonPushEvent
 * \param releaseFunction The function to execute when a button is released (popped), must take one
 *                        parameter of type ButtonReleaseEvent
 *
 * \return True on success, false on failure
 */
bool hmi_init(void (*pushFunction)(ButtonPushEvent), void (*releaseFunction)(ButtonReleaseEvent));

/**
 * Executes the appropriate button callback (push or release) once for each recorded button event.
 * This should be called as often as possible in the main loop.
 */
void hmi_run_button_callbacks();

/**
 * Switches the display to the specified page.
 *
 * \param page Page to switch the display to
 *
 * \return True on success, false on failure
 */
bool hmi_set_page(Page page);

/**
 * Changes the specified text field on the home page to the specified value.
 *
 * \param field The text field to modify
 * \param text The text \p field should be modified to
 *
 * \return True on success, false on failure
 */
bool hmi_set_text_field(TextField field, const char *text);

// Private Functions ==============================================================================

void push_button_callback(void *callback);
void release_button_callback(void *callback);

#endif
