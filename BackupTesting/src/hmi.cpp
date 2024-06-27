#include "hmi.h"
#include "utility.h"

// Pointer to callback functions
void (*release_callback_function)(ButtonReleaseEvent);
void (*push_callback_function)(ButtonPushEvent);

// Nextion instance pointer
Nextion *display = Nextion::GetInstance(Serial2);

// Page definitions
NexPage PageSplash = NexPage(display, 0, "Init");
NexPage PageHome = NexPage(display, 1, "Home");
NexPage PageTest = NexPage(display, 2, "Testing");
NexPage PageTestComplete = NexPage(display, 3, "TestComplete");
NexPage PageTestFail = NexPage(display, 4, "TestFail");
NexPage PagePressFail = NexPage(display, 5, "PressFail");
NexPage PageTestAbort = NexPage(display, 6, "Abort");

// Home page button definitions
NexButton btHomeTestDecrease  = NexButton(display, 1, 10, "b1", &PageHome);
NexButton btHomeSpeedDecrease = NexButton(display, 1, 11, "b2", &PageHome);
NexButton btHomeForceDecrease = NexButton(display, 1, 12, "b3", &PageHome);
NexButton btHomeSprayDecrease = NexButton(display, 1, 13, "b4", &PageHome);
NexButton btHomeTestIncrease  = NexButton(display, 1, 14, "b5", &PageHome);
NexButton btHomeSpeedIncrease = NexButton(display, 1, 15, "b6", &PageHome);
NexButton btHomeForceIncrease = NexButton(display, 1, 16, "b7", &PageHome);
NexButton btHomeSprayIncrease = NexButton(display, 1, 17, "b8", &PageHome);
NexButton btHomeStart = NexButton(display, 1, 1, "b0", &PageHome);

// Home page text definitions
NexText txHomeTest =  NexText(display, 1, 6, "t4", &PageHome);
NexText txHomeSpeed = NexText(display, 1, 7, "t5", &PageHome);
NexText txHomeForce = NexText(display, 1, 8, "t6", &PageHome);
NexText txHomeSpray = NexText(display, 1, 9, "t7", &PageHome);

// Test page button definitions
NexButton btTestAbort = NexButton(display, 2, 4, "b0", &PageTest);

// Test page text definitions
NexText txTestElapsed = NexText(display, 2, 2, "t1", &PageTest);
NexText txTestNum = NexText(display, 2, 3, "t2", &PageTest);

// TestComplete page button definitions
NexButton btTestCompleteOk = NexButton(display, 3, 4, "b0", &PageTestComplete);

// TestComplete page text definitions
NexText txTestCompleteElapsed = NexText(display, 3, 2, "t1", &PageTestComplete);
NexText txTestCompleteNum = NexText(display, 3, 3, "t2", &PageTestComplete);

// TestFail page button definitions
NexButton btTestFailOk = NexButton(display, 4, 4, "b0", &PageTestFail);

// TestFail page text definitions
NexText txTestFailElapsed = NexText(display, 4, 2, "t1", &PageTestFail);
NexText txTestFailNum = NexText(display, 4, 3, "t2", &PageTestFail);

// PressFail page button definitions
NexButton btPressFailOk = NexButton(display, 5, 2, "b0", &PagePressFail);

// Event callback list
NexTouch *nex_listen_list[] =
{
    &btHomeTestIncrease,
    &btHomeTestDecrease,
    &btHomeSpeedIncrease,
    &btHomeSpeedDecrease,
    &btHomeForceIncrease,
    &btHomeForceDecrease,
    &btHomeSprayIncrease,
    &btHomeSprayDecrease,
    &btHomeStart,
    &btTestAbort,
    &btTestCompleteOk,
    &btTestFailOk,
    &btPressFailOk,
    NULL
};

bool hmi_init(void (*pushFunction)(ButtonPushEvent), void (*releaseFunction)(ButtonReleaseEvent))
{
    // Nextion init
    if (!(*display).nexInit())
    {
        // Try again 3 times if failed due to init unreliability
        uint8_t tries = 3;
        while(tries != 0)
        {
            if((*display).nexInit())
            {
                break;
            }
            tries--;
        }

        // If ran out of tries, give up
        if(tries == 0)
        {
            return false;
        }
    }

    // Save callback functions
    push_callback_function = pushFunction;
    release_callback_function = releaseFunction;

    // Button push event instantiations
    static ButtonPushEvent homeTestIncreaseP = ButtonPushEvent::HomeTestIncrease;
    static ButtonPushEvent homeTestDecreaseP = ButtonPushEvent::HomeTestDecrease;
    static ButtonPushEvent homeSpeedIncreaseP = ButtonPushEvent::HomeSpeedIncrease;
    static ButtonPushEvent homeSpeedDecreaseP = ButtonPushEvent::HomeSpeedDecrease;
    static ButtonPushEvent homeForceIncreaseP = ButtonPushEvent::HomeForceIncrease;
    static ButtonPushEvent homeForceDecreaseP = ButtonPushEvent::HomeForceDecrease;
    static ButtonPushEvent homeSprayIncreaseP = ButtonPushEvent::HomeSprayIncrease;
    static ButtonPushEvent homeSprayDecreaseP = ButtonPushEvent::HomeSprayDecrease;
    static ButtonPushEvent homeStartP = ButtonPushEvent::HomeStart;
    static ButtonPushEvent testAbortP = ButtonPushEvent::TestAbort;
    static ButtonPushEvent testCompleteOkP = ButtonPushEvent::TestCompleteOk;
    static ButtonPushEvent testFailOkP = ButtonPushEvent::TestFailOk;
    static ButtonPushEvent pressFailOkP = ButtonPushEvent::PressFailOk;

    // Button release event instantiations
    static ButtonReleaseEvent homeTestIncreaseR = ButtonReleaseEvent::HomeTestIncrease;
    static ButtonReleaseEvent homeTestDecreaseR = ButtonReleaseEvent::HomeTestDecrease;
    static ButtonReleaseEvent homeSpeedIncreaseR = ButtonReleaseEvent::HomeSpeedIncrease;
    static ButtonReleaseEvent homeSpeedDecreaseR = ButtonReleaseEvent::HomeSpeedDecrease;
    static ButtonReleaseEvent homeForceIncreaseR = ButtonReleaseEvent::HomeForceIncrease;
    static ButtonReleaseEvent homeForceDecreaseR = ButtonReleaseEvent::HomeForceDecrease;
    static ButtonReleaseEvent homeSprayIncreaseR = ButtonReleaseEvent::HomeSprayIncrease;
    static ButtonReleaseEvent homeSprayDecreaseR = ButtonReleaseEvent::HomeSprayDecrease;
    static ButtonReleaseEvent homeStartR = ButtonReleaseEvent::HomeStart;
    static ButtonReleaseEvent testAbortR = ButtonReleaseEvent::TestAbort;
    static ButtonReleaseEvent testCompleteOkR = ButtonReleaseEvent::TestCompleteOk;
    static ButtonReleaseEvent testFailOkR = ButtonReleaseEvent::TestFailOk;
    static ButtonReleaseEvent pressFailOkR = ButtonReleaseEvent::PressFailOk;

    // Hook up all event handlers
    btHomeTestIncrease.attachPush(push_button_callback, &homeTestIncreaseP);
    btHomeTestIncrease.attachPop(release_button_callback, &homeTestIncreaseR);
    btHomeTestDecrease.attachPush(push_button_callback, &homeTestDecreaseP);
    btHomeTestDecrease.attachPop(release_button_callback, &homeTestDecreaseR);
    btHomeSpeedIncrease.attachPush(push_button_callback, &homeSpeedIncreaseP);
    btHomeSpeedIncrease.attachPop(release_button_callback, &homeSpeedIncreaseR);
    btHomeSpeedDecrease.attachPush(push_button_callback, &homeSpeedDecreaseP);
    btHomeSpeedDecrease.attachPop(release_button_callback, &homeSpeedDecreaseR);
    btHomeForceIncrease.attachPush(push_button_callback, &homeForceIncreaseP);
    btHomeForceIncrease.attachPop(release_button_callback, &homeForceIncreaseR);
    btHomeForceDecrease.attachPush(push_button_callback, &homeForceDecreaseP);
    btHomeForceDecrease.attachPop(release_button_callback, &homeForceDecreaseR);
    btHomeSprayIncrease.attachPush(push_button_callback, &homeSprayIncreaseP);
    btHomeSprayIncrease.attachPop(release_button_callback, &homeSprayIncreaseR);
    btHomeSprayDecrease.attachPush(push_button_callback, &homeSprayDecreaseP);
    btHomeSprayDecrease.attachPop(release_button_callback, &homeSprayDecreaseR);
    btHomeStart.attachPush(push_button_callback, &homeStartP);
    btHomeStart.attachPop(release_button_callback, &homeStartR);
    btTestAbort.attachPush(push_button_callback, &testAbortP);
    btTestAbort.attachPop(release_button_callback, &testAbortR);
    btTestCompleteOk.attachPush(push_button_callback, &testCompleteOkP);
    btTestCompleteOk.attachPop(release_button_callback, &testCompleteOkR);
    btTestFailOk.attachPush(push_button_callback, &testFailOkP);
    btTestFailOk.attachPop(release_button_callback, &testFailOkR);
    btPressFailOk.attachPush(push_button_callback, &pressFailOkP);
    btPressFailOk.attachPop(release_button_callback, &pressFailOkR);

    return true;
}

void hmi_run_button_callbacks()
{
    (*display).nexLoop(nex_listen_list);
}

bool hmi_set_page(Page page)
{
    switch(page)
    {
        case(Page::Splash):
            return PageSplash.show();

        case(Page::Home):
            return PageHome.show();

        case(Page::Test):
            return PageTest.show();

        case(Page::TestComplete):
            return PageTestComplete.show();

        case(Page::TestFail):
            return PageTestFail.show();

        case(Page::TestAbort):
            return PageTestAbort.show();

        case(Page::PressFail):
            return PagePressFail.show();

        default:
            dprint(Serial, "Error: hmi_set_page() switch statement reached default case!");
            return false;
    }
}

bool hmi_set_text_field(TextField field, const char *text)
{
    switch(field)
    {
        case(TextField::HomeTest):
            return txHomeTest.setText(text);

        case(TextField::HomeSpeed):
            return txHomeSpeed.setText(text);

        case(TextField::HomeForce):
            return txHomeForce.setText(text);

        case(TextField::HomeSpray):
            return txHomeSpray.setText(text);

        case(TextField::TestElapsed):
            return txTestElapsed.setText(text);

        case(TextField::TestNum):
            return txTestNum.setText(text);

        case(TextField::TestCompleteElapsed):
            return txTestCompleteElapsed.setText(text);

        case(TextField::TestCompleteNum):
            return txTestCompleteNum.setText(text);

        case(TextField::TestFailElapsed):
            return txTestFailElapsed.setText(text);

        case(TextField::TestFailNum):
            return txTestFailNum.setText(text);

        default:
            dprint(Serial, "Error: hmi_set_text_field() switch statement reached default case!");
            return false;
    }
}

void push_button_callback(void *callback)
{
    ButtonPushEvent button = *(ButtonPushEvent*)callback;
    push_callback_function(button);
}

void release_button_callback(void *callback)
{
    ButtonReleaseEvent button = *(ButtonReleaseEvent*)callback;
    release_callback_function(button);
}
