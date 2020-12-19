/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere demonstrates how to use a UART (serial port).
// The sample opens a UART with a baud rate of 115200. Pressing a button causes characters
// to be sent from the device over the UART; data received by the device from the UART is echoed to
// the Visual Studio Output Window.
//
// It uses the API for the following Azure Sphere application libraries:
// - UART (serial port)
// - GPIO (digital input for button)
// - log (displays messages in the Device Output window during debugging)
// - eventloop (system invokes handlers for timer events)

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <hw/avnet_mt3620_sk.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/uart.h>
#include <applibs/gpio.h>
#include <applibs/log.h>
#include <applibs/eventloop.h>

#include "eventloop_timer_utilities.h"
#include "peripheral_utilities.h"
#include "string_utilities.h"
#include "LoRa.h"

/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code. They must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_ButtonTimer_Consume = 2,
    ExitCode_ButtonTimer_GetValue = 3,
    ExitCode_Init_EventLoop = 4,
    ExitCode_Init_OpenButton = 5,
    ExitCode_Init_ButtonPollTimer = 6,
    ExitCode_Main_EventLoopFail = 7,
    ExitCode_Init_ReconnectTimer = 8,
    ExitCode_Init_SenMessageTimer = 9
} ExitCode;

// File descriptors - initialized to invalid value
static int gpioButtonFd = -1;

char sendMessage[] = "Hello World From LoRa";
char tmp_txt[ 50 ];
char sendHex[ 50 ];
char rspTxt[ 50 ];
char rsp_data[10];
uint8_t cnt;
uint8_t send_data;
uint8_t _data;
uint8_t rxState;
uint8_t txState;
char LORA_CMD_SYS_GET_VER[] = "sys get ver";
char LORA_CMD_MAC_PAUSE[] = "mac pause";
char LORA_CMD_RADIO_SET_WDT[] = "radio set wdt 0";
char LORA_ARG_0[] = "0";

static bool connected = false;

EventLoop *eventLoop = NULL;
EventLoopTimer *buttonPollTimer = NULL;
EventLoopTimer *reconnectTimer = NULL;
EventLoopTimer *sendMessageTimer = NULL;

// State variables
static GPIO_Value_Type buttonState = GPIO_Value_High;

// Termination state
static volatile sig_atomic_t exitCode = ExitCode_Success;

static void TerminationHandler(int signalNumber);
static void ButtonTimerEventHandler(EventLoopTimer *timer);
static ExitCode InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

static void TryConnectToLoRaNetwork(void)
{
    if (connected)
    {
        return;
    }

    lora_join( "otaa", &tmp_txt[0]);

    if ( strcmp(trim(tmp_txt), "accepted") == 0 ){
        Log_Debug("Device successfully connected.\n");
        connected = true;
    }
    else {
        Log_Debug("Device is not connected: %s\n", tmp_txt);
    }
}

static void ReconnectEventHandler(EventLoopTimer *timer)
{    
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }

    TryConnectToLoRaNetwork();
}

static void TrySendMessage(void)
{
    if (!connected)
    {
        Log_Debug("Cannot send a message since the device is offline.");
        return;
    }

    uint8_t resp = lora_mac_tx("cnf", "1", "48656C6C6F", &tmp_txt[0]);

    if ( resp != 0) {
        Log_Debug("Packet was not transmit: %d", resp);
    }
}

/// <summary>
///     Handle button timer event: if the button is pressed, send data over the UART.
/// </summary>
static void ButtonTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }

    // Check for a button press
    GPIO_Value_Type newButtonState;
    int result = GPIO_GetValue(gpioButtonFd, &newButtonState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_ButtonTimer_GetValue;
        return;
    }

    // If the button has just been pressed, send data over the UART
    // The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
    if (newButtonState != buttonState) {
        if (newButtonState == GPIO_Value_Low) {
            TrySendMessage();
            buttonState = newButtonState;
        }
    }
}

static void SendDeviceMessageHandler(EventLoopTimer *timer) 
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }

    TrySendMessage();
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>
///     ExitCode_Success if all resources were allocated successfully; otherwise another
///     ExitCode value which indicates the specific failure.
/// </returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    // Open SAMPLE_BUTTON_1 GPIO as input, and set up a timer to poll it
    Log_Debug("Opening SAMPLE_BUTTON_1 as input.\n");
    gpioButtonFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
    if (gpioButtonFd == -1) {
        Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_OpenButton;
    }
    struct timespec buttonPressCheckPeriod1Ms = {.tv_sec = 0, .tv_nsec = 1000 * 1000};
    buttonPollTimer = CreateEventLoopPeriodicTimer(eventLoop, ButtonTimerEventHandler,
                                                   &buttonPressCheckPeriod1Ms);
    if (buttonPollTimer == NULL) {
        return ExitCode_Init_ReconnectTimer;
    }

    lora_init();
    lora_process();

    // start
    lora_cmd( "mac reset 868", &tmp_txt[0]);
    lora_cmd( "mac set deveui 9ABB196487A3E9D3", &tmp_txt[0]);
    lora_cmd( "mac set appeui F33F1B9432896391", &tmp_txt[0]);
    lora_cmd( "mac set appkey D6FE7596B8974EBF09314AC0C17AB307", &tmp_txt[0]);
    lora_cmd( "mac set adr off", &tmp_txt[0]);
    lora_cmd( "mac set ar off", &tmp_txt[0]);
    lora_cmd( "mac save", &tmp_txt[0]);

    TryConnectToLoRaNetwork();

    struct timespec reconnectCheckPeriod1m = {.tv_sec = 60, .tv_nsec = 0};
    reconnectTimer = CreateEventLoopPeriodicTimer(eventLoop, ReconnectEventHandler,
                                                            &reconnectCheckPeriod1m);
    if (reconnectTimer == NULL) {
        return ExitCode_Init_ReconnectTimer;
    }

    struct timespec sendMessageCheckPeriod1m = {.tv_sec = 60, .tv_nsec = 0};
    sendMessageTimer = CreateEventLoopPeriodicTimer(eventLoop, SendDeviceMessageHandler,
                                                            &sendMessageCheckPeriod1m);
    if (sendMessageTimer == NULL) {
        return ExitCode_Init_SenMessageTimer;
    }    

    return ExitCode_Success;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(reconnectTimer);
    DisposeEventLoopTimer(sendMessageTimer);

    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(gpioButtonFd, "GpioButton");

    Log_Debug("Closing LoRa Device.\n");
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("UART application starting.\n");
    exitCode = InitPeripheralsAndHandlers();

    // Use event loop to wait for events and trigger handlers, until an error or SIGTERM happens
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}