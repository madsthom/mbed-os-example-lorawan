/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdint>
#include <stdio.h>
#include <string>
#include <mbed.h>

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

#define ON                              1
#define OFF                             0


/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);

// Initialise the digital pin LED3 and LED2 as an output
DigitalOut green_led(LED2);
DigitalOut blue_led(LED3);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

static uint8_t is_class_c = 0;

static void switch_to_class_c();

static void switch_to_class_a();

static uint8_t receive_count = 0;

static void send_specific_message(string message);

/**
 * Entry point for application
 */
int main(void)
{
    // setup tracing
    setup_trace();

    green_led = ON;

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n [main]: CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

static void switch_to_class_c()
{
    printf("\r\n Switching to class C... \r\n");
    int16_t retcode = lorawan.set_device_class(CLASS_C);
    if (retcode == LORAWAN_STATUS_OK) {
        printf("\r\n Switched to class C - Successful!\r\n");
    }
    blue_led = ON;
    green_led = OFF;
    is_class_c = 1;
    send_specific_message("ClassCSwitch");
}

static void switch_to_class_a()
{
    printf("\r\n Switching to class A... \r\n");
    int16_t retcode = lorawan.set_device_class(CLASS_A);
    if (retcode == LORAWAN_STATUS_OK) {
        printf("\r\n switched to class A - Successful!\r\n");
    }
    blue_led = OFF;
    green_led = ON;
    is_class_c = 0;
    send_specific_message("ClassAInit");
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    if (is_class_c)
        return;
    uint16_t packet_len;
    int16_t retcode;

    packet_len = sprintf((char *) tx_buffer, "DataFromEndDevice");

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("\r\n send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON && is_class_c == 0) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
    printf(" With the message: DataFromEndDevice\r\n");
}

/**
 * Sends a specific message to the Network Server
 */
static void send_specific_message(string message)
{
    if (is_class_c)
        return;
    uint16_t packet_len;
    int16_t retcode;

    uint8_t n = message.length();

    char string_arr[n + 1];

    strcpy(string_arr, message.c_str());

    packet_len = sprintf((char *) tx_buffer, string_arr);

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("\r\n send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        ev_queue.call_in(3000, send_message);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON && is_class_c == 0) {
                printf("\r\n Should send message now in class A \r\n");
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    receive_count++;
    printf("\r\n Packets receive count: %d \r\n", receive_count);
    uint8_t port;
    int flags;
    // retcode is also the number of bytes in the message ? :-P
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode == -1001) {
        printf("\r\n LoRaMAC have nothing to read. Probably just an ACK \r\n");
    } else if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");

    auto received_msg = (char *) &rx_buffer;

    printf("\r\n With message: %s \r\n", received_msg);

    if (strcmp(received_msg, "ClassCSwitch") == 0) {
        printf("\r\n We should switch to class C if not already \r\n");

        switch_to_class_c();
    }

    if (strcmp(received_msg, "ClassASwitch") == 0) {
        printf("\r\n We should switch to class A if not already \r\n");

        switch_to_class_a();
    }

    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                if (is_class_c == 1) {
                    send_specific_message("ClassCInit");
                } else {
                    send_specific_message("ClassAInit");
                }
            } else {
                // ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n TX_DONE \r\n");
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON && is_class_c == 1) {
                //receive_message();
            } else if (MBED_CONF_LORA_DUTY_CYCLE_ON && is_class_c == 0) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                // send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n RX_DONE \r\n");
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                // send_message();
            }
            break;
        case CLASS_CHANGED:
            printf("class changed");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
