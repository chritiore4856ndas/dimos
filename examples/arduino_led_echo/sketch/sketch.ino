/*
 * LED Echo — Example DimOS Arduino sketch.
 *
 * Receives Bool commands from the host to control the built-in LED.
 * Echoes the LED state back so the host can confirm it changed.
 *
 * Demonstrates the simplest possible ArduinoModule:
 *   - One input stream  (Bool → LED on/off)
 *   - One output stream  (Bool → confirm LED state)
 *   - DimosSerial debug prints
 */

#include "dimos_arduino.h"
#include <util/delay.h>

#define LED_PIN 13

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    DimosSerial.println("LED Echo ready");
}

void loop() {
    while (dimos_check_message()) {
        enum dimos_topic topic = dimos_message_topic();
        const uint8_t   *data  = dimos_message_data();
        uint16_t         len   = dimos_message_len();

        switch (topic) {

        case DIMOS_TOPIC__LED_CMD: {
            dimos_msg__Bool cmd;
            int decoded = dimos_msg__Bool__decode(data, 0, len, &cmd);
            if (decoded < 0) {
                DimosSerial.println("ERR: decode failed");
                break;
            }

            /* Set the LED */
            digitalWrite(LED_PIN, cmd.data ? HIGH : LOW);
            DimosSerial.print("LED ");
            DimosSerial.println(cmd.data ? "ON" : "OFF");

            /* Echo back the state */
            uint8_t buf[1];
            int encoded = dimos_msg__Bool__encode(buf, 0, sizeof(buf), &cmd);
            if (encoded > 0) {
                dimos_send(DIMOS_TOPIC__LED_STATE, buf, encoded);
            }
            break;
        }

        default:
            break;
        }
    }

    _delay_ms(1);
}
