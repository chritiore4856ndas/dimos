/*
 * Twist Echo — Example DimOS Arduino sketch.
 *
 * Receives Twist commands from the host, echoes them back.
 * Demonstrates:
 *   - dimos_init() / dimos_check_message() / dimos_send()
 *   - Switch on dimos_message_topic() to handle different streams
 *   - Using generated encode/decode functions
 *   - DimosSerial.println() going through the DSP debug channel
 *   - Config values available as #defines
 *
 * NOTE: We use _delay_ms() from <util/delay.h> instead of Arduino's delay()
 * because delay() relies on timer 0 interrupts which don't fire in QEMU's
 * AVR model.  _delay_ms is a pure busy loop and works in any simulator.
 */

#include "dimos_arduino.h"
#include <util/delay.h>

/* Shared state — accessible across all topic handlers */
dimos_msg__Twist last_twist;
uint32_t msg_count = 0;

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    DimosSerial.println("TwistEcho ready");
}

void loop() {
    while (dimos_check_message()) {
        enum dimos_topic  topic = dimos_message_topic();
        const uint8_t    *data  = dimos_message_data();
        uint16_t          len   = dimos_message_len();

        switch (topic) {

        case DIMOS_TOPIC__TWIST_IN: {
            int decoded = dimos_msg__Twist__decode(data, 0, len, &last_twist);
            if (decoded < 0) {
                DimosSerial.println("ERR: failed to decode Twist");
                break;
            }

            msg_count++;
            DimosSerial.print("Got twist #");
            DimosSerial.print(msg_count);
            DimosSerial.print(": linear.x=");
            DimosSerial.println(last_twist.linear.x);

            /* Echo it back.  Buffer size must match
             * dimos_msg__Twist__encoded_size() — we assert at the first
             * iteration so drift in the wire format is caught loudly
             * rather than silently truncated. */
            constexpr int TWIST_BUF_SIZE = 48;
            static bool size_checked = false;
            if (!size_checked) {
                if (dimos_msg__Twist__encoded_size() != TWIST_BUF_SIZE) {
                    DimosSerial.println("ERR: Twist wire size drift");
                    break;
                }
                size_checked = true;
            }
            uint8_t buf[TWIST_BUF_SIZE];
            int encoded = dimos_msg__Twist__encode(buf, 0, sizeof(buf), &last_twist);
            if (encoded > 0) {
                dimos_send(DIMOS_TOPIC__TWIST_ECHO_OUT, buf, encoded);
            }
            break;
        }

        default:
            break;
        }
    }

    /* Short delay to yield CPU.  Must be short enough that the 2-byte
     * USART hardware FIFO does not overflow between polls — at 115200
     * baud a byte arrives every ~87µs, so 1ms gives comfortable margin
     * while still preventing a tight spin.  The original 50ms caused
     * data loss on real hardware (no interrupt-driven RX buffering). */
    _delay_ms(1);
}
