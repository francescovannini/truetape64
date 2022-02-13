#include "msg.h"

inline uint8_t compute_checksum(volatile msg_t *msg) {
    uint8_t *msg_ptr = (uint8_t *) msg;
    uint8_t checksum = 8;
    for (uint8_t i = 0; i < sizeof(msg_t) - 1; i++) {
        checksum = (checksum + (msg_ptr[i] & 0x3f)) & 0x3f;
    }
    checksum = (checksum + (msg_ptr[sizeof(msg_t) - 1] & 0b00000011)) & 0x3f;
    return checksum;
}
