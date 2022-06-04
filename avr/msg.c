#include "msg.h"

//bool checksum(volatile msg_t *msg, volatile msg_t *msg_cs_update, volatile msg_t *msg_cs_compare, volatile flags_t *fl) {
//    uint8_t checksum = 8;
//
//    uint8_t *msg_ptr = (uint8_t *) msg;
//    for (uint8_t i = 0; i < (uint8_t) sizeof(msg_t) - 1; i++) {
//        checksum = (checksum + (msg_ptr[i] & 0x3f)) & 0x3f;
//    }
//
//    checksum = (checksum + (msg_ptr[sizeof(msg_t) - 1] & 0b00000011)) & 0x3f;
//
//    if (msg_cs_update) {
//        msg_cs_update->checksum = checksum;
//    }
//
//    if (msg_cs_compare) {
//        if (msg_cs_compare->checksum != checksum) {
//            *fl |= 1 << FL_ERR_CHECKSUM;
//        } else {
//            return true;
//        }
//    }
//    return false;
//}

