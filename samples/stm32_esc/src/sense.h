#ifndef SENSE_H
#define SENSE_H

#include <stdint.h>

typedef enum {
    SENSE_CHANNEL_BEMF_A = 0,
    SENSE_CHANNEL_BEMF_B,
    SENSE_CHANNEL_BEMF_C,
    SENSE_CHANNEL_ISENSE_A,
    SENSE_CHANNEL_ISENSE_B,
    SENSE_CHANNEL_ISENSE_C,
    SENSE_CHANNEL_VBUS,
    SENSE_CHANNEL_COUNT,
} sense_channel_E;

void sense_init();
uint32_t sense_read(sense_channel_E chan);

#endif // SENSE_H
