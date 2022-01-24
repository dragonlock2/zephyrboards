#ifndef FAKEDISK_H
#define FAKEDISK_H

#include <zephyr.h>

// MBR

typedef struct {
    uint8_t status;
    uint8_t chs_first[3];
    uint8_t partition_type;
    uint8_t chs_last[3];
    uint32_t lba_first;
    uint32_t num_sectors;
} __attribute__((__packed__)) partition_entry_t;

typedef struct {
    uint8_t rsvd1[446];
    partition_entry_t partitions[4];
    uint16_t boot_sig;
} __attribute__((__packed__)) mbr_t;

static const mbr_t MBR = {
    .partitions = {
        {
            .status = 0,
            .chs_first = {0xfe, 0xff, 0xff},
            .partition_type = 0x0b,
            .chs_last = {0xfe, 0xff, 0xff},
            .lba_first = 1,
            .num_sectors = CONFIG_DISK_FAKE_SECTOR_COUNT-1
        }
    },
    .boot_sig = 0xaa55,
};

// FAT32

#endif // FAKEDISK_H