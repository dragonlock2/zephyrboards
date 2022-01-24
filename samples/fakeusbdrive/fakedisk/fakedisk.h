#ifndef FAKEDISK_H
#define FAKEDISK_H

#include <zephyr.h>

#define SECTOR_SIZE 512 // most compatible value

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

// FAT32 boot sector

typedef struct {
    uint8_t jump_instr[3];
    uint8_t oem_name[8];
    uint16_t bytes_per_sector;
    uint8_t sectors_per_cluster;
    uint16_t num_rsvd_sectors;
    uint8_t num_fat_copies;
    uint16_t num_root_dir_ent;
    uint16_t num_sectors;
    uint8_t media_descriptor;
    uint16_t sectors_per_fat;
    uint16_t sectors_per_track;
    uint16_t num_heads;
    uint32_t num_hidden_sectors;
    uint32_t num_sectors_fat32;
    uint32_t sectors_per_fat32;
    uint16_t mirror_flags;
    uint16_t version;
    uint32_t root_dir_first_cluster;
    uint16_t fsinfo_sector_num;
    uint16_t backup_boot_sector;
    uint8_t rsvd1[12];
    uint8_t physical_drive_num;
    uint8_t rsvd2;
    uint8_t ext_sig;
    uint32_t volume_id;
    uint8_t volume_label[11];
    uint8_t fs_type[8];
    uint8_t rsvd3[420];
    uint16_t boot_sig;
} __attribute__((__packed__)) fat32_boot_t;

static const fat32_boot_t FAT32_BOOT = {
    .jump_instr = {0xeb, 0x58, 0x90},
    .oem_name = {'B', 'S', 'D', ' ', ' ', '4', '.', '4'},
    .bytes_per_sector = SECTOR_SIZE,
    .sectors_per_cluster = 64,
    .num_rsvd_sectors = 32,
    .num_fat_copies = 2,
    .num_root_dir_ent = 0,
    .num_sectors = 0,
    .media_descriptor = 0xf8,
    .sectors_per_fat = 0,
    .sectors_per_track = 32,
    .num_heads = 255,
    .num_hidden_sectors = 1,
    .num_sectors_fat32 = CONFIG_DISK_FAKE_SECTOR_COUNT-1,
    .sectors_per_fat32 = 0x3a6c, // TODO try diff number?
    .mirror_flags = 0,
    .version = 0,
    .root_dir_first_cluster = 2, // TODO change values
    .fsinfo_sector_num = 1,
    .backup_boot_sector = 0,
    .physical_drive_num = 0x80,
    .ext_sig = 0x29,
    .volume_id = 0x69696969,
    .volume_label = {'U', 'N', 'T', 'I', 'T', 'L', 'E', 'D', ' ', ' ', ' '},
    .fs_type = {'F', 'A', 'T', '3', '2', ' ', ' ',  ' '},
    .boot_sig = 0xaa55,
};

// FAT32 FS Info Sector

#endif // FAKEDISK_H