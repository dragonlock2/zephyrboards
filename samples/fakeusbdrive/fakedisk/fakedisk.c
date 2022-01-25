#include <device.h>
#include <errno.h>
#include <drivers/disk.h>
#include <logging/log.h>

#include "fakedisk.h"

// inspired by drivers/disk/ramdisk.c

LOG_MODULE_REGISTER(fakedisk, CONFIG_LOG_DEFAULT_LEVEL);

static int disk_fake_access_init(struct disk_info *disk) {
    return 0;
}

static int disk_fake_access_status(struct disk_info *disk) {
    return DISK_STATUS_OK;
}

static int disk_fake_access_read(struct disk_info *disk, uint8_t *buff,
                                 uint32_t sector, uint32_t count) {
    if (count != 1) {
        return -EIO;
    }

    if (sector == 0) {
        memcpy(buff, &MBR, SECTOR_SIZE);
    } else if (sector == 1) {
        memcpy(buff, &FAT32_BOOT, SECTOR_SIZE);
    } else if (sector == 2) {
        memcpy(buff, &FAT32_FSINFO, SECTOR_SIZE);
    } else if (sector == 33 || sector == 33 + SECTORS_PER_FAT) {
        memcpy(buff, &FAT32_ROOT_TABLE, SECTOR_SIZE);
    } else if (sector < 33 + 2 * SECTORS_PER_FAT) {
        memcpy(buff, &FAT32_TABLE, SECTOR_SIZE);
    } else if (sector >= (33 + 2 * SECTORS_PER_FAT)) {
        memcpy(buff, &FAT32_ROOTDIR, SECTOR_SIZE);
    } else {
        printk("no! %d %d\r\n", sector, count);
        memset(buff, 0, SECTOR_SIZE);
    }

    return 0;
}

static int disk_fake_access_write(struct disk_info *disk, const uint8_t *buff,
                                  uint32_t sector, uint32_t count) {
    return 0;
}

static int disk_fake_access_ioctl(struct disk_info *disk, uint8_t cmd,
                                  void *buff) {
    switch (cmd) {
        case DISK_IOCTL_CTRL_SYNC:
            break;
        case DISK_IOCTL_GET_SECTOR_COUNT:
            *(uint32_t *)buff = SECTOR_COUNT;
            break;
        case DISK_IOCTL_GET_SECTOR_SIZE:
            *(uint32_t *)buff = SECTOR_SIZE;
            break;
        case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
            *(uint32_t *)buff = 1U;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static const struct disk_operations fake_disk_ops = {
    .init = disk_fake_access_init,
    .status = disk_fake_access_status,
    .read = disk_fake_access_read,
    .write = disk_fake_access_write,
    .ioctl = disk_fake_access_ioctl,
};

static struct disk_info fake_disk = {
    .name = CONFIG_DISK_FAKE_VOLUME_NAME,
    .ops = &fake_disk_ops,
};

static int disk_fake_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return disk_access_register(&fake_disk);
}

SYS_INIT(disk_fake_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
