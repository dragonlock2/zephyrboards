#include "common.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(sd_test, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_PROP(TEST_NODE, sd)

#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>

int sd_test(void) {
    int rc;
    FATFS fat_fs;
    struct fs_mount_t mp = {
        .type = FS_FATFS,
        .fs_data = &fat_fs,
        .mnt_point = "/SD:"
    };

    LOG_INF("initializing SD card");
    rc = disk_access_init("SD");
    if (rc) {
        LOG_ERR("SD initialization failed! %d", rc);
        return rc;
    }

    LOG_INF("mounting FS");
    rc = fs_mount(&mp);
    if (rc) {
        LOG_ERR("FS mounting failed! %d", rc);
        return rc;
    }

    LOG_INF("listing files in root directory");
    struct fs_dir_t dirp;
    struct fs_dirent entry;
    fs_dir_t_init(&dirp);
    fs_opendir(&dirp, mp.mnt_point);
    while (1) {
        fs_readdir(&dirp, &entry);
        if (entry.name[0] == 0) {
            break;
        }
        LOG_INF("type: %d name: %s size: %d", entry.type, entry.name, entry.size);
    }
    fs_closedir(&dirp);
    LOG_INF("unmounting FS");
    rc = fs_unmount(&mp);
    if (rc) {
        LOG_ERR("FS unmounting failed! %d", rc);
        return rc;
    }

    return 0;
}

#endif // DT_PROP(TEST_NODE, sd)
