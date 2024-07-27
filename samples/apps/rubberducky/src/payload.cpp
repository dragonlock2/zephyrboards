#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/llext/llext.h>
#include <zephyr/llext/loader.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include "error.h"
#include "hid.h"
#include "rgb.h"
#include "payload.h"

LOG_MODULE_REGISTER(payload, CONFIG_LOG_DEFAULT_LEVEL);

#define DRIVE_NAME "SD"
#define MOUNT_PT   "/SD:"
#define ROOT_DIR   "/SD:/"

struct llext_file_loader {
    struct fs_file_t payload;
    struct llext_loader loader;
};

static struct {
    FATFS fat_fs;
    struct fs_mount_t mp;
    struct llext_file_loader loader;
    struct llext *payload;
    void (*run)(void);
} data;

static int llext_file_read(struct llext_loader *ldr, void *out, size_t len) {
    struct llext_file_loader *lf = CONTAINER_OF(ldr, struct llext_file_loader, loader);
    return fs_read(&lf->payload, out, len) == (ssize_t) len ? 0 : -1;
}

static int llext_file_seek(struct llext_loader *ldr, size_t pos) {
    struct llext_file_loader *lf = CONTAINER_OF(ldr, struct llext_file_loader, loader);
    return fs_seek(&lf->payload, pos, FS_SEEK_SET);
}

void payload_init(void) {
    // mount filesystem
    if (disk_access_init(DRIVE_NAME) != 0) {
        LOG_ERR("failed init drive");
        error_fatal(error_reason::PAYLOAD);
    }

    data.mp.type = FS_FATFS;
    data.mp.fs_data = &data.fat_fs;
    data.mp.mnt_point = MOUNT_PT;
    if (fs_mount(&data.mp) != 0) {
        LOG_ERR("failed fatfs mount");
        error_fatal(error_reason::PAYLOAD);
    }

    // get payload name
    struct fs_file_t cfg;
    fs_file_t_init(&cfg);
    if (fs_open(&cfg, ROOT_DIR "config.txt", FS_O_READ) != 0) {
        LOG_ERR("failed open config.txt");
        error_fatal(error_reason::PAYLOAD);
    }

    char payload_name[strlen(ROOT_DIR) + 128] = ROOT_DIR;
    ssize_t end = fs_read(&cfg, &payload_name[strlen(ROOT_DIR)], 128);
    if (end < 0) {
        LOG_ERR("failed read config.txt");
        error_fatal(error_reason::PAYLOAD);
    }
    if (end == 128) {
        LOG_ERR("payload filename too long");
        error_fatal(error_reason::PAYLOAD);
    }
    payload_name[strlen(ROOT_DIR) + end] = '\0';
    LOG_INF("using \"%s\"", &payload_name[strlen(ROOT_DIR)]);

    if (fs_close(&cfg) != 0) {
        LOG_ERR("failed close config.txt");
        error_fatal(error_reason::PAYLOAD);
    }

    // load payload
    fs_file_t_init(&data.loader.payload);
    if (fs_open(&data.loader.payload, payload_name, FS_O_READ) != 0) {
        LOG_ERR("failed open payload");
        error_fatal(error_reason::PAYLOAD);
    }

    data.loader.loader.read = llext_file_read;
    data.loader.loader.seek = llext_file_seek;
    data.loader.loader.peek = NULL;
    struct llext_load_param loader_param = LLEXT_LOAD_PARAM_DEFAULT;
    if (llext_load(&data.loader.loader, "payload", &data.payload, &loader_param) == 0) {
        data.run = reinterpret_cast<void (*)(void)>(llext_find_sym(&data.payload->exp_tab, "run"));
    } else {
        LOG_ERR("failed load payload");
    }

    if (fs_close(&data.loader.payload) != 0) {
        LOG_ERR("failed close payload");
        error_fatal(error_reason::PAYLOAD);
    }

    // unmount filesystem
    if (fs_unmount(&data.mp) != 0) {
        LOG_ERR("failed fatfs unmount");
        error_fatal(error_reason::PAYLOAD);
    }
}

void payload_run(void) {
    if (data.run) {
        LOG_INF("start");
        data.run();
        LOG_INF("done");
    } else {
        LOG_ERR("no payload run()");
        error_fatal(error_reason::PAYLOAD);
    }
}

EXPORT_SYMBOL(hid_get_os);
EXPORT_SYMBOL(hid_keyboard_raw);
EXPORT_SYMBOL(hid_mouse_raw);
EXPORT_SYMBOL(rgb_write);
