#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/llext/llext.h>
#include <zephyr/llext/buf_loader.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include "error.h"
#include "payload.h"

LOG_MODULE_REGISTER(payload, CONFIG_LOG_DEFAULT_LEVEL);

#define DRIVE_NAME "SD"
#define MOUNT_PT   "/SD:"
#define ROOT_DIR   "/SD:/"

static struct {
    FATFS fat_fs;
    struct fs_mount_t mp;
    uint8_t buffer[CONFIG_LLEXT_HEAP_SIZE * 1024] __aligned(4);
    struct llext_buf_loader loader;
    struct llext *payload;
    void (*run)(void);
} data;

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

    // read payload
    struct fs_file_t payload;
    fs_file_t_init(&payload);
    if (fs_open(&payload, payload_name, FS_O_READ) != 0) {
        LOG_ERR("failed open payload");
        error_fatal(error_reason::PAYLOAD);
    }

    ssize_t payload_len = fs_read(&payload, data.buffer, sizeof(data.buffer));
    if (payload_len < 0) {
        LOG_ERR("failed read payload");
        error_fatal(error_reason::PAYLOAD);
    }
    if (payload_len == sizeof(data.buffer)) {
        LOG_ERR("payload too large");
        error_fatal(error_reason::PAYLOAD);
    }

    if (fs_close(&payload) != 0) {
        LOG_ERR("failed close payload");
        error_fatal(error_reason::PAYLOAD);
    }

    // unmount filesystem
    if (fs_unmount(&data.mp) != 0) {
        LOG_ERR("failed fatfs unmount");
        error_fatal(error_reason::PAYLOAD);
    }

    // load payload
    data.loader = LLEXT_BUF_LOADER(data.buffer, sizeof(data.buffer));
    struct llext_load_param loader_param = LLEXT_LOAD_PARAM_DEFAULT;
    if (llext_load(&data.loader.loader, "payload", &data.payload, &loader_param) != 0) {
        LOG_ERR("failed load payload");
        error_fatal(error_reason::PAYLOAD);
    }

    data.run = reinterpret_cast<void (*)(void)>(llext_find_sym(&data.payload->exp_tab, "run"));
    if (data.run == NULL) {
        LOG_ERR("payload doesn't contain run()");
        error_fatal(error_reason::PAYLOAD);
    }
}

void payload_run(void) {
    if (data.run) {
        data.run();
    }
}
