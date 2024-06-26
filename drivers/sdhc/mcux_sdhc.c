#define DT_DRV_COMPAT nxp_mcux_sdhc

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

/* inspired by nxp,imx-usdhc in imx_usdhc.c and nxp,mcux-sdif in mcux_sdif.c */

LOG_MODULE_REGISTER(mcux_sdhc, CONFIG_SDHC_LOG_LEVEL);

#include <fsl_sdhc.h>

enum transfer_callback_status {
    TRANSFER_CMD_COMPLETE  = BIT(0),
    TRANSFER_CMD_FAILED    = BIT(1),
    TRANSFER_DATA_COMPLETE = BIT(2),
    TRANSFER_DATA_FAILED   = BIT(3),
};

#define TRANSFER_CMD_FLAGS  (TRANSFER_CMD_COMPLETE | TRANSFER_CMD_FAILED)
#define TRANSFER_DATA_FLAGS (TRANSFER_DATA_COMPLETE | TRANSFER_DATA_FAILED)

#define MCUX_SDHC_DMA_BUFFER_LEN  (64)
#define MCUX_SDHC_DEFAULT_TIMEOUT (5000)

struct mcux_sdhc_host_transfer {
    sdhc_transfer_t *transfer;
    k_timeout_t command_timeout;
    k_timeout_t data_timeout;
};

struct mcux_sdhc_config {
    SDHC_Type *base;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    uint32_t max_current_330;
    uint32_t max_current_300;
    uint32_t max_current_180;
    uint32_t power_delay_ms;
    uint32_t min_bus_freq;
    uint32_t max_bus_freq;
    const struct pinctrl_dev_config *pincfg;
    void (*irq_config_func)(void);
};

struct mcux_sdhc_data {
    struct sdhc_host_props props;
    struct k_sem transfer_sem;
    volatile uint32_t transfer_status;
    sdhc_handle_t transfer_handle;
    struct sdhc_io host_io;
    struct k_mutex access_mutex;
    sdhc_interrupt_cb_t sdhc_cb;
    void *sdhc_cb_user_data;
    uint32_t dma_descriptor[MCUX_SDHC_DMA_BUFFER_LEN] __aligned(4);
};

static void mcux_sdhc_card_inserted_cb(SDHC_Type *sdhc, void *user_data) {
    const struct device *dev = user_data;
    struct mcux_sdhc_data *data = dev->data;
    if (data->sdhc_cb) {
        data->sdhc_cb(dev, SDHC_INT_INSERTED, data->sdhc_cb_user_data);
    }
}

static void mcux_sdhc_card_removed_cb(SDHC_Type *sdhc, void *user_data) {
    const struct device *dev = user_data;
    struct mcux_sdhc_data *data = dev->data;
    if (data->sdhc_cb) {
        data->sdhc_cb(dev, SDHC_INT_REMOVED, data->sdhc_cb_user_data);
    }
}

static void mcux_sdhc_sdio_interrupt_cb(SDHC_Type *sdhc, void *user_data) {
    const struct device *dev = user_data;
    struct mcux_sdhc_data *data = dev->data;
    if (data->sdhc_cb) {
        data->sdhc_cb(dev, SDHC_INT_SDIO, data->sdhc_cb_user_data);
    }
}

static void mcux_sdhc_transfer_complete_cb(SDHC_Type *sdhc, sdhc_handle_t *handle, status_t status, void *user_data) {
    const struct device *dev = user_data;
    struct mcux_sdhc_data *data = dev->data;

    if (status == kStatus_SDHC_TransferDataFailed) {
        data->transfer_status |= TRANSFER_DATA_FAILED;
    } else if (status == kStatus_SDHC_TransferDataComplete) {
        data->transfer_status |= TRANSFER_DATA_COMPLETE;
    } else if (status == kStatus_SDHC_SendCommandFailed) {
        data->transfer_status |= TRANSFER_CMD_FAILED;
    } else if (status == kStatus_SDHC_TransferCommandComplete) {
        data->transfer_status |= TRANSFER_CMD_COMPLETE;
    } else {
        LOG_ERR("unknown status %d", status);
    }
    k_sem_give(&data->transfer_sem);
}

static int mcux_sdhc_reset(const struct device *dev) {
    const struct mcux_sdhc_config *cfg = dev->config;
    return SDHC_Reset(cfg->base, kSDHC_ResetAll, 100) ? 0 : -ETIMEDOUT;
}

static int mcux_sdhc_transfer(const struct device *dev, struct mcux_sdhc_host_transfer *request) {
    const struct mcux_sdhc_config *cfg = dev->config;
    struct mcux_sdhc_data *dev_data = dev->data;

    // start transfer
    dev_data->transfer_status = 0;
    k_sem_reset(&dev_data->transfer_sem);
    status_t error = SDHC_TransferNonBlocking(cfg->base, &dev_data->transfer_handle,
        dev_data->dma_descriptor, MCUX_SDHC_DMA_BUFFER_LEN, request->transfer);
    if (error != kStatus_Success) {
        return -EIO;
    }

    // wait for command
    while ((dev_data->transfer_status & TRANSFER_CMD_FLAGS) == 0) {
        if (k_sem_take(&dev_data->transfer_sem, request->command_timeout)) {
            return -ETIMEDOUT;
        }
    }
    if (dev_data->transfer_status & TRANSFER_CMD_FAILED) {
        return -EIO;
    }

    // wait for data
    if (request->transfer->data) {
        while ((dev_data->transfer_status & TRANSFER_DATA_FLAGS) == 0) {
            if (k_sem_take(&dev_data->transfer_sem, request->data_timeout)) {
                return -ETIMEDOUT;
            }
        }
        if (dev_data->transfer_status & TRANSFER_DATA_FAILED) {
            return -EIO;
        }
    }
    return 0;
}

static void mcux_sdhc_stop_transmission(const struct device *dev) {
    sdhc_command_t stop_cmd = {
        .index        = SD_STOP_TRANSMISSION,
        .argument     = 0,
        .type         = kCARD_CommandTypeAbort,
        .responseType = SD_RSP_TYPE_R1b,
    };
    sdhc_transfer_t transfer = {
        .command = &stop_cmd,
        .data    = NULL,
    };
    struct mcux_sdhc_host_transfer request = {
        .transfer        = &transfer,
        .command_timeout = K_MSEC(MCUX_SDHC_DEFAULT_TIMEOUT),
        .data_timeout    = K_MSEC(MCUX_SDHC_DEFAULT_TIMEOUT),
    };
    mcux_sdhc_transfer(dev, &request);
}

static int mcux_sdhc_card_busy(const struct device *dev) {
    const struct mcux_sdhc_config *cfg = dev->config;
    return (SDHC_GetPresentStatusFlags(cfg->base) &
        (kSDHC_Data0LineLevelFlag | kSDHC_Data1LineLevelFlag |
         kSDHC_Data2LineLevelFlag | kSDHC_Data3LineLevelFlag)) ? 0 : 1;
}

static int mcux_sdhc_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data) {
    const struct mcux_sdhc_config *cfg = dev->config;
    struct mcux_sdhc_data *dev_data = dev->data;

    if (cmd->opcode == SD_GO_IDLE_STATE) {
        SDHC_SetCardActive(cfg->base, 0xFFFF);
    }

    // setup transfer
    sdhc_command_t host_cmd = {
        .index        = cmd->opcode,
        .argument     = cmd->arg,
        .responseType = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK),
    };
    sdhc_transfer_t transfer = {
        .command = &host_cmd,
        .data    = NULL,
    };
    struct mcux_sdhc_host_transfer request = {
        .transfer        = &transfer,
        .command_timeout = (cmd->timeout_ms == SDHC_TIMEOUT_FOREVER) ? K_FOREVER : K_MSEC(cmd->timeout_ms),
        .data_timeout    = K_NO_WAIT,
    };
    sdhc_data_t host_data = {0};
    if (data) {
        host_data.blockSize  = data->block_size;
        host_data.blockCount = data->blocks;
        switch (cmd->opcode) {
            case SD_WRITE_SINGLE_BLOCK:
            case SD_WRITE_MULTIPLE_BLOCK:
                host_data.enableAutoCommand12 = true;
                host_data.txData              = data->data;
                break;
            case SD_READ_SINGLE_BLOCK:
            case SD_READ_MULTIPLE_BLOCK:
                host_data.enableAutoCommand12 = true;
                host_data.rxData              = data->data;
                break;
            case SD_APP_SEND_SCR:
            case SD_SWITCH:
            case SD_APP_SEND_NUM_WRITTEN_BLK:
                host_data.rxData = data->data;
                break;
            default:
                return -ENOTSUP;
        }
        transfer.data = &host_data;
        request.data_timeout = (data->timeout_ms == SDHC_TIMEOUT_FOREVER) ? K_FOREVER : K_MSEC(cmd->timeout_ms);
    }

    // do transfer
    if (k_mutex_lock(&dev_data->access_mutex, K_MSEC(cmd->timeout_ms)) != 0) {
        return -EBUSY;
    }
    int ret = 0;
    do {
        ret = mcux_sdhc_transfer(dev, &request);
        if (ret && data) {
            LOG_WRN("transfer failed, sending CMD12");
            SDHC_DisableInterruptSignal   (cfg->base, kSDHC_CommandFlag | kSDHC_DataFlag | kSDHC_DataDMAFlag);
            SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_CommandFlag | kSDHC_DataFlag | kSDHC_DataDMAFlag);
            mcux_sdhc_stop_transmission(dev);
            int busy_timeout = MCUX_SDHC_DEFAULT_TIMEOUT;
            while (busy_timeout > 0) {
                if (!mcux_sdhc_card_busy(dev)) {
                    break;
                }
                k_busy_wait(125);
                busy_timeout -= 125;
            }
            if (busy_timeout <= 0) {
                LOG_ERR("card did not idle after CMD12");
                k_mutex_unlock(&dev_data->access_mutex);
                return -ETIMEDOUT;
            }
        }
    } while (ret != 0 && (cmd->retries-- > 0));

    // save response
    k_mutex_unlock(&dev_data->access_mutex);
    memcpy(cmd->response, host_cmd.response, sizeof(cmd->response));
    if (data) {
        data->bytes_xfered = dev_data->transfer_handle.transferredWords;
    }
    return ret;
}

static int mcux_sdhc_set_io(const struct device *dev, struct sdhc_io *ios) {
    const struct mcux_sdhc_config *cfg = dev->config;
    struct mcux_sdhc_data *data = dev->data;

    // clock
    uint32_t src_clk_hz;
    if (clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &src_clk_hz)) {
        return -EINVAL;
    }
    if (ios->clock && (ios->clock > data->props.f_max || ios->clock < data->props.f_min)) {
        return -EINVAL;
    }
    if (data->host_io.clock != ios->clock) {
        if (ios->clock != 0) {
            if (SDHC_SetSdClock(cfg->base, src_clk_hz, ios->clock) == 0) {
                return -ENOTSUP;
            }
        }
        data->host_io.clock = ios->clock;
    }

    // bus mode
    if (ios->bus_mode != SDHC_BUSMODE_PUSHPULL) {
        return -ENOTSUP;
    }

    // power mode
    if (data->host_io.power_mode != ios->power_mode) {
        if (ios->power_mode == SDHC_POWER_OFF) {
            LOG_WRN("TODO add power toggle support, some cards require it");
        }
        data->host_io.power_mode = ios->power_mode;
    }

    // bus width
    if (data->host_io.bus_width != ios->bus_width) {
        switch (ios->bus_width) {
            case SDHC_BUS_WIDTH1BIT: SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth1Bit); break;
            case SDHC_BUS_WIDTH4BIT: SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth4Bit); break;
            case SDHC_BUS_WIDTH8BIT: SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth8Bit); break;
            default: return -ENOTSUP;
        }
        data->host_io.bus_width = ios->bus_width;
    }

    // timing
    if (data->host_io.timing != ios->timing) {
        switch (ios->timing) {
            case SDHC_TIMING_LEGACY:
            case SDHC_TIMING_HS:
            case SDHC_TIMING_SDR12:
            case SDHC_TIMING_SDR25:
            case SDHC_TIMING_SDR50:
                break;
            default:
                return -ENOTSUP;
        }
        data->host_io.timing = ios->timing;
    }

    // signal voltage
    if (ios->signal_voltage != SD_VOL_3_3_V) {
        return -ENOTSUP;
    }

    return 0;
}

static int mcux_sdhc_get_card_present(const struct device *dev) {
    const struct mcux_sdhc_config *cfg = dev->config;
    return (SDHC_GetPresentStatusFlags(cfg->base) & kSDHC_CardInsertedFlag) ? 1 : 0;
}

static int mcux_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props) {
    struct mcux_sdhc_data *data = dev->data;
    *props = data->props;
    return 0;
}

static int mcux_sdhc_isr(const struct device *dev) {
    const struct mcux_sdhc_config *cfg = dev->config;
    struct mcux_sdhc_data *data = dev->data;
    SDHC_TransferHandleIRQ(cfg->base, &data->transfer_handle);
    return 0;
}

static int mcux_sdhc_init(const struct device *dev) {
    const struct mcux_sdhc_config *cfg = dev->config;
    struct mcux_sdhc_data *data = dev->data;
    int ret;

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret) {
        return ret;
    }
    cfg->irq_config_func();

    sdhc_config_t sdhc_cfg = {
        .cardDetectDat3      = true,
        .endianMode          = kSDHC_EndianModeLittle,
        .dmaMode             = kSDHC_DmaModeAdma2,
        .readWatermarkLevel  = 128,
        .writeWatermarkLevel = 128,
    };
    SDHC_Init(cfg->base, &sdhc_cfg);

    sdhc_transfer_callback_t sdhc_cb = {
        .CardInserted     = mcux_sdhc_card_inserted_cb,
        .CardRemoved      = mcux_sdhc_card_removed_cb,
        .SdioInterrupt    = mcux_sdhc_sdio_interrupt_cb,
        .SdioBlockGap     = NULL,
        .TransferComplete = mcux_sdhc_transfer_complete_cb,
    };
    SDHC_TransferCreateHandle(cfg->base, &data->transfer_handle, &sdhc_cb, (void*) dev);

    sdhc_capability_t caps;
    SDHC_GetCapability(cfg->base, &caps);
    data->props.f_max                         = cfg->max_bus_freq;
    data->props.f_min                         = cfg->min_bus_freq;
    data->props.power_delay                   = cfg->power_delay_ms;
    data->props.max_current_330               = cfg->max_current_330;
    data->props.max_current_300               = cfg->max_current_300;
    data->props.max_current_180               = cfg->max_current_180;
    data->props.is_spi                        = false;
    data->props.host_caps.max_blk_len         = (bool)(caps.maxBlockLength);
    data->props.host_caps.bus_8_bit_support   = (bool)(caps.flags & kSDHC_Support8BitFlag);
    data->props.host_caps.bus_4_bit_support   = (bool)(caps.flags & kSDHC_Support4BitFlag);
    data->props.host_caps.adma_2_support      = (bool)(caps.flags & kSDHC_SupportAdmaFlag);
    data->props.host_caps.high_spd_support    = (bool)(caps.flags & kSDHC_SupportHighSpeedFlag);
    data->props.host_caps.suspend_res_support = (bool)(caps.flags & kSDHC_SupportSuspendResumeFlag);
    data->props.host_caps.vol_330_support     = (bool)(caps.flags & kSDHC_SupportV330Flag);
    data->props.host_caps.vol_300_support     = 0; // kSDHC_SupportV300Flag
    data->props.host_caps.vol_180_support     = 0; // kSDHC_SupportV180Flag

    data->host_io.clock          = 0;
    data->host_io.bus_mode       = SDHC_BUSMODE_PUSHPULL;
    data->host_io.power_mode     = SDHC_POWER_ON; // TODO add power toggle support
    data->host_io.bus_width      = SDHC_BUS_WIDTH1BIT;
    data->host_io.timing         = SDHC_TIMING_LEGACY;
    data->host_io.driver_type    = SD_DRIVER_TYPE_B;
    data->host_io.signal_voltage = SD_VOL_3_3_V;

    k_mutex_init(&data->access_mutex);
    k_sem_init(&data->transfer_sem, 0, 1);

    return 0;
}

static const struct sdhc_driver_api mcux_sdhc_api = {
    .reset             = mcux_sdhc_reset,
    .request           = mcux_sdhc_request,
    .set_io            = mcux_sdhc_set_io,
    .get_card_present  = mcux_sdhc_get_card_present,
    .card_busy         = mcux_sdhc_card_busy,
    .get_host_props    = mcux_sdhc_get_host_props,
};

#define MCUX_SDHC_INIT(n)                                           \
    static void mcux_sdhc_##n##_irq_config_func(void) {             \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),      \
            mcux_sdhc_isr, DEVICE_DT_INST_GET(n), 0);               \
        irq_enable(DT_INST_IRQN(n));                                \
    }                                                               \
                                                                    \
    PINCTRL_DT_INST_DEFINE(n);                                      \
                                                                    \
    static const struct mcux_sdhc_config mcux_sdhc_##n##_config = { \
        .base            = (SDHC_Type*) DT_INST_REG_ADDR(n),        \
        .clock_dev       = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),   \
        .clock_subsys    =                                          \
            (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),   \
        .max_current_330 = DT_INST_PROP(n, max_current_330),        \
        .max_current_300 = DT_INST_PROP(n, max_current_300),        \
        .max_current_180 = DT_INST_PROP(n, max_current_180),        \
        .power_delay_ms  = DT_INST_PROP(n, power_delay_ms),         \
        .min_bus_freq    = DT_INST_PROP(n, min_bus_freq),           \
        .max_bus_freq    = DT_INST_PROP(n, max_bus_freq),           \
        .pincfg          = PINCTRL_DT_INST_DEV_CONFIG_GET(n),       \
        .irq_config_func = mcux_sdhc_##n##_irq_config_func,         \
    };                                                              \
                                                                    \
    static struct mcux_sdhc_data mcux_sdhc_##n##_data;              \
                                                                    \
    DEVICE_DT_INST_DEFINE(n,                                        \
        &mcux_sdhc_init,                                            \
        NULL,                                                       \
        &mcux_sdhc_##n##_data,                                      \
        &mcux_sdhc_##n##_config,                                    \
        POST_KERNEL,                                                \
        CONFIG_SDHC_INIT_PRIORITY,                                  \
        &mcux_sdhc_api                                              \
    );

DT_INST_FOREACH_STATUS_OKAY(MCUX_SDHC_INIT)
