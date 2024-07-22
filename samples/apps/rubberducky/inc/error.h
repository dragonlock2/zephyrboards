#pragma once

enum class error_reason {
    HID,
    PAYLOAD,
};

void error_fatal(error_reason code);
