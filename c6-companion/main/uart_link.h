// UART receiver: decodes inter-chip protocol frames from the P4
// and dispatches them. See common/iridium_protocol/iridium_protocol.h.

#pragma once

#include "esp_err.h"

esp_err_t uart_link_init(void);

// Set true after we receive IRP_TYPE_BOOT_COMPLETE.
// Web UI uses this for the "P4 ready" indicator.
bool uart_link_p4_ready(void);
