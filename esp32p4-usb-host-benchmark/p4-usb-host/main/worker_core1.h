#ifndef WORKER_CORE1_H
#define WORKER_CORE1_H

#include "esp_err.h"
#include "dsp_processor.h"

esp_err_t worker_core1_init();
void worker_core1_push_burst(const detected_burst_t *burst);

#endif
