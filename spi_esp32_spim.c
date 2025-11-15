/*
 * Copyright (c) 2020-2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// LOOK INTO THIS
// binds driver to compatible = "expressif, esp32-spi" noes
#define DT_DRV_COMPAT espressif_esp32_spi

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <hal/spi_hal.h>    // ESP-IDF hal
#include <esp_attr.h>       
#include <esp_clk_tree.h>   // ESP-IDF hal

// zephyr's logging stuff
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_spi, CONFIG_SPI_LOG_LEVEL);

#include <soc.h>
#include <esp_memory_utils.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

// GDMA STUFF?
// configures DMA transfer...?
// use zephyr DMA API (NOT ESP-IDF DMA API)
#ifdef SOC_GDMA_SUPPORTED
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#endif

#include <zephyr/drivers/clock_control.h>
#include "spi_context.h"
#include "spi_esp32_spim.h"

#if defined(CONFIG_SOC_SERIES_ESP32S2) && defined(CONFIG_ADC_ESP32_DMA) &&                         \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(spi3)) && DT_PROP(DT_NODELABEL(spi3), dma_enabled)
#error "spi3 must not have dma-enabled if ADC_ESP32_DMA is enabled for ESP32-S2"
#endif

#define SPI_DMA_MAX_BUFFER_SIZE 4092

#define SPI_DMA_RX 0
#define SPI_DMA_TX 1

// HELPER FUNCTION (static lijmits visibility to just this file
// is transfer still ongoing?
// spi_esp32_data will contain spi_context (zephyr helper)
static bool spi_esp32_transfer_ongoing(struct spi_esp32_data *data)
{
  // tx_on and rx_on are helpers; return true if still data to send
  // i relaly just need to check out spi_contextd
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

// centralizes cleanup after a transfer finishes
// HELPER FUNCTION
static inline void spi_esp32_complete(const struct device *dev,
				      struct spi_esp32_data *data __maybe_unused,
				      spi_dev_t *spi, int status)
{
  // check this out later 
#ifdef CONFIG_SPI_ESP32_INTERRUPT
  // hall low-level (LL) helper to disable SPI peripheral interrupts
  // prevents further SPI interrupt callbacks from firing while we clean up,
  // report completion.
  // basically prevents re-entrancy of ISR 
	spi_ll_disable_int(spi);
  // clears any pending interrup tstatus flags in SPI peripheral registers
  // thus -- avoids immediately re-triggering an interrupt or leaving stale
  // flags set
	spi_ll_clear_int_stat(spi);
#endif

  // controls the device/sCS lines according to spi_context state
  // false -- deassert CS so SPI device is no lnoger needed
  // internally -- handles whether CS is a GPIO under driver control?
  // ULTIMATLEY -- peripheral deslected
	spi_context_cs_control(&data->ctx, false);

  // notifies spi context that the transaction is complete
  // behavior depends on sync / async use
  // for sync (transceive blcoking): wakes teh waiting thread and sets return
  // status for caller
  // async: runs registered callback.
  // basically 0 hands the reslt back to whever requested the SPI transfer...
#ifdef CONFIG_SPI_ESP32_INTERRUPT
	spi_context_complete(&data->ctx, dev, status);
#endif
}

