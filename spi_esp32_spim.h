/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_
#define ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_

// used to configure SCLK, MOSI, MISO, CS pins
#include <zephyr/drivers/pinctrl.h>

// vendor HAL types for SPI controller manipulation
// includes register configuration, transactions, timing...
#include <hal/spi_hal.h>

// LOOK INTO THIS
// GDMA: newer ESPDMA engine. code can selectively compile DMA support?
#ifdef SOC_GDMA_SUPPORTED
#include <hal/gdma_hal.h>
#endif

// frequency macros
// maps common bus speeds to approximate divisors of ABP_CLK_FREQ (peripheral
// abp clock
// used to pick clock dividers that the SPI HAL expects?
// SLAVE DRIVER WILL NOT GENERATE CLOCK, so we are good for that. unless we
// want some internal timing / sample delay calibration??
#define SPI_MASTER_FREQ_8M      (APB_CLK_FREQ/10)
#define SPI_MASTER_FREQ_9M      (APB_CLK_FREQ/9)    /* 8.89MHz */
#define SPI_MASTER_FREQ_10M     (APB_CLK_FREQ/8)    /* 10MHz */
#define SPI_MASTER_FREQ_11M     (APB_CLK_FREQ/7)    /* 11.43MHz */
#define SPI_MASTER_FREQ_13M     (APB_CLK_FREQ/6)    /* 13.33MHz */
#define SPI_MASTER_FREQ_16M     (APB_CLK_FREQ/5)    /* 16MHz */
#define SPI_MASTER_FREQ_20M     (APB_CLK_FREQ/4)    /* 20MHz */
#define SPI_MASTER_FREQ_26M     (APB_CLK_FREQ/3)    /* 26.67MHz */
#define SPI_MASTER_FREQ_40M     (APB_CLK_FREQ/2)    /* 40MHz */
#define SPI_MASTER_FREQ_80M     (APB_CLK_FREQ/1)    /* 80MHz */

/*
 * NOTES FOR LATER: this is built for master behavior.
 * LOOK AT NXP TO SEE HOW THEY DIFFERENTIATE CONFIGS
 * for slave, we want: cs_gpio (pin used for slave CS to detect edge
 * slave_mode_flags
 * sample_edge or input_delay_ns (still relevant)
 * whether to allow continuous transferes when CS is low (?)
 * FIFO thresholds for RX/TX interrupts
 * DMA channel mapping for slave transfers (?)
 */
struct spi_esp32_config {
  // pointer to vendor HAL device structure repping the SPI peripheral instance
  // (the register set).
  // check spi_hal.h
	spi_dev_t *spi;

  // zephyr binding for clock control?
  // enables/disables peripheral clocks. queries clock frequency.
	const struct device *clock_dev;

  // MASTER SPECIFIC
  // ratio of high/low SCLK? either way, not used for slave bcz slave follows
  // master clock
	int duty_cycle;

  // delay inserted on input sampling. used to shift the sampling edge relative
  // to SCLK for signal integrity.
  // important for mathing master's timing in slave mode -- slaves need 2 align
  // sampling edge.
	int input_delay_ns;

  // IRQ source idnetifiers (platofrm-specific)
  // priority and zephyr IRQ flags used when connecting ISR
  // ??
	int irq_source;
	int irq_priority;
	int irq_flags;

  // pointer to pinctrl config for SPI instance.
  // controls which pins are assigned to SPI signals via devicetree / pinctrl
	const struct pinctrl_dev_config *pcfg;

  // subystem; used w/ zephyr_clk_api to control peripheral clock
	clock_control_subsys_t clock_subsys;

  // whether to use IOMUX (internal pin matrix) vs GPIO matrix routing
	bool use_iomux;

  // whether DMA is used for transfers
  // master driver commonly supports DMA for larger transfers
  // LOOK INTO THIS
	bool dma_enabled;

  // DMA host idntifier. used when configuring DMA
	int dma_host;

  // if GDMA SUPPORTED (ignore for now
#if defined(SOC_GDMA_SUPPORTED)
	const struct device *dma_dev;
	uint8_t dma_tx_ch;
	uint8_t dma_rx_ch;
#else
  // legacy clock source identifier
	int dma_clk_src;
#endif

  // chipselect timing in clock cycles / microseconds.
  // how long ot assert CS before and after?
  // MASTER-SPECIFIC. defines delays applied to CS around transfers
  // for slave, CS timing is controlled by master, but driver should monitor CS
  // for transaction framing.
	int cs_setup;
	int cs_hold;

  // whether the SPI lines idle low (affects polarity?)
  
	bool line_idle_low;

  // HAL-speicifc type choosing clock source for SPI peripheral (ABP, PLL, etc)
  // mostly master
	spi_clock_source_t clock_source;
};

struct spi_esp32_data {
  // zephyr helper struct
  // enapuslates tx/rx buffers, transfer state, sync primitives, and helper
  // functions to manage transfersacrosss sync/async APIs
  // RESUSE THIS!!!!
  // to support options like SPI_TRANSCEIVE, SPI_ASYNC, ...
	struct spi_context ctx;

  // HAL runtime context for SPI register-level ops
  // holds state required by venor HAL to manipulate SPI hardware
	spi_hal_context_t hal;

  // hal config parameters (bit order, etc)
  // populated at initiation or prior to transaction
	spi_hal_config_t hal_config;

  // GDMA stuff
#ifdef SOC_GDMA_SUPPORTED
	gdma_hal_context_t hal_gdma;
#endif

  // hal STRUCTURE -- HOLDS TIMING-RELATED CONFIGURATION.
  // useful to tune sapmling window; both master and slave
	spi_hal_timing_conf_t timing_config;

  // device-level config -- FIFO sizes, hardware quircks, number of data lines
  // enabled...
	spi_hal_dev_config_t dev_config;

  // transaction-level config--per-transfer settings (bit length, direction,
  // dummy cycles)
	spi_hal_trans_config_t trans_config;

  // data frame size (bits per word) -- eg 8 for byte transfers
	uint8_t dfs;

  // DMA descirptors for TX/RX when using leegacy DMA
	lldesc_t dma_desc_tx;
	lldesc_t dma_desc_rx;

  // numeric value of clock source frequency used to calculate divisors,
  // timings
	uint32_t clock_source_hz;
};

#endif /* ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_ */

/*
 * DMA? (PERSONAL NOTES)
 * 
 * normally, when data moves b/ta peripheral like PSI and RAM, CPU would need
 * to read/write every byte:
 *  1. CPU reads byte from SPI RX register -> stores in RAM
 *  2. CPU load byte form RAM -> writes to SPI TX register
 * thus, every single byte requries CPU instructions, context switcing,
 * interrupt handling....
 *
 * DMA: letes hardware copy data direclty b/t RAM and the peripheral, w/o the
 * CPU. the CPU only configures the DMA, starts the transfer, gets interrupted
 * when finished... 
 *
 * essentially:
 * spi DMA allows large buffers and continuous streaming. in SPI slave mode, if
 * the master clocks bytes faster than CPU can serice them through inteerrupts,
 * you lose data and the transfer corrupts.  it guanrantees timing
 *
 * In practice:
 * TX (slave -> master)
 *  1. driver prepares a TX buffer
 *  2. DMA pointed to buffer
 *  3. when master clocks data, DMA feeds bytes into Spi TX FIFO automaticlaly
 *
 * RX (master -> slave)
 *  1. driver allocates RX buffer
 *  2. DMA ponted at it
 *  3. as master clocks bytes, DMA writes tehm directly to RAM
 */
