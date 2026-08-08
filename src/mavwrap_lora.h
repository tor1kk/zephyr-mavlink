/**
 * @file mavwrap_lora.h
 * @brief LoRa transport definitions
 *
 * This file contains LoRa-specific structures and definitions.
 * Only included when CONFIG_MAVWRAP_TRANSPORT_LORA is enabled.
 */

#ifndef MAVWRAP_LORA_H
#define MAVWRAP_LORA_H

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * LoRa transport compile-time configuration (populated from DTS).
 * Stores raw human-readable values that are mapped to Zephyr lora_modem_config
 * enums during mavwrap_lora_init().
 */
struct mavwrap_lora_config {
	uint32_t frequency;       /**< Center frequency in Hz, e.g. 868000000 */
	uint16_t bandwidth;       /**< Bandwidth in kHz: 125, 250 or 500 */
	uint8_t  datarate;        /**< Spreading factor: 5..12 */
	uint8_t  coding_rate;     /**< Coding rate denominator: 5=4/5, 6=4/6, 7=4/7, 8=4/8 */
	int8_t   tx_power;        /**< TX power in dBm */
	uint16_t preamble_len;    /**< Preamble length in symbols */
	bool     public_network;  /**< True for LoRaWAN public network sync word */
	bool     iq_inverted;     /**< IQ signal inversion */
	bool     crc_disabled;    /**< Disable 16-bit payload CRC */
};

/**
 * LoRa transport runtime data.
 */
struct mavwrap_lora_data {
	const struct device *lora_dev;

	struct lora_modem_config runtime_cfg; /**< Mutable, updated via set_property */
	struct lora_modem_config dt_cfg;      /**< Read-only copy of DTS defaults */

	mavwrap_transport_rx_cb_t rx_callback;
	void *user_data;

	struct k_mutex config_mutex;

	int16_t last_rx_rssi;
	int8_t last_rx_snr;
};


#ifdef __cplusplus
}
#endif

#endif /* MAVWRAP_LORA_H */
