#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>

#include "mavwrap_common.h"
#include "mavwrap_lora.h"


LOG_MODULE_DECLARE(mavwrap);


/**
 * Map raw DTS values from mavwrap_lora_config to Zephyr lora_modem_config enums.
 * Returns 0 on success, -EINVAL if any value is out of range.
 */
static int lora_cfg_from_mavwrap(const struct mavwrap_lora_config *src,
                                  struct lora_modem_config *dst)
{
	dst->frequency          = src->frequency;
	dst->tx_power           = src->tx_power;
	dst->preamble_len       = src->preamble_len;
	dst->public_network     = src->public_network;
	dst->iq_inverted        = src->iq_inverted;
	dst->packet_crc_disable = src->crc_disabled;
	dst->tx                 = false; 

	switch (src->bandwidth) {
	case 125:
	case 250:
	case 500:
		dst->bandwidth = (enum lora_signal_bandwidth)src->bandwidth;
		break;
	default:
		LOG_ERR("Invalid LoRa bandwidth: %u kHz", src->bandwidth);
		return -EINVAL;
	}

	if (src->datarate < 5 || src->datarate > 12) {
		LOG_ERR("Invalid LoRa spreading factor: %u (valid: 5..12)", src->datarate);
		return -EINVAL;
	}
	dst->datarate = (enum lora_datarate)src->datarate;

	switch (src->coding_rate) {
	case 5: dst->coding_rate = CR_4_5; break;
	case 6: dst->coding_rate = CR_4_6; break;
	case 7: dst->coding_rate = CR_4_7; break;
	case 8: dst->coding_rate = CR_4_8; break;
	default:
		LOG_ERR("Invalid LoRa coding rate: 4/%u", src->coding_rate);
		return -EINVAL;
	}

	return 0;
}


/**
 * Internal LoRa async RX callback.
 * user_data points to the mavwrap device — passed via lora_recv_async().
 * Forwards data to mavwrap transport callback and re-arms RX.
 */
static void lora_internal_rx_cb(const struct device *lora_dev,
                                 uint8_t *buf, uint16_t len,
                                 int16_t rssi, int8_t snr,
                                 void *user_data)
{
	const struct device *dev = user_data;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;

	if (len > 0 && lora_data->rx_callback) {
		lora_data->last_rx_rssi = rssi;
		lora_data->last_rx_snr = snr;
		lora_data->rx_callback(dev, buf, len, lora_data->user_data);
	}

	/* Re-arm async RX */
	lora_recv_async(lora_dev, lora_internal_rx_cb, user_data);
}


static int mavwrap_lora_init(const struct device *dev)
{
	const struct mavwrap_config *cfg = dev->config;
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;
	const struct mavwrap_lora_config *lora_cfg = cfg->transport_config;
	const struct device *lora_dev = cfg->transport_dev;
	int ret;

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("[%s] LoRa device %s not ready", dev->name, lora_dev->name);
		return -ENODEV;
	}

	lora_data->lora_dev = lora_dev;

	k_mutex_init(&lora_data->config_mutex);

	ret = lora_cfg_from_mavwrap(lora_cfg, &lora_data->dt_cfg);
	if (ret < 0) {
		return ret;
	}

	lora_data->runtime_cfg = lora_data->dt_cfg;

	ret = lora_config(lora_dev, &lora_data->runtime_cfg);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to configure LoRa modem: %d", dev->name, ret);
		return ret;
	}

	LOG_INF("[%s] LoRa transport initialized (%u Hz, SF%u, BW%u kHz)",
	        dev->name, lora_cfg->frequency, lora_cfg->datarate, lora_cfg->bandwidth);

	return 0;
}


static int mavwrap_lora_send(const struct device *dev,
                              const uint8_t *buf, size_t len)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;
	int ret;

	if (!buf || len == 0) {
		return -EINVAL;
	}

	k_mutex_lock(&lora_data->config_mutex, K_FOREVER);

	/* Stop async RX before reconfiguring — lora_config returns -EBUSY otherwise */
	if (lora_data->rx_callback) {
		lora_recv_async(lora_data->lora_dev, NULL, NULL);
	}

	/* Switch to TX mode */
	lora_data->runtime_cfg.tx = true;
	ret = lora_config(lora_data->lora_dev, &lora_data->runtime_cfg);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to switch to TX mode: %d", dev->name, ret);
		goto out;
	}

	ret = lora_send(lora_data->lora_dev, (uint8_t *)buf, len);
	if (ret < 0) {
		LOG_ERR("[%s] lora_send failed: %d", dev->name, ret);
	}

out:
	/* Switch back to RX mode */
	lora_data->runtime_cfg.tx = false;
	lora_config(lora_data->lora_dev, &lora_data->runtime_cfg);

	k_mutex_unlock(&lora_data->config_mutex);

	if (lora_data->rx_callback) {
		lora_recv_async(lora_data->lora_dev, lora_internal_rx_cb, (void *)dev);
	}

	return ret;
}


static int mavwrap_lora_set_rx_callback(const struct device *dev,
                                         mavwrap_transport_rx_cb_t callback,
                                         void *user_data)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;

	lora_data->rx_callback = callback;
	lora_data->user_data   = user_data;

	return lora_recv_async(lora_data->lora_dev, lora_internal_rx_cb, (void *)dev);
}


static int mavwrap_lora_set_property(const struct device *dev,
                                      const struct mavwrap_property_value *prop)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;
	int ret = 0;

	k_mutex_lock(&lora_data->config_mutex, K_FOREVER);

	switch (prop->type) {
	case MAVWRAP_PROPERTY_LORA_FREQUENCY:
		lora_data->runtime_cfg.frequency = prop->value.u32;
		break;
	case MAVWRAP_PROPERTY_LORA_TX_POWER:
		lora_data->runtime_cfg.tx_power = (int8_t)prop->value.u32;
		break;
	case MAVWRAP_PROPERTY_LORA_BANDWIDTH:
		switch (prop->value.u32) {
		case 125:
		case 250:
		case 500:
			lora_data->runtime_cfg.bandwidth =
				(enum lora_signal_bandwidth)prop->value.u32;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}
		break;
	case MAVWRAP_PROPERTY_LORA_DATARATE:
		if (prop->value.u32 < 5 || prop->value.u32 > 12) {
			ret = -EINVAL;
			goto out;
		}
		lora_data->runtime_cfg.datarate = (enum lora_datarate)prop->value.u32;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	if (prop->apply_immediately) {
		if (lora_data->rx_callback) {
			lora_recv_async(lora_data->lora_dev, NULL, NULL);
		}
		lora_data->runtime_cfg.tx = false;
		ret = lora_config(lora_data->lora_dev, &lora_data->runtime_cfg);
		if (ret < 0) {
			LOG_ERR("[%s] Failed to apply LoRa config: %d", dev->name, ret);
		}
	}

out:
	k_mutex_unlock(&lora_data->config_mutex);

	if (prop->apply_immediately && lora_data->rx_callback) {
		lora_recv_async(lora_data->lora_dev, lora_internal_rx_cb, (void *)dev);
	}

	return ret;
}


static int mavwrap_lora_get_property(const struct device *dev,
                                      struct mavwrap_property_value *prop)
{
	struct mavwrap_data *data = dev->data;
	struct mavwrap_lora_data *lora_data = data->transport_data;
	int ret = 0;

	k_mutex_lock(&lora_data->config_mutex, K_FOREVER);

	switch (prop->type) {
	case MAVWRAP_PROPERTY_LORA_FREQUENCY:
		prop->value.u32 = lora_data->runtime_cfg.frequency;
		break;
	case MAVWRAP_PROPERTY_LORA_TX_POWER:
		prop->value.u32 = (uint32_t)(int32_t)lora_data->runtime_cfg.tx_power;
		break;
	case MAVWRAP_PROPERTY_LORA_BANDWIDTH:
		prop->value.u32 = (uint32_t)lora_data->runtime_cfg.bandwidth;
		break;
	case MAVWRAP_PROPERTY_LORA_DATARATE:
		prop->value.u32 = (uint32_t)lora_data->runtime_cfg.datarate;
		break;
	case MAVWRAP_PROPERTY_LORA_LAST_RSSI:
		prop->value.u16 = (uint_16_t)lora_data->last_rx_rssi;
		break;
	case MAVWRAP_PROPERTY_LORA_LAST_SNR:
		prop->value.u16 = (uint_16_t)lora_data->last_rx_snr;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&lora_data->config_mutex);
	return ret;
}


const struct mavwrap_transport_ops mavwrap_lora_ops = {
	.init             = mavwrap_lora_init,
	.send             = mavwrap_lora_send,
	.set_rx_callback  = mavwrap_lora_set_rx_callback,
	.set_property     = mavwrap_lora_set_property,
	.get_property     = mavwrap_lora_get_property,
};
