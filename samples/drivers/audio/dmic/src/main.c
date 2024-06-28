/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_pdm.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(dmic_sample, LOG_LEVEL_NONE);

#define NUMBER_OF_CHANNELS 1

#define MAX_SAMPLE_RATE	 16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT	 1000

#define BLOCK_COUNT		    8
#define MILLIS_TO_MEASURE_PER_BLOCK 1

/* Size of a block for 100 ms of audio data. */
// #define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
// 	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Size of a block for 1 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
	(BYTES_PER_SAMPLE * (_sample_rate * (MILLIS_TO_MEASURE_PER_BLOCK / 1000.0)) *              \
	 _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MAX_SAMPLE_RATE, NUMBER_OF_CHANNELS)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

#define GPIO_PIN_ENABLE_3V3 26

uint32_t tick_counter = 0;

static int do_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg)
{
	int ret;

	LOG_DBG("PCM output rate: %u, channels: %u", cfg->streams[0].pcm_rate,
		cfg->channel.req_num_chan);

	ret = dmic_configure(dmic_dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}

	while (true) {
		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		if (ret < 0) {
			LOG_ERR("START trigger failed: %d", ret);
			return ret;
		}
		void *buffer;
		uint32_t size;
		uint32_t volume_sum = 0;
		int32_t volume = 0;

		uint8_t loop_count = (NUMBER_OF_CHANNELS * BLOCK_COUNT);
		// uint8_t loop_count = 5;

		for (int i = 0; i < loop_count; ++i) {
			ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
			if (ret < 0) {
				// LOG_ERR("%d - read failed: %d", i, ret);
				LOG_ERR("read failed: %d", ret);
				return ret;
			}
			LOG_DBG("%d/%d: got buffer %p of %u bytes", i + 1, loop_count, buffer,
				size);

			// Add to the volume sum
			int16_t *samples = (int16_t *)buffer;
			for (int j = 0; j < size / BYTES_PER_SAMPLE; ++j) {
				volume_sum += abs(samples[j]);
				volume += samples[j];
			}

			k_mem_slab_free(&mem_slab, &buffer);
		}

		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		if (ret < 0) {
			LOG_ERR("STOP trigger failed: %d", ret);
			return ret;
		}

		// Divide volume sum by the number of samples to get the average volume
		volume_sum /= (loop_count * (size / BYTES_PER_SAMPLE));

		if (++tick_counter == 1) {

			LOG_INF("Volume sum: %u", volume_sum);
			printk("%d, %d, %d, %d,\n", volume_sum, 0, 0, 0);

			tick_counter = 0;
		}
		// Release the buffer

		// k_msleep(10);
		// break;
	}

	return ret;
}

// This function should run a loop and measure the volume of the PDM input source
void main(void)
{
	const struct device *dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	int ret;

	// Set 3.3V pin to output low
	const struct device *dev = device_get_binding("GPIO_0");
	if (dev == NULL) {
		LOG_ERR("Could not get GPIO device");
		return;
	}

	gpio_pin_configure(dev, GPIO_PIN_ENABLE_3V3, GPIO_OUTPUT);
	gpio_pin_set(dev, GPIO_PIN_ENABLE_3V3, 0);

	LOG_INF("DMIC volume measurement loop");

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
		.block_size = MAX_BLOCK_SIZE,
		.pcm_rate = MAX_SAMPLE_RATE,
	};
	struct dmic_cfg cfg = {
		.io =
			{
				/* These fields can be used to limit the PDM clock
				 * configurations that the driver is allowed to use
				 * to those supported by the microphone.
				 */
				.min_pdm_clk_freq = 1000000,
				.max_pdm_clk_freq = 2400000,
				.min_pdm_clk_dc = 40,
				.max_pdm_clk_dc = 60,
			},
		.streams = &stream,
		.channel =
			{
				.req_num_streams = 1,
				.req_num_chan = 1,
				.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
			},
	};

	ret = do_pdm_transfer(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to transfer audio data: %d", ret);
		return;
	}
}