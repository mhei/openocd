// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Bitbang driver for Linux GPIO descriptors through libgpiod
 * Copyright (C) 2020 Antonio Borneo <borneo.antonio@gmail.com>
 *
 * Largely based on sysfsgpio driver
 * Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au
 * Copyright (C) 2014 by Jean-Christian de Rivaz <jc@eclis.ch>
 * Copyright (C) 2014 by Paul Fertser <fercerpav@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gpiod.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#ifdef HAVE_LIBGPIOD_V2

struct gpiod_line {
	struct gpiod_chip *chip;
	struct gpiod_line_request *line_request;
	unsigned int gpio_num;
};

static inline int gpiod_line_get_value(struct gpiod_line *gpio_line)
{
	return gpiod_line_request_get_value(gpio_line->line_request, gpio_line->gpio_num);
}

static inline int gpiod_line_set_value(struct gpiod_line *gpio_line, int value)
{
	return gpiod_line_request_set_value(gpio_line->line_request, gpio_line->gpio_num, value);
}

static struct gpiod_chip *gpiod_chip_open_by_number(unsigned int chip_num)
{
	char chip_path[32];
	int l;

	l = snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%u", chip_num);
	if (l < 0 || l >= (int)sizeof(chip_path))
		return NULL;

	return gpiod_chip_open(chip_path);
}

static struct gpiod_line *gpiod_line_get(struct gpiod_chip *chip, int gpio_num)
{
	struct gpiod_line *rv;

	rv = calloc(sizeof(struct gpiod_line), 1);
	if (!rv)
		return NULL;

	rv->chip = chip;
	rv->gpio_num = gpio_num;

	return rv;
}

static void gpiod_line_release(struct gpiod_line *gpio_line)
{
	/* note: we must not touch the gpio_line->chip here */

	gpiod_line_request_release(gpio_line->line_request);
	free(gpio_line);
}

static int gpiod_line_request_input(struct gpiod_line *gpio_line, const char *consumer)
{
	struct gpiod_line_settings *line_settings = NULL;
	struct gpiod_line_config *line_config = NULL;
	struct gpiod_request_config *req_cfg = NULL;
	int retval = -1;

	line_settings = gpiod_line_settings_new();
	line_config = gpiod_line_config_new();
	req_cfg = gpiod_request_config_new();

	if (!line_settings || !line_config || !req_cfg)
		goto err_out;

	retval = gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_INPUT);
	if (retval != 0)
		goto err_out;

	retval = gpiod_line_config_add_line_settings(line_config, &gpio_line->gpio_num, 1, line_settings);
	if (retval != 0)
		goto err_out;

	gpiod_request_config_set_consumer(req_cfg, consumer);

	gpio_line->line_request = gpiod_chip_request_lines(gpio_line->chip, req_cfg, line_config);
	if (!gpio_line->line_request)
		goto err_out;

	retval = 0;

err_out:
	gpiod_line_settings_free(line_settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(req_cfg);
	return retval;
}

static int gpiod_line_request_output(struct gpiod_line *gpio_line, const char *consumer, int value)
{
	struct gpiod_line_settings *line_settings = NULL;
	struct gpiod_line_config *line_config = NULL;
	struct gpiod_request_config *req_cfg = NULL;
	int retval = -1;

	line_settings = gpiod_line_settings_new();
	line_config = gpiod_line_config_new();
	req_cfg = gpiod_request_config_new();

	if (!line_settings || !line_config || !req_cfg)
		goto err_out;

	retval = gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_OUTPUT);
	if (retval != 0)
		goto err_out;

	retval = gpiod_line_settings_set_output_value(line_settings,
			value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
	if (retval != 0)
		goto err_out;

	retval = gpiod_line_config_add_line_settings(line_config, &gpio_line->gpio_num, 1, line_settings);
	if (retval != 0)
		goto err_out;

	gpiod_request_config_set_consumer(req_cfg, consumer);

	gpio_line->line_request = gpiod_chip_request_lines(gpio_line->chip, req_cfg, line_config);
	if (!gpio_line->line_request)
		goto err_out;

	retval = 0;

err_out:
	gpiod_line_settings_free(line_settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(req_cfg);
	return retval;
}
#endif

static struct gpiod_chip *gpiod_chip[ADAPTER_GPIO_IDX_NUM] = {};
static struct gpiod_line *gpiod_line[ADAPTER_GPIO_IDX_NUM] = {};

static int last_swclk;
static int last_swdio;
static bool last_stored;
static bool swdio_input;

static const struct adapter_gpio_config *adapter_gpio_config;

/*
 * Helper function to determine if gpio config is valid
 *
 * Assume here that there will be less than 10000 gpios per gpiochip, and less
 * than 1000 gpiochips.
 */
static bool is_gpio_config_valid(enum adapter_gpio_config_index idx)
{
	return adapter_gpio_config[idx].chip_num < 1000
		&& adapter_gpio_config[idx].gpio_num < 10000;
}

/* Bitbang interface read of TDO */
static bb_value_t linuxgpiod_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_line[ADAPTER_GPIO_IDX_TDO]);
	if (retval < 0) {
		LOG_WARNING("reading tdo failed");
		return 0;
	}

	return retval ? BB_HIGH : BB_LOW;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Seeing as this is the only function where the outputs are changed,
 * we can cache the old value to avoid needlessly writing it.
 */
static int linuxgpiod_write(int tck, int tms, int tdi)
{
	static int last_tck;
	static int last_tms;
	static int last_tdi;

	static int first_time;

	int retval;

	if (!first_time) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		first_time = 1;
	}

	if (tdi != last_tdi) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TDI], tdi);
		if (retval < 0)
			LOG_WARNING("writing tdi failed");
	}

	if (tms != last_tms) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TMS], tms);
		if (retval < 0)
			LOG_WARNING("writing tms failed");
	}

	/* write clk last */
	if (tck != last_tck) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TCK], tck);
		if (retval < 0)
			LOG_WARNING("writing tck failed");
	}

	last_tdi = tdi;
	last_tms = tms;
	last_tck = tck;

	return ERROR_OK;
}

static int linuxgpiod_swdio_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO]);
	if (retval < 0) {
		LOG_WARNING("Fail read swdio");
		return 0;
	}

	return retval;
}

static void linuxgpiod_swdio_drive(bool is_output)
{
	int retval;

#ifndef HAVE_LIBGPIOD_V2
	/*
	 * change direction requires release and re-require the line in libgpiod < 2.x
	 * https://stackoverflow.com/questions/58735140/
	 */
	gpiod_line_release(gpiod_line[ADAPTER_GPIO_IDX_SWDIO]);
#endif

	if (is_output) {
		if (gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR]) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR], 1);
			if (retval < 0)
				LOG_WARNING("Fail set swdio_dir");
		}
		retval = gpiod_line_request_output(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], "OpenOCD", 1);
		if (retval < 0)
			LOG_WARNING("Fail request_output line swdio");
	} else {
		retval = gpiod_line_request_input(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], "OpenOCD");
		if (retval < 0)
			LOG_WARNING("Fail request_input line swdio");
		if (gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR]) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR], 0);
			if (retval < 0)
				LOG_WARNING("Fail set swdio_dir");
		}
	}

	last_stored = false;
	swdio_input = !is_output;
}

static int linuxgpiod_swd_write(int swclk, int swdio)
{
	int retval;

	if (!swdio_input) {
		if (!last_stored || (swdio != last_swdio)) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], swdio);
			if (retval < 0)
				LOG_WARNING("Fail set swdio");
		}
	}

	/* write swclk last */
	if (!last_stored || (swclk != last_swclk)) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWCLK], swclk);
		if (retval < 0)
			LOG_WARNING("Fail set swclk");
	}

	last_swdio = swdio;
	last_swclk = swclk;
	last_stored = true;

	return ERROR_OK;
}

static int linuxgpiod_blink(int on)
{
	int retval;

	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_LED))
		return ERROR_OK;

	retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_LED], on);
	if (retval < 0)
		LOG_WARNING("Fail set led");
	return retval;
}

static struct bitbang_interface linuxgpiod_bitbang = {
	.read = linuxgpiod_read,
	.write = linuxgpiod_write,
	.swdio_read = linuxgpiod_swdio_read,
	.swdio_drive = linuxgpiod_swdio_drive,
	.swd_write = linuxgpiod_swd_write,
	.blink = linuxgpiod_blink,
};

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static int linuxgpiod_reset(int trst, int srst)
{
	int retval1 = 0, retval2 = 0;

	LOG_DEBUG("linuxgpiod_reset");

	/*
	 * active low behaviour handled by "adaptor gpio" command and
	 * GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW flag when requesting the line.
	 */
	if (gpiod_line[ADAPTER_GPIO_IDX_SRST]) {
		retval1 = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SRST], srst);
		if (retval1 < 0)
			LOG_WARNING("set srst value failed");
	}

	if (gpiod_line[ADAPTER_GPIO_IDX_TRST]) {
		retval2 = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TRST], trst);
		if (retval2 < 0)
			LOG_WARNING("set trst value failed");
	}

	return ((retval1 < 0) || (retval2 < 0)) ? ERROR_FAIL : ERROR_OK;
}

static bool linuxgpiod_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TCK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TMS))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDI))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDO))
		return false;
	return true;
}

static bool linuxgpiod_swd_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWCLK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO))
		return false;
	return true;
}

static inline void helper_release(enum adapter_gpio_config_index idx)
{
	if (gpiod_line[idx]) {
		gpiod_line_release(gpiod_line[idx]);
		gpiod_line[idx] = NULL;
	}
	if (gpiod_chip[idx]) {
		gpiod_chip_close(gpiod_chip[idx]);
		gpiod_chip[idx] = NULL;
	}
}

static int linuxgpiod_quit(void)
{
	LOG_DEBUG("linuxgpiod_quit");
	for (int i = 0; i < ADAPTER_GPIO_IDX_NUM; ++i)
		helper_release(i);

	return ERROR_OK;
}

#ifndef HAVE_LIBGPIOD_V2
static int helper_get_line(enum adapter_gpio_config_index idx)
{
	if (!is_gpio_config_valid(idx))
		return ERROR_OK;

	int dir = GPIOD_LINE_REQUEST_DIRECTION_INPUT, flags = 0, val = 0, retval;

	gpiod_chip[idx] = gpiod_chip_open_by_number(adapter_gpio_config[idx].chip_num);
	if (!gpiod_chip[idx]) {
		LOG_ERROR("Cannot open LinuxGPIOD chip %d for %s", adapter_gpio_config[idx].chip_num,
			adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	gpiod_line[idx] = gpiod_chip_get_line(gpiod_chip[idx], adapter_gpio_config[idx].gpio_num);
	if (!gpiod_line[idx]) {
		LOG_ERROR("Error get line %s", adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	switch (adapter_gpio_config[idx].init_state) {
	case ADAPTER_GPIO_INIT_STATE_INPUT:
		dir = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
		break;
	case ADAPTER_GPIO_INIT_STATE_INACTIVE:
		dir = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
		val = 0;
		break;
	case ADAPTER_GPIO_INIT_STATE_ACTIVE:
		dir = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
		val = 1;
		break;
	}

	switch (adapter_gpio_config[idx].drive) {
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN;
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE;
		break;
	}

	switch (adapter_gpio_config[idx].pull) {
	case ADAPTER_GPIO_PULL_NONE:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
#endif
		break;
	case ADAPTER_GPIO_PULL_UP:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
#else
		LOG_WARNING("linuxgpiod: ignoring request for pull-up on %s: not supported by gpiod v%s",
			adapter_gpio_get_name(idx), gpiod_version_string());
#endif
		break;
	case ADAPTER_GPIO_PULL_DOWN:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
#else
		LOG_WARNING("linuxgpiod: ignoring request for pull-down on %s: not supported by gpiod v%s",
			adapter_gpio_get_name(idx), gpiod_version_string());
#endif
		break;
	}

	if (adapter_gpio_config[idx].active_low)
		flags |= GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;

	struct gpiod_line_request_config config = {
		.consumer = "OpenOCD",
		.request_type = dir,
		.flags = flags,
	};

	retval = gpiod_line_request(gpiod_line[idx], &config, val);
	if (retval < 0) {
		LOG_ERROR("Error requesting gpio line %s", adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}
#else
static int helper_get_line(enum adapter_gpio_config_index idx)
{
	struct gpiod_line_settings *line_settings = NULL;
	struct gpiod_line_config *line_config = NULL;
	struct gpiod_request_config *req_cfg = NULL;
	int retval = ERROR_JTAG_INIT_FAILED;

	if (!is_gpio_config_valid(idx))
		return ERROR_OK;

	gpiod_chip[idx] = gpiod_chip_open_by_number(adapter_gpio_config[idx].chip_num);
	if (!gpiod_chip[idx]) {
		LOG_ERROR("Cannot open LinuxGPIOD chip %d for %s", adapter_gpio_config[idx].chip_num,
			adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	gpiod_line[idx] = gpiod_line_get(gpiod_chip[idx], adapter_gpio_config[idx].gpio_num);
	line_settings = gpiod_line_settings_new();
	line_config = gpiod_line_config_new();
	req_cfg = gpiod_request_config_new();

	if (!gpiod_line[idx] || !line_settings || !line_config || !req_cfg) {
		LOG_ERROR("Cannot configure LinuxGPIOD line for %s", adapter_gpio_get_name(idx));
		goto err_out;
	}

	gpiod_request_config_set_consumer(req_cfg, "OpenOCD");

	switch (adapter_gpio_config[idx].init_state) {
	case ADAPTER_GPIO_INIT_STATE_INPUT:
		gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_INPUT);
		break;
	case ADAPTER_GPIO_INIT_STATE_INACTIVE:
		gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_OUTPUT);
		gpiod_line_settings_set_output_value(line_settings, GPIOD_LINE_VALUE_INACTIVE);
		break;
	case ADAPTER_GPIO_INIT_STATE_ACTIVE:
		gpiod_line_settings_set_direction(line_settings, GPIOD_LINE_DIRECTION_OUTPUT);
		gpiod_line_settings_set_output_value(line_settings, GPIOD_LINE_VALUE_ACTIVE);
		break;
	}

	if (adapter_gpio_config[idx].init_state != ADAPTER_GPIO_INIT_STATE_INPUT) {
		switch (adapter_gpio_config[idx].drive) {
		case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
			gpiod_line_settings_set_drive(line_settings, GPIOD_LINE_DRIVE_PUSH_PULL);
			break;
		case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
			gpiod_line_settings_set_drive(line_settings, GPIOD_LINE_DRIVE_OPEN_DRAIN);
			break;
		case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
			gpiod_line_settings_set_drive(line_settings, GPIOD_LINE_DRIVE_OPEN_SOURCE);
			break;
		}
	}

	switch (adapter_gpio_config[idx].pull) {
	case ADAPTER_GPIO_PULL_NONE:
		gpiod_line_settings_set_bias(line_settings, GPIOD_LINE_BIAS_DISABLED);
		break;
	case ADAPTER_GPIO_PULL_UP:
		gpiod_line_settings_set_bias(line_settings, GPIOD_LINE_BIAS_PULL_UP);
		break;
	case ADAPTER_GPIO_PULL_DOWN:
		gpiod_line_settings_set_bias(line_settings, GPIOD_LINE_BIAS_PULL_DOWN);
		break;
	}

	gpiod_line_settings_set_active_low(line_settings, adapter_gpio_config[idx].active_low);

	retval = gpiod_line_config_add_line_settings(line_config,
			&adapter_gpio_config[idx].gpio_num, 1, line_settings);
	if (retval < 0) {
		LOG_ERROR("Error configuring gpio line %s", adapter_gpio_get_name(idx));
		retval = ERROR_JTAG_INIT_FAILED;
		goto err_out;
	}

	gpiod_line[idx]->line_request = gpiod_chip_request_lines(gpiod_chip[idx], req_cfg, line_config);
	if (!gpiod_line[idx]->line_request) {
		LOG_ERROR("Error requesting gpio line %s", adapter_gpio_get_name(idx));
		retval = ERROR_JTAG_INIT_FAILED;
		goto err_out;
	}

	retval = ERROR_OK;
	goto out;

err_out:
	free(gpiod_line[idx]);
	gpiod_line[idx] = NULL;
out:
	gpiod_line_settings_free(line_settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(req_cfg);
	return retval;
}
#endif

static int linuxgpiod_init(void)
{
	LOG_INFO("Linux GPIOD JTAG/SWD bitbang driver");

	bitbang_interface = &linuxgpiod_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	/*
	 * Configure JTAG/SWD signals. Default directions and initial states are handled
	 * by adapter.c and "adapter gpio" command.
	 */

	if (transport_is_jtag()) {
		if (!linuxgpiod_jtag_mode_possible()) {
			LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
			goto out_error;
		}

		if (helper_get_line(ADAPTER_GPIO_IDX_TDO) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TDI) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TCK) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TMS) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TRST) != ERROR_OK)
				goto out_error;
	}

	if (transport_is_swd()) {
		int retval1, retval2;
		if (!linuxgpiod_swd_mode_possible()) {
			LOG_ERROR("Require swclk and swdio gpio for SWD mode");
			goto out_error;
		}

		/*
		 * swdio and its buffer should be initialized in the order that prevents
		 * two outputs from being connected together. This will occur if the
		 * swdio GPIO is configured as an output while the external buffer is
		 * configured to send the swdio signal from the target to the GPIO.
		 */
		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].init_state == ADAPTER_GPIO_INIT_STATE_INPUT) {
			retval1 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO);
			retval2 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO_DIR);
		} else {
			retval1 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO_DIR);
			retval2 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO);
		}
		if (retval1 != ERROR_OK || retval2 != ERROR_OK)
			goto out_error;

		if (helper_get_line(ADAPTER_GPIO_IDX_SWCLK) != ERROR_OK)
			goto out_error;
	}

	if (helper_get_line(ADAPTER_GPIO_IDX_SRST) != ERROR_OK ||
		helper_get_line(ADAPTER_GPIO_IDX_LED) != ERROR_OK)
			goto out_error;

	return ERROR_OK;

out_error:
	linuxgpiod_quit();

	return ERROR_JTAG_INIT_FAILED;
}

static const char *const linuxgpiod_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface linuxgpiod_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver linuxgpiod_adapter_driver = {
	.name = "linuxgpiod",
	.transports = linuxgpiod_transport,

	.init = linuxgpiod_init,
	.quit = linuxgpiod_quit,
	.reset = linuxgpiod_reset,

	.jtag_ops = &linuxgpiod_interface,
	.swd_ops = &bitbang_swd,
};
