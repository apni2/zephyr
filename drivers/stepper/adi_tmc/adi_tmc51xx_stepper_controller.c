/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Prevas A/S
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc51xx

#include <stdlib.h>

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

#include "adi_tmc_spi.h"
#include "adi_tmc5xxx_common.h"
#include "adi_tmc5xxx_stepper_controller.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc51xx, CONFIG_STEPPER_LOG_LEVEL);

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL

static void tmc51xx_rampstat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);

	struct tmc5xxx_stepper_data *stepper_data =
		CONTAINER_OF(dwork, struct tmc5xxx_stepper_data, rampstat_callback_dwork);

	if (tmc5xxx_rampstat_work_handler(work) > 0) {
		k_work_reschedule(
			&stepper_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
	}
}

static void reschedule_rampstat_dwork(struct tmc5xxx_stepper_data *stepper_data)
{
	if (stepper_data->callback) {
		k_work_reschedule(
			&stepper_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
	}
}

#endif

static int tmc51xx_stepper_move_by(const struct device *dev, const int32_t micro_steps)
{
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	err = tmc5xxx_stepper_move_by(dev, micro_steps);
	if (err != 0) {
		return err;
	}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL
	reschedule_rampstat_dwork(data);
#endif
	return 0;
}

static int tmc51xx_stepper_move_to(const struct device *dev, const int32_t micro_steps)
{
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	err = tmc5xxx_stepper_move_to(dev, micro_steps);
	if (err != 0) {
		return err;
	}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL
	reschedule_rampstat_dwork(data);
#endif
	return 0;
}

static int tmc51xx_stepper_run(const struct device *dev, const enum stepper_direction direction)
{
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	err = tmc5xxx_stepper_run(dev, direction);
	if (err != 0) {
		return err;
	}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL
	reschedule_rampstat_dwork(data);
#endif
	return 0;
}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN

int tmc51xx_stepper_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	int err;

	err = tmc5xxx_stepper_set_ramp(dev, &config->default_ramp_config);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC51XX_THIGH, ramp_data->thigh);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC51XX_TCOOLTHRS, ramp_data->tcoolthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC51XX_TPWMTHRS, ramp_data->tpwmthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC51XX_TPOWER_DOWN, ramp_data->tpowerdown);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC51XX_IHOLD_IRUN, ramp_data->iholdrun);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

#endif

static int tmc51xx_init(const struct device *dev)
{
	LOG_DBG("TMC51XX stepper motor controller %s initialized", dev->name);
	int err;

	err = tmc5xxx_init(dev);
	if (err != 0) {
		return -EIO;
	}

	/* Read and write GSTAT register to clear any SPI Datagram errors. */
	uint32_t gstat_value;

	err = tmc5xxx_read(dev, TMC5XXX_GSTAT, &gstat_value);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5xxx_write(dev, TMC5XXX_GSTAT, gstat_value);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Device %s initialized", dev->name);
	return 0;
}

static int tmc51xx_stepper_init(const struct device *dev)
{
	const struct tmc5xxx_stepper_config *stepper_config = dev->config;
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	err = tmc5xxx_stepper_init(dev);
	if (err != 0) {
		return -EIO;
	}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN
	err = tmc51xx_stepper_set_ramp(dev, &stepper_config->default_ramp_config);
	if (err != 0) {
		return -EIO;
	}
#endif

#if CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL
	k_work_init_delayable(&data->rampstat_callback_dwork, tmc51xx_rampstat_work_handler);
	k_work_reschedule(&data->rampstat_callback_dwork,
			  K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
#endif
	return 0;
}

static DEVICE_API(stepper, tmc51xx_stepper_api) = {
	.enable = tmc5xxx_stepper_enable,
	.disable = tmc5xxx_stepper_disable,
	.is_moving = tmc5xxx_stepper_is_moving,
	.move_by = tmc51xx_stepper_move_by,
	.set_micro_step_res = tmc5xxx_stepper_set_micro_step_res,
	.get_micro_step_res = tmc5xxx_stepper_get_micro_step_res,
	.set_reference_position = tmc5xxx_stepper_set_reference_position,
	.get_actual_position = tmc5xxx_stepper_get_actual_position,
	.move_to = tmc51xx_stepper_move_to,
	.run = tmc51xx_stepper_run,
	.set_event_callback = tmc5xxx_stepper_set_event_callback,
};

#define TMC51XX_SHAFT_CONFIG(child)								\
	(DT_PROP(child, invert_direction) << TMC51XX_GCONF_SHAFT_SHIFT)

#define TMC51XX_STEPPER_CONFIG_DEFINE(child)							\
	COND_CODE_1(DT_PROP_EXISTS(child, stallguard_threshold_velocity),			\
	BUILD_ASSERT(DT_PROP(child, stallguard_threshold_velocity),				\
		     "stallguard threshold velocity must be a positive value"), ());		\
	IF_ENABLED(CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN, (CHECK_RAMP_DT_DATA(child)));		\
	static const struct tmc5xxx_stepper_config tmc5xxx_stepper_config_##child = {		\
		.controller = DEVICE_DT_GET(DT_PARENT(child)),					\
		.default_micro_step_res = DT_PROP(child, micro_step_res),			\
		.index = DT_REG_ADDR(child),							\
		.sg_threshold = DT_PROP(child, stallguard2_threshold),				\
		.sg_threshold_velocity = DT_PROP(child, stallguard_threshold_velocity),		\
		.sg_velocity_check_interval_ms = DT_PROP(child,					\
						stallguard_velocity_check_interval_ms),		\
		.is_sg_enabled = DT_PROP(child, activate_stallguard2),				\
		IF_ENABLED(CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN,					\
		(.default_ramp_config = TMC_RAMP_DT_SPEC_GET_TMC51XX(child))) };

#define TMC51XX_STEPPER_DATA_DEFINE(child)							\
	static struct tmc5xxx_stepper_data tmc5xxx_stepper_data_##child = {			\
		.stepper = DEVICE_DT_GET(child),};

#define TMC51XX_STEPPER_DEFINE(child)								\
	DEVICE_DT_DEFINE(child, tmc51xx_stepper_init, NULL, &tmc5xxx_stepper_data_##child,	\
			 &tmc5xxx_stepper_config_##child, POST_KERNEL,				\
			 CONFIG_STEPPER_INIT_PRIORITY, &tmc51xx_stepper_api);

#define TMC51XX_DEFINE(inst)									\
	BUILD_ASSERT(DT_INST_CHILD_NUM(inst) <= 1, "tmc51xx can drive one stepper motor");	\
	BUILD_ASSERT((DT_INST_PROP(inst, clock_frequency) > 0),					\
		     "clock frequency must be non-zero positive value");			\
	static struct tmc5xxx_data tmc5xxx_data_##inst;						\
	static const struct tmc5xxx_config tmc5xxx_config_##inst = {				\
		.gconf = (									\
		(DT_INST_PROP(inst, en_pwm_mode) << TMC51XX_GCONF_EN_PWM_MODE_SHIFT) |		\
		(DT_INST_PROP(inst, test_mode) << TMC51XX_GCONF_TEST_MODE_SHIFT) |		\
		DT_INST_FOREACH_CHILD(inst, TMC51XX_SHAFT_CONFIG)),				\
		.spi = SPI_DT_SPEC_INST_GET(inst, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |	\
					SPI_MODE_CPOL | SPI_MODE_CPHA |	SPI_WORD_SET(8)), 0),	\
		.clock_frequency = DT_INST_PROP(inst, clock_frequency)};			\
	DT_INST_FOREACH_CHILD(inst, TMC51XX_STEPPER_CONFIG_DEFINE);				\
	DT_INST_FOREACH_CHILD(inst, TMC51XX_STEPPER_DATA_DEFINE);				\
	DT_INST_FOREACH_CHILD(inst, TMC51XX_STEPPER_DEFINE);					\
	DEVICE_DT_INST_DEFINE(inst, tmc51xx_init, NULL, &tmc5xxx_data_##inst,			\
			      &tmc5xxx_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC51XX_DEFINE)
