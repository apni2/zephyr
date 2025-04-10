/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc50xx

#include <stdlib.h>

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

#include "adi_tmc_spi.h"
#include "adi_tmc5xxx_common.h"
#include "adi_tmc5xxx_stepper_controller.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc50xx, CONFIG_STEPPER_LOG_LEVEL);

static DEVICE_API(stepper, tmc50xx_stepper_api) = {
	.enable = tmc50xx_stepper_enable,
	.disable = tmc50xx_stepper_disable,
	.is_moving = tmc50xx_stepper_is_moving,
	.move_by = tmc50xx_stepper_move_by,
	.set_micro_step_res = tmc50xx_stepper_set_micro_step_res,
	.get_micro_step_res = tmc50xx_stepper_get_micro_step_res,
	.set_reference_position = tmc50xx_stepper_set_reference_position,
	.get_actual_position = tmc50xx_stepper_get_actual_position,
	.move_to = tmc50xx_stepper_move_to,
	.run = tmc50xx_stepper_run,
	.set_event_callback = tmc50xx_stepper_set_event_callback,
};

#define TMC50XX_SHAFT_CONFIG(child)								\
	(DT_PROP(child, invert_direction) << TMC50XX_GCONF_SHAFT_SHIFT(DT_REG_ADDR(child))) |

#define TMC50XX_STEPPER_CONFIG_DEFINE(child)							\
	COND_CODE_1(DT_PROP_EXISTS(child, stallguard_threshold_velocity),			\
	BUILD_ASSERT(DT_PROP(child, stallguard_threshold_velocity),				\
		     "stallguard threshold velocity must be a positive value"), ());		\
	IF_ENABLED(CONFIG_STEPPER_ADI_TMC50XX_RAMP_GEN, (CHECK_RAMP_DT_DATA(child)));		\
	static const struct tmc50xx_stepper_config tmc50xx_stepper_config_##child = {		\
		.controller = DEVICE_DT_GET(DT_PARENT(child)),					\
		.default_micro_step_res = DT_PROP(child, micro_step_res),			\
		.index = DT_REG_ADDR(child),							\
		.sg_threshold = DT_PROP(child, stallguard2_threshold),				\
		.sg_threshold_velocity = DT_PROP(child, stallguard_threshold_velocity),		\
		.sg_velocity_check_interval_ms = DT_PROP(child,					\
						stallguard_velocity_check_interval_ms),		\
		.is_sg_enabled = DT_PROP(child, activate_stallguard2),				\
		IF_ENABLED(CONFIG_STEPPER_ADI_TMC50XX_RAMP_GEN,					\
		(.default_ramp_config = TMC_RAMP_DT_SPEC_GET_TMC50XX(child))) };

#define TMC50XX_STEPPER_DATA_DEFINE(child)							\
	static struct tmc50xx_stepper_data tmc50xx_stepper_data_##child = {			\
		.stepper = DEVICE_DT_GET(child),};

#define TMC50XX_STEPPER_DEFINE(child)								\
	DEVICE_DT_DEFINE(child, tmc50xx_stepper_init, NULL, &tmc50xx_stepper_data_##child,	\
			 &tmc50xx_stepper_config_##child, POST_KERNEL,				\
			 CONFIG_STEPPER_INIT_PRIORITY, &tmc50xx_stepper_api);

#define TMC50XX_DEFINE(inst)									\
	BUILD_ASSERT(DT_INST_CHILD_NUM(inst) <= 2, "tmc50xx can drive two steppers at max");	\
	BUILD_ASSERT((DT_INST_PROP(inst, clock_frequency) > 0),					\
		     "clock frequency must be non-zero positive value");			\
	static struct tmc50xx_data tmc50xx_data_##inst;						\
	static const struct tmc50xx_config tmc50xx_config_##inst = {				\
		.gconf = (									\
		(DT_INST_PROP(inst, poscmp_enable) << TMC50XX_GCONF_POSCMP_ENABLE_SHIFT) |	\
		(DT_INST_PROP(inst, test_mode) << TMC50XX_GCONF_TEST_MODE_SHIFT) |		\
		DT_INST_FOREACH_CHILD(inst, TMC50XX_SHAFT_CONFIG)				\
		(DT_INST_PROP(inst, lock_gconf) << TMC50XX_LOCK_GCONF_SHIFT)),			\
		.spi = SPI_DT_SPEC_INST_GET(inst, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |	\
					SPI_MODE_CPOL | SPI_MODE_CPHA |	SPI_WORD_SET(8)), 0),	\
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),};			\
	DT_INST_FOREACH_CHILD(inst, TMC50XX_STEPPER_CONFIG_DEFINE);				\
	DT_INST_FOREACH_CHILD(inst, TMC50XX_STEPPER_DATA_DEFINE);				\
	DT_INST_FOREACH_CHILD(inst, TMC50XX_STEPPER_DEFINE);					\
	DEVICE_DT_INST_DEFINE(inst, tmc50xx_init, NULL, &tmc50xx_data_##inst,			\
			      &tmc50xx_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC50XX_DEFINE)
