/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

#include "adi_tmc_spi.h"
#include "adi_tmc5xxx_common.h"
#include "adi_tmc5xxx_stepper_controller.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc5xxx, CONFIG_STEPPER_LOG_LEVEL);

int tmc5xxx_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val)
{
	const struct tmc5xxx_config *config = dev->config;
	struct tmc5xxx_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = tmc_spi_write_register(&bus, TMC5XXX_WRITE_BIT, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err) {
		LOG_ERR("Failed to write register 0x%x with value 0x%x", reg_addr, reg_val);
		return err;
	}
	return 0;
}

int tmc5xxx_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5xxx_config *config = dev->config;
	struct tmc5xxx_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = tmc_spi_read_register(&bus, TMC5XXX_ADDRESS_MASK, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err) {
		LOG_ERR("Failed to read register 0x%x", reg_addr);
		return err;
	}
	return 0;
}

int tmc5xxx_stepper_set_event_callback(const struct device *dev,
					stepper_event_callback_t callback, void *user_data)
{
	struct tmc5xxx_stepper_data *data = dev->data;

	data->callback = callback;
	data->event_cb_user_data = user_data;
	return 0;
}

int stallguard_enable(const struct device *dev, const bool enable)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_SWMODE(config->index), &reg_value);
	if (err) {
		LOG_ERR("Failed to read SWMODE register");
		return -EIO;
	}

	if (enable) {
		reg_value |= TMC5XXX_SW_MODE_SG_STOP_ENABLE;

		int32_t actual_velocity;

		err = tmc5xxx_read(config->controller, TMC5XXX_VACTUAL(config->index),
				   &actual_velocity);
		if (err) {
			LOG_ERR("Failed to read VACTUAL register");
			return -EIO;
		}

		actual_velocity = (actual_velocity << (31 - TMC_RAMP_VACTUAL_SHIFT)) >>
				  (31 - TMC_RAMP_VACTUAL_SHIFT);
		if (actual_velocity != 0) {
			LOG_DBG("actual velocity: %d", actual_velocity);
		}
		if (abs(actual_velocity) < config->sg_threshold_velocity) {
			return -EAGAIN;
		}
	} else {
		reg_value &= ~TMC5XXX_SW_MODE_SG_STOP_ENABLE;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_SWMODE(config->index), reg_value);
	if (err) {
		LOG_ERR("Failed to write SWMODE register");
		return -EIO;
	}
	return 0;
}

void stallguard_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc5xxx_stepper_data *stepper_data =
		CONTAINER_OF(dwork, struct tmc5xxx_stepper_data, stallguard_dwork);
	int err;
	const struct tmc5xxx_stepper_config *stepper_config = stepper_data->stepper->config;

	err = stallguard_enable(stepper_data->stepper, true);
	if (err == -EAGAIN) {
		k_work_reschedule(dwork, K_MSEC(stepper_config->sg_velocity_check_interval_ms));
	}
	if (err == -EIO) {
		LOG_ERR("Failed to enable stallguard because of I/O error");
		return;
	}
}

#if defined CONFIG_STEPPER_ADI_TMC50XX_RAMPSTAT_POLL || \
	defined CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL

static void execute_callback(const struct device *dev, const enum stepper_event event)
{
	struct tmc5xxx_stepper_data *data = dev->data;

	if (!data->callback) {
		LOG_WRN_ONCE("No callback registered");
		return;
	}
	data->callback(dev, event, data->event_cb_user_data);
}

int tmc5xxx_rampstat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);

	struct tmc5xxx_stepper_data *stepper_data =
		CONTAINER_OF(dwork, struct tmc5xxx_stepper_data, rampstat_callback_dwork);
	const struct tmc5xxx_stepper_config *stepper_config = stepper_data->stepper->config;

	__ASSERT_NO_MSG(stepper_config->controller != NULL);

	uint32_t drv_status;
	int err;

	err = tmc5xxx_read(stepper_config->controller, TMC5XXX_DRVSTATUS(stepper_config->index),
			   &drv_status);
	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", stepper_data->stepper->name);
		return -EIO;
	}

	if (FIELD_GET(TMC5XXX_DRV_STATUS_SG_STATUS_MASK, drv_status) == 1U) {
		LOG_INF("%s: Stall detected", stepper_data->stepper->name);
		err = tmc5xxx_write(stepper_config->controller,
				    TMC5XXX_RAMPMODE(stepper_config->index),
				    TMC5XXX_RAMPMODE_HOLD_MODE);
		if (err != 0) {
			LOG_ERR("%s: Failed to stop motor", stepper_data->stepper->name);
			return -EIO;
		}
	}

	uint32_t rampstat_value;

	err = tmc5xxx_read(stepper_config->controller, TMC5XXX_RAMPSTAT(stepper_config->index),
			   &rampstat_value);
	if (err != 0) {
		LOG_ERR("%s: Failed to read RAMPSTAT register", stepper_data->stepper->name);
		return -EIO;
	}

	const uint8_t ramp_stat_values = FIELD_GET(TMC5XXX_RAMPSTAT_INT_MASK, rampstat_value);

	if (ramp_stat_values > 0) {
		switch (ramp_stat_values) {

		case TMC5XXX_STOP_LEFT_EVENT:
			LOG_DBG("RAMPSTAT %s:Left end-stop detected", stepper_data->stepper->name);
			execute_callback(stepper_data->stepper,
					 STEPPER_EVENT_LEFT_END_STOP_DETECTED);
			break;

		case TMC5XXX_STOP_RIGHT_EVENT:
			LOG_DBG("RAMPSTAT %s:Right end-stop detected", stepper_data->stepper->name);
			execute_callback(stepper_data->stepper,
					 STEPPER_EVENT_RIGHT_END_STOP_DETECTED);
			break;

		case TMC5XXX_POS_REACHED_EVENT:
			LOG_DBG("RAMPSTAT %s:Position reached", stepper_data->stepper->name);
			execute_callback(stepper_data->stepper, STEPPER_EVENT_STEPS_COMPLETED);
			break;

		case TMC5XXX_STOP_SG_EVENT:
			LOG_DBG("RAMPSTAT %s:Stall detected", stepper_data->stepper->name);
			stallguard_enable(stepper_data->stepper, false);
			execute_callback(stepper_data->stepper, STEPPER_EVENT_STALL_DETECTED);
			break;
		default:
			LOG_ERR("Illegal ramp stat bit field: 0x%02x", ramp_stat_values);
			break;
		}
	}
	return (ramp_stat_values == 0);
}

#endif

int tmc5xxx_stepper_enable(const struct device *dev)
{
	LOG_DBG("Enabling Stepper motor controller %s", dev->name);
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_CHOPCONF(config->index), &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value |= TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc5xxx_write(config->controller, TMC5XXX_CHOPCONF(config->index), reg_value);
}

int tmc5xxx_stepper_disable(const struct device *dev)
{
	LOG_DBG("Disabling Stepper motor controller %s", dev->name);
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_CHOPCONF(config->index), &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc5xxx_write(config->controller, TMC5XXX_CHOPCONF(config->index), reg_value);
}

int tmc5xxx_stepper_is_moving(const struct device *dev, bool *is_moving)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_DRVSTATUS(config->index), &reg_value);

	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return -EIO;
	}

	*is_moving = (FIELD_GET(TMC5XXX_DRV_STATUS_STST_BIT, reg_value) != 1U);
	LOG_DBG("Stepper motor controller %s is moving: %d", dev->name, *is_moving);
	return 0;
}

int tmc5xxx_stepper_move_by(const struct device *dev, const int32_t micro_steps)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	if (config->is_sg_enabled) {
		err = stallguard_enable(dev, false);
		if (err != 0) {
			return -EIO;
		}
	}

	int32_t position;

	err = stepper_get_actual_position(dev, &position);
	if (err != 0) {
		return -EIO;
	}
	int32_t target_position = position + micro_steps;

	err = tmc5xxx_write(config->controller, TMC5XXX_RAMPMODE(config->index),
			    TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Stepper motor controller %s moved to %d by steps: %d", dev->name, target_position,
		micro_steps);
	err = tmc5xxx_write(config->controller, TMC5XXX_XTARGET(config->index), target_position);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	return 0;
}

int tmc5xxx_stepper_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	const struct tmc5xxx_config *tmc5xxx_config = config->controller->config;
	const uint32_t clock_frequency = tmc5xxx_config->clock_frequency;
	uint32_t velocity_fclk;
	int err;

	velocity_fclk = tmc5xxx_calculate_velocity_from_hz_to_fclk(velocity, clock_frequency);

	err = tmc5xxx_write(config->controller, TMC5XXX_VMAX(config->index), velocity_fclk);
	if (err != 0) {
		LOG_ERR("%s: Failed to set max velocity", dev->name);
		return -EIO;
	}
	return 0;
}

int tmc5xxx_stepper_set_micro_step_res(const struct device *dev,
					enum stepper_micro_step_resolution res)
{
	if (!VALID_MICRO_STEP_RES(res)) {
		LOG_ERR("Invalid micro step resolution %d", res);
		return -ENOTSUP;
	}

	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_CHOPCONF(config->index), &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value |= ((MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - LOG2(res))
		      << TMC5XXX_CHOPCONF_MRES_SHIFT);

	err = tmc5xxx_write(config->controller, TMC5XXX_CHOPCONF(config->index), reg_value);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Stepper motor controller %s set micro step resolution to 0x%x", dev->name,
		reg_value);
	return 0;
}

int tmc5xxx_stepper_get_micro_step_res(const struct device *dev,
					enum stepper_micro_step_resolution *res)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_CHOPCONF(config->index), &reg_value);
	if (err != 0) {
		return -EIO;
	}
	reg_value &= TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value >>= TMC5XXX_CHOPCONF_MRES_SHIFT;
	*res = (1 << (MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - reg_value));
	LOG_DBG("Stepper motor controller %s get micro step resolution: %d", dev->name, *res);
	return 0;
}

int tmc5xxx_stepper_set_reference_position(const struct device *dev, const int32_t position)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	int err;

	err = tmc5xxx_write(config->controller, TMC5XXX_RAMPMODE(config->index),
			    TMC5XXX_RAMPMODE_HOLD_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5xxx_write(config->controller, TMC5XXX_XACTUAL(config->index), position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Stepper motor controller %s set actual position to %d", dev->name, position);
	return 0;
}

int tmc5xxx_stepper_get_actual_position(const struct device *dev, int32_t *position)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	int err;

	err = tmc5xxx_read(config->controller, TMC5XXX_XACTUAL(config->index), position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("%s actual position: %d", dev->name, *position);
	return 0;
}

int tmc5xxx_stepper_move_to(const struct device *dev, const int32_t micro_steps)
{
	LOG_DBG("Stepper motor controller %s set target position to %d", dev->name, micro_steps);
	const struct tmc5xxx_stepper_config *config = dev->config;
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	if (config->is_sg_enabled) {
		stallguard_enable(dev, false);
	}

	err = tmc5xxx_write(config->controller, TMC5XXX_RAMPMODE(config->index),
			    TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_XTARGET(config->index), micro_steps);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	return 0;
}

int tmc5xxx_stepper_run(const struct device *dev, const enum stepper_direction direction)
{
	LOG_DBG("Stepper motor controller %s run", dev->name);
	const struct tmc5xxx_stepper_config *config = dev->config;
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	if (config->is_sg_enabled) {
		err = stallguard_enable(dev, false);
		if (err != 0) {
			return -EIO;
		}
	}

	switch (direction) {
	case STEPPER_DIRECTION_POSITIVE:
		err = tmc5xxx_write(config->controller, TMC5XXX_RAMPMODE(config->index),
				    TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;

	case STEPPER_DIRECTION_NEGATIVE:
		err = tmc5xxx_write(config->controller, TMC5XXX_RAMPMODE(config->index),
				    TMC5XXX_RAMPMODE_NEGATIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	return 0;
}

#if defined CONFIG_STEPPER_ADI_TMC50XX_RAMP_GEN || defined CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN

int tmc5xxx_stepper_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data)
{
	LOG_DBG("Stepper motor controller %s set ramp", dev->name);
	const struct tmc5xxx_stepper_config *config = dev->config;
	int err;

	err = tmc5xxx_write(config->controller, TMC5XXX_VSTART(config->index), ramp_data->vstart);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_A1(config->index), ramp_data->a1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_AMAX(config->index), ramp_data->amax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_D1(config->index), ramp_data->d1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_DMAX(config->index), ramp_data->dmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_V1(config->index), ramp_data->v1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_VMAX(config->index), ramp_data->vmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_VSTOP(config->index), ramp_data->vstop);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5xxx_write(config->controller, TMC5XXX_TZEROWAIT(config->index),
			    ramp_data->tzerowait);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

#endif

int tmc5xxx_init(const struct device *dev)
{
	struct tmc5xxx_data *data = dev->data;
	const struct tmc5xxx_config *config = dev->config;
	int err;

	k_sem_init(&data->sem, 1, 1);

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	/* Init non motor-index specific registers here. */
	LOG_DBG("GCONF: %d", config->gconf);
	err = tmc5xxx_write(dev, TMC5XXX_GCONF, config->gconf);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

int tmc5xxx_stepper_init(const struct device *dev)
{
	const struct tmc5xxx_stepper_config *stepper_config = dev->config;
	struct tmc5xxx_stepper_data *data = dev->data;
	int err;

	LOG_DBG("Controller: %s, Stepper: %s", stepper_config->controller->name, dev->name);

	if (stepper_config->is_sg_enabled) {
		k_work_init_delayable(&data->stallguard_dwork, stallguard_work_handler);

		err = tmc5xxx_write(stepper_config->controller,
				    TMC5XXX_SWMODE(stepper_config->index), BIT(10));
		if (err != 0) {
			return -EIO;
		}

		LOG_DBG("Setting stall guard to %d with delay %d ms", stepper_config->sg_threshold,
			stepper_config->sg_velocity_check_interval_ms);
		if (!IN_RANGE(stepper_config->sg_threshold, TMC5XXX_SG_MIN_VALUE,
			      TMC5XXX_SG_MAX_VALUE)) {
			LOG_ERR("Stallguard threshold out of range");
			return -EINVAL;
		}

		int32_t stall_guard_threshold = (int32_t)stepper_config->sg_threshold;

		err = tmc5xxx_write(
			stepper_config->controller, TMC5XXX_COOLCONF(stepper_config->index),
			stall_guard_threshold << TMC5XXX_COOLCONF_SG2_THRESHOLD_VALUE_SHIFT);
		if (err != 0) {
			return -EIO;
		}
		k_work_reschedule(&data->stallguard_dwork, K_NO_WAIT);
	}

	err = tmc5xxx_stepper_set_micro_step_res(dev, stepper_config->default_micro_step_res);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}
