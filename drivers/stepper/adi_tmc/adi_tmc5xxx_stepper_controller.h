/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Prevas A/S
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_STEPPER_CONTROLLER_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_STEPPER_CONTROLLER_H_

#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

struct tmc5xxx_data {
	struct k_sem sem;
};

struct tmc5xxx_config {
	const uint32_t gconf;
	struct spi_dt_spec spi;
	const uint32_t clock_frequency;
};

struct tmc5xxx_stepper_data {
	struct k_work_delayable stallguard_dwork;
	/* Work item to run the callback in a thread context. */
#if defined CONFIG_STEPPER_ADI_TMC50XX_RAMPSTAT_POLL || \
	defined CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL
	struct k_work_delayable rampstat_callback_dwork;
#endif
	/* device pointer required to access config in k_work */
	const struct device *stepper;
	stepper_event_callback_t callback;
	void *event_cb_user_data;
};

struct tmc5xxx_stepper_config {
	const uint8_t index;
	const uint16_t default_micro_step_res;
	const int8_t sg_threshold;
	const bool is_sg_enabled;
	const uint32_t sg_velocity_check_interval_ms;
	const uint32_t sg_threshold_velocity;
	/* parent controller required for bus communication */
	const struct device *controller;
#if defined CONFIG_STEPPER_ADI_TMC50XX_RAMP_GEN || defined CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN
	const struct tmc_ramp_generator_data default_ramp_config;
#endif
};

int tmc5xxx_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val);
int tmc5xxx_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val);
int tmc5xxx_rampstat_work_handler(struct k_work *work);
int tmc5xxx_stepper_set_event_callback(const struct device *dev,
					stepper_event_callback_t callback, void *user_data);
int stallguard_enable(const struct device *dev, const bool enable);
void stallguard_work_handler(struct k_work *work);
int tmc5xxx_stepper_enable(const struct device *dev);
int tmc5xxx_stepper_disable(const struct device *dev);
int tmc5xxx_stepper_is_moving(const struct device *dev, bool *is_moving);
int tmc5xxx_stepper_move_by(const struct device *dev, const int32_t micro_steps);
int tmc5xxx_stepper_set_micro_step_res(const struct device *dev,
					enum stepper_micro_step_resolution res);
int tmc5xxx_stepper_get_micro_step_res(const struct device *dev,
					enum stepper_micro_step_resolution *res);
int tmc5xxx_stepper_set_reference_position(const struct device *dev, const int32_t position);
int tmc5xxx_stepper_get_actual_position(const struct device *dev, int32_t *position);
int tmc5xxx_stepper_move_to(const struct device *dev, const int32_t micro_steps);
int tmc5xxx_stepper_run(const struct device *dev, const enum stepper_direction direction);
int tmc5xxx_stepper_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data);
int tmc5xxx_init(const struct device *dev);
int tmc5xxx_stepper_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_STEPPER_CONTROLLER_H_ */
