#include "neck.h"
#include "pinout.h"
#include "state.h"
#include "sound.h"
#include "motor.h"
#include "driver/pulse_cnt.h"

#define NECK_MAX_SPEED 245
#define NECK_ACC 1100

// Motor PWM settings
#define MOTOR_PWM_FREQ 30000
#define MOTOR_PWM_CHANNEL 4
#define MOTOR_PWM_RESOLUTION 8

#define ENC_TICK_PER_NECK_ROTATION (19687 / 2)
#define ENC_TICK_LOW_LIMIT -ENC_TICK_PER_NECK_ROTATION / 2
#define ENC_TICK_HIGH_LIMIT ENC_TICK_PER_NECK_ROTATION / 2
// Setting PWM properties
pcnt_unit_handle_t pcnt_unit = NULL;

static bool available = false;

void IRAM_ATTR hallSensorRising() {
	hallSensor = true;
}

void setupNeckZeroSensor() {
	pinMode(PIN_NECK_ZERO_SENSOR, INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_NECK_ZERO_SENSOR), hallSensorRising, CHANGE);
}

void setupNeckMotorDrive() {
	// sets the pins as outputs:
	pinMode(PIN_MOTOR1_IN1, OUTPUT);
	pinMode(PIN_MOTOR1_IN2, OUTPUT);
	pinMode(PIN_MOTOR1_PWM, OUTPUT);
	// configure LEDC PWM
	ledcAttachChannel(PIN_MOTOR1_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION, MOTOR_PWM_CHANNEL);
}


void updateNeckPositionFromXY(int32_t x, int32_t y) {
	float mag = sqrt((float)(x * x + y * y));
	if(mag > 300) {
		neckPositionMaxSpeed = map(mag, 0, 720, 50, NECK_MAX_SPEED);
		neckPositionTarget = atan2(x, -y) * 180 / PI;
		//Serial.printf("Angle: %.2f speed %d\n", neckPositionTarget, neckPositionMaxSpeed);
	}
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
	BaseType_t high_task_wakeup;
	QueueHandle_t queue = (QueueHandle_t)user_ctx;

	// Increment overflow counter on overflow
	if (edata->watch_point_value == ENC_TICK_HIGH_LIMIT) {
		neckPositionOverflow += ENC_TICK_HIGH_LIMIT;
	}
	if (edata->watch_point_value == ENC_TICK_LOW_LIMIT) {
		neckPositionOverflow += ENC_TICK_LOW_LIMIT;
	}
//	if (edata->watch_point_value == 0) {
//		if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_NEG_POS) {
//			neckPositionOverflow += 1;
//		} else if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_POS_NEG) {
//			neckPositionOverflow -= 1;
//		} if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_NEG_ZERO) {
//			neckPositionOverflow += 1;
//		} if (edata->zero_cross_mode == PCNT_UNIT_ZERO_CROSS_POS_ZERO) {
//			//neckPositionOverflow -= 1;
//		}
//	}

	// send event data to queue, from this interrupt callback
	xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
	return (high_task_wakeup == pdTRUE);
}
QueueHandle_t queue = xQueueCreate(10, sizeof(int));

void updateNeckSpeed() {
	if (neckSpeedTarget != neckSpeedCurrent) {
		int accelerationStep = (NECK_ACC * deltatime_ms) / 1000; // Scale acceleration by delta time

		// Determine the direction of acceleration
		int speedDiff = neckSpeedTarget - neckSpeedCurrent;
		neckSpeedCurrent += min(abs(speedDiff), accelerationStep) * (speedDiff > 0 ? 1 : -1);
	}

	// Clamp neck speed within the allowed range
	neckSpeedCurrent = max(-NECK_MAX_SPEED, min(NECK_MAX_SPEED, neckSpeedCurrent));

	// Only send update if speed has changed
	if (neckSpeedCurrent != lastSentNeckSpeed) {
		//Serial.printf("Neck speed target: %d current %d\n", neckSpeedTarget, neckSpeedCurrent);
		setMotor(neckSpeedCurrent, PIN_MOTOR1_PWM, PIN_MOTOR1_IN1, PIN_MOTOR1_IN2, 5);
		lastSentNeckSpeed = neckSpeedCurrent;
	}
}

void clearNeckPosition() {
	neckZeroFound = true;
	//Serial.printf("clear pcnt unit\n");
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	neckPositionOverflow = 0;
	neckPositionTarget = 0;
	neckPositionCurrent = 0;
}

void zeroFound() {
	clearNeckPosition();
	neckSpeedTarget = 0;
	updateNeckSpeed();
	playNextSound();
}

float shortestAngleDifference(float angle1, float angle2) {
	float delta = fmod((angle2 - angle1 + 180), 360) - 180;
	return (delta < -180) ? delta + 360 : delta;
}

void updateNeckPosition() {
	int e = shortestAngleDifference(neckPositionTarget, neckPositionCurrent);
	int dir = e >= 0 ? 1 : -1;
	int delta = abs(e);
	if(delta <= 1) {
		neckSpeedTarget = 0;
	} else {
		neckSpeedTarget = dir * map(delta, 0, 180, 25, NECK_MAX_SPEED);
	}
	//if(printSerialLimited) Serial.printf("dir %d, delta %d, neckSpeedTarget %d\n", dir, delta, neckSpeedTarget);
}

void updateNeckPositionPID(float currentPos) {

	// PID constants
	float kp = 2;
	float kd = 0.025; //0.025;
	float ki = 0.0;

	// time difference
	long currT = micros();
	float deltaT = ((float) (currT - prevT))/( 1.0e6f );
	prevT = currT;

	// error
	int e = shortestAngleDifference(neckPositionTarget, currentPos);

	// derivative
	float dedt = (e-eprev)/(deltaT);

	// integral
	eintegral = eintegral + e*deltaT;

	// control signal
	float u = kp*e + kd*dedt + ki*eintegral;

	// motor power
	float pwr = fabs(u);
	if( pwr > neckPositionMaxSpeed ){
		pwr = neckPositionMaxSpeed;
	}
	if(pwr < 30) {
		pwr = 70;
	}
	Serial.printf("Pwr %.2f\n", pwr);

	// motor direction
	int dir = 1;
	if(u<0){
		dir = -1;
	}

	neckSpeedTarget = pwr * dir;

	// store previous error
	eprev = e;
}

void setupNeckEncoder() {
	pcnt_unit_config_t unit_config = {
			.low_limit = ENC_TICK_LOW_LIMIT,
			.high_limit = ENC_TICK_HIGH_LIMIT,
	};
	unit_config.flags.accum_count = 0;
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

	pcnt_glitch_filter_config_t filter_config = {
			.max_glitch_ns = 1000,
	};
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

	pcnt_chan_config_t chan_a_config = {
			.edge_gpio_num = PIN_ENCODER_A,
			.level_gpio_num = PIN_ENCODER_B,
	};
	pcnt_channel_handle_t pcnt_chan_a = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
	pcnt_chan_config_t chan_b_config = {
			.edge_gpio_num = PIN_ENCODER_B,
			.level_gpio_num = PIN_ENCODER_A,
	};
	pcnt_channel_handle_t pcnt_chan_b = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.high_limit));
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.low_limit));
	//ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 0));
	pcnt_event_callbacks_t cbs = {
			.on_reach = example_pcnt_on_reach,
	};
	ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void setupNeck() {
	setupNeckZeroSensor();
	setupNeckMotorDrive();
	setupNeckEncoder();

	neckSpeedTarget = 100;
	updateNeckSpeed();
	available = true;
}

void loopNeck() {
	if(!available) return;
	updateNeckSpeed();

	if(!neckZeroFound) {
		if(hallSensor) {
			hallSensor = false;
			detachInterrupt(digitalPinToInterrupt(PIN_NECK_ZERO_SENSOR));
			zeroFound();
		}
	}
	int pulse_count = 0;
	ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
	auto tot = neckPositionOverflow + pulse_count;
	auto rot = tot / ENC_TICK_PER_NECK_ROTATION;
	neckPositionCurrent = 360.0f * (float) (tot % ENC_TICK_PER_NECK_ROTATION) / (float) (ENC_TICK_PER_NECK_ROTATION);
	//if(printSerialLimited) Serial.printf("Pulse count: ticks:%d overflowed:%ld total:%ld rot: %ld deg: %.2f (target %.2f)\n", pulse_count, neckPositionOverflow, neckPositionOverflow + pulse_count, rot, deg, neckPositionTarget);
	updateNeckPosition();
}
