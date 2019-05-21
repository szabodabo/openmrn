#ifndef _SIGNAL_CONTROLLER_ANIMATED_LAMP_H
#define _SIGNAL_CONTROLLER_ANIMATED_LAMP_H

#include <math.h>
#include <stdlib.h>

#define LED_JITTER true

class AbstractAnimatedLamp {
public:
	AbstractAnimatedLamp(double max_brightness_ratio, double jitter_ratio)
		: max_brightness_ratio_(max_brightness_ratio),
		  jitter_factor_(jitter_ratio) {}

	enum Workflow {
		SOLID_ON = 1,
		SOLID_OFF,
		FLASHING,
	};

	void Poll_1khz() {
		++current_state_elapsed_ms_;
		Poll_1khz_Internal();
	}

	virtual void Poll_1khz_Internal() = 0;

	uint16_t GetScaledValue(uint16_t scale_max) {
		return value_ * (double) scale_max;
	}

	void SetWorkflow(Workflow new_workflow) {
		workflow_ = new_workflow;
	}

protected:
	void SetState(int new_state, uint32_t new_state_duration_ms) {
		state_ = new_state;
		current_state_elapsed_ms_ = 0;
#if LED_JITTER
		const uint16_t max_jitter_ms = new_state_duration_ms*jitter_factor_;
		const int16_t jitter_ms = (rand() % max_jitter_ms) - max_jitter_ms/2;
		current_state_duration_ms_ = new_state_duration_ms + jitter_ms;
#else
		current_state_duration_ms_ = new_state_duration_ms;
#endif
	}

	int state_ = 0;
	Workflow workflow_ = Workflow::SOLID_ON;
	uint32_t current_state_elapsed_ms_ = 0;
	uint32_t current_state_duration_ms_ = 500;
	double value_ = 0.0;
	double max_brightness_ratio_ = 1.0;
	double jitter_factor_ = 0.0;
};

class AnimatedLedLamp : public AbstractAnimatedLamp {
public:
	AnimatedLedLamp(double max_brightness_ratio, double jitter_ratio)
			: AbstractAnimatedLamp(max_brightness_ratio, jitter_ratio) {}
	virtual void Poll_1khz_Internal() override {
		if (workflow_ == Workflow::SOLID_ON && state_ == STATE_ON) {
			return;
		} else if (workflow_ == Workflow::SOLID_OFF && state_ == STATE_OFF) {
			return;
		}

		// We must be waiting for the current state to expire, then flip.
		if (current_state_elapsed_ms_ > current_state_duration_ms_) {
			SetState(state_ == STATE_ON ? STATE_OFF : STATE_ON, LED_DURATION_MS);
		}
	}

protected:
	void SetState(int state, uint32_t duration) {
		value_ = state == STATE_ON ? max_brightness_ratio_ : 0;
		AbstractAnimatedLamp::SetState(state, duration);
	}
private:
	static const int STATE_OFF = 0;
	static const int STATE_ON = 1;
	static const uint32_t LED_DURATION_MS = 800;
};

class AnimatedIncandescentLamp : public AbstractAnimatedLamp {
public:
	AnimatedIncandescentLamp(double max_brightness_ratio, double jitter_ratio)
		: AbstractAnimatedLamp(max_brightness_ratio, jitter_ratio) {}

	virtual void Poll_1khz_Internal() override {
		if (workflow_ == Workflow::SOLID_ON && state_ == STATE_WAITING_ON) {
			return;
		} else if (workflow_ == Workflow::SOLID_OFF && state_ == STATE_WAITING_OFF) {
			return;
		}

		if (current_state_elapsed_ms_ > current_state_duration_ms_) {
			if (state_ == STATE_WAITING_OFF) {
				SetState(STATE_TURNING_ON, 135);
			} else if (state_ == STATE_TURNING_ON) {
				SetState(STATE_WAITING_ON, 400);
			} else if (state_ == STATE_WAITING_ON) {
				SetState(STATE_TURNING_OFF, 100);
			} else if (state_ == STATE_TURNING_OFF) {
				SetState(STATE_WAITING_OFF, 400);
			} else {
				HASSERT(0);
			}
			return;
		}

		if (state_ == STATE_WAITING_OFF || state_ == STATE_WAITING_ON) {
			return;
		}

		// Use a parabola to make brightness fade appear linear.
		const double coef = (double) 10000 * max_brightness_ratio_ / (double) pow(current_state_duration_ms_, 2);
		const double x = state_ == STATE_TURNING_ON ? current_state_elapsed_ms_ : (current_state_duration_ms_-current_state_elapsed_ms_);
		value_ = (coef * pow(x, 2)) / (double) 10000;
	}

private:
	static const int STATE_WAITING_OFF = 0;
	static const int STATE_TURNING_ON = 1;
	static const int STATE_WAITING_ON = 2;
	static const int STATE_TURNING_OFF = 3;
};

#endif // _SIGNAL_CONTROLLER_ANIMATED_LAMP_H
