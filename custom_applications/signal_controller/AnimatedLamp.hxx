#ifndef _SIGNAL_CONTROLLER_ANIMATED_LAMP_H
#define _SIGNAL_CONTROLLER_ANIMATED_LAMP_H

#include <math.h>
#include <stdlib.h>

class AbstractLamp {
public:
	// TODO: Looks like this fn only computes flash animations.
	// Steady on/dark should be implemented better.
	virtual uint16_t GetValue_1ms() = 0;
	void SetMaxBrightness(uint16_t b) {
		max_brightness_ = b;
	}

	uint16_t GetMaxBrightness() {
		return max_brightness_;
	}

	void SetValue(uint16_t val) {
		value_ = val;
	}

	void SetFlashing(bool f) {
		is_flashing_ = f;
	}

	bool IsFlashing() { return is_flashing_; }
protected:
	uint16_t max_brightness_ = 0;
	bool is_flashing_ = false;
	uint16_t value_ = 0;
};

#define LED_DURATION_MS 900

class LedLamp : public AbstractLamp {
public:
	uint16_t GetValue_1ms() override {
		if (!is_flashing_) { return value_; }

		const uint16_t max_jitter_ms = LED_DURATION_MS/2.5;
		const int16_t jitter_ms = (rand() % max_jitter_ms) - max_jitter_ms/2;
		const uint16_t duration_ms = LED_DURATION_MS + jitter_ms;

		elapsed_millis_in_state_++;

		if (value_ == 0 && elapsed_millis_in_state_ > duration_ms) {
			value_ = max_brightness_;
			elapsed_millis_in_state_ = 0;
		} else if (value_ > 0 && elapsed_millis_in_state_ > duration_ms) {
			value_ = 0;
			elapsed_millis_in_state_ = 0;
		}
		return value_;
	}
private:
	uint32_t elapsed_millis_in_state_ = 0;
};

class IncandescentLamp : public AbstractLamp {
public:
	uint16_t GetValue_1ms() override {
		uint16_t state_duration_ms_ = 0;
		State next = TURNING_ON;
		if (state_ == TURNING_ON) {
			state_duration_ms_ = 100;
			next = WAITING_ON;
		} else if (state_ == WAITING_ON) {
			if (!is_flashing_) {
				return value_;
			}
			state_duration_ms_ = 400;
			next = TURNING_OFF;
		} else if (state_ == TURNING_OFF) {
			state_duration_ms_ = 70;
			next = WAITING_OFF;
		} else if (state_ == WAITING_OFF) {
			if (!is_flashing_) {
				return value_;
			}
			state_duration_ms_ = 400;
			next = TURNING_ON;
		}

		if (++elapsed_millis_in_state_ > state_duration_ms_) {
			elapsed_millis_in_state_ = 0;
			state_ = next;
		}

		if (state_ == WAITING_ON || state_ == WAITING_OFF) {
			return value_;
		}

		// Use a parabola to make brightness fade appear linear.
		const double coef = (double) max_brightness_ / pow(state_duration_ms_, 2);
		const double x = (state_ == TURNING_ON ? elapsed_millis_in_state_ : state_duration_ms_-elapsed_millis_in_state_);

		value_ = coef * pow(x, 2);
		return value_;
	}
private:
	enum State {
		TURNING_ON = 1,
		WAITING_ON,
		TURNING_OFF,
		WAITING_OFF,
	};

	State state_ = WAITING_OFF;
	uint32_t elapsed_millis_in_state_ = 0;
};

#endif // _SIGNAL_CONTROLLER_ANIMATED_LAMP_H
