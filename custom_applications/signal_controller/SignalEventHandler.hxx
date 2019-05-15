#ifndef _SIGNAL_CONTROLLER_SIGNAL_EVENT_HANDLER_H
#define _SIGNAL_CONTROLLER_SIGNAL_EVENT_HANDLER_H

#include "AnimatedLamp.hxx"
#include "config.hxx"

struct SignalLampContainer {
	openlcb::EventId event_on_solid, event_on_flashing;
	std::unique_ptr<AbstractLamp> lamp;
};

class SignalHeadContainer {
public:
	SignalLampContainer red, yellow, green;
	openlcb::EventId event_dark;

	void TurnAllOff() {
		red.lamp->SetFlashing(false);
		red.lamp->SetValue(0);
		yellow.lamp->SetFlashing(false);
		yellow.lamp->SetValue(0);
		green.lamp->SetFlashing(false);
		green.lamp->SetValue(0);
	}
};

#define NUM_SIGNAL_HEADS 8
#define BRIGHTNESS_MAX 4000

inline uint16_t ScaleBrightnessPct(uint8_t pct) {
	HASSERT(1000 < BRIGHTNESS_MAX);
	return pct * (BRIGHTNESS_MAX/100);
}


class SignalEventHandler : DefaultConfigUpdateListener,
		private openlcb::SimpleEventHandler {
public:
	SignalEventHandler(const openlcb::ConfigDef config,
			openlcb::Node* node) : config_(config), node_(node) {
		for (int i = 0; i < NUM_SIGNAL_HEADS; i++) {
			heads_[i].red.lamp.reset(new IncandescentLamp());
			heads_[i].red.lamp->SetMaxBrightness(300);
			heads_[i].yellow.lamp.reset(new IncandescentLamp());
			heads_[i].yellow.lamp->SetMaxBrightness(1200);
			heads_[i].green.lamp.reset(new IncandescentLamp());
			heads_[i].green.lamp->SetMaxBrightness(500);
		}
	}

	virtual UpdateAction apply_configuration(int fd, bool initial_load,
				BarrierNotifiable* done) override {
		AutoNotify an(done);

		if (!initial_load) {
			openlcb::EventRegistry::instance()->unregister_handler(this);
		}

		signal_controller::SignalConfigWrapper signal_wrapper = config_.seg().signal_config();
		HASSERT(signal_wrapper.heads().num_repeats() == NUM_SIGNAL_HEADS);

		for (unsigned i = 0; i < NUM_SIGNAL_HEADS; i++) {
			// Outputs on chip vs config entries are reversed:
			//    SH1 in config is SH8 on chip.
			const signal_controller::SignalHeadConfig head = signal_wrapper.heads().entry(NUM_SIGNAL_HEADS-i-1);

			// TODO: signal aspects are forgotten on apply_config.
			if (head.appearance().lamp_style().read(fd) == 1) {
				heads_[i].red.lamp.reset(new IncandescentLamp());
				heads_[i].yellow.lamp.reset(new IncandescentLamp());
				heads_[i].green.lamp.reset(new IncandescentLamp());
			} else {
				heads_[i].red.lamp.reset(new LedLamp());
				heads_[i].yellow.lamp.reset(new LedLamp());
				heads_[i].green.lamp.reset(new LedLamp());
			}

			heads_[i].red.event_on_solid = head.events().red_event().read(fd);
			heads_[i].red.event_on_flashing = head.events().flash_red_event().read(fd);
			heads_[i].red.lamp->SetMaxBrightness(
					ScaleBrightnessPct(head.appearance().red_brightness().read(fd)));

			heads_[i].yellow.event_on_solid = head.events().yellow_event().read(fd);
			heads_[i].yellow.event_on_flashing = head.events().flash_yellow_event().read(fd);
			heads_[i].yellow.lamp->SetMaxBrightness(
					ScaleBrightnessPct(head.appearance().yellow_brightness().read(fd)));

			heads_[i].green.event_on_solid = head.events().green_event().read(fd);
			heads_[i].green.event_on_flashing = head.events().flash_green_event().read(fd);
			heads_[i].green.lamp->SetMaxBrightness(
					ScaleBrightnessPct(head.appearance().green_brightness().read(fd)));

			if (initial_load) {
				// TODO: initial aspect should be special somehow.
				heads_[i].red.lamp->SetFlashing(true);
				heads_[i].red.lamp->SetValue(heads_[i].red.lamp->GetMaxBrightness());
			}

			heads_[i].event_dark = head.events().dark_event().read(fd);

			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].red.event_on_solid), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].red.event_on_flashing), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].yellow.event_on_solid), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].yellow.event_on_flashing), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].green.event_on_solid), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].green.event_on_flashing), /*mask=*/0);
			openlcb::EventRegistry::instance()->register_handler(
					EventRegistryEntry(this, heads_[i].event_dark), /*mask=*/0);
		}

		return REINIT_NEEDED;
	}

	/**
	 * If this Node consumes the given event, give an update to the bus
	 * about its current status (VALID=active, INVALID=inactive, UNKNOWN).
	 * If this Node does not consume this event, don't respond.
	 */
	void handle_identify_consumer(const EventRegistryEntry& entry,
			EventReport* envelope, BarrierNotifiable* done) override {
		return handle_identify_global(entry, envelope, done);
	}

	/**
	 * If this Node produces the given event, give an update to the bus
	 * about its current status (VALID=active, INVALID=inactive, UNKNOWN).
	 * If this Node does not consume this event, don't respond.
	 */
	void handle_identify_producer(const EventRegistryEntry& entry,
			EventReport* envelope, BarrierNotifiable* done) override {
		return handle_identify_global(entry, envelope, done);
	}

	/*
	 * Called on startup or when an IdentifyGlobal message arrives.
	 * superclass comment says EventReport* is NULL, but sample code disagrees.
	 * It appears EventReport's event is null, but not entry.event!
	 */
	void handle_identify_global(const EventRegistryEntry& entry,
			EventReport* envelope, BarrierNotifiable* done) override {
		AutoNotify an(done);
		if (addressed_to_someone_else(envelope)) {
			return;
		}

		const openlcb::EventId& event = entry.event;
		HASSERT(event > 0);

		std::set<openlcb::EventId> aspect_events;

		for (int i = 0; i < NUM_SIGNAL_HEADS; i++) {
			aspect_events.insert(heads_[i].event_dark);
			for (SignalLampContainer* l : {&heads_[i].red, &heads_[i].yellow, &heads_[i].green}) {
				aspect_events.insert(l->event_on_flashing);
				aspect_events.insert(l->event_on_solid);
			}
		}

		if (aspect_events.count(event) > 0) {
			envelope->event_write_helper<1>()->WriteAsync(node_,
				// TODO: Respond with actual aspect state.
				openlcb::Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN,
				openlcb::WriteHelper::global(),
				openlcb::eventid_to_buffer(event), done->new_child());
		}
	}

	void handle_event_report(const EventRegistryEntry& entry,
		EventReport* envelope, BarrierNotifiable* done) override {
		AutoNotify an(done);

		for (int i = 0; i < NUM_SIGNAL_HEADS; i++) {
			SignalHeadContainer& head = heads_[i];
			if (envelope->event == head.event_dark) {
				head.TurnAllOff();
				return;
			}
			for (SignalLampContainer* l : {&heads_[i].red, &heads_[i].yellow, &heads_[i].green}) {
				if (envelope->event == l->event_on_solid
				 || envelope->event == l->event_on_flashing) {
					head.TurnAllOff();
					l->lamp->SetFlashing(envelope->event == l->event_on_flashing);
					l->lamp->SetValue(l->lamp->GetMaxBrightness());
					return;
				}
			}
		}
	}

	virtual void factory_reset(int fd) override {
		config_.userinfo().name().write(fd,
			openlcb::SNIP_STATIC_DATA.model_name);
		config_.userinfo().description().write(fd, "SAFETY FIRST");

		for (int i = 0; i < NUM_SIGNAL_HEADS; i++) {
			signal_controller::SignalHeadConfig head = config_.seg().signal_config().heads().entry(i);
			CDI_FACTORY_RESET(head.appearance().red_brightness);
			CDI_FACTORY_RESET(head.appearance().yellow_brightness);
			CDI_FACTORY_RESET(head.appearance().green_brightness);
			CDI_FACTORY_RESET(head.appearance().lamp_style);
			head.name().write(fd, "");
		}
	}

	void CopyValuesToArray(volatile uint16_t* array) {
		for (int headIdx = 0; headIdx < NUM_SIGNAL_HEADS; headIdx++) {
			SignalHeadContainer& head = heads_[headIdx];
			const int greenIdx = headIdx*3;
			array[greenIdx] = head.green.lamp->GetValue_1ms();
			array[greenIdx+1] = head.yellow.lamp->GetValue_1ms();
			array[greenIdx+2] = head.red.lamp->GetValue_1ms();
		}
	}

private:
	inline bool addressed_to_someone_else(const EventReport* envelope) const {
		return envelope->dst_node && envelope->dst_node != node_;
	}

	const openlcb::ConfigDef config_;
	openlcb::Node* node_;
	SignalHeadContainer heads_[8];
};

#endif // _SIGNAL_CONTROLLER_SIGNAL_EVENT_HANDLER_H
