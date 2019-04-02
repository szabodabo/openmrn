#ifndef CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERMANAGER_HXX_
#define CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERMANAGER_HXX_

#include <set>

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ServoConsumerConfig.hxx"
#include "config.hxx"

namespace crossing_controller {

class CrossingManager: private DefaultConfigUpdateListener,
		private openlcb::SimpleEventHandler {
public:
	CrossingManager(const openlcb::ConfigDef config, openlcb::Node* node,
			const Gpio* snd_enable, bool* crossing_active) :
			config_(config), node_(node), snd_enable_(snd_enable), 
			crossing_active_(crossing_active) {
	}

	virtual UpdateAction apply_configuration(int fd, bool initial_load,
			BarrierNotifiable* done) override {
		//fd_ = fd;

		bool replace_all_events = false;
		unsigned num_events_in_new_config = 0;
		for (unsigned i = 0;
				i < cfg().events_to_activate_crossing().num_repeats(); i++) {
			const openlcb::EventConfigEntry& event_slot =
					cfg().events_to_activate_crossing().entry(i);
			const openlcb::EventId& event = event_slot.read(fd);
			if (!is_activation_event(event)) {
				replace_all_events = true;
			}
			++num_events_in_new_config;
		}
		if (num_events_in_new_config != events_to_activate_crossing_.size()) {
			replace_all_events = true;
		} else if (cfg().active_event().read(fd) != crossing_active_event_) {
			replace_all_events = true;
		} else if (cfg().dormant_event().read(fd) != crossing_dormant_event_) {
			replace_all_events = true;
		}

		// TODO: Set non-event options from config here.

		if (replace_all_events) {
			events_to_activate_crossing_.clear();
			if (!initial_load) {
				// On initial load, we won't have yet registered.
				openlcb::EventRegistry::instance()->unregister_handler(this);
			}

			// Since unregister removes *all* the handlers,
			// we should re-register everything.

			for (unsigned i = 0;
					i < cfg().events_to_activate_crossing().num_repeats();
					i++) {
				const openlcb::EventConfigEntry& event_slot =
						cfg().events_to_activate_crossing().entry(i);
				const openlcb::EventId& event = event_slot.read(fd);
				events_to_activate_crossing_.emplace(event);
				openlcb::EventRegistry::instance()->register_handler(
						EventRegistryEntry(this, event), /*mask=*/0);
			}

			crossing_active_event_ = cfg().active_event().read(fd);
			crossing_dormant_event_ = cfg().dormant_event().read(fd);
			for (const openlcb::EventId& event : { crossing_active_event_,
					crossing_dormant_event_ }) {
				openlcb::EventRegistry::instance()->register_handler(
						EventRegistryEntry(this, event), /*mask=*/0);
			}

			return REINIT_NEEDED;
		}

		return UPDATED;
	}

	virtual void factory_reset(int fd) override {
		//fd_ = fd;

		// OpenMRN handles factory reset of openlcb::EventConfigEntry.
		// events_to_activate_crossing
		// crossing_post_activated_event
		// crossing_post_deactivated_event

		config_.userinfo().name().write(fd,
				openlcb::SNIP_STATIC_DATA.model_name);
		config_.userinfo().description().write(fd,
				"The best darn Grade Crossing controller money can't buy!");

		CDI_FACTORY_RESET(cfg().crossbuck_brightness);
		CDI_FACTORY_RESET(cfg().crossbuck_flash_rate);
		CDI_FACTORY_RESET(cfg().crossbuck_style);
		CDI_FACTORY_RESET(cfg().false_alarm_timeout_seconds);
		CDI_FACTORY_RESET(cfg().light_sensitivity);

		for (unsigned i = 0; i < cfg().servo_consumers().num_repeats(); i++) {
			const openlcb::ServoConsumerConfig& servo =
					cfg().servo_consumers().entry(i);
			SERVOCONSUMERCONFIG_RESET(servo);
		}
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
		// crossing_post_activated_event
		// crossing_post_deactivated_event
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

		if (is_activation_event(entry.event)) {
			envelope->event_write_helper<1>()->WriteAsync(node_,
					openlcb::Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(entry.event), done->new_child());
		} else if (entry.event == crossing_active_event_) {
			openlcb::Defs::MTI mti =
					*crossing_active_ ?
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID :
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_INVALID;
			envelope->event_write_helper<2>()->WriteAsync(node_, mti,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(entry.event), done->new_child());
		} else if (entry.event == crossing_dormant_event_) {
			openlcb::Defs::MTI mti =
					*crossing_active_ ?
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_INVALID :
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID;
			envelope->event_write_helper<3>()->WriteAsync(node_, mti,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(entry.event), done->new_child());
		}
	}

	void handle_event_report(const EventRegistryEntry& entry,
			EventReport* envelope, BarrierNotifiable* done) override {
		AutoNotify an(done);
		if (is_activation_event(entry.event)) {
			*crossing_active_ = true;
			envelope->event_write_helper<1>()->WriteAsync(node_,
					openlcb::Defs::MTI_EVENT_REPORT,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(crossing_active_event_),
					done->new_child());
			snd_enable_->set();

			// - kick off servo rotation
		} else if (entry.event == crossing_dormant_event_) {
			// TODO: This should be the other way around
			// (event published after crossing is done).
			// Do it this way for debug during development.

			// TODO: Need a higher-level workflow for disabling:
			//       Lights should only go off when gates are fully up.
			*crossing_active_ = false;
			snd_enable_->clr();
		}
	}

private:

	inline bool addressed_to_someone_else(const EventReport* envelope) const {
		return envelope->dst_node && envelope->dst_node != node_;
	}

	inline bool is_activation_event(const openlcb::EventId& event) {
		return events_to_activate_crossing_.find(event)
				!= events_to_activate_crossing_.end();
	}

	inline const crossing_controller::CrossingControllerConfig cfg() const {
		return config_.seg().crossing_controller_config();
	}

	// Store the events here because we want to know if a
	// config update is changing them or not.
	std::set<openlcb::EventId> events_to_activate_crossing_;
	openlcb::EventId crossing_active_event_ { 0 };
	openlcb::EventId crossing_dormant_event_ { 0 };

	const openlcb::ConfigDef config_; // this is just the schema.
	//int fd_ { -1 };  // config file descriptor.
	openlcb::Node* node_;
	const Gpio* snd_enable_;
	bool* crossing_active_;
};

}  // namespace crossing_controller

#endif /* CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERMANAGER_HXX_ */
