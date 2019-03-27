#include <set>

#include "nmranet_config.h"
#include "os/os.h"

#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ServoConsumerConfig.hxx"

#include "config.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "hardware.hxx"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3xx.h"

#define SNIFF_ON_SERIAL
//#define SNIFF_ON_USB
//#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 1300);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
// #define STM32_UUID_LOC 0x1FFFF7AC;
extern const openlcb::NodeID NODE_ID;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);
// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char * const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE = cfg.seg().size()
		+ cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 600, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char * const openlcb::SNIP_DYNAMIC_FILENAME =
		openlcb::CONFIG_FILENAME;

// TODO: Attach CDI config schema to hardware things.

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(stack.node(), {/*producer_sw1.polling()*/});

// each of events_to_activate_crossing modeled as its own biteventconsumer.
// if any event is consumed, produce the crossing_post_activated_event event.

// need a biteventinterface to do the hardware-side stuff,
// like kick off servo rotation. 
// high-level gate actuation modeled as a BitEventPC
// (activation events above produce the event; hardware consumes the event.)

class CrossingManager: private DefaultConfigUpdateListener,
		private openlcb::SimpleEventHandler {
public:
	static const bool CROSSING_ACTIVE = false;

	CrossingManager(const crossing_controller::CrossingControllerConfig& config,
			openlcb::Node* node) :
			config_(config), node_(node) {

	}

	virtual UpdateAction apply_configuration(int fd, bool initial_load,
			BarrierNotifiable* done) override {
		//fd_ = fd;

		bool replace_all_events = false;
		unsigned num_events_in_new_config = 0;
		for (unsigned i = 0;
				i < config_.events_to_activate_crossing().num_repeats(); i++) {
			const openlcb::EventConfigEntry& event_slot =
					config_.events_to_activate_crossing().entry(i);
			const openlcb::EventId& event = event_slot.read(fd);
			if (!is_activation_event(event)) {
				replace_all_events = true;
			}
			++num_events_in_new_config;
		}
		if (num_events_in_new_config != events_to_activate_crossing_.size()) {
			replace_all_events = true;
		} else if (config_.active_event().read(fd) != crossing_active_event_) {
			replace_all_events = true;
		} else if (config_.dormant_event().read(fd)
				!= crossing_dormant_event_) {
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
					i < config_.events_to_activate_crossing().num_repeats();
					i++) {
				const openlcb::EventConfigEntry& event_slot =
						config_.events_to_activate_crossing().entry(i);
				const openlcb::EventId& event = event_slot.read(fd);
				events_to_activate_crossing_.emplace(event);
				openlcb::EventRegistry::instance()->register_handler(
						EventRegistryEntry(this, event), /*mask=*/0);
			}

			crossing_active_event_ = config_.active_event().read(fd);
			crossing_dormant_event_ = config_.dormant_event().read(fd);
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

		cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
		cfg.userinfo().description().write(fd,
				"The best darn Grade Crossing controller money can't buy!");

		CDI_FACTORY_RESET(config_.crossbuck_brightness);
		CDI_FACTORY_RESET(config_.crossbuck_flash_rate);
		CDI_FACTORY_RESET(config_.crossbuck_style);
		CDI_FACTORY_RESET(config_.false_alarm_timeout_seconds);
		CDI_FACTORY_RESET(config_.light_sensitivity);

		for (unsigned i = 0; i < config_.servo_consumers().num_repeats(); i++) {
			const openlcb::ServoConsumerConfig& servo =
					config_.servo_consumers().entry(i);
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
					CROSSING_ACTIVE ?
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID :
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_INVALID;
			envelope->event_write_helper<2>()->WriteAsync(node_, mti,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(entry.event), done->new_child());
		} else if (entry.event == crossing_dormant_event_) {
			openlcb::Defs::MTI mti =
					CROSSING_ACTIVE ?
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_INVALID :
							openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID;
			envelope->event_write_helper<3>()->WriteAsync(node_, mti,
					openlcb::WriteHelper::global(),
					openlcb::eventid_to_buffer(entry.event), done->new_child());
		}
	}

	void handle_event_report(const EventRegistryEntry& entry,
			EventReport* event, BarrierNotifiable* done) override {
		AutoNotify an(done);
	}

private:

	inline bool addressed_to_someone_else(const EventReport* envelope) const {
		return envelope->dst_node && envelope->dst_node != node_;
	}

	inline bool is_activation_event(const openlcb::EventId& event) {
		return events_to_activate_crossing_.find(event)
				!= events_to_activate_crossing_.end();
	}

	// Store the events here because we want to know if a
	// config update is changing them or not.
	std::set<openlcb::EventId> events_to_activate_crossing_;
	openlcb::EventId crossing_active_event_ { 0 };
	openlcb::EventId crossing_dormant_event_ { 0 };

	crossing_controller::CrossingControllerConfig config_; // this is just the schema.
	//int fd_ { -1 };  // config file descriptor.
	openlcb::Node* node_;

} config_manager_instance { cfg.seg().crossing_controller_config(), stack.node() };

//class GateOnOffInterface : public openlcb::BitEventInterface {
//public:
//    void set_state(bool new_value) override {
//        bool old_value = get_local_state();
//        openlcb::BitEventInterface::set_state(new_value);
//        if (old_value == new_value) { return; }
//
//        if (new_value) {
//            // Arm the gates
//            // seems like arming and disarming need a state machine:
//            // - start crossbucks flashing
//            // - enable sound
//            // - kick off servo rotation
//        } else {
//            // Disarm the gates
//            // - kick off servo rotation. when it's done,
//            //     - disable sound
//            //     - stop crossbucks flashing
//        }
//    }
//};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[]) {
	stack.check_version_and_factory_reset(cfg.seg().internal_config(),
			openlcb::CANONICAL_VERSION, false);

	// The necessary physical ports must be added to the stack.
	//
	// It is okay to enable multiple physical ports, in which case the stack
	// will behave as a bridge between them. For example enabling both the
	// physical CAN port and the USB port will make this firmware act as an
	// USB-CAN adapter in addition to the producers/consumers created above.
	//
	// If a port is enabled, it must be functional or else the stack will
	// freeze waiting for that port to send the packets out.
#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
	stack.add_gridconnect_port("/dev/ser0");
#endif

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	// This command donates the main thread to the operation of the
	// stack. Alternatively the stack could be started in a separate stack and
	// then application-specific business logic could be executed ion a busy
	// loop in the main thread.
	stack.loop_executor();
	return 0;
}
