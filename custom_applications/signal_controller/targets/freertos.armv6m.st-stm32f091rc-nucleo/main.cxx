/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file main.cxx
 *
 * Main file for the io board application on the STM32 Nucleo board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "config.hxx"
#include "hardware.hxx"
#include "BusActivityBlinky.hxx"
#include "AnimatedLamp.hxx"
#include "SignalEventHandler.hxx"

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
//#define SNIFF_ON_SERIAL
//#define SNIFF_ON_USB
#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 1300);

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
extern const char *const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 2048, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

class UpdatableEventHandler : public DefaultConfigUpdateListener,
						      public openlcb::SimpleEventHandler {
public:
	UpdatableEventHandler(openlcb::Node* node)
			: node_(node) {}

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

protected:
	inline bool addressed_to_someone_else(const EventReport* envelope) const {
		return envelope->dst_node && envelope->dst_node != node_;
	}

	openlcb::Node* node_;
};

class SimplePinProducer : public DefaultConfigUpdateListener {
public:
	SimplePinProducer(openlcb::Node* node,
					  const signal_controller::InputPinConfig config,
					  const Gpio* gpio)
			: node_(node), config_(config), pin_(gpio),
			  producer_(QuiesceDebouncer::Options(3), node, 0, 0, gpio) {
	}

	virtual UpdateAction apply_configuration(int fd, bool initial_load,
			BarrierNotifiable* done) override {
		AutoNotify an(done);

		openlcb::EventId event_hi, event_lo;
		int pin_on_value = config_.pin_on_value().read(fd);

		if (pin_on_value == 0) {
			event_lo = config_.event_on().read(fd);
			event_hi = config_.event_off().read(fd);
		} else {
			event_lo = config_.event_off().read(fd);
			event_hi = config_.event_on().read(fd);
		}

		producer_.~PolledProducer();
		new (&producer_) openlcb::PolledProducer<QuiesceDebouncer, openlcb::GPIOBit>(
			QuiesceDebouncer::Options(3), node_, event_hi, event_lo, pin_);
		return REINIT_NEEDED;
	}

	virtual void factory_reset(int fd) override {
        config_.name().write(fd, "");
		CDI_FACTORY_RESET(config_.pin_on_value);
	}

	openlcb::Polling* polling() {
		return &producer_;
	}

private:
	openlcb::Node* node_;
    const signal_controller::InputPinConfig config_;
    const Gpio* pin_;
    openlcb::PolledProducer<QuiesceDebouncer, openlcb::GPIOBit> producer_;
};

SimplePinProducer producer_plug1(
    stack.node(), cfg.seg().plug_inputs().entry<0>(), Plug_Input1_Pin::instance());
SimplePinProducer producer_plug2(
    stack.node(), cfg.seg().plug_inputs().entry<1>(), Plug_Input2_Pin::instance());
SimplePinProducer producer_plug3(
    stack.node(), cfg.seg().plug_inputs().entry<2>(), Plug_Input3_Pin::instance());
SimplePinProducer producer_plug4(
    stack.node(), cfg.seg().plug_inputs().entry<3>(), Plug_Input4_Pin::instance());
SimplePinProducer producer_plug5(
    stack.node(), cfg.seg().plug_inputs().entry<4>(), Plug_Input5_Pin::instance());
SimplePinProducer producer_plug6(
    stack.node(), cfg.seg().plug_inputs().entry<5>(), Plug_Input6_Pin::instance());
SimplePinProducer producer_plug7(
    stack.node(), cfg.seg().plug_inputs().entry<6>(), Plug_Input7_Pin::instance());
SimplePinProducer producer_plug8(
    stack.node(), cfg.seg().plug_inputs().entry<7>(), Plug_Input8_Pin::instance());

SignalEventHandler signal_event_handler(cfg, stack.node());

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    BusActivityBlinky activity_blinky(&stack);

    CalculateLedDataFn = [&](volatile uint16_t* arr) {
    	signal_event_handler.CopyValuesToArray(arr);
    };

    // The producers need to be polled repeatedly for changes and to execute the
    // debouncing algorithm. This class instantiates a refreshloop and adds the two
    // producers to it.
    openlcb::RefreshLoop loop(stack.node(),
    		{&activity_blinky,
    		 producer_plug1.polling(), producer_plug2.polling(),
			 producer_plug3.polling(), producer_plug4.polling(),
			 producer_plug5.polling(), producer_plug6.polling(),
			 producer_plug7.polling(), producer_plug8.polling()});
    
    stack.check_version_and_factory_reset(
        cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, false);

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

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
