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
#include "openlcb/ServoConsumer.hxx"
#include "openlcb/ServoConsumerConfig.hxx"
#include "utils/Hub.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "config.hxx"
#include "hardware.hxx"

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

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011816ULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

class BusActivityBlinky : public openlcb::Polling, public HubPort {
public:
    BusActivityBlinky(openlcb::SimpleCanStack* stack) : HubPort(stack->service()) {
        stack->gridconnect_hub()->register_port(this);
    }

    Action entry() override {
        activity_request_ = true;
        return release_and_exit();
    }

    virtual void poll_33hz(openlcb::WriteHelper* helper, Notifiable* done) override {
        if (activity_request_ && !activity_on_) {
            activity_request_ = false;
            LED_GREEN_RAW_Pin::set(1);
            activity_on_ = true;
        } else if (activity_on_) {
            activity_on_ = false;
            LED_GREEN_RAW_Pin::set(0);
        }
        done->notify();
    }

    void request_blink() {
        activity_request_ = true;
    }

private:
    bool activity_request_ = false;
    bool activity_on_ = false;
};

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
static_assert(openlcb::CONFIG_FILE_SIZE <= 300, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

class ServoAndRelayConsumer : public openlcb::ServoConsumer, public openlcb::SimpleEventHandler {
public:
    ServoAndRelayConsumer(openlcb::Node *node, const openlcb::ServoConsumerConfig &cfg,
        const uint32_t pwmCountPerMs, PWM *pwm, uint8_t* relay_set, uint8_t* relay_reset)
        : openlcb::ServoConsumer(node, cfg, pwmCountPerMs, pwm), cfg_(cfg), relay_set_bit_(relay_set), relay_reset_bit_(relay_reset) {
    }

    UpdateAction apply_configuration(int fd, bool initial_load, BarrierNotifiable* done) OVERRIDE {
        AutoNotify an(done);

    	// Spy on the on/off event to set the relay.
    	// This class is implemented as a wrapper because the relay doesn't need
    	// its own config items. If it did, we'd need a new config descriptor.
    	// Also, because the LCC-compliant consumer chatter is implemented in ServoConsumer,
    	// we can register a very dumb EventHandler here to be the spy.
    	if (!initial_load) {
    		openlcb::EventRegistry::instance()->unregister_handler(this);
    	}

    	const openlcb::EventId srv_min = cfg_.event_rotate_min().read(fd);
    	const openlcb::EventId srv_max = cfg_.event_rotate_max().read(fd);

    	openlcb::EventRegistry::instance()->register_handler(
    	            openlcb::EventRegistryEntry(this, srv_min, /*user_arg=*/EVENT_SERVO_MIN), /*mask=*/0);
    	openlcb::EventRegistry::instance()->register_handler(
    	    	            openlcb::EventRegistryEntry(this, srv_max, /*user_arg=*/EVENT_SERVO_MAX), /*mask=*/0);

    	return openlcb::ServoConsumer::apply_configuration(fd, initial_load, done->new_child());
    }

    // TODO: confirm if this works in the startup case
    void handle_event_report(const EventRegistryEntry &registry_entry,
                             EventReport *event,
                             BarrierNotifiable *done) OVERRIDE {
    	AutoNotify an(done);
    	if (registry_entry.user_arg == EVENT_SERVO_MIN) {
    		*relay_set_bit_ = 0;
    		*relay_reset_bit_ = 1;
    		UpdateRelays();
    	} else if (registry_entry.user_arg == EVENT_SERVO_MAX) {
    		*relay_set_bit_ = 1;
    		*relay_reset_bit_ = 0;
    		UpdateRelays();
    	}
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry, EventReport *event, BarrierNotifiable *done)
            OVERRIDE {
    	return done->notify();
    }

private:
    const openlcb::ServoConsumerConfig cfg_;
    uint8_t *relay_set_bit_, *relay_reset_bit_;

    enum EventMapping {
    	EVENT_SERVO_MIN = 1,
		EVENT_SERVO_MAX = 2,
    };
};

extern uint8_t RELAY_DATA[];
uint8_t RELAY_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};

ServoAndRelayConsumer srv0(
    stack.node(), cfg.seg().servo_consumers().entry<0>(),
    servoPwmCountPerMs, servo_channels[0], &RELAY_DATA[0], &RELAY_DATA[1]);
ServoAndRelayConsumer srv1(
    stack.node(), cfg.seg().servo_consumers().entry<1>(),
    servoPwmCountPerMs, servo_channels[1], &RELAY_DATA[2], &RELAY_DATA[3]);
ServoAndRelayConsumer srv2(
    stack.node(), cfg.seg().servo_consumers().entry<2>(),
    servoPwmCountPerMs, servo_channels[2], &RELAY_DATA[4], &RELAY_DATA[5]);
ServoAndRelayConsumer srv3(
    stack.node(), cfg.seg().servo_consumers().entry<3>(),
    servoPwmCountPerMs, servo_channels[3], &RELAY_DATA[6], &RELAY_DATA[7]);


// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The ConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the configuration configuration from the CDI definition, and
// the hardware pin definition. The virtual node pointer comes from the stack
// object. The configuration structure comes from the CDI definition object,
// segment 'seg', in which there is a repeated group 'consumers', and we assign
// the individual entries to the individual consumers. Each consumer gets its
// own GPIO pin.
openlcb::ConfiguredConsumer consumer_green(
    stack.node(), cfg.seg().consumers().entry<0>(), LED_GREEN_Pin());

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    BusActivityBlinky activity_blinky(&stack);

    // The producers need to be polled repeatedly for changes and to execute the
    // debouncing algorithm. This class instantiates a refreshloop and adds the two
    // producers to it.
    openlcb::RefreshLoop loop(stack.node(), {&activity_blinky});
    
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
