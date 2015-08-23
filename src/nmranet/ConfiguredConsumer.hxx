/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file ConfiguredConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a GPIO template structure to
 * export a single bit as two event consumers to OpenLCB.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _NMRANET_CONFIGUREDCONSUMER_HXX_
#define _NMRANET_CONFIGUREDCONSUMER_HXX_

#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "nmranet/RefreshLoop.hxx"

namespace nmranet
{

CDI_GROUP(ConsumerConfig);
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Event On"),
    Description("Receiving this event ID will turn the output on."));
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("Receiving this event ID will turn the output off."));
CDI_GROUP_END();


CDI_GROUP(PulseConsumerConfig);
CDI_GROUP_ENTRY(event, EventConfigEntry, //
    Name("Event"),
    Description("Receiving this event ID will generate a pulso on the output."));
CDI_GROUP_ENTRY(duration, Uint8ConfigEntry, //
    Name("Pulse duration"),
    Description("Length of the pulse to output (unit of 30 msec)."));
CDI_GROUP_END();


class ConfiguredConsumer : public ConfigUpdateListener
{
public:
    using Impl = GPIOBit;

    ConfiguredConsumer(Node *node, const ConsumerConfig &cfg, Gpio *gpio)
        : impl_(node, 0, 0, gpio)
        , consumer_(&impl_)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    template <class HW>
    ConfiguredConsumer(Node *node, const ConsumerConfig &cfg, const HW &)
        : impl_(node, 0, 0, HW::instance())
        , consumer_(&impl_)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        if (cfg_event_off != impl_.event_off() ||
            cfg_event_on != impl_.event_on())
        {
            auto saved_gpio = impl_.gpio_;
            auto saved_node = impl_.node();
            // Need to reinitialize the consumer. We do this with in-place
            // destruction and construction.
            consumer_.~BitEventConsumer();
            impl_.~Impl();
            new (&impl_)
                Impl(saved_node, cfg_event_on, cfg_event_off, saved_gpio);
            new (&consumer_) BitEventConsumer(&impl_);
            return REINIT_NEEDED; // Causes events identify.
        }
        return UPDATED;
    }

    ///@TODO(balazs.racz): implement
    void factory_reset(int fd) OVERRIDE
    {
    }

private:
    Impl impl_;
    BitEventConsumer consumer_;
    const ConsumerConfig cfg_;
};

class ConfiguredPulseConsumer : public ConfigUpdateListener, private SimpleEventHandler, public Polling
{
public:
    template <class HW>
    ConfiguredPulseConsumer(Node *node, const PulseConsumerConfig &cfg, const HW &)
        : node_(node), gpio_(HW::instance()), cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ConfiguredPulseConsumer(Node *node, const PulseConsumerConfig &cfg, Gpio* gpio)
        : node_(node), gpio_(gpio), cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredPulseConsumer() {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);
        EventId cfg_event_on = cfg_.event().read(fd);
        pulseLength_ = cfg_.duration().read(fd);
        if (cfg_event_on == event_) return UPDATED; // nothing to do
        if (!initial_load) {
            do_unregister();
        }
        event_ = cfg_event_on;
        do_register();
        return REINIT_NEEDED; // Causes events identify.
    }

    ///@TODO(balazs.racz): implement
    void factory_reset(int fd) OVERRIDE
    {
    }

private:
    /// Registers the event handler with the global event registry.
    void do_register() {
        EventRegistry::instance()->register_handlerr(this, event_, 0);
    }

    /// Registers the event handler with the global event registry.
    void do_unregister() {
        EventRegistry::instance()->unregister_handlerr(this, event_, 0);
    }
    
    // Implementations for the event handler functions.

    void HandleIdentifyGlobal(EventReport* event,
                              BarrierNotifiable* done) OVERRIDE {
        if (event->dst_node && event->dst_node != node_) {
            return done->notify();
        }
        SendConsumerIdentified(done);
    }

    void SendConsumerIdentified(BarrierNotifiable* done) OVERRIDE {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        if (!pulseRemaining_) {
            mti++; // INVALID
        }
        event_write_helper3.WriteAsync(node_, mti, WriteHelper::global(),
                                       eventid_to_buffer(event_),
                                       done);
    }
    
    void HandleIdentifyConsumer(EventReport* event,
                                BarrierNotifiable* done) OVERRIDE {
        if (event->event != event_) {
            return done->notify();
        }
        SendConsumerIdentified(done);
    }

    void HandleEventReport(EventReport* event, BarrierNotifiable* done) OVERRIDE {
        if (event->event == event_) {
            pulseRemaining_ = pulseLength_;
        }
        done->notify();
    }

    // Polling interface
    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE {
        if (pulseRemaining_ > 0) {
            gpio_->set();
            --pulseRemaining_;
        } else {
            gpio_->clr();
        }
        done->notify();
    }

    Node* node_; //< virtual node to export the consumer on
    Gpio* gpio_; //< hardware output pin to drive
    EventId event_{0}; //< Event ID to listen for
    const PulseConsumerConfig cfg_; //< offset to the config in EEPROM
    uint8_t pulseLength_{1}; //< length of pulse (in polling count)
    uint8_t pulseRemaining_{0}; //< remaining polling count to keep pulse on
};

} // namespace nmranet

#endif // _NMRANET_CONFIGUREDCONSUMER_HXX_
