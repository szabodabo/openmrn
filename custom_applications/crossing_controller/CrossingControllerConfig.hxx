#ifndef CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERCONFIG_HXX_
#define CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERCONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

namespace crossing_controller {

CDI_GROUP(EventWrapper);
CDI_GROUP_ENTRY(
    event, openlcb::EventConfigEntry, Name("Event ID that activates the crossing."));
CDI_GROUP_END(); // EventWrapper

using ActivationEvents = openlcb::RepeatedGroup<EventWrapper, 16>;
using ServoConsumers = openlcb::RepeatedGroup<openlcb::ServoConsumerConfig, 4>;

CDI_GROUP(CrossingControllerConfig, Name("Crossing Controller Options"));

CDI_GROUP_ENTRY(events_to_activate_crossing, ActivationEvents,
    Name("Activation Events"),
    Description("Events that, when seen, should activate the crossing."), RepName("Event"));
CDI_GROUP_ENTRY(active_event, openlcb::EventConfigEntry,
    Name("Post-Activated Event"),
    Description("Event to publish after crossing becomes active."));
CDI_GROUP_ENTRY(dormant_event, openlcb::EventConfigEntry,
    Name("Post-Deactivated Event"),
    Description("Event to publish after crossing becomes inactive."));
CDI_GROUP_ENTRY(servo_consumers, ServoConsumers, Name("Servo outputs"),
    Description("Servo outputs for crossing gate arms."), RepName("Servo"));
CDI_GROUP_ENTRY(crossbuck_brightness, openlcb::Uint8ConfigEntry, Default(100), Min(0),
    Max(100), Name("Crossbuck lamp brightness"),
    Description("Crossbuck lamp brightness, 0-100."));
CDI_GROUP_ENTRY(crossbuck_flash_rate, openlcb::Uint16ConfigEntry, Default(1100), Min(250),
    Max(2000), Name("Crossbuck lamp flash rate"),
    Description("Crossbuck lamp flash rate in milliseconds, 0-2000."));
CDI_GROUP_ENTRY(crossbuck_style, openlcb::Uint8ConfigEntry, Default(0), Min(0), Max(1),
    Name("Crossbuck lamp style"),
    Description("0 for LED, 1 for incandescent."));
CDI_GROUP_ENTRY(false_alarm_timeout_seconds, openlcb::Uint16ConfigEntry, Default(60),
    Min(5), Max(500), Name("Crossing gate timeout (seconds)"),
    Description("How long to wait before the controller loses faith that a "
                "train is actually approaching."));
CDI_GROUP_ENTRY(light_sensitivity, openlcb::Uint8ConfigEntry, Default(100), Min(0),
    Max(100), Name("Light detector sensitivity"),
    Description("How sensitive the light detectors are: 0 is least sensitive, "
                "100 is most sensitive."));

CDI_GROUP_END(); // CrossingControllerConfig

}  // namespace crossing_controller



#endif /* CUSTOM_APPLICATIONS_CROSSING_CONTROLLER_CROSSINGCONTROLLERCONFIG_HXX_ */
