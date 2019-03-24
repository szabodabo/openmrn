#ifndef _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/ServoConsumerConfig.hxx"

namespace openlcb
{

extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,                           // 4 (version info, always 4 by the standard
    "Dakota Szabo",              // Manufacturer name
    "Grade Crossing Controller", // Model name
    "F303RE-Nucleo-V1",          // Hardware version
    "OpenMRN 2019-03"            // Software version
};

/// Modify this value every time the EEPROM needs to be cleared on the node
/// after an update.
static constexpr uint16_t CANONICAL_VERSION = 0x184c;

CDI_GROUP(EventWrapper);
CDI_GROUP_ENTRY(
    event, EventConfigEntry, Name("Event ID that activates the crossing."));
CDI_GROUP_END(); // EventWrapper

/// Defines the main segment in the configuration CDI. This is laid out at
/// origin 128 to give space for the ACDI user data at the beginning.
CDI_GROUP(IoBoardSegment, Segment(MemoryConfigDefs::SPACE_CONFIG), Offset(128));
CDI_GROUP_ENTRY(internal_config, InternalConfigData);

using ActivationEvents = RepeatedGroup<EventWrapper, 16>;
using ServoConsumers = RepeatedGroup<ServoConsumerConfig, 4>;

CDI_GROUP_ENTRY(events_to_activate_crossing, ActivationEvents,
    Name("Activation Events"),
    Description("Events that, when seen, should activate the crossing."));
CDI_GROUP_ENTRY(crossing_post_activated_event, EventConfigEntry,
    Name("Post-Activated Event"),
    Description("Event to publish after crossing becomes active."));
CDI_GROUP_ENTRY(crossing_post_deactivated_event, EventConfigEntry,
    Name("Post-Deactivated Event"),
    Description("Event to publish after crossing becomes inactive."));
CDI_GROUP_ENTRY(servo_consumers, ServoConsumers, Name("Servo outputs"),
    Description("Servo outputs for crossing gate arms."));
CDI_GROUP_ENTRY(crossbuck_brightness, Uint8ConfigEntry, Default(100), Min(0),
    Max(100), Name("Crossbuck lamp brightness"),
    Description("Crossbuck lamp brightness, 0-100."));
CDI_GROUP_ENTRY(crossbuck_flash_rate, Uint16ConfigEntry, Default(1100), Min(250),
    Max(2000), Name("Crossbuck lamp flash rate"),
    Description("Crossbuck lamp flash rate in milliseconds, 0-2000."));
CDI_GROUP_ENTRY(crossbuck_style, Uint8ConfigEntry, Default(0), Min(0), Max(1),
    Name("Crossbuck lamp style"),
    Description("0 for LED, 1 for incandescent."));
CDI_GROUP_ENTRY(false_alarm_timeout_seconds, Uint16ConfigEntry, Default(60),
    Min(5), Max(500), Name("Crossing gate timeout (seconds)"),
    Description("How long to wait before the controller loses faith that a "
                "train is actually approaching."));
CDI_GROUP_ENTRY(light_sensitivity, Uint8ConfigEntry, Default(100), Min(0),
    Max(100), Name("Light detector sensitivity"),
    Description("How sensitive the light detectors are: 0 is least sensitive, "
                "100 is most sensitive."));

CDI_GROUP_END();

/// This segment is only needed temporarily until there is program code to set
/// the ACDI user data version byte.
CDI_GROUP(VersionSeg, Segment(MemoryConfigDefs::SPACE_CONFIG),
    Name("Version information"));
CDI_GROUP_ENTRY(acdi_user_version, Uint8ConfigEntry,
    Name("ACDI User Data version"), Description("Set to 2 and do not change."));
CDI_GROUP_END();

/// The main structure of the CDI. ConfigDef is the symbol we use in main.cxx
/// to refer to the configuration defined here.
CDI_GROUP(ConfigDef, MainCdi());
/// Adds the <identification> tag with the values from SNIP_STATIC_DATA above.
CDI_GROUP_ENTRY(ident, Identification);
/// Adds an <acdi> tag.
CDI_GROUP_ENTRY(acdi, Acdi);
/// Adds a segment for changing the values in the ACDI user-defined
/// space. UserInfoSegment is defined in the system header.
CDI_GROUP_ENTRY(userinfo, UserInfoSegment);
/// Adds the main configuration segment.
CDI_GROUP_ENTRY(seg, IoBoardSegment);
/// Adds the versioning segment.
CDI_GROUP_ENTRY(version, VersionSeg);
CDI_GROUP_END();

} // namespace openlcb

#endif // _APPLICATIONS_IO_BOARD_TARGET_CONFIG_HXX_
