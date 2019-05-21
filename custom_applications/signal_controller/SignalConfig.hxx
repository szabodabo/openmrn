#ifndef CUSTOM_APPLICATIONS_SIGNAL_CONTROLLER_SIGNALCONFIG_HXX_
#define CUSTOM_APPLICATIONS_SIGNAL_CONTROLLER_SIGNALCONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

namespace signal_controller {

CDI_GROUP(HeadEvents, Name("Signal Head Events"));

CDI_GROUP_ENTRY(
		green_event, openlcb::EventConfigEntry,
		Name("Green"), Description("Head goes Solid Green on this Event"));
CDI_GROUP_ENTRY(
		yellow_event, openlcb::EventConfigEntry,
		Name("Yellow"), Description("Head goes Solid Yellow on this Event"));
CDI_GROUP_ENTRY(
		red_event, openlcb::EventConfigEntry,
		Name("Red"), Description("Head goes Solid Red on this Event"));

CDI_GROUP_ENTRY(
		flash_green_event, openlcb::EventConfigEntry,
		Name("Flashing Green"), Description("Head goes Flashing Green on this Event"));
CDI_GROUP_ENTRY(
		flash_yellow_event, openlcb::EventConfigEntry,
		Name("Flashing Yellow"), Description("Head goes Flashing Yellow on this Event"));
CDI_GROUP_ENTRY(
		flash_red_event, openlcb::EventConfigEntry,
		Name("Flashing Red"), Description("Head goes Flashing Red on this Event"));

CDI_GROUP_ENTRY(
		dark_event, openlcb::EventConfigEntry,
		Name("Dark"), Description("Head goes Dark on this Event"));

CDI_GROUP_END(); // HeadEvents

CDI_GROUP(HeadAppearance, Name("Signal Head Appearance"));

CDI_GROUP_ENTRY(
		green_brightness, openlcb::Uint8ConfigEntry,
		Default(7), Max(100), Min(0),
		Name("Green Brightness"),
		Description("Brightness percentage of Green lamp, 0-100"));
CDI_GROUP_ENTRY(
		yellow_brightness, openlcb::Uint8ConfigEntry,
		Default(25), Max(100), Min(0),
		Name("Yellow Brightness"),
		Description("Brightness percentage of Yellow lamp, 0-100"));
CDI_GROUP_ENTRY(
		red_brightness, openlcb::Uint8ConfigEntry,
		Default(20), Max(100), Min(0),
		Name("Red Brightness"),
		Description("Brightness percentage of Red lamp, 0-100"));

CDI_GROUP_ENTRY(lamp_style, openlcb::Uint8ConfigEntry, Default(1), Min(0), Max(1),
    Name("Lamp Style"), Description("0 for LED, 1 for incandescent."));
CDI_GROUP_ENTRY(lamp_jitter, openlcb::Uint8ConfigEntry, Default(32), Min(0), Max(100),
    Name("Jitter Percentage"), Description("Lamp effect randomness (%, 0-100)."));

CDI_GROUP_END(); // HeadAppearance

CDI_GROUP(SignalHeadConfig, Name("Signal Settings"));

CDI_GROUP_ENTRY(name, openlcb::StringConfigEntry<64>, Name("Name"),
		Description("Friendly name (likely containing location)"));
CDI_GROUP_ENTRY(events, HeadEvents, Name("Events"));
CDI_GROUP_ENTRY(appearance, HeadAppearance, Name("Appearance"));

CDI_GROUP_END(); // SignalHeadConfig


using SignalHeadConfigs = openlcb::RepeatedGroup<SignalHeadConfig, 8>;

CDI_GROUP(SignalConfigWrapper, Name("Signal Heads"));
CDI_GROUP_ENTRY(heads, SignalHeadConfigs, Name("Signal Heads"), RepName("Signal Head"));
CDI_GROUP_END();  // SignalConfigWrapper

} // namespace signal_controller

#endif // CUSTOM_APPLICATIONS_SIGNAL_CONTROLLER_SIGNALCONFIG_HXX_
