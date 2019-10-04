#ifndef CUSTOMAPPLICATIONS_SIGNALCONTROLLER_INPUTPINCONFIG_HXX_
#define CUSTOMAPPLICATIONS_SIGNALCONTROLLER_INPUTPINCONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/MemoryConfig.hxx"

namespace signal_controller {

CDI_GROUP(InputPinConfig, Name("Input Pins"));

CDI_GROUP_ENTRY(name, openlcb::StringConfigEntry<64>, Name("Name"));
CDI_GROUP_ENTRY(event_on, openlcb::EventConfigEntry, Name("Event On"),
		Description("Event to produce when pin is \"on\"."));
CDI_GROUP_ENTRY(event_off, openlcb::EventConfigEntry, Name("Event Off"),
		Description("Event to produce when pin is \"off\"."));
CDI_GROUP_ENTRY(pin_on_value, openlcb::Uint8ConfigEntry, Name("Pin \"On\" Value"),
		Min(0), Max(1), Default(0),
		Description("0 if pin LOW means ON (active low); 1 if pin HIGH means ON (active high)."));

CDI_GROUP_END(); // InputPinConfig

}  // namespace signal_controller

#endif // CUSTOMAPPLICATIONS_SIGNALCONTROLLER_INPUTPINCONFIG_HXX_
