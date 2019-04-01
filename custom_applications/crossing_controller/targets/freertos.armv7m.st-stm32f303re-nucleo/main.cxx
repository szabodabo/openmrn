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
#include "CrossingControllerManager.hxx"

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

bool CROSSING_ACTIVE = false;

// TODO: Attach CDI config schema to hardware things.

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(stack.node(), {/*producer_sw1.polling()*/});

GPIO_PIN(SndEnable, GpioOutputSafeLow, C, 10);
static const Gpio* snd_enable_gpio = SndEnable_Pin::instance();

void EnableCBLampTimer() {
	__HAL_TIM_ENABLE_IT(&tim1_handle, TIM_IT_UPDATE);
	 __HAL_TIM_ENABLE(&tim1_handle);
	 TIM_CCxChannelCmd(tim1_handle.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	 TIM_CCxChannelCmd(tim1_handle.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
	 __HAL_TIM_MOE_ENABLE(&tim1_handle);
}

void DisableCBLampTimer() {
	__HAL_TIM_DISABLE_IT(&tim1_handle, TIM_IT_UPDATE);
	 __HAL_TIM_DISABLE(&tim1_handle);
	 TIM_CCxChannelCmd(tim1_handle.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	 TIM_CCxChannelCmd(tim1_handle.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	 __HAL_TIM_MOE_DISABLE(&tim1_handle);
}

crossing_controller::CrossingManager crossing(cfg, stack.node(),
	snd_enable_gpio, &CROSSING_ACTIVE);

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

	// TODO: figure out why calling HAL_TIM_PWM_Enable_IT fails FreeRTOS assert.
	// TODO: looks like LED is on even when fet gate is 0 :[!
	//       but only for one fet channel... could be bad hardware.
	EnableCBLampTimer();
	 

	// This command donates the main thread to the operation of the
	// stack. Alternatively the stack could be started in a separate stack and
	// then application-specific business logic could be executed ion a busy
	// loop in the main thread.
	stack.loop_executor();
	return 0;
}
