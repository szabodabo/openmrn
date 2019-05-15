#include "utils/Hub.hxx"
#include "openlcb/SimpleStack.hxx"

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
            LED_BLUE_RAW_Pin::set(1);
            activity_on_ = true;
        } else if (activity_on_) {
            activity_on_ = false;
            LED_BLUE_RAW_Pin::set(0);
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
