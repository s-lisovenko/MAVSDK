#include "mavlink_parameter_subscription.h"

namespace mavsdk {

void MavlinkParameterSubscription::subscribe_param_changed(
    const std::string& name, const ParamChangedCallbacks& callback, const void* cookie)
{
    std::lock_guard<std::mutex> lock(_subscriptions_mutex);
    _subscriptions.emplace_back(Subscription{name, callback, cookie});
}

void MavlinkParameterSubscription::unsubscribe_param_changed(
    const std::string& name, const void* cookie)
{
    std::lock_guard<std::mutex> lock(_subscriptions_mutex);
    _subscriptions.erase(
        std::remove_if(
            _subscriptions.begin(),
            _subscriptions.end(),
            [&](const auto& entry) { return entry.param_name == name && entry.cookie == cookie; }),
        _subscriptions.end());
}

void MavlinkParameterSubscription::find_and_call_subscriptions_value_changed(
    const std::string& param_name, const ParamValue& value)
{
    std::lock_guard<std::mutex> lock(_subscriptions_mutex);
    for (const auto& subscription : _subscriptions) {
        if (subscription.param_name != param_name) {
            continue;
        }

        // We have a subscription on this param name, now check if the subscription is for the right
        // type and call the callback when matching
        if (std::get_if<ParamFloatChangedCallback>(&subscription.callback) && value.get_float()) {
            std::get<ParamFloatChangedCallback>(subscription.callback)(value.get_float().value());
        } else if (
            std::get_if<ParamIntChangedCallback>(&subscription.callback) && value.get_int()) {
            std::get<ParamIntChangedCallback>(subscription.callback)(value.get_int().value());
        } else if (
            std::get_if<ParamCustomChangedCallback>(&subscription.callback) && value.get_custom()) {
            std::get<ParamCustomChangedCallback>(subscription.callback)(value.get_custom().value());
        } else {
            // The callback we have set is not for this type.
            LogErr() << "Type and callback mismatch";
        }
    }
}

} // namespace mavsdk
