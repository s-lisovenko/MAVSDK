#pragma once

#include "param_value.h"
#include <mutex>
#include <vector>

namespace mavsdk {

class MavlinkParameterSubscription {
public:
    template<class T> using ParamChangedCallback = std::function<void(T value)>;
    using ParamFloatChangedCallback = ParamChangedCallback<float>;
    using ParamIntChangedCallback = ParamChangedCallback<int>;
    using ParamCustomChangedCallback = ParamChangedCallback<std::string>;

    using ParamChangedCallbacks = std::
        variant<ParamFloatChangedCallback, ParamIntChangedCallback, ParamCustomChangedCallback>;

    void subscribe_param_changed(
        const std::string& name, const ParamChangedCallbacks& callback, const void* cookie);

    void unsubscribe_param_changed(const std::string& name, const void* cookie);

    void find_and_call_subscriptions_value_changed(
        const std::string& param_name, const ParamValue& new_param_value);

private:
    struct Subscription {
        std::string param_name;
        ParamChangedCallbacks callback;
        const void* cookie;
    };
    std::mutex _subscriptions_mutex{};
    std::vector<Subscription> _subscriptions{};
};

} // namespace mavsdk
