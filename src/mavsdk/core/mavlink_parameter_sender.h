#pragma once

#include "log.h"
#include "mavlink_include.h"
#include "timeout_s_callback.h"
#include "locked_queue.h"
#include "param_value.h"
#include "mavlink_parameter_subscription.h"
#include "mavlink_parameter_set.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <functional>
#include <utility>
#include <vector>
#include <map>
#include <optional>
#include <variant>

namespace mavsdk {

class Sender;
class MavlinkMessageHandler;
class TimeoutHandler;

class MavlinkParameterSender : public MavlinkParameterSubscription {
public:
    MavlinkParameterSender() = delete;
    explicit MavlinkParameterSender(
        Sender& parent,
        MavlinkMessageHandler& message_handler,
        TimeoutHandler& timeout_handler,
        TimeoutSCallback timeout_s_callback,
        uint8_t target_component_id = MAV_COMP_ID_AUTOPILOT1,
        bool use_extended_protocol = false);
    ~MavlinkParameterSender();

    // Non-copyable
    MavlinkParameterSender(const MavlinkParameterSender&) = delete;
    const MavlinkParameterSender& operator=(const MavlinkParameterSender&) = delete;

    enum class Result {
        Success,
        Timeout,
        ConnectionError,
        WrongType,
        ParamNameTooLong,
        NotFound,
        ValueUnsupported,
        Failed,
        ParamValueTooLong,
        StringTypeUnsupported,
        InconsistentData,
        UnknownError,
    };

    Result set_param(const std::string& name, const ParamValue& value);

    using SetParamCallback = std::function<void(Result result)>;

    // This call requires knowing the exact type of the parameter to set.
    void set_param_async(
        const std::string& name,
        const ParamValue& value,
        const SetParamCallback& callback,
        const void* cookie);

    Result set_param_int(const std::string& name, int32_t value);

    void set_param_int_async(
        const std::string& name,
        int32_t value,
        const SetParamCallback& callback,
        const void* cookie);

    Result set_param_float(const std::string& name, float value);

    void set_param_float_async(
        const std::string& name, float value, const SetParamCallback& callback, const void* cookie);

    Result set_param_custom(const std::string& name, const std::string& value);

    void set_param_custom_async(
        const std::string& name,
        const std::string& value,
        const SetParamCallback& callback,
        const void* cookie = nullptr);

    using GetParamAnyCallback = std::function<void(Result, ParamValue)>;

    void get_param_async(
        const std::string& name, const GetParamAnyCallback& callback, const void* cookie);
    std::pair<Result, ParamValue> get_param(const std::string& name);

    // The ParamValue is constructed just to infer the type.
    void get_param_async(
        const std::string& name,
        const ParamValue& value_type,
        const GetParamAnyCallback& callback,
        const void* cookie);

    // This could replace the code above.
    // We use get_param_async to get the current type and value for a parameter, then check the type
    // of the obtained parameter and return the result with the appropriate error codes via the
    // callback.
    template<class T> using GetParamTypesafeCallback = std::function<void(Result, T value)>;

    template<class T>
    void get_param_async_typesafe(
        const std::string& name, GetParamTypesafeCallback<T> callback, const void* cookie);

    std::pair<Result, float> get_param_float(const std::string& name);

    using GetParamFloatCallback = std::function<void(Result, float)>;

    void get_param_float_async(
        const std::string& name, const GetParamFloatCallback& callback, const void* cookie);

    std::pair<Result, int32_t> get_param_int(const std::string& name);

    using GetParamIntCallback = std::function<void(Result, int32_t)>;

    void get_param_int_async(
        const std::string& name, const GetParamIntCallback& callback, const void* cookie);

    std::pair<Result, std::string> get_param_custom(const std::string& name);

    using GetParamCustomCallback = std::function<void(Result, const std::string& value)>;

    void get_param_custom_async(
        const std::string& name, const GetParamCustomCallback& callback, const void* cookie);

    using GetAllParamsCallback =
        std::function<void(Result result, std::map<std::string, ParamValue> set)>;

    void get_all_params_async(GetAllParamsCallback callback, void* cookie);
    std::pair<Result, std::map<std::string, ParamValue>> get_all_params();

    void cancel_all_param(const void* cookie);

    void clear_cache();

    void do_work();

    friend std::ostream& operator<<(std::ostream&, const Result&);
    friend std::ostream& operator<<(std::ostream&, const Result&);

private:
    struct WorkItemSet {
        const std::string param_name;
        const ParamValue param_value;
        const SetParamCallback callback;
    };

    struct WorkItemGet {
        // We can get a parameter from the server by either its string id or index.
        const std::variant<std::string, int16_t> param_identifier;
        const GetParamAnyCallback callback;
    };

    struct WorkItemGetAll {
        const GetAllParamsCallback callback;
    };

    struct WorkItem {
        using WorkItemVariant = std::variant<WorkItemGet, WorkItemSet, WorkItemGetAll>;
        WorkItemVariant work_item_variant;
        const void* cookie{nullptr};
        unsigned retries_to_do = 3;
        bool already_requested{false};

        WorkItem() = delete;
        WorkItem(WorkItemVariant new_work_item_variant, const void* new_cookie) :
            work_item_variant(std::move(new_work_item_variant)),
            cookie(new_cookie){};
    };
    void process_param_value(const mavlink_message_t& message);
    void process_param_ext_value(const mavlink_message_t& message);
    void process_param_ext_ack(const mavlink_message_t& message);
    void receive_timeout();
    void receive_get_all_params_timeout();

    mavlink_message_t create_set_param_message(WorkItemSet& work_item);
    mavlink_message_t create_get_param_message(WorkItemGet& work_item);
    mavlink_message_t create_request_list_message();

    Sender& _sender;
    MavlinkMessageHandler& _message_handler;
    TimeoutHandler& _timeout_handler;
    TimeoutSCallback _timeout_s_callback;
    uint8_t _target_component_id = MAV_COMP_ID_AUTOPILOT1;
    bool _use_extended = false;

    // These are specific depending on the work item type
    LockedQueue<WorkItem> _work_queue{};

    void* _timeout_cookie = nullptr;

    std::mutex _all_params_mutex{};
    GetAllParamsCallback _all_params_callback;
    void* _all_params_timeout_cookie{nullptr};
    ParamSetFromServer _param_set_from_server;

    bool _parameter_debugging = true;

    // Validate if the response matches what was given in the work queue
    static bool validate_id_or_index(
        const std::variant<std::string, int16_t>& original,
        const std::string& param_id,
        int16_t param_index);

    // This adds the given parameter to the parameter set cache (if possible) and then checks and
    // call the _all_params_callback() if it is set and the parameter set has become complete after
    // adding this parameter.
    void add_param_to_cached_parameter_set(
        const std::string& safe_param_id,
        uint16_t param_idx,
        uint16_t all_param_count,
        const ParamValue& received_value);
    // Create a callback for a WorkItemGet that performs the following steps:
    // 1) Check if any parameter of the parameter set is missing.
    // 2) If yes, request the first missing parameter and recursively add the same callback to check
    // and request again 3) If no, terminate.
    GetParamAnyCallback create_recursive_callback();
};

} // namespace mavsdk
