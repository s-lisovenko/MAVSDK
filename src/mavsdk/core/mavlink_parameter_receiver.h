#pragma once

#include "mavlink_parameter_subscription.h"
#include "sender.h"
#include "mavlink_message_handler.h"
#include "timeout_handler.h"
#include "timeout_s_callback.h"
#include "param_value.h"
#include "locked_queue.h"
#include "mavlink_parameter_cache.h"
#include "mavlink_parameter_subscription.h"

#include <map>
#include <string>
#include <list>
#include <utility>

namespace mavsdk {

/**
 * This class provide parameters to be accessed via MAVLink.
 *
 * Note that the server side is much simpler as it does not need to worry about
 * re-transmission.
 *
 * Note regarding non-extended and extended parameters protocol:
 *
 * In addition to the parameter types from the non-extended parameter protocol,
 * the extended parameter protocol also supports string parameter values.
 *
 * This class supports clients using both the non-extended and the extended
 * parameter protocol, but hides the string parameter values from non-extended
 * protocol clients. Therefore, if the server has std:.string parameters but is
 * talking to a non-extended client, param_index and param_count are different
 * compared to talking to a client who doesn't speak extended.
 */
class MavlinkParameterReceiver {
public:
    MavlinkParameterReceiver() = delete;
    explicit MavlinkParameterReceiver(Sender& parent, MavlinkMessageHandler& message_handler);
    ~MavlinkParameterReceiver();

    enum class Result {
        Success,
        WrongType,
        ParamNameTooLong,
        NotFound,
        ParamValueTooLong,
        ParamExistsAlready,
        TooManyParams,
        ParamNotFound,
    };

    /*
     * Note that it is recommended to not change the parameter set after the
     * first communication with any client.
     *
     * Also see https://mavlink.io/en/services/parameter_ext.html#parameters_invariant
     */
    Result provide_server_param(const std::string& name, const ParamValue& param_value);
    Result provide_server_param_float(const std::string& name, float value);
    Result provide_server_param_int(const std::string& name, int32_t value);
    Result provide_server_param_custom(const std::string& name, const std::string& value);

    [[nodiscard]] std::map<std::string, ParamValue> retrieve_all_server_params() const;

    template<class T>
    [[nodiscard]] std::pair<Result, T> retrieve_server_param(const std::string& name) const;

    [[nodiscard]] std::pair<Result, float>
    retrieve_server_param_float(const std::string& name) const;
    [[nodiscard]] std::pair<Result, int32_t>
    retrieve_server_param_int(const std::string& name) const;
    [[nodiscard]] std::pair<Result, std::string>
    retrieve_server_param_custom(const std::string& name) const;

    void subscribe_param_changed(
        const std::string& name,
        const MavlinkParameterSubscription::ParamChangedCallbacks& callback,
        const void* cookie);
    void subscribe_param_float_changed(
        const std::string& name,
        const MavlinkParameterSubscription::ParamFloatChangedCallback& callback,
        const void* cookie);
    void subscribe_param_int_changed(
        const std::string& name,
        const MavlinkParameterSubscription::ParamIntChangedCallback& callback,
        const void* cookie);
    void subscribe_param_custom_changed(
        const std::string& name,
        const MavlinkParameterSubscription::ParamCustomChangedCallback& callback,
        const void* cookie);
    void unsubscribe_param_changed(const std::string& name, const void* cookie);

    void do_work();

    friend std::ostream& operator<<(std::ostream&, const Result&);

    // Non-copyable
    MavlinkParameterReceiver(const MavlinkParameterReceiver&) = delete;
    const MavlinkParameterReceiver& operator=(const MavlinkParameterReceiver&) = delete;

private:
    void process_param_set_internally(
        const std::string& param_id, const ParamValue& value_to_set, bool extended);
    void process_param_set(const mavlink_message_t& message);
    void process_param_ext_set(const mavlink_message_t& message);

    Sender& _sender;
    MavlinkMessageHandler& _message_handler;

    mutable std::mutex _all_params_mutex{};
    MavlinkParameterCache _param_cache{};

    void process_param_request_read(const mavlink_message_t& message);
    void process_param_ext_request_read(const mavlink_message_t& message);

    void internal_process_param_request_read_by_id(const std::string& id, bool extended);
    void internal_process_param_request_read_by_index(uint16_t index, bool extended);

    void process_param_request_list(const mavlink_message_t& message);
    void process_param_ext_request_list(const mavlink_message_t& message);
    void broadcast_all_parameters(bool extended);

    mutable std::mutex _param_subscriptions_mutex{};
    MavlinkParameterSubscription _param_subscriptions{};

    struct WorkItemValue {
        const uint16_t param_index;
        const uint16_t param_count;
        const bool extended;
    };

    struct WorkItemAck {
        const PARAM_ACK param_ack;
    };

    struct WorkItem {
        // A response always has a valid param id
        const std::string param_id;
        // as well as a valid param value
        const ParamValue param_value;
        using WorkItemVariant = std::variant<WorkItemValue, WorkItemAck>;
        const WorkItemVariant work_item_variant;
        explicit WorkItem(
            std::string new_param_id,
            ParamValue new_param_value,
            WorkItemVariant new_work_item_variant) :
            param_id(std::move(new_param_id)),
            param_value(std::move(new_param_value)),
            work_item_variant(std::move(new_work_item_variant)){};
    };
    LockedQueue<WorkItem> _work_queue{};

    bool target_matches(uint16_t target_sys_id, uint16_t target_comp_id, bool is_request) const;

    // Helper for safely handling a request_read or ext_request_read message (which have the exact
    // same layout). returns the identifier that should be used or nothing if the message is
    // ill-formed. See https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ and
    // https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_READ
    static std::variant<std::monostate, std::string, uint16_t>
    extract_request_read_param_identifier(int16_t param_index, const char* param_id);

    bool _debugging = false;
};

} // namespace mavsdk
