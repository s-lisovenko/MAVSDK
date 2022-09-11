#include "mavlink_parameter_sender.h"
#include "mavlink_message_handler.h"
#include "timeout_handler.h"
#include "system_impl.h"
#include "plugin_base.h"
#include "unused.h"
#include <algorithm>
#include <future>
#include <cassert>
#include <utility>

// FIXME:
//  - go through debug, warnings
//  - test with drops
//  - check all unknown results
//  - use queue for callbacks

namespace mavsdk {

MavlinkParameterSender::MavlinkParameterSender(
    Sender& sender,
    MavlinkMessageHandler& message_handler,
    TimeoutHandler& timeout_handler,
    TimeoutSCallback timeout_s_callback,
    const uint8_t target_component_id,
    bool use_extended) :
    _sender(sender),
    _message_handler(message_handler),
    _timeout_handler(timeout_handler),
    _timeout_s_callback(std::move(timeout_s_callback)),
    _target_component_id(target_component_id),
    _use_extended(use_extended)
{
    if (const char* env_p = std::getenv("MAVSDK_PARAMETER_DEBUGGING")) {
        if (std::string(env_p) == "1") {
            LogDebug() << "Parameter debugging is on.";
            _parameter_debugging = true;
        }
    }
    if (_use_extended) {
        _message_handler.register_one(
            MAVLINK_MSG_ID_PARAM_EXT_VALUE,
            [this](const mavlink_message_t& message) { process_param_ext_value(message); },
            this);

        _message_handler.register_one(
            MAVLINK_MSG_ID_PARAM_EXT_ACK,
            [this](const mavlink_message_t& message) { process_param_ext_ack(message); },
            this);
    } else {
        _message_handler.register_one(
            MAVLINK_MSG_ID_PARAM_VALUE,
            [this](const mavlink_message_t& message) { process_param_value(message); },
            this);
    }
}

MavlinkParameterSender::~MavlinkParameterSender()
{
    _message_handler.unregister_all(this);
}

MavlinkParameterSender::Result
MavlinkParameterSender::set_param(const std::string& name, const ParamValue& value)
{
    auto prom = std::promise<Result>();
    auto res = prom.get_future();
    set_param_async(
        name, value, [&prom](Result result) { prom.set_value(result); }, this);
    return res.get();
}

void MavlinkParameterSender::set_param_async(
    const std::string& name,
    const ParamValue& value,
    const SetParamCallback& callback,
    const void* cookie)
{
    if (name.size() > MavlinkParameterSet::PARAM_ID_LEN) {
        LogErr() << "Param name too long";
        if (callback) {
            callback(Result::ParamNameTooLong);
        }
        return;
    }
    if (value.is<std::string>() && !_use_extended) {
        LogErr() << "String needs extended parameter protocol";
        if (callback) {
            callback(Result::StringTypeUnsupported);
        }
        return;
    }
    auto new_work = std::make_shared<WorkItem>(WorkItemSet{name, value, callback}, cookie);
    _work_queue.push_back(new_work);
}

void MavlinkParameterSender::set_param_int_async(
    const std::string& name, int32_t value, const SetParamCallback& callback, const void* cookie)
{
    if (name.size() > MavlinkParameterSet::PARAM_ID_LEN) {
        LogErr() << "Param name too long";
        if (callback) {
            callback(Result::ParamNameTooLong);
        }
        return;
    }

    // PX4 only uses int32_t, so we can be sure and don't need to check the exact type first
    // by getting the param, or checking the cache.
    if (_sender.autopilot() == SystemImpl::Autopilot::Px4) {
        ParamValue value_to_set;
        value_to_set.set(static_cast<int32_t>(value));
        set_param_async(name, value_to_set, callback, cookie);
    } else {
        // We don't know which exact int type the server wants, so we have to get the param
        // first to see the type before setting it.
        auto param_opt = _param_set_from_server.lookup_parameter(name);
        if (param_opt.has_value()) {
            // we have the parameter cached
            auto param = param_opt.value();
            if (param.set_int(value)) {
                // we have successfully written whatever int the user provided into the int type
                // that is actually stored
                set_param_async(name, param, callback, cookie);
            } else {
                // We didn't find compatibility and give up.
                if (callback) {
                    LogErr() << "Wrong type for int in cache";
                    callback(Result::WrongType);
                }
                return;
            }
        } else {
            // parameter is not cached. Request it and then perform the appropriate action once we
            // know it
            auto send_message_once_type_is_known = [this, name, value, callback, cookie](
                                                       Result result,
                                                       ParamValue fetched_param_value) {
                if (result == Result::Success) {
                    if (fetched_param_value.set_int(value)) {
                        // Since the callback itself is called with the work queue locked, we have
                        // to make sure that the work queue guard is removed before we call the
                        // finalizing callback of a work item.
                        set_param_async(name, fetched_param_value, callback, cookie);
                    } else {
                        // The param type returned does is not compatible with an int, give up.
                        if (callback) {
                            LogErr() << "Wrong type for int returned";
                            callback(Result::WrongType);
                        }
                    }
                } else {
                    // Failed to get the param to get the type, pass on the error.
                    callback(result);
                }
            };
            get_param_async(name, send_message_once_type_is_known, cookie);
        }
    }
}

MavlinkParameterSender::Result
MavlinkParameterSender::set_param_int(const std::string& name, int32_t value)
{
    auto prom = std::promise<Result>();
    auto res = prom.get_future();
    set_param_int_async(
        name, value, [&prom](Result result) { prom.set_value(result); }, this);
    return res.get();
}

void MavlinkParameterSender::set_param_float_async(
    const std::string& name, float value, const SetParamCallback& callback, const void* cookie)
{
    ParamValue value_to_set;
    value_to_set.set_float(value);
    set_param_async(name, value_to_set, callback, cookie);
}

MavlinkParameterSender::Result
MavlinkParameterSender::set_param_float(const std::string& name, float value)
{
    auto prom = std::promise<Result>();
    auto res = prom.get_future();

    set_param_float_async(
        name, value, [&prom](Result result) { prom.set_value(result); }, this);

    return res.get();
}

void MavlinkParameterSender::set_param_custom_async(
    const std::string& name,
    const std::string& value,
    const SetParamCallback& callback,
    const void* cookie)
{
    if (name.size() > MavlinkParameterSet::PARAM_ID_LEN) {
        LogErr() << "Param name too long";
        if (callback) {
            callback(Result::ParamNameTooLong);
        }
        return;
    }

    if (value.size() > sizeof(mavlink_param_ext_set_t::param_value)) {
        LogErr() << "Param value too long";
        if (callback) {
            callback(Result::ParamValueTooLong);
        }
        return;
    }
    ParamValue value_to_set;
    value_to_set.set_custom(value);
    set_param_async(name, value_to_set, callback, cookie);
}

MavlinkParameterSender::Result
MavlinkParameterSender::set_param_custom(const std::string& name, const std::string& value)
{
    auto prom = std::promise<Result>();
    auto res = prom.get_future();
    set_param_custom_async(
        name, value, [&prom](Result result) { prom.set_value(result); }, this);
    return res.get();
}

void MavlinkParameterSender::get_param_async(
    const std::string& name, const GetParamAnyCallback& callback, const void* cookie)
{
    if (_parameter_debugging) {
        LogDebug() << "Getting param " << name << ", extended: " << (_use_extended ? "yes" : "no");
    }
    if (name.size() > MavlinkParameterSet::PARAM_ID_LEN) {
        LogErr() << "Param name too long";
        if (callback) {
            callback(Result::ParamNameTooLong, {});
        }
        return;
    }

    auto new_work = std::make_shared<WorkItem>(WorkItemGet{name, callback}, cookie);
    _work_queue.push_back(new_work);
}

void MavlinkParameterSender::get_param_async(
    const std::string& name,
    const ParamValue& value_type,
    const GetParamAnyCallback& callback,
    const void* cookie)
{
    // We need to delay the type checking until we get a response from the server.
    GetParamAnyCallback callback_future_result = [callback,
                                                  value_type](Result result, ParamValue value) {
        if (result == Result::Success) {
            if (value.is_same_type(value_type)) {
                callback(Result::Success, std::move(value));
            } else {
                callback(Result::WrongType, {});
            }
        } else {
            callback(result, {});
        }
    };
    get_param_async(name, callback, cookie);
}

template<class T>
void MavlinkParameterSender::get_param_async_typesafe(
    const std::string& name, const GetParamTypesafeCallback<T> callback, const void* cookie)
{
    // We need to delay the type checking until we get a response from the server.
    GetParamAnyCallback callback_future_result = [callback](Result result, ParamValue value) {
        if (result == Result::Success) {
            if (value.is<T>()) {
                callback(Result::Success, value.get<T>());
            } else {
                callback(Result::WrongType, {});
            }
        } else {
            callback(result, {});
        }
    };
    get_param_async(name, callback_future_result, cookie);
}

void MavlinkParameterSender::get_param_float_async(
    const std::string& name, const GetParamFloatCallback& callback, const void* cookie)
{
    get_param_async_typesafe<float>(name, callback, cookie);
}

void MavlinkParameterSender::get_param_int_async(
    const std::string& name, const GetParamIntCallback& callback, const void* cookie)
{
    get_param_async_typesafe<int32_t>(name, callback, cookie);
}

void MavlinkParameterSender::get_param_custom_async(
    const std::string& name, const GetParamCustomCallback& callback, const void* cookie)
{
    get_param_async_typesafe<std::string>(name, callback, cookie);
}

std::pair<MavlinkParameterSender::Result, ParamValue>
MavlinkParameterSender::get_param(const std::string& name)
{
    auto prom = std::promise<std::pair<Result, ParamValue>>();
    auto res = prom.get_future();
    get_param_async(
        name,
        [&prom](Result result, ParamValue new_value) {
            prom.set_value(std::make_pair<>(result, std::move(new_value)));
        },
        this);
    return res.get();
}

std::pair<MavlinkParameterSender::Result, int32_t>
MavlinkParameterSender::get_param_int(const std::string& name)
{
    auto prom = std::promise<std::pair<Result, int32_t>>();
    auto res = prom.get_future();
    get_param_int_async(
        name,
        [&prom](Result result, int32_t value) { prom.set_value(std::make_pair<>(result, value)); },
        this);
    return res.get();
}

std::pair<MavlinkParameterSender::Result, float>
MavlinkParameterSender::get_param_float(const std::string& name)
{
    auto prom = std::promise<std::pair<Result, float>>();
    auto res = prom.get_future();
    get_param_float_async(
        name,
        [&prom](Result result, float value) { prom.set_value(std::make_pair<>(result, value)); },
        this);
    return res.get();
}

std::pair<MavlinkParameterSender::Result, std::string>
MavlinkParameterSender::get_param_custom(const std::string& name)
{
    auto prom = std::promise<std::pair<Result, std::string>>();
    auto res = prom.get_future();
    get_param_custom_async(
        name,
        [&prom](Result result, const std::string& value) {
            prom.set_value(std::make_pair<>(result, value));
        },
        this);
    return res.get();
}

void MavlinkParameterSender::get_all_params_async(GetAllParamsCallback callback, void* cookie)
{
    if (_parameter_debugging) {
        LogDebug() << "Getting all params, extended: " << (_use_extended ? "yes" : "no");
    }

    std::lock_guard<std::mutex> lock(_all_params_mutex);

    auto new_work = std::make_shared<WorkItem>(WorkItemGetAll{std::move(callback)}, cookie);
    _work_queue.push_back(new_work);
}

std::pair<MavlinkParameterSender::Result, std::map<std::string, ParamValue>>
MavlinkParameterSender::get_all_params()
{
    std::promise<std::pair<MavlinkParameterSender::Result, std::map<std::string, ParamValue>>> prom;
    auto res = prom.get_future();
    get_all_params_async(
        // Make sure to NOT use a reference for all_params here, pass by value.
        // Since for example on a timeout, the empty all_params result is constructed in-place and
        // then goes out of scope when the callback returns.
        [&prom](Result result, std::map<std::string, ParamValue> set) {
            prom.set_value({result, std::move(set)});
        },
        this);
    return res.get();
}

void MavlinkParameterSender::cancel_all_param(const void* cookie)
{
    LockedQueue<WorkItem>::Guard work_queue_guard(_work_queue);

    // We don't call any callbacks before erasing them as this is just used on destruction
    // where we don't care anymore.
    _work_queue.erase(std::remove_if(_work_queue.begin(), _work_queue.end(), [&](auto&& item) {
        return (item->cookie == cookie);
    }));
}

void MavlinkParameterSender::clear_cache()
{
    _param_set_from_server.clear();
}

void MavlinkParameterSender::do_work()
{
    auto work_queue_guard = std::make_unique<LockedQueue<WorkItem>::Guard>(_work_queue);
    auto work = work_queue_guard->get_front();

    if (!work) {
        return;
    }

    if (work->already_requested) {
        return;
    }

    std::visit(
        overloaded{
            [&](WorkItemSet& item) {
                auto message = create_set_param_message(item);
                if (!_sender.send_message(message)) {
                    LogErr() << "Send message failed";
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        work_queue_guard.reset();
                        callback(Result::ConnectionError);
                    }
                    return;
                }
                work->already_requested = true;
                // We want to get notified if a timeout happens
                _timeout_handler.add(
                    [this] { receive_timeout(); }, _timeout_s_callback(), &_timeout_cookie);
            },
            [&](WorkItemGet& item) {
                auto message = create_get_param_message(item);
                if (!_sender.send_message(message)) {
                    LogErr() << "Send message failed";
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        work_queue_guard.reset();
                        item.callback(Result::ConnectionError, ParamValue{});
                    }
                    return;
                }
                work->already_requested = true;
                // We want to get notified if a timeout happens
                _timeout_handler.add(
                    [this] { receive_timeout(); }, _timeout_s_callback(), &_timeout_cookie);
            },
            [&](WorkItemGetAll& item) {
                auto message = create_request_list_message();

                if (!_sender.send_message(message)) {
                    LogErr() << "Send message failed";
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        work_queue_guard.reset();
                        item.callback(Result::ConnectionError, {});
                    }
                    return;
                }
                work->already_requested = true;
                // We want to get notified if a timeout happens
                _timeout_handler.add(
                    [this] { receive_timeout(); }, _timeout_s_callback(), &_timeout_cookie);
            }},
        work->work_item_variant);
}

mavlink_message_t MavlinkParameterSender::create_set_param_message(WorkItemSet& work_item)
{
    auto param_id = MavlinkParameterSet::param_id_to_message_buffer(work_item.param_name);

    mavlink_message_t message;
    if (_use_extended) {
        const auto param_value_buf = work_item.param_value.get_128_bytes();
        if (_parameter_debugging) {
            LogDebug() << "Sending param_ext_set to:" << (int)_sender.get_own_system_id() << ":"
                       << (int)_sender.get_own_component_id();
        }
        mavlink_msg_param_ext_set_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id,
            param_id.data(),
            param_value_buf.data(),
            work_item.param_value.get_mav_param_ext_type());
    } else {
        if (_parameter_debugging) {
            LogDebug() << "Sending param_set to:" << (int)_sender.get_own_system_id() << ":"
                       << (int)_sender.get_own_component_id();
        }

        const float value_set = (_sender.autopilot() == SystemImpl::Autopilot::ArduPilot) ?
                                    work_item.param_value.get_4_float_bytes_cast() :
                                    work_item.param_value.get_4_float_bytes_bytewise();

        mavlink_msg_param_set_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id,
            param_id.data(),
            value_set,
            work_item.param_value.get_mav_param_type());
    }
    return message;
}

mavlink_message_t MavlinkParameterSender::create_get_param_message(WorkItemGet& work_item)
{
    std::array<char, MavlinkParameterSet::PARAM_ID_LEN> param_id_buff{};
    int16_t param_index = -1;
    if (auto str = std::get_if<std::string>(&work_item.param_identifier)) {
        param_id_buff = MavlinkParameterSet::param_id_to_message_buffer(*str);
    } else {
        // param_id_buff doesn't matter
        param_index = std::get<int16_t>(work_item.param_identifier);
    }

    mavlink_message_t message;

    if (_use_extended) {
        if (_parameter_debugging) {
            LogDebug() << "Send param_ext_request_read: " << (int)_sender.get_own_system_id() << ":"
                       << (int)_sender.get_own_component_id() << " to "
                       << (int)_sender.get_system_id() << ":" << (int)_target_component_id;
        }

        mavlink_msg_param_ext_request_read_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id,
            param_id_buff.data(),
            param_index);

    } else {
        if (_parameter_debugging) {
            LogDebug() << "Send param_request_read: " << (int)_sender.get_own_system_id() << ":"
                       << (int)_sender.get_own_component_id() << " to "
                       << (int)_sender.get_system_id() << ":" << (int)_target_component_id;
        }

        mavlink_msg_param_request_read_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id,
            param_id_buff.data(),
            param_index);
    }

    return message;
}

mavlink_message_t MavlinkParameterSender::create_request_list_message()
{
    mavlink_message_t message;
    if (_use_extended) {
        mavlink_msg_param_ext_request_list_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id);
    } else {
        mavlink_msg_param_request_list_pack(
            _sender.get_own_system_id(),
            _sender.get_own_component_id(),
            &message,
            _sender.get_system_id(),
            _target_component_id);
    }

    return message;
}

void MavlinkParameterSender::process_param_value(const mavlink_message_t& message)
{
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&message, &param_value);
    const std::string safe_param_id =
        MavlinkParameterSet::extract_safe_param_id(param_value.param_id);
    if (safe_param_id.empty()) {
        LogWarn() << "Got ill-formed param_value message (param_id empty)";
        return;
    }

    ParamValue received_value;
    const bool set_value_success = received_value.set_from_mavlink_param_value(
        param_value,
        (_sender.autopilot() == SystemImpl::Autopilot::ArduPilot) ?
            ParamValue::Conversion::Cast :
            ParamValue::Conversion::Bitwise);
    if (!set_value_success) {
        LogWarn() << "Got ill-formed param_ext_value message (param_type unknown)";
        return;
    }

    if (_parameter_debugging) {
        LogDebug() << "process_param_value: " << safe_param_id << " " << received_value;
    }

    // add_param_to_cached_parameter_set(
    //    safe_param_id, param_value.param_index, param_value.param_count, received_value);
    //// TODO I think we need to consider more edge cases here
    // find_and_call_subscriptions_value_changed(safe_param_id, received_value);

    // We need to use a unique pointer here to remove the lock from the work queue manually "early"
    // before calling the (perhaps user-provided) callback. Otherwise, we might end up in a deadlock
    // if the callback wants to push another work item onto the queue. By using a unique ptr there
    // is no risk of forgetting to remove the lock - it is destroyed (if still valid) after going
    // out of scope.
    auto work_queue_guard = std::make_unique<LockedQueue<WorkItem>::Guard>(_work_queue);
    const auto work = work_queue_guard->get_front();
    if (!work) {
        return;
    }

    if (!work->already_requested) {
        return;
    }

    std::visit(
        overloaded{
            [&](WorkItemSet& item) {
                if (item.param_name != safe_param_id) {
                    // No match, let's just return the borrowed work item.
                    return;
                }

                // We are done, inform caller and go back to idle
                const auto result = item.param_value == received_value ?
                                        MavlinkParameterSender::Result::Success :
                                        MavlinkParameterSender::Result::Failed;

                _timeout_handler.remove(_timeout_cookie);
                // LogDebug() << "time taken: " <<
                // _sender.get_time().elapsed_since_s(_last_request_time);
                work_queue_guard->pop_front();
                if (item.callback) {
                    auto callback = item.callback;
                    work_queue_guard.reset();
                    callback(result);
                }
            },
            [&](WorkItemGet& item) {
                if (!validate_id_or_index(
                        item.param_identifier,
                        safe_param_id,
                        static_cast<int16_t>(param_value.param_index))) {
                    LogDebug() << "Got unexpected response on work item";
                    // No match, let's just return the borrowed work item.
                    return;
                }
                _timeout_handler.remove(_timeout_cookie);
                // LogDebug() << "time taken: " <<
                // _sender.get_time().elapsed_since_s(_last_request_time);
                work_queue_guard->pop_front();
                if (item.callback) {
                    auto callback = item.callback;
                    work_queue_guard.reset();
                    callback(Result::Success, received_value);
                }
            },
            [&](WorkItemGetAll& item) {
                // TODO
                UNUSED(item);
            }},
        work->work_item_variant);
}

void MavlinkParameterSender::process_param_ext_value(const mavlink_message_t& message)
{
    mavlink_param_ext_value_t param_ext_value;
    mavlink_msg_param_ext_value_decode(&message, &param_ext_value);
    const auto safe_param_id = MavlinkParameterSet::extract_safe_param_id(param_ext_value.param_id);
    if (safe_param_id.empty()) {
        LogDebug() << "Got ill-formed param_ext_value message (param_id empty)";
        return;
    }
    ParamValue received_value;
    if (!received_value.set_from_mavlink_param_ext_value(param_ext_value)) {
        LogDebug() << "Got ill-formed param_ext_value message (param_type unknown)";
        return;
    }

    if (_parameter_debugging) {
        LogDebug() << "process param_ext_value: " << safe_param_id << " " << received_value;
    }

    // add_param_to_cached_parameter_set(
    //    safe_param_id, param_ext_value.param_index, param_ext_value.param_count, received_value);
    //// TODO I think we need to consider more edge cases here
    // find_and_call_subscriptions_value_changed(safe_param_id, received_value);

    // See comments on process_param_value for use of unique_ptr
    auto work_queue_guard = std::make_unique<LockedQueue<WorkItem>::Guard>(_work_queue);
    auto work = work_queue_guard->get_front();
    if (!work) {
        return;
    }
    if (!work->already_requested) {
        return;
    }

    std::visit(
        overloaded{
            [&](WorkItemSet&) {
                if (_parameter_debugging) {
                    LogDebug() << "Unexpected ParamExtValue response.";
                }
            },
            [&](WorkItemGet& item) {
                if (!validate_id_or_index(
                        item.param_identifier,
                        safe_param_id,
                        static_cast<int16_t>(param_ext_value.param_index))) {
                    LogDebug() << "Got unexpected response on work item";
                    // No match, let's just return the borrowed work item.
                    return;
                }
                _timeout_handler.remove(_timeout_cookie);
                // LogDebug() << "time taken: " <<
                // _sender.get_time().elapsed_since_s(_last_request_time);
                work_queue_guard->pop_front();
                if (item.callback) {
                    auto callback = item.callback;
                    work_queue_guard.reset();
                    callback(Result::Success, received_value);
                }
            },
            [&](WorkItemGetAll& item) {
                UNUSED(item);
                // TODO
            },
        },
        work->work_item_variant);
}

void MavlinkParameterSender::process_param_ext_ack(const mavlink_message_t& message)
{
    mavlink_param_ext_ack_t param_ext_ack;
    mavlink_msg_param_ext_ack_decode(&message, &param_ext_ack);
    const auto safe_param_id = MavlinkParameterSet::extract_safe_param_id(param_ext_ack.param_id);

    if (_parameter_debugging) {
        LogDebug() << "process param_ext_value: " << safe_param_id << " "
                   << (int)param_ext_ack.param_result;
    }

    // See comments on process_param_value for use of unique_ptr
    auto work_queue_guard = std::make_unique<LockedQueue<WorkItem>::Guard>(_work_queue);
    auto work = work_queue_guard->get_front();
    if (!work) {
        return;
    }
    if (!work->already_requested) {
        return;
    }

    std::visit(
        overloaded{
            [&](WorkItemSet& item) {
                if (item.param_name != safe_param_id) {
                    // No match, let's just return the borrowed work item.
                    return;
                }
                if (param_ext_ack.param_result == PARAM_ACK_ACCEPTED) {
                    _timeout_handler.remove(_timeout_cookie);
                    // LogDebug() << "time taken: " <<
                    // _sender.get_time().elapsed_since_s(_last_request_time);
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        // We are done, inform caller and go back to idle
                        work_queue_guard.reset();
                        callback(Result::Success);
                    }
                } else if (param_ext_ack.param_result == PARAM_ACK_IN_PROGRESS) {
                    // Reset timeout and wait again.
                    _timeout_handler.refresh(_timeout_cookie);

                } else {
                    LogErr() << "Somehow we did not get an ack, we got: "
                             << int(param_ext_ack.param_result);
                    _timeout_handler.remove(_timeout_cookie);
                    // LogDebug() << "time taken: " <<
                    // _sender.get_time().elapsed_since_s(_last_request_time);
                    work_queue_guard->pop_front();
                    work_queue_guard.reset();
                    if (item.callback) {
                        auto callback = item.callback;
                        auto result = [&]() {
                            switch (param_ext_ack.param_result) {
                                case PARAM_ACK_FAILED:
                                    return Result::Failed;
                                case PARAM_ACK_VALUE_UNSUPPORTED:
                                    return Result::ValueUnsupported;
                                default:
                                    return Result::UnknownError;
                            }
                        }();
                        work_queue_guard.reset();
                        callback(result);
                    }
                }
            },
            [&](WorkItemGet&) { LogWarn() << "Unexpected ParamExtAck response."; },
            [&](WorkItemGetAll&) { LogWarn() << "Unexpected ParamExtAck response."; }},
        work->work_item_variant);
}

void MavlinkParameterSender::receive_timeout()
{
    // See comments on process_param_value for use of unique_ptr
    auto work_queue_guard = std::make_unique<LockedQueue<WorkItem>::Guard>(_work_queue);

    auto work = work_queue_guard->get_front();
    if (!work) {
        LogErr() << "Received timeout without work";
        return;
    }
    if (!work->already_requested) {
        LogErr() << "Received timeout without already having work requested";
        return;
    }

    std::visit(
        overloaded{
            [&](WorkItemSet& item) {
                if (work->retries_to_do > 0) {
                    // We're not sure the command arrived, let's retransmit.
                    LogWarn() << "sending again, retries to do: " << work->retries_to_do << "  ("
                              << item.param_name << ").";

                    auto message = create_set_param_message(item);

                    if (!_sender.send_message(message)) {
                        LogErr() << "connection send error in retransmit (" << item.param_name
                                 << ").";
                        work_queue_guard->pop_front();

                        if (item.callback) {
                            auto callback = item.callback;
                            work_queue_guard.reset();
                            callback(Result::ConnectionError);
                        }
                    } else {
                        --work->retries_to_do;
                        _timeout_handler.add(
                            [this] { receive_timeout(); }, _timeout_s_callback(), &_timeout_cookie);
                    }
                } else {
                    // We have tried retransmitting, giving up now.
                    LogErr() << "Error: Retrying failed get param busy timeout: "
                             << item.param_name;
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        work_queue_guard.reset();
                        callback(Result::Timeout);
                    }
                }
            },
            [&](WorkItemGet& item) {
                if (work->retries_to_do > 0) {
                    // We're not sure the command arrived, let's retransmit.
                    LogWarn() << "sending again, retries to do: " << work->retries_to_do;
                    auto message = create_get_param_message(item);
                    if (!_sender.send_message(message)) {
                        LogErr() << "connection send error in retransmit ";
                        work_queue_guard->pop_front();
                        if (item.callback) {
                            auto callback = item.callback;
                            work_queue_guard.reset();
                            callback(Result::ConnectionError, {});
                        }
                    } else {
                        --work->retries_to_do;
                        _timeout_handler.add(
                            [this] { receive_timeout(); }, _timeout_s_callback(), &_timeout_cookie);
                    }
                } else {
                    // We have tried retransmitting, giving up now.
                    LogErr() << "Error: Retrying failed get param busy timeout: ";
                    work_queue_guard->pop_front();
                    if (item.callback) {
                        auto callback = item.callback;
                        work_queue_guard.reset();
                        callback(Result::Timeout, {});
                    }
                }
            },
            [&](WorkItemGetAll&) {
#if 0
          LogWarn() << "Unexpected ParamExtAck response.";

          if (_parameter_debugging) {
              LogDebug() << "All params receive timeout with " << _param_set_from_server.to_string();
          }

          if (!_param_set_from_server.param_count_known()) {
              // We got 0 messages back from the server (param count unknown). Most likely the "list
              // request" got lost before making it to the server,
              // TODO maybe re-transmit the list request message just like it is done with the other
              // work items.
              _all_params_callback(Result::Timeout, {});
              _all_params_callback = nullptr;
          } else {
              // We know the n of parameters on the server, now check how many are still missing
              const auto missing_param_indices = _param_set_from_server.get_missing_param_indices();
              if (missing_param_indices.empty()) {
                  assert(_param_set_from_server.is_complete());
                  _all_params_callback(
                      Result::Success, _param_set_from_server.get_all_params());
                  _all_params_callback = nullptr;
              }
              // We request all the missing parameters. for that, we can use the work queue.
              // We add the first missing parameter to the work queue, once we've gotten a result of
              // this operation we can fetch the next one.
              const auto first_missing_param = missing_param_indices.at(0);

              if (_parameter_debugging) {
                  LogDebug() << "Requesting missing parameter " << (int)first_missing_param;
              }

              auto new_work = std::make_shared<WorkItem>(
                  _timeout_s_callback(),
                  WorkItemGet{static_cast<int16_t>(first_missing_param), create_recursive_callback()},
                  this);
              _work_queue.push_back(new_work);
          }
#endif
            }},
        work->work_item_variant);
}

std::ostream& operator<<(std::ostream& str, const MavlinkParameterSender::Result& result)
{
    switch (result) {
        case MavlinkParameterSender::Result::Success:
            return str << "Success";
        case MavlinkParameterSender::Result::Timeout:
            return str << "Timeout";
        case MavlinkParameterSender::Result::ConnectionError:
            return str << "ConnectionError";
        case MavlinkParameterSender::Result::WrongType:
            return str << "WrongType";
        case MavlinkParameterSender::Result::ParamNameTooLong:
            return str << "ParamNameTooLong";
        case MavlinkParameterSender::Result::NotFound:
            return str << "NotFound";
        case MavlinkParameterSender::Result::ValueUnsupported:
            return str << "ValueUnsupported";
        case MavlinkParameterSender::Result::Failed:
            return str << "Failed";
        case MavlinkParameterSender::Result::UnknownError:
            // Fallthrough
        default:
            return str << "UnknownError";
    }
}

bool MavlinkParameterSender::validate_id_or_index(
    const std::variant<std::string, int16_t>& original,
    const std::string& param_id,
    const int16_t param_index)
{
    if (const auto str = std::get_if<std::string>(&original)) {
        if (param_id != *str) {
            // We requested by string id, but response doesn't match
            return false;
        }
    } else {
        const auto tmp = std::get<int16_t>(original);
        if (param_index != tmp) {
            // We requested by index, but response doesn't match
            return false;
        }
    }
    return true;
}

void MavlinkParameterSender::add_param_to_cached_parameter_set(
    const std::string& safe_param_id,
    const uint16_t param_idx,
    const uint16_t all_param_count,
    const ParamValue& received_value)
{
    std::lock_guard<std::mutex> lock(_all_params_mutex);
    const bool success = _param_set_from_server.add_new_parameter(
        safe_param_id, param_idx, all_param_count, received_value);
    if (!success) {
        // This can result in unwanted behaviour, but never crashes / terminates the sender.
        LogWarn() << "Invariant parameter set detected";
    }
    if (_all_params_callback) {
        if (!success) {
            // we cannot do a full parameter synchronization with this server, it provides
            // inconsistent data.
            _timeout_handler.remove(_all_params_timeout_cookie);
            _all_params_callback(Result::InconsistentData, {});
            _all_params_callback = nullptr;
            return;
        }
        if (_param_set_from_server.is_complete()) {
            LogDebug() << "Param set complete " << _param_set_from_server.to_string();
            _timeout_handler.remove(_all_params_timeout_cookie);
            _all_params_callback(Result::Success, _param_set_from_server.get_all_params());
            _all_params_callback = nullptr;
        } else {
            LogDebug() << "Param set not yet complete";
            // update the timeout handler, messages are still coming in.
            _timeout_handler.refresh(&_all_params_timeout_cookie);
        }
    }
}

MavlinkParameterSender::GetParamAnyCallback MavlinkParameterSender::create_recursive_callback()
{
    const auto callback = [this](Result res, ParamValue&&) {
        std::lock_guard<std::mutex> lock(_all_params_mutex);
        if (res == Result::Success) {
            const auto missing = _param_set_from_server.get_missing_param_indices();
            if (missing.empty()) {
                assert(_param_set_from_server.is_complete());
                // we are done, the parameter set is complete.
            } else {
                // Request the next parameter still missing
                const auto next_missing_param = missing.at(0);
                LogInfo() << "Requesting missing parameter " << (int)next_missing_param << " of "
                          << missing.size();
                auto new_work = std::make_shared<WorkItem>(
                    WorkItemGet{
                        static_cast<int16_t>(next_missing_param), create_recursive_callback()},
                    this);
                _work_queue.push_back(new_work);
            }
        } else {
            Result conv_result = Result::UnknownError;
            if (res == Result::Timeout) {
                conv_result = Result::Timeout;
            } else if (res == Result::ConnectionError) {
                conv_result = Result::ConnectionError;
            }
            LogDebug() << "Get param used for GetAllParameters failed";
            if (_all_params_callback) {
                _all_params_callback(conv_result, {});
                _all_params_callback = nullptr;
            }
        }
    };
    return callback;
}

} // namespace mavsdk
