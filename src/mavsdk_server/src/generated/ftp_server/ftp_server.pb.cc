// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ftp_server/ftp_server.proto

#include "ftp_server/ftp_server.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
#include "google/protobuf/generated_message_tctable_impl.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::google::protobuf;
namespace _pbi = ::google::protobuf::internal;
namespace _fl = ::google::protobuf::internal::field_layout;
namespace mavsdk {
namespace rpc {
namespace ftp_server {

inline constexpr SetRootDirRequest::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : path_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        _cached_size_{0} {}

template <typename>
PROTOBUF_CONSTEXPR SetRootDirRequest::SetRootDirRequest(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct SetRootDirRequestDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SetRootDirRequestDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~SetRootDirRequestDefaultTypeInternal() {}
  union {
    SetRootDirRequest _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SetRootDirRequestDefaultTypeInternal _SetRootDirRequest_default_instance_;

inline constexpr FtpServerResult::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : result_str_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        result_{static_cast< ::mavsdk::rpc::ftp_server::FtpServerResult_Result >(0)},
        _cached_size_{0} {}

template <typename>
PROTOBUF_CONSTEXPR FtpServerResult::FtpServerResult(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct FtpServerResultDefaultTypeInternal {
  PROTOBUF_CONSTEXPR FtpServerResultDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~FtpServerResultDefaultTypeInternal() {}
  union {
    FtpServerResult _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 FtpServerResultDefaultTypeInternal _FtpServerResult_default_instance_;

inline constexpr SetRootDirResponse::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : _cached_size_{0},
        ftp_server_result_{nullptr} {}

template <typename>
PROTOBUF_CONSTEXPR SetRootDirResponse::SetRootDirResponse(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct SetRootDirResponseDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SetRootDirResponseDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~SetRootDirResponseDefaultTypeInternal() {}
  union {
    SetRootDirResponse _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SetRootDirResponseDefaultTypeInternal _SetRootDirResponse_default_instance_;
}  // namespace ftp_server
}  // namespace rpc
}  // namespace mavsdk
static ::_pb::Metadata file_level_metadata_ftp_5fserver_2fftp_5fserver_2eproto[3];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_ftp_5fserver_2fftp_5fserver_2eproto[1];
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_ftp_5fserver_2fftp_5fserver_2eproto = nullptr;
const ::uint32_t TableStruct_ftp_5fserver_2fftp_5fserver_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    ~0u,  // no _has_bits_
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::SetRootDirRequest, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::SetRootDirRequest, _impl_.path_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::SetRootDirResponse, _impl_._has_bits_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::SetRootDirResponse, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::SetRootDirResponse, _impl_.ftp_server_result_),
    0,
    ~0u,  // no _has_bits_
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::FtpServerResult, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::FtpServerResult, _impl_.result_),
    PROTOBUF_FIELD_OFFSET(::mavsdk::rpc::ftp_server::FtpServerResult, _impl_.result_str_),
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        {0, -1, -1, sizeof(::mavsdk::rpc::ftp_server::SetRootDirRequest)},
        {9, 18, -1, sizeof(::mavsdk::rpc::ftp_server::SetRootDirResponse)},
        {19, -1, -1, sizeof(::mavsdk::rpc::ftp_server::FtpServerResult)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::mavsdk::rpc::ftp_server::_SetRootDirRequest_default_instance_._instance,
    &::mavsdk::rpc::ftp_server::_SetRootDirResponse_default_instance_._instance,
    &::mavsdk::rpc::ftp_server::_FtpServerResult_default_instance_._instance,
};
const char descriptor_table_protodef_ftp_5fserver_2fftp_5fserver_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\033ftp_server/ftp_server.proto\022\025mavsdk.rp"
    "c.ftp_server\032\024mavsdk_options.proto\"!\n\021Se"
    "tRootDirRequest\022\014\n\004path\030\001 \001(\t\"W\n\022SetRoot"
    "DirResponse\022A\n\021ftp_server_result\030\001 \001(\0132&"
    ".mavsdk.rpc.ftp_server.FtpServerResult\"\302"
    "\001\n\017FtpServerResult\022=\n\006result\030\001 \001(\0162-.mav"
    "sdk.rpc.ftp_server.FtpServerResult.Resul"
    "t\022\022\n\nresult_str\030\002 \001(\t\"\\\n\006Result\022\022\n\016RESUL"
    "T_UNKNOWN\020\000\022\022\n\016RESULT_SUCCESS\020\001\022\031\n\025RESUL"
    "T_DOES_NOT_EXIST\020\002\022\017\n\013RESULT_BUSY\020\0032{\n\020F"
    "tpServerService\022g\n\nSetRootDir\022(.mavsdk.r"
    "pc.ftp_server.SetRootDirRequest\032).mavsdk"
    ".rpc.ftp_server.SetRootDirResponse\"\004\200\265\030\001"
    "B&\n\024io.mavsdk.ftp_serverB\016FtpServerProto"
    "b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_deps[1] =
    {
        &::descriptor_table_mavsdk_5foptions_2eproto,
};
static ::absl::once_flag descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto = {
    false,
    false,
    568,
    descriptor_table_protodef_ftp_5fserver_2fftp_5fserver_2eproto,
    "ftp_server/ftp_server.proto",
    &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_once,
    descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_deps,
    1,
    3,
    schemas,
    file_default_instances,
    TableStruct_ftp_5fserver_2fftp_5fserver_2eproto::offsets,
    file_level_metadata_ftp_5fserver_2fftp_5fserver_2eproto,
    file_level_enum_descriptors_ftp_5fserver_2fftp_5fserver_2eproto,
    file_level_service_descriptors_ftp_5fserver_2fftp_5fserver_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_getter() {
  return &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_ftp_5fserver_2fftp_5fserver_2eproto(&descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto);
namespace mavsdk {
namespace rpc {
namespace ftp_server {
const ::google::protobuf::EnumDescriptor* FtpServerResult_Result_descriptor() {
  ::google::protobuf::internal::AssignDescriptors(&descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto);
  return file_level_enum_descriptors_ftp_5fserver_2fftp_5fserver_2eproto[0];
}
PROTOBUF_CONSTINIT const uint32_t FtpServerResult_Result_internal_data_[] = {
    262144u, 0u, };
bool FtpServerResult_Result_IsValid(int value) {
  return 0 <= value && value <= 3;
}
#if (__cplusplus < 201703) && \
  (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

constexpr FtpServerResult_Result FtpServerResult::RESULT_UNKNOWN;
constexpr FtpServerResult_Result FtpServerResult::RESULT_SUCCESS;
constexpr FtpServerResult_Result FtpServerResult::RESULT_DOES_NOT_EXIST;
constexpr FtpServerResult_Result FtpServerResult::RESULT_BUSY;
constexpr FtpServerResult_Result FtpServerResult::Result_MIN;
constexpr FtpServerResult_Result FtpServerResult::Result_MAX;
constexpr int FtpServerResult::Result_ARRAYSIZE;

#endif  // (__cplusplus < 201703) &&
        // (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
// ===================================================================

class SetRootDirRequest::_Internal {
 public:
};

SetRootDirRequest::SetRootDirRequest(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.ftp_server.SetRootDirRequest)
}
inline PROTOBUF_NDEBUG_INLINE SetRootDirRequest::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : path_(arena, from.path_),
        _cached_size_{0} {}

SetRootDirRequest::SetRootDirRequest(
    ::google::protobuf::Arena* arena,
    const SetRootDirRequest& from)
    : ::google::protobuf::Message(arena) {
  SetRootDirRequest* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);

  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.ftp_server.SetRootDirRequest)
}
inline PROTOBUF_NDEBUG_INLINE SetRootDirRequest::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : path_(arena),
        _cached_size_{0} {}

inline void SetRootDirRequest::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
}
SetRootDirRequest::~SetRootDirRequest() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.ftp_server.SetRootDirRequest)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void SetRootDirRequest::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  _impl_.path_.Destroy();
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void SetRootDirRequest::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.ftp_server.SetRootDirRequest)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.path_.ClearToEmpty();
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* SetRootDirRequest::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<0, 1, 0, 52, 2> SetRootDirRequest::_table_ = {
  {
    0,  // no _has_bits_
    0, // no _extensions_
    1, 0,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967294,  // skipmap
    offsetof(decltype(_table_), field_entries),
    1,  // num_field_entries
    0,  // num_aux_entries
    offsetof(decltype(_table_), field_names),  // no aux_entries
    &_SetRootDirRequest_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    // string path = 1;
    {::_pbi::TcParser::FastUS1,
     {10, 63, 0, PROTOBUF_FIELD_OFFSET(SetRootDirRequest, _impl_.path_)}},
  }}, {{
    65535, 65535
  }}, {{
    // string path = 1;
    {PROTOBUF_FIELD_OFFSET(SetRootDirRequest, _impl_.path_), 0, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
  }},
  // no aux_entries
  {{
    "\47\4\0\0\0\0\0\0"
    "mavsdk.rpc.ftp_server.SetRootDirRequest"
    "path"
  }},
};

::uint8_t* SetRootDirRequest::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.ftp_server.SetRootDirRequest)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  // string path = 1;
  if (!this->_internal_path().empty()) {
    const std::string& _s = this->_internal_path();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "mavsdk.rpc.ftp_server.SetRootDirRequest.path");
    target = stream->WriteStringMaybeAliased(1, _s, target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.ftp_server.SetRootDirRequest)
  return target;
}

::size_t SetRootDirRequest::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.ftp_server.SetRootDirRequest)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string path = 1;
  if (!this->_internal_path().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_path());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData SetRootDirRequest::_class_data_ = {
    SetRootDirRequest::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* SetRootDirRequest::GetClassData() const {
  return &_class_data_;
}

void SetRootDirRequest::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<SetRootDirRequest*>(&to_msg);
  auto& from = static_cast<const SetRootDirRequest&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.ftp_server.SetRootDirRequest)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_path().empty()) {
    _this->_internal_set_path(from._internal_path());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void SetRootDirRequest::CopyFrom(const SetRootDirRequest& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.ftp_server.SetRootDirRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool SetRootDirRequest::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* SetRootDirRequest::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void SetRootDirRequest::InternalSwap(SetRootDirRequest* PROTOBUF_RESTRICT other) {
  using std::swap;
  auto* arena = GetArena();
  ABSL_DCHECK_EQ(arena, other->GetArena());
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.path_, &other->_impl_.path_, arena);
}

::google::protobuf::Metadata SetRootDirRequest::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_getter, &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_once,
      file_level_metadata_ftp_5fserver_2fftp_5fserver_2eproto[0]);
}
// ===================================================================

class SetRootDirResponse::_Internal {
 public:
  using HasBits = decltype(std::declval<SetRootDirResponse>()._impl_._has_bits_);
  static constexpr ::int32_t kHasBitsOffset =
    8 * PROTOBUF_FIELD_OFFSET(SetRootDirResponse, _impl_._has_bits_);
  static const ::mavsdk::rpc::ftp_server::FtpServerResult& ftp_server_result(const SetRootDirResponse* msg);
  static void set_has_ftp_server_result(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::mavsdk::rpc::ftp_server::FtpServerResult& SetRootDirResponse::_Internal::ftp_server_result(const SetRootDirResponse* msg) {
  return *msg->_impl_.ftp_server_result_;
}
SetRootDirResponse::SetRootDirResponse(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.ftp_server.SetRootDirResponse)
}
inline PROTOBUF_NDEBUG_INLINE SetRootDirResponse::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : _has_bits_{from._has_bits_},
        _cached_size_{0} {}

SetRootDirResponse::SetRootDirResponse(
    ::google::protobuf::Arena* arena,
    const SetRootDirResponse& from)
    : ::google::protobuf::Message(arena) {
  SetRootDirResponse* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);
  ::uint32_t cached_has_bits = _impl_._has_bits_[0];
  _impl_.ftp_server_result_ = (cached_has_bits & 0x00000001u)
                ? CreateMaybeMessage<::mavsdk::rpc::ftp_server::FtpServerResult>(arena, *from._impl_.ftp_server_result_)
                : nullptr;

  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.ftp_server.SetRootDirResponse)
}
inline PROTOBUF_NDEBUG_INLINE SetRootDirResponse::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : _cached_size_{0} {}

inline void SetRootDirResponse::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  _impl_.ftp_server_result_ = {};
}
SetRootDirResponse::~SetRootDirResponse() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.ftp_server.SetRootDirResponse)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void SetRootDirResponse::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  delete _impl_.ftp_server_result_;
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void SetRootDirResponse::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.ftp_server.SetRootDirResponse)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    ABSL_DCHECK(_impl_.ftp_server_result_ != nullptr);
    _impl_.ftp_server_result_->Clear();
  }
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* SetRootDirResponse::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<0, 1, 1, 0, 2> SetRootDirResponse::_table_ = {
  {
    PROTOBUF_FIELD_OFFSET(SetRootDirResponse, _impl_._has_bits_),
    0, // no _extensions_
    1, 0,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967294,  // skipmap
    offsetof(decltype(_table_), field_entries),
    1,  // num_field_entries
    1,  // num_aux_entries
    offsetof(decltype(_table_), aux_entries),
    &_SetRootDirResponse_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    // .mavsdk.rpc.ftp_server.FtpServerResult ftp_server_result = 1;
    {::_pbi::TcParser::FastMtS1,
     {10, 0, 0, PROTOBUF_FIELD_OFFSET(SetRootDirResponse, _impl_.ftp_server_result_)}},
  }}, {{
    65535, 65535
  }}, {{
    // .mavsdk.rpc.ftp_server.FtpServerResult ftp_server_result = 1;
    {PROTOBUF_FIELD_OFFSET(SetRootDirResponse, _impl_.ftp_server_result_), _Internal::kHasBitsOffset + 0, 0,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
  }}, {{
    {::_pbi::TcParser::GetTable<::mavsdk::rpc::ftp_server::FtpServerResult>()},
  }}, {{
  }},
};

::uint8_t* SetRootDirResponse::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.ftp_server.SetRootDirResponse)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // .mavsdk.rpc.ftp_server.FtpServerResult ftp_server_result = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        1, _Internal::ftp_server_result(this),
        _Internal::ftp_server_result(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.ftp_server.SetRootDirResponse)
  return target;
}

::size_t SetRootDirResponse::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.ftp_server.SetRootDirResponse)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .mavsdk.rpc.ftp_server.FtpServerResult ftp_server_result = 1;
  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size +=
        1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.ftp_server_result_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData SetRootDirResponse::_class_data_ = {
    SetRootDirResponse::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* SetRootDirResponse::GetClassData() const {
  return &_class_data_;
}

void SetRootDirResponse::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<SetRootDirResponse*>(&to_msg);
  auto& from = static_cast<const SetRootDirResponse&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.ftp_server.SetRootDirResponse)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if ((from._impl_._has_bits_[0] & 0x00000001u) != 0) {
    _this->_internal_mutable_ftp_server_result()->::mavsdk::rpc::ftp_server::FtpServerResult::MergeFrom(
        from._internal_ftp_server_result());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void SetRootDirResponse::CopyFrom(const SetRootDirResponse& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.ftp_server.SetRootDirResponse)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool SetRootDirResponse::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* SetRootDirResponse::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void SetRootDirResponse::InternalSwap(SetRootDirResponse* PROTOBUF_RESTRICT other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  swap(_impl_.ftp_server_result_, other->_impl_.ftp_server_result_);
}

::google::protobuf::Metadata SetRootDirResponse::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_getter, &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_once,
      file_level_metadata_ftp_5fserver_2fftp_5fserver_2eproto[1]);
}
// ===================================================================

class FtpServerResult::_Internal {
 public:
};

FtpServerResult::FtpServerResult(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:mavsdk.rpc.ftp_server.FtpServerResult)
}
inline PROTOBUF_NDEBUG_INLINE FtpServerResult::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : result_str_(arena, from.result_str_),
        _cached_size_{0} {}

FtpServerResult::FtpServerResult(
    ::google::protobuf::Arena* arena,
    const FtpServerResult& from)
    : ::google::protobuf::Message(arena) {
  FtpServerResult* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);
  _impl_.result_ = from._impl_.result_;

  // @@protoc_insertion_point(copy_constructor:mavsdk.rpc.ftp_server.FtpServerResult)
}
inline PROTOBUF_NDEBUG_INLINE FtpServerResult::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : result_str_(arena),
        _cached_size_{0} {}

inline void FtpServerResult::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  _impl_.result_ = {};
}
FtpServerResult::~FtpServerResult() {
  // @@protoc_insertion_point(destructor:mavsdk.rpc.ftp_server.FtpServerResult)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void FtpServerResult::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  _impl_.result_str_.Destroy();
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void FtpServerResult::Clear() {
// @@protoc_insertion_point(message_clear_start:mavsdk.rpc.ftp_server.FtpServerResult)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.result_str_.ClearToEmpty();
  _impl_.result_ = 0;
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* FtpServerResult::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<1, 2, 0, 56, 2> FtpServerResult::_table_ = {
  {
    0,  // no _has_bits_
    0, // no _extensions_
    2, 8,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967292,  // skipmap
    offsetof(decltype(_table_), field_entries),
    2,  // num_field_entries
    0,  // num_aux_entries
    offsetof(decltype(_table_), field_names),  // no aux_entries
    &_FtpServerResult_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    // string result_str = 2;
    {::_pbi::TcParser::FastUS1,
     {18, 63, 0, PROTOBUF_FIELD_OFFSET(FtpServerResult, _impl_.result_str_)}},
    // .mavsdk.rpc.ftp_server.FtpServerResult.Result result = 1;
    {::_pbi::TcParser::SingularVarintNoZag1<::uint32_t, offsetof(FtpServerResult, _impl_.result_), 63>(),
     {8, 63, 0, PROTOBUF_FIELD_OFFSET(FtpServerResult, _impl_.result_)}},
  }}, {{
    65535, 65535
  }}, {{
    // .mavsdk.rpc.ftp_server.FtpServerResult.Result result = 1;
    {PROTOBUF_FIELD_OFFSET(FtpServerResult, _impl_.result_), 0, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kOpenEnum)},
    // string result_str = 2;
    {PROTOBUF_FIELD_OFFSET(FtpServerResult, _impl_.result_str_), 0, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
  }},
  // no aux_entries
  {{
    "\45\0\12\0\0\0\0\0"
    "mavsdk.rpc.ftp_server.FtpServerResult"
    "result_str"
  }},
};

::uint8_t* FtpServerResult::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:mavsdk.rpc.ftp_server.FtpServerResult)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  // .mavsdk.rpc.ftp_server.FtpServerResult.Result result = 1;
  if (this->_internal_result() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
        1, this->_internal_result(), target);
  }

  // string result_str = 2;
  if (!this->_internal_result_str().empty()) {
    const std::string& _s = this->_internal_result_str();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "mavsdk.rpc.ftp_server.FtpServerResult.result_str");
    target = stream->WriteStringMaybeAliased(2, _s, target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:mavsdk.rpc.ftp_server.FtpServerResult)
  return target;
}

::size_t FtpServerResult::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:mavsdk.rpc.ftp_server.FtpServerResult)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string result_str = 2;
  if (!this->_internal_result_str().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_result_str());
  }

  // .mavsdk.rpc.ftp_server.FtpServerResult.Result result = 1;
  if (this->_internal_result() != 0) {
    total_size += 1 +
                  ::_pbi::WireFormatLite::EnumSize(this->_internal_result());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData FtpServerResult::_class_data_ = {
    FtpServerResult::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* FtpServerResult::GetClassData() const {
  return &_class_data_;
}

void FtpServerResult::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<FtpServerResult*>(&to_msg);
  auto& from = static_cast<const FtpServerResult&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:mavsdk.rpc.ftp_server.FtpServerResult)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_result_str().empty()) {
    _this->_internal_set_result_str(from._internal_result_str());
  }
  if (from._internal_result() != 0) {
    _this->_internal_set_result(from._internal_result());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void FtpServerResult::CopyFrom(const FtpServerResult& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:mavsdk.rpc.ftp_server.FtpServerResult)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool FtpServerResult::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* FtpServerResult::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void FtpServerResult::InternalSwap(FtpServerResult* PROTOBUF_RESTRICT other) {
  using std::swap;
  auto* arena = GetArena();
  ABSL_DCHECK_EQ(arena, other->GetArena());
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.result_str_, &other->_impl_.result_str_, arena);
  swap(_impl_.result_, other->_impl_.result_);
}

::google::protobuf::Metadata FtpServerResult::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_getter, &descriptor_table_ftp_5fserver_2fftp_5fserver_2eproto_once,
      file_level_metadata_ftp_5fserver_2fftp_5fserver_2eproto[2]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace ftp_server
}  // namespace rpc
}  // namespace mavsdk
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"