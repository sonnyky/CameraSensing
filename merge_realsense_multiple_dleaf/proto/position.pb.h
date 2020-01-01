// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: proto/position.proto

#ifndef PROTOBUF_proto_2fposition_2eproto__INCLUDED
#define PROTOBUF_proto_2fposition_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_proto_2fposition_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultspeople_positionImpl();
void InitDefaultspeople_position();
inline void InitDefaults() {
  InitDefaultspeople_position();
}
}  // namespace protobuf_proto_2fposition_2eproto
namespace zaboom {
class people_position;
class people_positionDefaultTypeInternal;
extern people_positionDefaultTypeInternal _people_position_default_instance_;
}  // namespace zaboom
namespace zaboom {

// ===================================================================

class people_position : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:zaboom.people_position) */ {
 public:
  people_position();
  virtual ~people_position();

  people_position(const people_position& from);

  inline people_position& operator=(const people_position& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  people_position(people_position&& from) noexcept
    : people_position() {
    *this = ::std::move(from);
  }

  inline people_position& operator=(people_position&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const people_position& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const people_position* internal_default_instance() {
    return reinterpret_cast<const people_position*>(
               &_people_position_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(people_position* other);
  friend void swap(people_position& a, people_position& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline people_position* New() const PROTOBUF_FINAL { return New(NULL); }

  people_position* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const people_position& from);
  void MergeFrom(const people_position& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(people_position* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated int64 x = 1;
  int x_size() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int64 x(int index) const;
  void set_x(int index, ::google::protobuf::int64 value);
  void add_x(::google::protobuf::int64 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
      x() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
      mutable_x();

  // repeated int64 y = 2;
  int y_size() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int64 y(int index) const;
  void set_y(int index, ::google::protobuf::int64 value);
  void add_y(::google::protobuf::int64 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
      y() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
      mutable_y();

  // @@protoc_insertion_point(class_scope:zaboom.people_position)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 > x_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int64 > y_;
  friend struct ::protobuf_proto_2fposition_2eproto::TableStruct;
  friend void ::protobuf_proto_2fposition_2eproto::InitDefaultspeople_positionImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// people_position

// repeated int64 x = 1;
inline int people_position::x_size() const {
  return x_.size();
}
inline void people_position::clear_x() {
  x_.Clear();
}
inline ::google::protobuf::int64 people_position::x(int index) const {
  // @@protoc_insertion_point(field_get:zaboom.people_position.x)
  return x_.Get(index);
}
inline void people_position::set_x(int index, ::google::protobuf::int64 value) {
  x_.Set(index, value);
  // @@protoc_insertion_point(field_set:zaboom.people_position.x)
}
inline void people_position::add_x(::google::protobuf::int64 value) {
  x_.Add(value);
  // @@protoc_insertion_point(field_add:zaboom.people_position.x)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
people_position::x() const {
  // @@protoc_insertion_point(field_list:zaboom.people_position.x)
  return x_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
people_position::mutable_x() {
  // @@protoc_insertion_point(field_mutable_list:zaboom.people_position.x)
  return &x_;
}

// repeated int64 y = 2;
inline int people_position::y_size() const {
  return y_.size();
}
inline void people_position::clear_y() {
  y_.Clear();
}
inline ::google::protobuf::int64 people_position::y(int index) const {
  // @@protoc_insertion_point(field_get:zaboom.people_position.y)
  return y_.Get(index);
}
inline void people_position::set_y(int index, ::google::protobuf::int64 value) {
  y_.Set(index, value);
  // @@protoc_insertion_point(field_set:zaboom.people_position.y)
}
inline void people_position::add_y(::google::protobuf::int64 value) {
  y_.Add(value);
  // @@protoc_insertion_point(field_add:zaboom.people_position.y)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int64 >&
people_position::y() const {
  // @@protoc_insertion_point(field_list:zaboom.people_position.y)
  return y_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int64 >*
people_position::mutable_y() {
  // @@protoc_insertion_point(field_mutable_list:zaboom.people_position.y)
  return &y_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace zaboom

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_proto_2fposition_2eproto__INCLUDED
