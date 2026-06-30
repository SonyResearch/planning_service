#include "client/cpp/planning_service_client/internal/proto_base.h"
#include "client/cpp/planning_service_client/test_data/dummy.pb.h"

namespace planning_service_client {
class DummyMessage : public internal::ProtoBase<dummy_protos::DummyMessage> {
 public:
  DummyMessage() = default;
  const std::string_view field() const {
    return field_;
  }
  const std::vector<std::string>& repeated_fields() const {
    return repeated_fields_;
  }
  void set_field(const std::string_view field) {
    field_ = field.data();
  }
  auto* mutable_repeated_fields() {
    return &repeated_fields_;
  }

 private:
  dummy_protos::DummyMessage ToProtoImpl() const override {
    dummy_protos::DummyMessage msg;
    msg.set_field(field_);
    *msg.mutable_repeated_fields() = {repeated_fields_.begin(),
                                      repeated_fields_.end()};

    return msg;
  }
  void FromProtoImpl(const dummy_protos::DummyMessage& msg) override {
    field_ = msg.field();
    repeated_fields_ = {msg.repeated_fields().begin(),
                        msg.repeated_fields().end()};
  }
  std::string field_;
  std::vector<std::string> repeated_fields_;
};
}  // namespace planning_service_client
