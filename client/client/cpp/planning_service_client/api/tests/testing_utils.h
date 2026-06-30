#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>

namespace client {
template <typename TClient, typename TService>
class ClientTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SetAddress();
    channel_ =
        grpc::CreateChannel(address_, grpc::InsecureChannelCredentials());
    SetService();
    SetStub();
    if (client_ == nullptr || service_ == nullptr) {
      throw std::runtime_error("Test fixture not properly initialized!");
    }
  }
  /** Return the port address for the given server. */
  void SetAddress() {
    int address_base {5000};  // only listen on ports 5XXX
    address_ = "localhost:" + std::to_string(address_base + std::rand() % 1000);
  }
  /** Set the service. */
  virtual void SetService() = 0;
  /** Set the client. */
  virtual void SetClient() = 0;

  std::shared_ptr<grpc::Channel> channel_ {};
  std::unique_ptr<TClient> client_ {};
  std::unique_ptr<TService> service_ {};
  std::string address_ {""};
};
}  // namespace client
