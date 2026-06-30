#include <gtest/gtest.h>

#include <future>
#include <string>
#include <vector>

#include "planning_service/common/override.h"
namespace common {

struct TestStruct {
  std::string name;
  TestStruct(const std::string_view n) : name(n.data()) {}
};
template <typename T>
class OverrideableStub : public Overrideable<T> {
 public:
  OverrideableStub() : Overrideable<T>() {}
  OverrideableStub(const T& val) : Overrideable<T>(val) {}
  void set(const T& value) {
    Overrideable<T>::set(token_, value);
  }
  void restore() {
    Overrideable<T>::restore(token_);
  }

 private:
  const uint64_t token_ {100};

  FRIEND_TEST(TestOverrideable, Ctor);
  FRIEND_TEST(TestOverrideable, Set);
  FRIEND_TEST(TestOverrideable, Restore);
  FRIEND_TEST(TestOverrideable, Clone);
  FRIEND_TEST(TestOverrideable, Overridden);
};

TEST(TestOverrideable, Ctor) {
  OverrideableStub<int> empty;
  EXPECT_FALSE(empty);
  EXPECT_FALSE(empty.has_value());
  EXPECT_THROW(empty.value(), std::runtime_error);
  EXPECT_THROW(*empty, std::runtime_error);
  auto nonempty = OverrideableStub<int>(42);
  EXPECT_TRUE(nonempty);
  EXPECT_TRUE(nonempty.has_value());
  EXPECT_NO_THROW(nonempty.value());
  EXPECT_NO_THROW(*nonempty);
  EXPECT_EQ(42, nonempty.value());
  EXPECT_EQ(42, *nonempty);
}

TEST(TestOverrideable, Set) {
  OverrideableStub<int> overrideable;
  overrideable.set(42);
  EXPECT_EQ(42, overrideable.value());
  overrideable.set(100);
  EXPECT_EQ(100, overrideable.value());
}

TEST(TestOverrideable, Overridden) {
  OverrideableStub<int> overrideable;
  EXPECT_FALSE(overrideable.overridden());
  EXPECT_NO_THROW(overrideable.set(42));
  EXPECT_FALSE(overrideable.overridden());
  EXPECT_NO_THROW(overrideable.set(100));
  EXPECT_THROW(overrideable.set(100), std::runtime_error)
      << "A value, once overridden, may not be changed.";
  overrideable.restore();
  EXPECT_FALSE(overrideable.overridden());
  EXPECT_NO_THROW(overrideable.set(100))
      << "After restoring, a value can be set again.";
}

TEST(TestOverrideable, Restore) {
  OverrideableStub<int> overrideable;
  EXPECT_NO_THROW(overrideable.restore())
      << "Restoring without setting a value should not throw.";
  overrideable.set(42);
  overrideable.set(100);
  EXPECT_EQ(100, overrideable.value());
  overrideable.restore();
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestOverrideable, Clone) {
  OverrideableStub<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  const auto clone = overrideable.clone();
  EXPECT_EQ(42, clone);
}

TEST(TestOverrideable, InitializeSetsValue) {
  Overrideable<int> overrideable;
  EXPECT_FALSE(overrideable.has_value());
  overrideable.initialize(42);
  EXPECT_TRUE(overrideable.has_value());
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestOverrideable, InitializeCanOnlyBeCalledOnce) {
  Overrideable<int> overrideable;
  overrideable.initialize(42);
  EXPECT_THROW(overrideable.initialize(100), std::runtime_error)
      << "initialize() should throw if called a second time.";
  EXPECT_EQ(42, overrideable.value())
      << "Value should remain unchanged after failed second initialize().";
}

TEST(TestOverrideable, InitializeThrowsIfValueConstructed) {
  Overrideable<int> overrideable(42);
  EXPECT_THROW(overrideable.initialize(100), std::runtime_error)
      << "initialize() should throw if the instance was value-constructed.";
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestOverrideable, InitializedValueUsableWithScopedOverride) {
  Overrideable<int> overrideable;
  overrideable.initialize(42);
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value())
      << "ScopedOverride should restore the initialized value on destruction.";
}

TEST(TestOverrideable, InitializeThrowsAfterScopedOverrideLifecycle) {
  Overrideable<int> overrideable;
  overrideable.initialize(42);
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
  }
  // After ScopedOverride restores, the initialized flag must still be set.
  EXPECT_EQ(42, overrideable.value());
  EXPECT_THROW(overrideable.initialize(200), std::runtime_error)
      << "initialize() should throw even after a full ScopedOverride "
         "lifecycle.";
}

TEST(TestScopedOverride, Ctor) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, Assign) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    EXPECT_THROW(scoped_override.acquire(overrideable), std::runtime_error)
        << "Cannot assign twice.";
  }
  EXPECT_NO_THROW(ScopedOverride<int> new_scoped_override(overrideable));
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, AssignAndSet) {
  Overrideable<TestStruct> overrideable(TestStruct("test"));
  EXPECT_EQ("test", overrideable.value().name);
  {
    ScopedOverride<TestStruct> scoped_override(overrideable);
    auto my_override = scoped_override.clone();
    my_override.name = "changed";
    scoped_override.set(my_override);
    EXPECT_EQ("changed", overrideable.value().name);
  }
  EXPECT_EQ("test", overrideable.value().name);
}

TEST(TestScopedOverride, ReleaseReassign) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    scoped_override.release();
    EXPECT_EQ(42, overrideable.value());
    EXPECT_NO_THROW(scoped_override.acquire(overrideable));
    EXPECT_EQ(42, overrideable.value());
    scoped_override.set(200);
    EXPECT_EQ(200, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, Owned) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  EXPECT_FALSE(overrideable.owned());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_TRUE(overrideable.owned());
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_FALSE(overrideable.owned());
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, OwnedDiffThread) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  EXPECT_FALSE(overrideable.owned());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_TRUE(overrideable.owned());
    EXPECT_EQ(100, overrideable.value());
    std::thread other_thread([&overrideable]() {
      EXPECT_TRUE(overrideable.owned()) << "Should be owned by another thread.";
    });
    other_thread.join();
  }
  EXPECT_FALSE(overrideable.owned());
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, AddlOwnerSameThread) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    EXPECT_THROW(auto new_override = ScopedOverride<int>(overrideable),
                 std::runtime_error)
        << "Cannot attach a second ScopedOverride instance in the same thread.";
  }
  EXPECT_EQ(42, overrideable.value());
}
TEST(TestScopedOverride, NewOwnerSameThread) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    scoped_override.release();
    EXPECT_NO_THROW(auto new_override = ScopedOverride<int>(overrideable))
        << "Should be able to attach a new ScopedOverride after release.";
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, AddlOwnerDiffThread) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    std::thread other_thread([&]() {
      EXPECT_NO_THROW(auto new_override = ScopedOverride<int>(overrideable))
          << "Should be able to attach a new ScopedOverride after release.";
    });
    scoped_override.release();
    other_thread.join();
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, AddlOwnerDiffThread_Timeout) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    std::thread other_thread([&]() {
      EXPECT_THROW(auto new_override = ScopedOverride<int>(overrideable),
                   std::runtime_error)
          << "Cannot attach a second ScopedOverride instance from another "
             "thread.";
    });
    other_thread.join();
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, DiffThread) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  std::thread other_thread([&overrideable]() {
    ScopedOverride<int> scoped_override(overrideable);
    EXPECT_NO_THROW(scoped_override.set(100));
    EXPECT_EQ(100, overrideable.value());
  });
  other_thread.join();
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, Exception) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  try {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    throw std::runtime_error("Test exception");
  } catch (const std::exception&) {
  }
  EXPECT_EQ(42, overrideable.value())
      << "Value should be restored after exception.";
}

TEST(TestScopedOverride, OnSet) {
  int value_plus_10 {0};
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override;
  scoped_override.on_event([&]() {
    value_plus_10 = overrideable.value() + 10;
  });
  scoped_override.acquire(overrideable);
  EXPECT_EQ(value_plus_10, 0);
  scoped_override.set(100);
  EXPECT_EQ(value_plus_10, 110);
}

TEST(TestScopedOverride, OnRestore) {
  int value_plus_10;
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_event([&]() {
      value_plus_10 = overrideable.value() + 10;
    });
    scoped_override.set(100);
  }
  EXPECT_EQ(value_plus_10, 52);
}

TEST(TestScopedOverride, OnSetAndRestore) {
  int value_plus_10;
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_event([&]() {
      value_plus_10 = overrideable.value() + 10;
    });
    scoped_override.set(100);
    EXPECT_EQ(value_plus_10, 110);
  }
  EXPECT_EQ(value_plus_10, 52);
}

TEST(TestScopedOverride, OnRestoreNoOp) {
  int value_plus_10 {0};
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_event([&]() {
      value_plus_10 = overrideable.value() + 10;
    });
  }
  EXPECT_EQ(value_plus_10, 0)
      << "Unset target should never trigger event function on restore.";
}

// Tests for the separate on_set() and on_restore() callback methods.
TEST(TestScopedOverride, OnSetCallbackOnly) {
  int set_count {0};
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_set([&]() {
      set_count++;
    });
    EXPECT_EQ(set_count, 0);
    scoped_override.set(100);
    EXPECT_EQ(set_count, 1)
        << "on_set callback should fire when set() is called";
  }
  EXPECT_EQ(set_count, 1) << "on_set callback should not fire on restore";
}

TEST(TestScopedOverride, OnRestoreCallbackOnly) {
  int restore_count {0};
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_restore([&]() {
      restore_count++;
    });
    scoped_override.set(100);
    EXPECT_EQ(restore_count, 0)
        << "on_restore callback should not fire when set() is called";
  }
  EXPECT_EQ(restore_count, 1)
      << "on_restore callback should fire exactly once on restore";
}

TEST(TestScopedOverride, OnRestoreCallbackNoOpIfNotSet) {
  int restore_count {0};
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_restore([&]() {
      restore_count++;
    });
    // No set() call — restore should be a no-op
  }
  EXPECT_EQ(restore_count, 0)
      << "on_restore callback should not fire if value was never set";
}

TEST(TestScopedOverride, OnSetAndRestoreIndependentCallbacks) {
  std::vector<std::string> events;
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_set([&]() {
      events.push_back("set:" + std::to_string(overrideable.value()));
    });
    scoped_override.on_restore([&]() {
      events.push_back("restore:" + std::to_string(overrideable.value()));
    });
    scoped_override.set(100);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0], "set:100");
  }
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1], "restore:42");
}

TEST(TestScopedOverride, OnSetDoesNotOverrideOnRestore) {
  // Setting on_set() should not affect an independently set on_restore().
  int set_count {0};
  int restore_count {0};
  Overrideable<int> overrideable(42);
  {
    ScopedOverride<int> scoped_override(overrideable);
    scoped_override.on_restore([&]() {
      restore_count++;
    });
    scoped_override.on_set([&]() {
      set_count++;
    });
    scoped_override.set(100);
  }
  EXPECT_EQ(set_count, 1);
  EXPECT_EQ(restore_count, 1);
}

TEST(TestScopedOverride, OnSetCallbackThrowsIfAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_set([&]() {});
  EXPECT_THROW(scoped_override.on_set([&]() {}), std::runtime_error)
      << "on_set should throw if a callback is already registered";
}

TEST(TestScopedOverride, OnRestoreCallbackThrowsIfAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_restore([&]() {});
  EXPECT_THROW(scoped_override.on_restore([&]() {}), std::runtime_error)
      << "on_restore should throw if a callback is already registered";
}

TEST(TestScopedOverride, OnEventThrowsIfOnSetAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_set([&]() {});
  EXPECT_THROW(scoped_override.on_event([&]() {}), std::runtime_error)
      << "on_event should throw if on_set is already registered";
}

TEST(TestScopedOverride, OnEventThrowsIfOnRestoreAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_restore([&]() {});
  EXPECT_THROW(scoped_override.on_event([&]() {}), std::runtime_error)
      << "on_event should throw if on_restore is already registered";
}

TEST(TestScopedOverride, OnSetThrowsIfOnEventAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_event([&]() {});
  EXPECT_THROW(scoped_override.on_set([&]() {}), std::runtime_error)
      << "on_set should throw if on_event (which sets on_set_) is already "
         "registered";
}

TEST(TestScopedOverride, OnRestoreThrowsIfOnEventAlreadySet) {
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_event([&]() {});
  EXPECT_THROW(scoped_override.on_restore([&]() {}), std::runtime_error)
      << "on_restore should throw if on_event (which sets on_restore_) is "
         "already registered";
}

TEST(TestScopedOverride, ExplicitReleaseFiresOnRestoreCallback) {
  int restore_count {0};
  Overrideable<int> overrideable(42);
  ScopedOverride<int> scoped_override(overrideable);
  scoped_override.on_restore([&]() {
    restore_count++;
  });
  scoped_override.set(100);
  EXPECT_EQ(restore_count, 0);
  scoped_override.release();
  EXPECT_EQ(restore_count, 1)
      << "on_restore callback should fire on explicit release()";
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, WaitForOwnership) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  EXPECT_FALSE(overrideable.owned());
  const int thread_wait_time_ms {100};
  std::promise<void> acquired_promise;
  auto acquired_future = acquired_promise.get_future();
  std::thread thread([&]() {
    auto override = ScopedOverride<int>(overrideable);
    acquired_promise.set_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(thread_wait_time_ms));
  });
  acquired_future.wait();
  EXPECT_TRUE(overrideable.owned()) << "Should be owned by another thread.";
  const auto start {std::chrono::steady_clock::now()};
  auto new_override = ScopedOverride<int>(overrideable);
  const auto elapsed_ms {std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start)
                             .count()};

  EXPECT_LE(std::abs(elapsed_ms - thread_wait_time_ms), 10)
      << "Should have waited for ownership.";
  EXPECT_EQ(42, overrideable.value());
  thread.join();
}
TEST(TestScopedOverride, MoveConstruct) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_TRUE(scoped_override.has_target());
    EXPECT_EQ(100, overrideable.value());
    ScopedOverride<int> moved_override(std::move(scoped_override));
    EXPECT_TRUE(moved_override.has_target());
    EXPECT_FALSE(scoped_override.has_target());
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value());
}

TEST(TestScopedOverride, MoveAssign) {
  Overrideable<int> overrideable(42);
  EXPECT_EQ(42, overrideable.value());
  {
    ScopedOverride<int> scoped_override(overrideable, 100);
    EXPECT_EQ(100, overrideable.value());
    ScopedOverride<int> moved_override;
    EXPECT_FALSE(moved_override.has_target());
    EXPECT_TRUE(scoped_override.has_target());
    moved_override = std::move(scoped_override);
    EXPECT_TRUE(moved_override.has_target());
    EXPECT_FALSE(scoped_override.has_target());
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value());
}
TEST(TestScopedOverride, MoveDiffThread) {
  Overrideable<int> overrideable(42);
  {
    EXPECT_EQ(42, overrideable.value());
    std::promise<void> promise;
    auto future = promise.get_future();
    ScopedOverride<int> scoped_override(overrideable, 100);
    std::thread thread([&]() {
      EXPECT_EQ(100, overrideable.value());
      ScopedOverride<int> moved_override;
      EXPECT_THROW(moved_override = std::move(scoped_override),
                   std::runtime_error)
          << "Cannot move ownership across threads.";
      promise.set_value();
    });
    future.wait();
    thread.join();
    EXPECT_EQ(100, overrideable.value());
  }
  EXPECT_EQ(42, overrideable.value());
}

}  // namespace common
