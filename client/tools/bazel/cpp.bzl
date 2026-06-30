"Protobuf/gRPC rules specific to C++."

load("@grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_test")

def proto_generate_cpp(srcs, generate_grpc = False, visibility = ["//visibility:public"]):
    """Compile the language-specific libraries for C++.

    Args:
      srcs: Proto target.
      generate_grpc: If true, generate the gRPC code as well as the protobuf code
      visibility: The visibilty assigned to the generated code
    """
    proto_name = srcs[0].lstrip(":").removesuffix("_proto") + "_cpp_proto"
    cc_proto_library(
        name = proto_name,
        deps = srcs,
        visibility = visibility,
        tags = ["proto", "cpp"],
    )
    if generate_grpc:
        grpc_name = proto_name.replace("cpp_proto", "cpp_grpc")
        cc_grpc_library(
            name = grpc_name,
            srcs = srcs,
            deps = [proto_name],
            visibility = visibility,
            generate_mocks = True,
            grpc_only = True,
            tags = ["grpc", "cpp"],
        )

def cc_googletest(name, srcs, deps = [], **kwargs):
    """Create a C++ test using GoogleTest. Simple wrapper for DRYness.

    Args:
            name: Name of the test target.
            srcs: Source files for the test.
            deps: Dependencies for the test.
            **kwargs: Additional keyword arguments for the cc_test rule.
    """

    # Use default 'short' timeout unless specified via kwargs
    cc_test(
        name = name,
        srcs = srcs,
        deps = deps + ["@googletest//:gtest_main"],
        timeout = kwargs.pop("timeout", "short"),
        **kwargs
    )
