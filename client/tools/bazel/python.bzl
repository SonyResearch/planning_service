"Protobuf/gRPC rules specific to Python."

load("@grpc//bazel:python_rules.bzl", "py_grpc_library", "py_proto_library")
load("@planning_service_client_dependencies//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

def proto_generate_python(srcs, generate_grpc = False, visibility = ["//visibility:public"]):
    """Compile the language-specific libraries for Python.

    Args:
      srcs: Proto target.
      generate_grpc: If true, generate the gRPC code as well as the protobuf code
      visibility: The visibilty assigned to the generated code
    """
    proto_name = srcs[0].lstrip(":").removesuffix("_proto") + "_py_pb2"
    py_proto_library(
        name = proto_name,
        deps = srcs,
        visibility = visibility,
        tags = ["proto", "python"],
    )

    if generate_grpc:
        grpc_name = proto_name.replace("_py_pb2", "_py_pb2_grpc")
        py_grpc_library(
            name = grpc_name,
            srcs = srcs,
            deps = [proto_name],
            visibility = visibility,
            tags = ["grpc", "python"],
        )

def py_pytest(name, srcs, deps = [], args = [], data = [], size = "small", **kwargs):
    # Use Label() so the wrapper resolves in this repo (@planning_service_client)
    # rather than in whichever repo calls this macro.
    _wrapper = Label("//tools/bazel:pytest_wrapper.py")
    py_test(
        name = name,
        srcs = [_wrapper] + srcs,
        main = _wrapper,
        args = args + ["$(location :%s)" % x for x in srcs],
        deps = deps + [
            requirement("pytest"),
            requirement("pytest-timeout"),
        ],
        size = size,
        data = data,
        **kwargs
    )
