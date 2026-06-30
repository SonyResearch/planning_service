"Rule to generate protobuf and gRPC code for enabled langauges."

load("//tools/bazel:cpp.bzl", "proto_generate_cpp")
load("//tools/bazel:python.bzl", "proto_generate_python")

CPP = "cpp"
PYTHON = "python"

def proto_generate(srcs, languages = [], generate_grpc = False, visibility = ["//visibility:public"]):
    """Given a proto library target, generate code for the specified languages.

    Args:
      srcs: Name of the proto target which was generated
      languages: Set of permissible languages
      generate_grpc: If true, generate the gRPC code as well as the protobuf code
      visibility: The visibilty assigned to the generated code
    """
    if type(srcs) != type([]) or len(srcs) != 1:
        fail(msg = "Source should be provided as a single-element list!")
    if not srcs[0].endswith("_proto"):
        fail(msg = "This macro must be invoked on a proto file!")
    _supported_languages = [CPP, PYTHON]
    for language in languages:
        if language == CPP:
            proto_generate_cpp(
                srcs = srcs,
                generate_grpc = generate_grpc,
                visibility = visibility,
            )
        elif language == PYTHON:
            proto_generate_python(
                srcs = srcs,
                generate_grpc = generate_grpc,
                visibility = visibility,
            )
        else:
            fail(msg = "Requested language '{}' is not supported! Please select from: {}".format(language, _supported_languages))
