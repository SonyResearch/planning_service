project(planning_service_client)

message("----------------------------------------")
message("planning_service_client CMakeLists.txt")
message("----------------------------------------")
message("CMAKE_CXX_STANDARD: " ${CMAKE_CXX_STANDARD})
message("CMAKE_CXX_STANDARD_REQUIRED: " ${CMAKE_CXX_STANDARD_REQUIRED})
message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
message("CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER})

# Protobuf
include(FindProtobuf)
find_package(Protobuf REQUIRED)
# gRPC must be located via pkg-config
find_package(PkgConfig REQUIRED)
pkg_search_module(GRPC REQUIRED grpc)
pkg_search_module(GRPCPP REQUIRED grpc++>=1.16.1)
# plugin which will generate gRPC C++ code
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
if (NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR "grpc_cpp_plugin not found!")
endif()
message("----------------------------------------")
message("Protobuf_VERSION: " ${Protobuf_VERSION})
message("Protobuf_IMPORT_DIRS: " ${Protobuf_IMPORT_DIRS})
message("Protobuf_INCLUDE_DIRS: " ${Protobuf_INCLUDE_DIRS})
message("----------------------------------------")
set (PROTO_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
# glob all .proto files together
file(GLOB PROTODEF_FILES
    ${PROTO_IMPORT_DIRS}/proto/*.proto
)
# create a library from the proto definitions
add_library(protos SHARED ${PROTODEF_FILES})
# link against protobuf and gRPC
target_link_libraries(protos PUBLIC
    protobuf::libprotobuf
    grpc
    grpc++
)
set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
# generate C++ code for Protobuf message types
protobuf_generate(
    TARGET protos
    OUT_VAR PROTO_GENERATED_FILES
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}
)
set_source_files_properties(${PROTO_GENERATED_FILES} PROPERTIES GENERATED TRUE)
# generate C++ code for gRPC methods
protobuf_generate(
    TARGET protos
    OUT_VAR GRPC_GENERATED_FILES
    LANGUAGE grpc
    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
    GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
    PLUGIN "protoc-gen-grpc=${GRPC_CPP_PLUGIN}"
    PROTOC_OUT_DIR ${PROTO_BINARY_DIR}
)
set_source_files_properties(${GRPC_GENERATED_FILES} PROPERTIES  GENERATED TRUE)

target_include_directories(protos PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")

add_library(planning_service_client SHARED ${PROTO_GENERATED_FILES} ${GRPC_GENERATED_FILES})
target_link_libraries(planning_service_client PUBLIC protos)
target_include_directories(planning_service_client PUBLIC client/cpp/include)
