package(
    default_visibility = ["//visibility:private"],
)

licenses(["notice"])

cc_library(
    name = "geometry_defs",
    srcs = [
        "angle_helper.cc",
        "geometry_defs.cc",
    ],
    hdrs = [
        "angle_helper.h",
        "geometry_defs.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//common:type_traits",
    ],
)

cc_library(
    name = "epsilons",
    srcs = [
        "epsilons.cc",
    ],
    hdrs = [
        "epsilons.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//common:type_traits",
    ],
)

cc_library(
    name = "geometry_basics",
    srcs = [
        "frame.cc",
        "transform.cc",
        "unit_quaternion.cc",
        "unit_vector3d.cc",
        "vector3d.cc",
    ],
    hdrs = [
        "const_vector3d.h",
        "frame.h",
        "transform.h",
        "unit_quaternion.h",
        "unit_vector3d.h",
        "vector3d.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":epsilons",
        ":geometry_defs",
        "//common:error_or",
    ],
)

cc_library(
    name = "frame_manager",
    srcs = [
        "frame_manager.cc",
        "frame_node.cc",
    ],
    hdrs = [
        "frame_manager.h",
        "frame_node.h",
        "transform_provider.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":epsilons",
        ":geometry_basics",
        ":geometry_defs",
        "//common:error_or",
    ],
)

cc_test(
    name = "geometry_defs_test",
    srcs = [
        "geometry_defs_test.cc",
    ],
    deps = [
        ":geometry_defs",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "unit_vector3d_test",
    srcs = [
        "unit_vector3d_test.cc",
    ],
    deps = [
        ":geometry_basics",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vector3d_test",
    srcs = [
        "vector3d_test.cc",
    ],
    deps = [
        ":geometry_basics",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "unit_quaternion_test",
    srcs = [
        "unit_quaternion_test.cc",
    ],
    deps = [
        ":geometry_basics",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "transform_test",
    srcs = [
        "transform_test.cc",
    ],
    deps = [
        ":geometry_basics",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "angle_helper_test",
    srcs = [
        "angle_helper_test.cc",
    ],
    deps = [
        ":epsilons",
        ":geometry_defs",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_test",
    srcs = [
        "frame_test.cc",
    ],
    deps = [
        ":geometry_basics",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_node_test",
    srcs = [
        "frame_node_test.cc",
    ],
    deps = [
        ":frame_manager",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "frame_manager_test",
    srcs = [
        "frame_manager_test.cc",
    ],
    deps = [
        ":frame_manager",
        "@com_googletest//:gtest_main",
    ],
)
