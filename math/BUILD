package(
    default_visibility = ["//visibility:private"],
)

licenses(["notice"])

cc_library(
    name = "geometry_defs",
    hdrs = [
        "geometry_defs.h",
    ],
    srcs = [
        "geometry_defs.cc",
    ],
    deps = [
        "//common:type_traits",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "epsilons",
    hdrs = [
        "epsilons.h",
    ],
    srcs = [
        "epsilons.cc",
    ],
    deps = [
        "//common:type_traits",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vector",
    hdrs = [
        "const_vector3d.h",
        "unit_vector3d.h",
        "vector3d.h",
    ],
    srcs = [
        "unit_vector3d.cc",
        "vector3d.cc",
    ],
    deps = [
        ":epsilons",
        ":geometry_defs",
        "//common:error_or",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "quaternion",
    hdrs = [
        "unit_quaternion.h",
    ],
    srcs = [
        "unit_quaternion.cc",
    ],
    deps = [
        ":epsilons",
        ":geometry_defs",
        "//common:error_or",
    ],
    visibility = ["//visibility:public"],
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
        ":vector",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vector3d_test",
    srcs = [
        "vector3d_test.cc",
    ],
    deps = [
        ":vector",
        "@com_googletest//:gtest_main",
    ],
)

cc_test(
    name = "unit_quaternion_test",
    srcs = [
        "unit_quaternion_test.cc",
    ],
    deps = [
        ":quaternion",
        "@com_googletest//:gtest_main",
    ],
)
