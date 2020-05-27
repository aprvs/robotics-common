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
