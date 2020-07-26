package(
    default_visibility = ["//visibility:private"],
)

licenses(["notice"])

cc_library(
    name = "link",
    hdrs = [
        "link.h",
    ],
    srcs = [
    ],
    deps = [
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "joint",
    hdrs = [
        "joint.h",
    ],
    srcs = [
    ],
    deps = [
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "robot",
    hdrs = [
        "robot.h",
    ],
    srcs = [
        "robot.cc",
    ],
    deps = [
        "//common:error_or",
        "//robotics-common/math:frame",
        "//robotics-common/math:geometry_basics",
        ":link",
        ":joint",
    ],
    visibility = ["//visibility:public"],
)
