load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_raw_predictor_base",
    deps = [
        "//modules/localization/lmd/predictor:localization_lmd_predictor",
    ],
)

cc_library(
    name = "localization_lmd_raw_predictor_imu",
    srcs = [
        "predictor_imu.cc",
    ],
    hdrs = [
        "predictor_imu.h",
    ],
    deps = [
        ":localization_lmd_raw_predictor_base",
        "//modules/common:log",
        "//modules/localization/proto:imu_proto",
    ],
)

cc_library(
    name = "localization_lmd_raw_predictor_gps",
    srcs = [
        "predictor_gps.cc",
    ],
    hdrs = [
        "predictor_gps.h",
    ],
    deps = [
        ":localization_lmd_raw_predictor_base",
        "//modules/common:log",
        "//modules/localization/proto:gps_proto",
    ],
)

cpplint()
