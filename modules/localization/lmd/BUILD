load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lmd_localization",
    srcs = [
        "lm_bag_processor.cc",
        "lm_provider.cc",
        "lm_sampler.cc",
        "lmd_particle_filter.cc",
        "lmd_localization.cc",
        "pc_map.cc",
        "pc_registrator.cc",
    ],
    hdrs = [
        "lm_bag_processor.h",
        "lm_provider.h",
        "lm_sampler.h",
        "lmd_particle_filter.h",
        "lmd_localization.h",
        "pc_map.h",
        "pc_registrator.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:odometry_lane_marker_proto",
        "@gtest",
        "@ros//:ros_common",
    ],
)

cc_test(
    name = "lmd_localization_test",
    size = "small",
    srcs = [
        "lm_bag_processor_test.cc",
        "lm_provider_test.cc",
        "lm_sampler_test.cc",
        "lmd_localization_test.cc",
        "pc_map_test.cc",
        "pc_registrator_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/lmd:lmd_localization",
        "@gtest//:main",
    ],
)

cpplint()
