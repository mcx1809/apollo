load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_common",
    hdrs = [
        "tm_list.h",
    ],
)

cc_test(
    name = "localization_lmd_common_test",
    size = "small",
    srcs = [
        "tm_list_test.cc",
    ],
    deps = [
        "//modules/localization/lmd/common:localization_lmd_common",
        "@gtest//:main",
    ],
)

cpplint()
