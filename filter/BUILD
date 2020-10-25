package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.h"],
    hdrs = ["kalman_filter.h"],
)

cc_test(
    name = "test_kalman_filter",
    srcs = [
        "test/test_kalman_filter.cc",
    ],
    deps = [
        ":kalman_filter",
        "@eigen",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)
