workspace(name = "camera_lidar_lane_fusion")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# google test
git_repository(
    name = "gtest",
    remote = "https://github.com/google/googletest",
    tag = "release-1.10.0",
)

# eigen
http_archive(
    name = "eigen",
    build_file = "//third_party:eigen.BUILD",
    sha256 = "d56fbad95abf993f8af608484729e3d87ef611dd85b3380a8bad1d5cbc373a57",
    strip_prefix = "eigen-3.3.7",
    url = "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz",
)
