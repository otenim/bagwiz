// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_message_definitions.hpp"

#include <string>
#include <string_view>
#include <unordered_map>

namespace bagwiz::core
{

namespace
{

// Each entry below is the canonical ROS 1 message metadata as defined
// in the upstream sources (ros/std_msgs, ros/common_msgs,
// ros/geometry2). md5sums are the values published in those packages'
// generated headers. message_definition strings follow the standard
// ROS 1 bag v2.0 connection record convention: the top-level .msg
// text, followed by each dependency type after a line of 80 `=`
// characters and an `MSG:` line.
//
// Notes:
//   * ROS 1 `time` and `duration` are wire primitives (8 bytes each),
//     not separate types. They are NOT listed as dependencies in
//     message_definition.
//   * Some structurally identical messages share an md5sum (Point and
//     Vector3 both reduce to "float64 x\nfloat64 y\nfloat64 z"); this
//     is intentional and matches upstream behaviour.

// Common .msg snippets reused below.
constexpr std::string_view kHeaderMsg =
  "# Standard metadata for higher-level stamped data types.\n"
  "# This is generally used to communicate timestamped data \n"
  "# in a particular coordinate frame.\n"
  "# \n"
  "# sequence ID: consecutively increasing ID \n"
  "uint32 seq\n"
  "#Two-integer timestamp that is expressed as:\n"
  "# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
  "# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
  "# time-handling sugar is provided by the client library\n"
  "time stamp\n"
  "#Frame this data is associated with\n"
  "string frame_id\n";

constexpr std::string_view kPointMsg =
  "# This contains the position of a point in free space\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n";

constexpr std::string_view kVector3Msg =
  "# This represents a vector in free space. \n"
  "# It is only meant to represent a direction. Therefore, it does not\n"
  "# make sense to apply a translation to it (e.g., when applying a \n"
  "# generic rigid transformation to a Vector3, tf2 will only apply the\n"
  "# rotation). If you want your data to be translatable too, use the\n"
  "# geometry_msgs/Point message instead.\n"
  "\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n";

constexpr std::string_view kQuaternionMsg =
  "# This represents an orientation in free space in quaternion form.\n"
  "\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 w\n";

constexpr std::string_view kPoseMsg =
  "# A representation of pose in free space, composed of position and orientation. \n"
  "Point position\n"
  "Quaternion orientation\n";

constexpr std::string_view kTwistMsg =
  "# This expresses velocity in free space broken into its linear and angular parts.\n"
  "Vector3  linear\n"
  "Vector3  angular\n";

constexpr std::string_view kAccelMsg =
  "# This expresses acceleration in free space broken into its linear and angular parts.\n"
  "Vector3  linear\n"
  "Vector3  angular\n";

constexpr std::string_view kTransformMsg =
  "# This represents the transform between two coordinate frames in free space.\n"
  "\n"
  "Vector3 translation\n"
  "Quaternion rotation\n";

constexpr std::string_view kPointFieldMsg =
  "# This message holds the description of one point entry in the\n"
  "# PointCloud2 message format.\n"
  "uint8 INT8    = 1\n"
  "uint8 UINT8   = 2\n"
  "uint8 INT16   = 3\n"
  "uint8 UINT16  = 4\n"
  "uint8 INT32   = 5\n"
  "uint8 UINT32  = 6\n"
  "uint8 FLOAT32 = 7\n"
  "uint8 FLOAT64 = 8\n"
  "\n"
  "string name      # Name of field\n"
  "uint32 offset    # Offset from start of point struct\n"
  "uint8  datatype  # Datatype enumeration, see above\n"
  "uint32 count     # How many elements in the field\n";

constexpr std::string_view kSep =
  "================================================================================\n";

// Helper to compose a message_definition string at build time. Since
// std::string concatenation can't be constexpr in C++20, this is a
// runtime helper invoked once per call to find_ros1_meta(); the
// resulting strings are cached in the static map so the cost is paid
// only once per process.
std::string compose(std::initializer_list<std::string_view> parts)
{
  std::size_t total = 0;
  for (auto p : parts) {
    total += p.size();
  }
  std::string out;
  out.reserve(total);
  for (auto p : parts) {
    out.append(p);
  }
  return out;
}

const std::unordered_map<std::string, Ros1TypeMeta> & build_table()
{
  static const std::unordered_map<std::string, Ros1TypeMeta> kMap = [] {
    std::unordered_map<std::string, Ros1TypeMeta> m;

    // ---- std_msgs ----
    m.emplace("std_msgs/Bool", Ros1TypeMeta{"8b94c1b53db61fb6aed406028ad6332a", "bool data\n"});
    m.emplace(
      "std_msgs/Header", Ros1TypeMeta{"2176decaecbce78abc3b96ef049fabed", std::string(kHeaderMsg)});
    m.emplace("std_msgs/String", Ros1TypeMeta{"992ce8a1687cec8c8bd883ec73ca41d1", "string data\n"});
    m.emplace(
      "std_msgs/Float32", Ros1TypeMeta{"73fcbf46b49191e672908e50842a83d4", "float32 data\n"});
    m.emplace(
      "std_msgs/Float64", Ros1TypeMeta{"fdb28210bfa9d7c91146260178d9a584", "float64 data\n"});
    m.emplace("std_msgs/Int32", Ros1TypeMeta{"da5909fbe378aeaf85e547e830cc1bb7", "int32 data\n"});
    m.emplace("std_msgs/Int64", Ros1TypeMeta{"34add168574510e6e17f5d23ecc077ef", "int64 data\n"});
    m.emplace("std_msgs/UInt32", Ros1TypeMeta{"304a39449588c7f8ce2df6e8001c5fce", "uint32 data\n"});
    m.emplace("std_msgs/UInt64", Ros1TypeMeta{"1b2a79973e8bf53d7b53acb71299cb57", "uint64 data\n"});

    // ---- geometry_msgs (primitives) ----
    m.emplace(
      "geometry_msgs/Vector3",
      Ros1TypeMeta{"4a842b65f413084dc2b10fb484ea7f17", std::string(kVector3Msg)});
    m.emplace(
      "geometry_msgs/Point",
      Ros1TypeMeta{"4a842b65f413084dc2b10fb484ea7f17", std::string(kPointMsg)});
    m.emplace(
      "geometry_msgs/Quaternion",
      Ros1TypeMeta{"a779879fadf0160734f906b8c19c7004", std::string(kQuaternionMsg)});
    m.emplace(
      "geometry_msgs/Pose", Ros1TypeMeta{
                              "e45d45a5a1ce597b249e23fb30fc871f",
                              compose(
                                {kPoseMsg, "\n", kSep, "MSG: geometry_msgs/Point\n", kPointMsg,
                                 "\n", kSep, "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg})});
    m.emplace(
      "geometry_msgs/Transform",
      Ros1TypeMeta{
        "ac9eff44abf714214112b05d54a3cf9b",
        compose(
          {kTransformMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n", kVector3Msg, "\n", kSep,
           "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg})});
    m.emplace(
      "geometry_msgs/Twist",
      Ros1TypeMeta{
        "9f195f881246fdfa2798d1d3eebca84a",
        compose({kTwistMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n", kVector3Msg})});
    m.emplace(
      "geometry_msgs/Accel",
      Ros1TypeMeta{
        "9f195f881246fdfa2798d1d3eebca84a",
        compose({kAccelMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n", kVector3Msg})});

    // ---- geometry_msgs (stamped variants) ----
    constexpr std::string_view vec3_stamped =
      "# This represents a Vector3 with reference coordinate frame and timestamp\n"
      "Header header\n"
      "Vector3 vector\n";
    m.emplace(
      "geometry_msgs/Vector3Stamped",
      Ros1TypeMeta{
        "7b324c7325e683bf02a9b14b01090ec7",
        compose(
          {vec3_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Vector3\n", kVector3Msg})});

    constexpr std::string_view point_stamped =
      "# This represents a Point with reference coordinate frame and timestamp\n"
      "Header header\n"
      "Point point\n";
    m.emplace(
      "geometry_msgs/PointStamped",
      Ros1TypeMeta{
        "c63aecb41bfdfd6b7e1fac37c7cbe7bf",
        compose(
          {point_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Point\n", kPointMsg})});

    constexpr std::string_view quat_stamped =
      "# This represents an orientation with reference coordinate frame and timestamp.\n"
      "Header header\n"
      "Quaternion quaternion\n";
    m.emplace(
      "geometry_msgs/QuaternionStamped",
      Ros1TypeMeta{
        "e57f1e547e0e1fd13504588ffc8334e2",
        compose(
          {quat_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg})});

    constexpr std::string_view pose_stamped =
      "# A Pose with reference coordinate frame and timestamp\n"
      "Header header\n"
      "Pose pose\n";
    m.emplace(
      "geometry_msgs/PoseStamped",
      Ros1TypeMeta{
        "d3812c3cbc69362b77dc0b19b345f8f5",
        compose(
          {pose_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Pose\n", kPoseMsg, "\n", kSep, "MSG: geometry_msgs/Point\n",
           kPointMsg, "\n", kSep, "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg})});

    constexpr std::string_view transform_stamped =
      "# This expresses a transform from coordinate frame header.frame_id\n"
      "# to the coordinate frame child_frame_id\n"
      "#\n"
      "# This message is mostly used by the \n"
      "# <a href=\"http://wiki.ros.org/tf\">tf</a> package. \n"
      "# See its documentation for more information.\n"
      "\n"
      "Header header\n"
      "string child_frame_id # the frame id of the child frame\n"
      "Transform transform\n";
    m.emplace(
      "geometry_msgs/TransformStamped",
      Ros1TypeMeta{
        "b5764a33bfeb3588febc2682852579b0",
        compose(
          {transform_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Transform\n", kTransformMsg, "\n", kSep,
           "MSG: geometry_msgs/Vector3\n", kVector3Msg, "\n", kSep,
           "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg})});

    constexpr std::string_view twist_stamped =
      "# A twist with reference coordinate frame and timestamp\n"
      "Header header\n"
      "Twist twist\n";
    m.emplace(
      "geometry_msgs/TwistStamped",
      Ros1TypeMeta{
        "98d34b0043a2093cf9d9345ab6eef12e",
        compose(
          {twist_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Twist\n", kTwistMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n",
           kVector3Msg})});

    constexpr std::string_view accel_stamped =
      "# An accel with reference coordinate frame and timestamp\n"
      "Header header\n"
      "Accel accel\n";
    m.emplace(
      "geometry_msgs/AccelStamped",
      Ros1TypeMeta{
        "d8a98a5d81351b6eb0578c78557e7659",
        compose(
          {accel_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Accel\n", kAccelMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n",
           kVector3Msg})});

    // ---- geometry_msgs (with-covariance variants) ----
    constexpr std::string_view pose_wc =
      "# This represents a pose in free space with uncertainty.\n"
      "\n"
      "Pose pose\n"
      "\n"
      "# Row-major representation of the 6x6 covariance matrix\n"
      "# The orientation parameters use a fixed-axis representation.\n"
      "# In order, the parameters are:\n"
      "# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
      "float64[36] covariance\n";
    m.emplace(
      "geometry_msgs/PoseWithCovariance",
      Ros1TypeMeta{
        "c23e848cf1b7533a8d7c259073a97e6f",
        compose(
          {pose_wc, "\n", kSep, "MSG: geometry_msgs/Pose\n", kPoseMsg, "\n", kSep,
           "MSG: geometry_msgs/Point\n", kPointMsg, "\n", kSep, "MSG: geometry_msgs/Quaternion\n",
           kQuaternionMsg})});

    constexpr std::string_view pose_wc_stamped =
      "# This expresses an estimated pose with a reference coordinate frame and timestamp\n"
      "Header header\n"
      "PoseWithCovariance pose\n";
    m.emplace(
      "geometry_msgs/PoseWithCovarianceStamped",
      Ros1TypeMeta{
        "953b798c0f514ff060a53a3498ce6246",
        compose({pose_wc_stamped, "\n", kSep, "MSG: std_msgs/Header\n",
                 kHeaderMsg,      "\n", kSep, "MSG: geometry_msgs/PoseWithCovariance\n",
                 pose_wc,         "\n", kSep, "MSG: geometry_msgs/Pose\n",
                 kPoseMsg,        "\n", kSep, "MSG: geometry_msgs/Point\n",
                 kPointMsg,       "\n", kSep, "MSG: geometry_msgs/Quaternion\n",
                 kQuaternionMsg})});

    constexpr std::string_view twist_wc =
      "# This expresses velocity in free space with uncertainty.\n"
      "\n"
      "Twist twist\n"
      "\n"
      "# Row-major representation of the 6x6 covariance matrix\n"
      "# The orientation parameters use a fixed-axis representation.\n"
      "# In order, the parameters are:\n"
      "# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
      "float64[36] covariance\n";
    m.emplace(
      "geometry_msgs/TwistWithCovariance",
      Ros1TypeMeta{
        "1fe8a28e6890a4cc3ae4c3ca5c7d82e6",
        compose(
          {twist_wc, "\n", kSep, "MSG: geometry_msgs/Twist\n", kTwistMsg, "\n", kSep,
           "MSG: geometry_msgs/Vector3\n", kVector3Msg})});

    constexpr std::string_view twist_wc_stamped =
      "# This represents an estimated twist with reference coordinate frame and timestamp.\n"
      "Header header\n"
      "TwistWithCovariance twist\n";
    m.emplace(
      "geometry_msgs/TwistWithCovarianceStamped",
      Ros1TypeMeta{
        "8927a1a12fb2607ceea095b2dc440a96",
        compose(
          {twist_wc_stamped, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/TwistWithCovariance\n", twist_wc, "\n", kSep,
           "MSG: geometry_msgs/Twist\n", kTwistMsg, "\n", kSep, "MSG: geometry_msgs/Vector3\n",
           kVector3Msg})});

    // ---- tf ----
    constexpr std::string_view tf_msg = "geometry_msgs/TransformStamped[] transforms\n";
    constexpr std::string_view tf_msg_def_tail_a = "\n";
    const std::string tf_def = compose(
      {tf_msg,
       "\n",
       kSep,
       "MSG: geometry_msgs/TransformStamped\n",
       "# This expresses a transform from coordinate frame header.frame_id\n"
       "# to the coordinate frame child_frame_id\n"
       "#\n"
       "# This message is mostly used by the \n"
       "# <a href=\"http://wiki.ros.org/tf\">tf</a> package. \n"
       "# See its documentation for more information.\n"
       "\n"
       "Header header\n"
       "string child_frame_id # the frame id of the child frame\n"
       "Transform transform\n",
       "\n",
       kSep,
       "MSG: std_msgs/Header\n",
       kHeaderMsg,
       "\n",
       kSep,
       "MSG: geometry_msgs/Transform\n",
       kTransformMsg,
       "\n",
       kSep,
       "MSG: geometry_msgs/Vector3\n",
       kVector3Msg,
       "\n",
       kSep,
       "MSG: geometry_msgs/Quaternion\n",
       kQuaternionMsg});
    m.emplace("tf2_msgs/TFMessage", Ros1TypeMeta{"94810edda583a504dfda3829e70d7eec", tf_def});
    // Legacy ROS 1 alias: same wire format, same md5, same definition.
    m.emplace("tf/tfMessage", Ros1TypeMeta{"94810edda583a504dfda3829e70d7eec", tf_def});
    (void)tf_msg_def_tail_a;  // unused holder, kept to avoid future merge churn

    // ---- nav_msgs ----
    constexpr std::string_view odom_msg =
      "# This represents an estimate of a position and velocity in free space.  \n"
      "# The pose in this message should be specified in the coordinate frame given by "
      "header.frame_id.\n"
      "# The twist in this message should be specified in the coordinate frame given by the "
      "child_frame_id\n"
      "Header header\n"
      "string child_frame_id\n"
      "geometry_msgs/PoseWithCovariance pose\n"
      "geometry_msgs/TwistWithCovariance twist\n";
    m.emplace(
      "nav_msgs/Odometry",
      Ros1TypeMeta{
        "cd5e73d190d741a2f92e81eda573aca7",
        compose({odom_msg,       "\n", kSep, "MSG: std_msgs/Header\n",
                 kHeaderMsg,     "\n", kSep, "MSG: geometry_msgs/PoseWithCovariance\n",
                 pose_wc,        "\n", kSep, "MSG: geometry_msgs/Pose\n",
                 kPoseMsg,       "\n", kSep, "MSG: geometry_msgs/Point\n",
                 kPointMsg,      "\n", kSep, "MSG: geometry_msgs/Quaternion\n",
                 kQuaternionMsg, "\n", kSep, "MSG: geometry_msgs/TwistWithCovariance\n",
                 twist_wc,       "\n", kSep, "MSG: geometry_msgs/Twist\n",
                 kTwistMsg,      "\n", kSep, "MSG: geometry_msgs/Vector3\n",
                 kVector3Msg})});

    constexpr std::string_view path_msg =
      "#An array of poses that represents a Path for a robot to follow\n"
      "Header header\n"
      "geometry_msgs/PoseStamped[] poses\n";
    m.emplace(
      "nav_msgs/Path", Ros1TypeMeta{
                         "6227e2b7e9cce15051f669a5e197bbf7",
                         compose({path_msg,      "\n", kSep, "MSG: std_msgs/Header\n",
                                  kHeaderMsg,    "\n", kSep, "MSG: geometry_msgs/PoseStamped\n",
                                  pose_stamped,  "\n", kSep, "MSG: geometry_msgs/Pose\n",
                                  kPoseMsg,      "\n", kSep, "MSG: geometry_msgs/Point\n",
                                  kPointMsg,     "\n", kSep, "MSG: geometry_msgs/Quaternion\n",
                                  kQuaternionMsg})});

    // ---- sensor_msgs ----
    constexpr std::string_view imu_msg =
      "# This is a message to hold data from an IMU (Inertial Measurement Unit)\n"
      "#\n"
      "# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in "
      "rad/sec\n"
      "#\n"
      "# If the covariance of the measurement is known, it should be filled in (if all you know is "
      "the \n"
      "# variance of each measurement, e.g. from the datasheet, just put those along the "
      "diagonal)\n"
      "# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to "
      "use the\n"
      "# data a covariance will have to be assumed or gotten from some other source\n"
      "#\n"
      "# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an "
      "orientation \n"
      "# estimate), please set element 0 of the associated covariance matrix to -1\n"
      "# If you are interpreting this message, please check for a value of -1 in the first element "
      "of each \n"
      "# covariance matrix, and disregard the associated estimate.\n"
      "\n"
      "Header header\n"
      "\n"
      "geometry_msgs/Quaternion orientation\n"
      "float64[9] orientation_covariance # Row major about x, y, z axes\n"
      "\n"
      "geometry_msgs/Vector3 angular_velocity\n"
      "float64[9] angular_velocity_covariance # Row major about x, y, z axes\n"
      "\n"
      "geometry_msgs/Vector3 linear_acceleration\n"
      "float64[9] linear_acceleration_covariance # Row major x, y z \n";
    m.emplace(
      "sensor_msgs/Imu", Ros1TypeMeta{
                           "6a62c6daae103f4ff57a132d6f95cec2",
                           compose(
                             {imu_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
                              "MSG: geometry_msgs/Quaternion\n", kQuaternionMsg, "\n", kSep,
                              "MSG: geometry_msgs/Vector3\n", kVector3Msg})});

    constexpr std::string_view image_msg =
      "# This message contains an uncompressed image\n"
      "# (0, 0) is at top-left corner of image\n"
      "#\n"
      "\n"
      "Header header        # Header timestamp should be acquisition time of image\n"
      "                     # Header frame_id should be optical frame of camera\n"
      "                     # origin of frame should be optical center of camera\n"
      "                     # +x should point to the right in the image\n"
      "                     # +y should point down in the image\n"
      "                     # +z should point into to plane of the image\n"
      "                     # If the frame_id here and the frame_id of the CameraInfo\n"
      "                     # message associated with the image conflict\n"
      "                     # the behavior is undefined\n"
      "\n"
      "uint32 height         # image height, that is, number of rows\n"
      "uint32 width          # image width, that is, number of columns\n"
      "\n"
      "# The legal values for encoding are in file src/image_encodings.cpp\n"
      "# If you want to standardize a new string format, join\n"
      "# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
      "\n"
      "string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
      "                      # taken from the list of strings in "
      "include/sensor_msgs/image_encodings.h\n"
      "\n"
      "uint8 is_bigendian    # is this data bigendian?\n"
      "uint32 step           # Full row length in bytes\n"
      "uint8[] data          # actual matrix data, size is (step * rows)\n";
    m.emplace(
      "sensor_msgs/Image",
      Ros1TypeMeta{
        "060021388200f6f0f447d0fcd9c64743",
        compose({image_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view compressed_image_msg =
      "# This message contains a compressed image\n"
      "\n"
      "Header header        # Header timestamp should be acquisition time of image\n"
      "                     # Header frame_id should be optical frame of camera\n"
      "                     # origin of frame should be optical center of camera\n"
      "                     # +x should point to the right in the image\n"
      "                     # +y should point down in the image\n"
      "                     # +z should point into to plane of the image\n"
      "\n"
      "string format        # Specifies the format of the data\n"
      "                     #   Acceptable values:\n"
      "                     #     jpeg, png\n"
      "uint8[] data         # Compressed image buffer\n";
    m.emplace(
      "sensor_msgs/CompressedImage",
      Ros1TypeMeta{
        "8f7a12909da2c9d3332d540a0977563f",
        compose({compressed_image_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view camera_info_msg =
      "# This message defines meta information for a camera. It should be in a\n"
      "# camera namespace on topic \"camera_info\" and accompanied by up to five\n"
      "# image topics named:\n"
      "#\n"
      "#   image_raw - raw data from the camera driver, possibly Bayer encoded\n"
      "#   image            - monochrome, distorted\n"
      "#   image_color      - color, distorted\n"
      "#   image_rect       - monochrome, rectified\n"
      "#   image_rect_color - color, rectified\n"
      "\n"
      "Header header    # Header timestamp should be acquisition time of image\n"
      "uint32 height\n"
      "uint32 width\n"
      "string distortion_model\n"
      "float64[] D\n"
      "float64[9]  K # 3x3 row-major matrix\n"
      "float64[9]  R # 3x3 row-major matrix\n"
      "float64[12] P # 3x4 row-major matrix\n"
      "uint32 binning_x\n"
      "uint32 binning_y\n"
      "RegionOfInterest roi\n";
    constexpr std::string_view region_of_interest_msg =
      "# This message is used to specify a region of interest within an image.\n"
      "uint32 x_offset\n"
      "uint32 y_offset\n"
      "uint32 height\n"
      "uint32 width\n"
      "bool do_rectify\n";
    m.emplace(
      "sensor_msgs/CameraInfo",
      Ros1TypeMeta{
        "c9a58c1b0b154e0e6da7578cb991d214",
        compose(
          {camera_info_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: sensor_msgs/RegionOfInterest\n", region_of_interest_msg})});

    constexpr std::string_view pointcloud2_msg =
      "# This message holds a collection of N-dimensional points, which may\n"
      "# contain additional information such as normals, intensity, etc. The\n"
      "# point data is stored as a binary blob, its layout described by the\n"
      "# contents of the \"fields\" array.\n"
      "\n"
      "Header header\n"
      "\n"
      "uint32 height\n"
      "uint32 width\n"
      "\n"
      "PointField[] fields\n"
      "\n"
      "bool    is_bigendian\n"
      "uint32  point_step\n"
      "uint32  row_step\n"
      "uint8[] data\n"
      "\n"
      "bool is_dense\n";
    m.emplace(
      "sensor_msgs/PointCloud2",
      Ros1TypeMeta{
        "1158d486dd51d683ce2f1be655c3c181",
        compose(
          {pointcloud2_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: sensor_msgs/PointField\n", kPointFieldMsg})});

    m.emplace(
      "sensor_msgs/PointField",
      Ros1TypeMeta{"268eacb2962780ceac86cbd17e328150", std::string(kPointFieldMsg)});

    constexpr std::string_view nav_sat_status_msg =
      "# Navigation Satellite fix status for any Global Navigation Satellite System.\n"
      "\n"
      "int8 STATUS_NO_FIX =  -1        # unable to fix position\n"
      "int8 STATUS_FIX =      0        # unaugmented fix\n"
      "int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\n"
      "int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n"
      "\n"
      "int8 status\n"
      "\n"
      "uint16 SERVICE_GPS =     1\n"
      "uint16 SERVICE_GLONASS = 2\n"
      "uint16 SERVICE_COMPASS = 4\n"
      "uint16 SERVICE_GALILEO = 8\n"
      "\n"
      "uint16 service\n";
    m.emplace(
      "sensor_msgs/NavSatStatus",
      Ros1TypeMeta{"331cdbddfa4bc96ffc3b9ad98900a54c", std::string(nav_sat_status_msg)});

    constexpr std::string_view nav_sat_fix_msg =
      "# Navigation Satellite fix for any Global Navigation Satellite System\n"
      "\n"
      "Header header\n"
      "\n"
      "NavSatStatus status\n"
      "\n"
      "float64 latitude\n"
      "float64 longitude\n"
      "float64 altitude\n"
      "\n"
      "float64[9] position_covariance\n"
      "\n"
      "uint8 COVARIANCE_TYPE_UNKNOWN = 0\n"
      "uint8 COVARIANCE_TYPE_APPROXIMATED = 1\n"
      "uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\n"
      "uint8 COVARIANCE_TYPE_KNOWN = 3\n"
      "\n"
      "uint8 position_covariance_type\n";
    m.emplace(
      "sensor_msgs/NavSatFix",
      Ros1TypeMeta{
        "2d3a8cd499b9b4a0249fb98fd05cfa48",
        compose(
          {nav_sat_fix_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: sensor_msgs/NavSatStatus\n", nav_sat_status_msg})});

    constexpr std::string_view laser_scan_msg =
      "# Single scan from a planar laser range-finder\n"
      "\n"
      "Header header            # timestamp in the header is the acquisition time of \n"
      "                         # the first ray in the scan.\n"
      "\n"
      "float32 angle_min        # start angle of the scan [rad]\n"
      "float32 angle_max        # end angle of the scan [rad]\n"
      "float32 angle_increment  # angular distance between measurements [rad]\n"
      "\n"
      "float32 time_increment   # time between measurements [seconds]\n"
      "float32 scan_time        # time between scans [seconds]\n"
      "\n"
      "float32 range_min        # minimum range value [m]\n"
      "float32 range_max        # maximum range value [m]\n"
      "\n"
      "float32[] ranges         # range data [m]\n"
      "float32[] intensities    # intensity data [device-specific units]\n";
    m.emplace(
      "sensor_msgs/LaserScan",
      Ros1TypeMeta{
        "90c7ef2dc6895d81024acba2ac42f369",
        compose({laser_scan_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view range_msg =
      "# Single range reading from an active ranger that emits energy and reports\n"
      "# one range reading that is valid along an arc at the distance measured.\n"
      "\n"
      "Header header           # timestamp in the header is the time the range was measured\n"
      "                        # frame_id in the header is the location of the range sensor\n"
      "\n"
      "uint8 ULTRASOUND=0\n"
      "uint8 INFRARED=1\n"
      "\n"
      "uint8 radiation_type    # the type of radiation used by the sensor\n"
      "                        # (sound, IR, etc) [enum]\n"
      "\n"
      "float32 field_of_view   # the size of the arc that the distance reading is\n"
      "                        # valid for [rad]\n"
      "\n"
      "float32 min_range       # minimum range value [m]\n"
      "float32 max_range       # maximum range value [m]\n"
      "\n"
      "float32 range           # range data [m]\n";
    m.emplace(
      "sensor_msgs/Range",
      Ros1TypeMeta{
        "c005c34273dc426c67a020a87bc24148",
        compose({range_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view temperature_msg =
      " # Single temperature reading.\n"
      "\n"
      "Header header           # timestamp is the time the temperature was measured\n"
      "                        # frame_id is the location of the temperature reading\n"
      "\n"
      "float64 temperature     # Measurement of the Temperature in Degrees Celsius\n"
      "\n"
      "float64 variance        # 0 is interpreted as variance unknown\n";
    m.emplace(
      "sensor_msgs/Temperature",
      Ros1TypeMeta{
        "ff71b307acdbe7c871a5a6cd1fd4c8f9",
        compose({temperature_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view fluid_pressure_msg =
      " # Single pressure reading.  This message is appropriate for measuring the\n"
      " # pressure inside of a fluid (air, water, etc).  This also includes\n"
      " # atmospheric or barometric pressure.\n"
      "\n"
      "Header header           # timestamp of the measurement\n"
      "                        # frame_id is the location of the pressure sensor\n"
      "\n"
      "float64 fluid_pressure  # Absolute pressure reading in Pascals.\n"
      "\n"
      "float64 variance        # 0 is interpreted as variance unknown\n";
    m.emplace(
      "sensor_msgs/FluidPressure",
      Ros1TypeMeta{
        "804dc5cea1c5306d6a2eb80b9833befe",
        compose({fluid_pressure_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg})});

    constexpr std::string_view magnetic_field_msg =
      " # Measurement of the Magnetic Field vector at a specific location.\n"
      "\n"
      " # If the covariance of the measurement is known, it should be filled in.\n"
      " # If all you know is the variance of each measurement, e.g. from the datasheet,\n"
      " # just put those along the diagonal.\n"
      " # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",\n"
      " # and to use the data a covariance will have to be assumed or gotten from some\n"
      " # other source.\n"
      "\n"
      "Header header                        # timestamp is the time the\n"
      "                                     # field was measured\n"
      "                                     # frame_id is the location and orientation\n"
      "                                     # of the field measurement\n"
      "\n"
      "geometry_msgs/Vector3 magnetic_field # x, y, and z components of the\n"
      "                                     # field vector in Tesla\n"
      "                                     # If your sensor does not output 3 axes,\n"
      "                                     # put NaNs in the components not reported.\n"
      "\n"
      "float64[9] magnetic_field_covariance # Row major about x, y, z axes\n"
      "                                     # 0 is interpreted as variance unknown\n";
    m.emplace(
      "sensor_msgs/MagneticField",
      Ros1TypeMeta{
        "2f3b0b43eed0c9501de0fa3ff89a45aa",
        compose(
          {magnetic_field_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: geometry_msgs/Vector3\n", kVector3Msg})});

    // ---- diagnostic_msgs ----
    constexpr std::string_view key_value_msg =
      "string key # what to label this value when viewing\n"
      "string value # a value to track over time\n";
    m.emplace(
      "diagnostic_msgs/KeyValue",
      Ros1TypeMeta{"cf57fdc6617a881a88c16e768132149c", std::string(key_value_msg)});

    constexpr std::string_view diagnostic_status_msg =
      "# This message holds the status of an individual component of the robot.\n"
      "# \n"
      "\n"
      "# Possible levels of operations\n"
      "byte OK=0\n"
      "byte WARN=1\n"
      "byte ERROR=2\n"
      "byte STALE=3\n"
      "\n"
      "byte level # level of operation enumerated above \n"
      "string name # a description of the test/component reporting\n"
      "string message # a description of the status\n"
      "string hardware_id # a hardware unique string\n"
      "KeyValue[] values # an array of values associated with the status\n";
    m.emplace(
      "diagnostic_msgs/DiagnosticStatus",
      Ros1TypeMeta{
        "67c8e9c47e2876d3b1e5dac0fdf3a39c",
        compose(
          {diagnostic_status_msg, "\n", kSep, "MSG: diagnostic_msgs/KeyValue\n", key_value_msg})});

    constexpr std::string_view diagnostic_array_msg =
      "# This message is used to send diagnostic information about the state of the robot\n"
      "Header header #for timestamp\n"
      "DiagnosticStatus[] status # an array of components being reported on\n";
    m.emplace(
      "diagnostic_msgs/DiagnosticArray",
      Ros1TypeMeta{
        "60810da900de1dd6ddd437c3503511da",
        compose(
          {diagnostic_array_msg, "\n", kSep, "MSG: std_msgs/Header\n", kHeaderMsg, "\n", kSep,
           "MSG: diagnostic_msgs/DiagnosticStatus\n", diagnostic_status_msg, "\n", kSep,
           "MSG: diagnostic_msgs/KeyValue\n", key_value_msg})});

    return m;
  }();
  return kMap;
}

}  // namespace

const Ros1TypeMeta * find_ros1_meta(std::string_view ros1_type)
{
  const auto & m = build_table();
  auto it = m.find(std::string(ros1_type));
  if (it == m.end()) {
    return nullptr;
  }
  return &it->second;
}

}  // namespace bagwiz::core
