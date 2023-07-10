cmake_minimum_required(VERSION 2.8.3)
project(stamped_msgs)

set(THIS_PACKAGE_DEPS
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_DEPS} message_generation)

add_message_files(
 FILES
  BoolArray.msg
  Bool.msg
  ByteArray.msg
  Byte.msg
  ByteMultiArray.msg
  ColorRGBAArray.msg
  ColorRGBA.msg
  DurationArray.msg
  Duration.msg
  Empty.msg
  Float32Array.msg
  Float32.msg
  Float32MultiArray.msg
  Float64Array.msg
  Float64.msg
  Float64MultiArray.msg
  CharArray.msg
  Char.msg
  Int16Array.msg
  Int16.msg
  Int16MultiArray.msg
  Int32Array.msg
  Int32.msg
  Int32MultiArray.msg
  Int64Array.msg
  Int64.msg
  Int64MultiArray.msg
  Int8Array.msg
  Int8.msg
  Int8MultiArray.msg
  StringArray.msg
  String.msg
  UInt16Array.msg
  UInt16.msg
  UInt16MultiArray.msg
  UInt32Array.msg
  UInt32.msg
  UInt32MultiArray.msg
  UInt64Array.msg
  UInt64.msg
  UInt64MultiArray.msg
  UInt8Array.msg
  UInt8.msg
  UInt8MultiArray.msg
)

generate_messages(DEPENDENCIES ${THIS_PACKAGE_DEPS})

catkin_package(
  CATKIN_DEPENDS message_runtime ${THIS_PACKAGE_DEPS}
)
