#ifndef MINKINDR_CONVERSIONS_KINDR_MSG_H
#define MINKINDR_CONVERSIONS_KINDR_MSG_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>

namespace tf {

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void quaternionKindrToMsg(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& kindr,
    geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr.toImplementation(), *msg);
}

template <typename Scalar>
void quaternionMsgToKindr(
    const geometry_msgs::Quaternion& msg,
    kindr::minimal::RotationQuaternionTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  quaternionMsgToEigen(msg, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}

// Also the Eigen implementation version of this.
template <typename Scalar>
void quaternionKindrToMsg(const Eigen::Quaternion<Scalar>& kindr,
                          geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void quaternionMsgToKindr(const geometry_msgs::Quaternion& msg,
                          Eigen::Quaternion<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<double> kindr_double;
  quaternionMsgToEigen(msg, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void pointKindrToMsg(const Eigen::Matrix<Scalar, 3, 1>& kindr,
                     geometry_msgs::Point* msg) {
  CHECK_NOTNULL(msg);
  pointEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void pointMsgToKindr(const geometry_msgs::Point& msg,
                     Eigen::Matrix<Scalar, 3, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  pointMsgToEigen(msg, *kindr);
}

template <typename Scalar>
void vectorKindrToMsg(const Eigen::Matrix<Scalar, 3, 1>& kindr,
                      geometry_msgs::Vector3* msg) {
  CHECK_NOTNULL(msg);
  vectorEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void vectorMsgToKindr(const geometry_msgs::Vector3& msg,
                      Eigen::Matrix<Scalar, 3, 1>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<double, 3, 1> kindr_double;
  vectorMsgToEigen(msg, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
template <typename Scalar>
void poseKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::Pose* msg) {
  CHECK_NOTNULL(msg);
  pointKindrToMsg(kindr.getPosition(), &msg->position);
  quaternionKindrToMsg(kindr.getRotation(), &msg->orientation);
}

template <typename Scalar>
void poseMsgToKindr(const geometry_msgs::Pose& msg,
                    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionMsgToKindr(msg.orientation, &rotation);
  pointMsgToKindr(msg.position, &position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

template <typename Scalar>
void poseStampedKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    const ros::Time& time, const std::string& reference_frame,
    geometry_msgs::PoseStamped* msg) {
  CHECK_NOTNULL(msg);
  msg->header.frame_id = reference_frame;
  msg->header.stamp = time;
  poseKindrToMsg(kindr, &msg->pose);
}

// Uses current time.
template <typename Scalar>
void poseStampedKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    const std::string& reference_frame, geometry_msgs::PoseStamped* msg) {
  poseStampedKindrToMsg(kindr, ros::Time(), reference_frame, msg);
}

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
template <typename Scalar>
void transformKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::Transform* msg) {
  CHECK_NOTNULL(msg);
  vectorKindrToMsg(kindr.getPosition(), &msg->translation);
  quaternionKindrToMsg(kindr.getRotation(), &msg->rotation);
}

template <typename Scalar>
void transformMsgToKindr(
    const geometry_msgs::Transform& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionMsgToKindr(msg.rotation, &rotation);
  vectorMsgToKindr(msg.translation, &position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_MSG_H
