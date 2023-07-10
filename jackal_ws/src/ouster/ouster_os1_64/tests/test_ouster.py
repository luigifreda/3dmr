#!/usr/bin/env python

import unittest

import rospy
import rosunit

from gazebo_msgs.srv import SpawnModel
from sensor_msgs.point_cloud2 import PointCloud2, read_points_list

boxy = """<robot name="boxy">
  <link name="boxy">
    <inertial>
      <origin xyz="1 1 0" />
      <mass value="1" />
      <inertia ixx="1" ixy="1" ixz="1" iyy="1" iyz="1" izz="1" />
    </inertial>
    <visual>
      <origin xyz="1 1 0" />
      <geometry><box size="1 1 1" /></geometry>
    </visual>
    <collision>
      <origin xyz="1 1 0" />
      <geometry><box size="1 1 1" /></geometry>
    </collision>
  </link>
  <gazebo reference="boxy"><material>Gazebo/Blue</material></gazebo>
</robot>"""


class OusterTest(unittest.TestCase):
    def test_insert_cube(self):
        rospy.init_node('ouster_test_node', anonymous=True)

        # Get 2 point clouds
        msg1 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=30)
        msg2 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=1)

        # Ensure they are different
        self.assertNotEqual(msg1.header.stamp, msg2.header.stamp)

        # The scene is the same: they should provide the same data
        self.assertEqual(msg1.data, msg2.data)

        # And there should be no interesting point to see
        points = [pt for pt in read_points_list(msg2) if pt.x != 0]
        self.assertEqual(len(points), 0)

        # Spawn a cube
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        spawn_sdf_model(model_name='boxy', model_xml=boxy, robot_namespace='/boxy', reference_frame='world')

        msg3 = rospy.wait_for_message("/os1_cloud_node/points", PointCloud2, timeout=1)

        # This time, the scene has changed: the data should be different
        self.assertNotEqual(msg2.data, msg3.data)

        # And there should be many interesting point to see
        points = [pt for pt in read_points_list(msg3) if pt.x != 0]
        self.assertGreater(len(points), 1000)

        # Finally, ensure those points are close enough to the cube
        margin = 0.05
        xs = [pt.x for pt in points]
        ys = [pt.y for pt in points]
        zs = [pt.z for pt in points]
        self.assertGreater(min(xs), 0.5 - margin)
        self.assertGreater(min(ys), 0.5 - margin)
        self.assertGreater(min(zs), 0 - margin)
        self.assertLess(max(xs), 1.5 + margin)
        self.assertLess(max(ys), 1.5 + margin)
        self.assertLess(max(zs), 1 + margin)


if __name__ == '__main__':
    rosunit.unitrun('ouster_os1_64', 'test_ouster', OusterTest)
