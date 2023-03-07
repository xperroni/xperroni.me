Title: Projecting Points with ROS Pinhole Camera Model
Date: 2017-11-05 17:00
Category: Code
Tags: Robotics, Tutorial, Python, ROS
Authors: Helio Perroni Filho
Summary: How to project world points into a camera plane in ROS.

A common task in robotics-related Computer Vision projects is [projecting](https://jordicenzano.name/front-test/2d-3d-paradigm-overview-2011/camera-model/) a 3D point into the 2D image plane of a camera, or vice-versa. In ROS this can be achieved very easily with the help of classes [tf.TransformListener](docs.ros.org/electric/api/tf/html/python/tf_python.html) and [image_geometry.PinholeCameraModel](http://docs.ros.org/diamondback/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html).

Given an arbitrary 3D point in world coordinates, the first step is to convert its coordinates from the global reference frame to a camera-centric frame. Given a tuple `p_3d` containing global coordinates of a point, and assuming there is a [tf](http://wiki.ros.org/tf) broadcaster publishing transforms between the `/world` (global) and `/base_link` (camera-centric) frames, the code below transforms `p_3d` to camera-centric coordinates:

    #!python
    p_world = PointStamped()
    p_world.header.seq = self.camera_image.header.seq
    p_world.header.stamp = stamp
    p_world.header.frame_id = '/world'
    p_world.point.x = p_3d[0]
    p_world.point.y = p_3d[1]
    p_world.point.z = p_3d[2]

    listener = tf.TransformListener()
    listener.waitForTransform('/base_link', '/world', stamp, rospy.Duration(1.0))
    p_camera = listener.transformPoint('/base_link', p_world)

The camera-centric point `p_camera` can then be projected into the image as in the code below:

    #!python
    # The navigation frame has X pointing forward, Y left and Z up, whereas the
    # vision frame has X pointing right, Y down and Z forward; hence the need to
    # reassign axes here.
    x = -p_camera.point.y
    y = -p_camera.point.z
    z = p_camera.point.x

    camera = PinholeCameraModel()
    camera.fromCameraInfo(camera_info)
    p_image = camera.project3dToPixel((x, y, z))

The `camera_info` argument is an instance of the [sensor_msgs.msg.CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) message class, it can be instantiated locally or received through a topic.

Notice that in real-world applications, rather than locally instantiating `tf.TransformListener` and `image_geometry.PinholeCameraModel` objects as in the code above, they'd more likely be instantiated once and kept as attributes of a node object.

