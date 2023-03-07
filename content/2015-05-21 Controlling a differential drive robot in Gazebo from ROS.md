Title: Controlling a differential drive robot in Gazebo from ROS
Date: 2015-05-21 21:12
Category: Code
Tags: Robotics, Tutorial, ROS
Authors: Helio Perroni Filho
Summary: How to control a pure SDF Gazebo robot using ROS.

[Gazebo](http://gazebosim.org/) is a simulation suite targeted at robotics projects, used by DARPA to implement the virtual part of its [Robotics Challenge](http://spectrum.ieee.org/automaton/robotics/robotics-software/osrf-prepares-for-darpa-virtual-robotics-challenge). It's a great companion to the [Robot Operating System](http://www.ros.org/), supporting [several ways](http://gazebosim.org/tutorials?cat=connect_ros) to interface with it.

One recurrent question regarding Gazebo is how to control robots described in [Simulation Description Format (SDF)](http://sdformat.org/) models from ROS. The more involved case of [porting a URDF model to Gazebo](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) and [manipulating it through ROS Control](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros) interfaces is well documented, but what about a robot that already has a full SDF model available? Isn't it possible to just paste it into a Gazebo workspace and get some ROS topics to manipulate it?

The tutorial below covers this simpler case. It is based on the SDF model for the [Pioneer 2DX](http://www-lar.deis.unibo.it/equipments/p2dx/) differential drive robot, which is available from Gazebo's default library. It also assumes ROS and Gazebo are already installed, as well as the necessary [integration packages](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) and [plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros).

First, save the SDF world model below to your local filesystem as `pioneer2dx_ros.world`:

    #!xml
    <?xml version="1.0" ?>
    <sdf version="1.4">
      <world name="default">
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name="ground_plane">
          <static>1</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>100</mu>
                    <mu2>50</mu2>
                  </ode>
                </friction>
                <bounce/>
                <contact>
                  <ode/>
                </contact>
              </surface>
              <max_contacts>10</max_contacts>
            </collision>
            <visual name="visual">
              <cast_shadows>0</cast_shadows>
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <script>
                  <uri>file://media/materials/scripts/gazebo.material</uri>
                  <name>Gazebo/Grey</name>
                </script>
              </material>
            </visual>
            <velocity_decay>
              <linear>0</linear>
              <angular>0</angular>
            </velocity_decay>
            <self_collide>0</self_collide>
            <kinematic>0</kinematic>
            <gravity>1</gravity>
          </link>
        </model>
        <physics type="ode">
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1</real_time_factor>
          <real_time_update_rate>1000</real_time_update_rate>
          <gravity>0 0 -9.8</gravity>
        </physics>
        <scene>
          <ambient>0.4 0.4 0.4 1</ambient>
          <background>0.7 0.7 0.7 1</background>
          <shadows>1</shadows>
        </scene>
        <spherical_coordinates>
          <surface_model>EARTH_WGS84</surface_model>
          <latitude_deg>0</latitude_deg>
          <longitude_deg>0</longitude_deg>
          <elevation>0</elevation>
          <heading_deg>0</heading_deg>
        </spherical_coordinates>
        <model name="pioneer2dx">
          <link name="chassis">
            <pose>0 0 0.16 0 -0 0</pose>
            <inertial>
              <mass>5.67</mass>
              <inertia>
                <ixx>0.07</ixx>
                <iyy>0.08</iyy>
                <izz>0.1</izz>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyz>0</iyz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry>
                <box>
                  <size>0.445 0.277 0.17</size>
                </box>
              </geometry>
              <max_contacts>10</max_contacts>
              <surface>
                <bounce/>
                <friction>
                  <ode/>
                </friction>
                <contact>
                  <ode/>
                </contact>
              </surface>
            </collision>
            <collision name="castor_collision">
              <pose>-0.2 0 -0.12 0 -0 0</pose>
              <geometry>
                <sphere>
                  <radius>0.04</radius>
                </sphere>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>0</mu>
                    <mu2>0</mu2>
                    <slip1>1</slip1>
                    <slip2>1</slip2>
                  </ode>
                </friction>
                <bounce/>
                <contact>
                  <ode/>
                </contact>
              </surface>
              <max_contacts>10</max_contacts>
            </collision>
            <visual name="visual">
              <pose>0 0 0.04 0 -0 0</pose>
              <geometry>
                <mesh>
                  <uri>model://pioneer2dx/meshes/chassis.dae</uri>
                </mesh>
              </geometry>
            </visual>
            <visual name="castor_visual">
              <pose>-0.2 0 -0.12 0 -0 0</pose>
              <geometry>
                <sphere>
                  <radius>0.04</radius>
                </sphere>
              </geometry>
              <material>
                <script>
                  <uri>file://media/materials/scripts/gazebo.material</uri>
                  <name>Gazebo/FlatBlack</name>
                </script>
              </material>
            </visual>
            <velocity_decay>
              <linear>0</linear>
              <angular>0</angular>
            </velocity_decay>
            <self_collide>0</self_collide>
            <kinematic>0</kinematic>
            <gravity>1</gravity>
          </link>
          <link name="right_wheel">
            <pose>0.1 -0.17 0.11 0 1.5707 1.5707</pose>
            <inertial>
              <mass>1.5</mass>
              <inertia>
                <ixx>0.0051</ixx>
                <iyy>0.0051</iyy>
                <izz>0.009</izz>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyz>0</iyz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>0.11</radius>
                  <length>0.05</length>
                </cylinder>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>100000</mu>
                    <mu2>100000</mu2>
                    <slip1>0</slip1>
                    <slip2>0</slip2>
                  </ode>
                </friction>
                <bounce/>
                <contact>
                  <ode/>
                </contact>
              </surface>
              <max_contacts>10</max_contacts>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>0.11</radius>
                  <length>0.05</length>
                </cylinder>
              </geometry>
              <material>
                <script>
                  <uri>file://media/materials/scripts/gazebo.material</uri>
                  <name>Gazebo/FlatBlack</name>
                </script>
              </material>
            </visual>
            <velocity_decay>
              <linear>0</linear>
              <angular>0</angular>
            </velocity_decay>
            <self_collide>0</self_collide>
            <kinematic>0</kinematic>
            <gravity>1</gravity>
          </link>
          <link name="left_wheel">
            <pose>0.1 0.17 0.11 0 1.5707 1.5707</pose>
            <inertial>
              <mass>1.5</mass>
              <inertia>
                <ixx>0.0051</ixx>
                <iyy>0.0051</iyy>
                <izz>0.009</izz>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyz>0</iyz>
              </inertia>
            </inertial>
            <collision name="collision">
              <geometry>
                <cylinder>
                  <radius>0.11</radius>
                  <length>0.05</length>
                </cylinder>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>100000</mu>
                    <mu2>100000</mu2>
                    <slip1>0</slip1>
                    <slip2>0</slip2>
                  </ode>
                </friction>
                <bounce/>
                <contact>
                  <ode/>
                </contact>
              </surface>
              <max_contacts>10</max_contacts>
            </collision>
            <visual name="visual">
              <geometry>
                <cylinder>
                  <radius>0.11</radius>
                  <length>0.05</length>
                </cylinder>
              </geometry>
              <material>
                <script>
                  <uri>file://media/materials/scripts/gazebo.material</uri>
                  <name>Gazebo/FlatBlack</name>
                </script>
              </material>
            </visual>
            <velocity_decay>
              <linear>0</linear>
              <angular>0</angular>
            </velocity_decay>
            <self_collide>0</self_collide>
            <kinematic>0</kinematic>
            <gravity>1</gravity>
          </link>
          <joint name="left_wheel_hinge" type="revolute">
            <pose>0 0 -0.03 0 -0 0</pose>
            <child>left_wheel</child>
            <parent>chassis</parent>
            <axis>
              <xyz>0 1 0</xyz>
              <limit>
                <lower>-1e+16</lower>
                <upper>1e+16</upper>
              </limit>
            </axis>
          </joint>
          <joint name="right_wheel_hinge" type="revolute">
            <pose>0 0 0.03 0 -0 0</pose>
            <child>right_wheel</child>
            <parent>chassis</parent>
            <axis>
              <xyz>0 1 0</xyz>
              <limit>
                <lower>-1e+16</lower>
                <upper>1e+16</upper>
              </limit>
            </axis>
          </joint>
          <!-- Replaced Gazebo's differential drive plugin with the ROS-friendly variant -->
          <!--
          <plugin filename="libDiffDrivePlugin.so" name="diff_drive">
          <left_joint>left_wheel_hinge</left_joint>
          <right_joint>right_wheel_hinge</right_joint>
          <torque>5</torque>
          </plugin>
          -->
          <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>100</updateRate>
            <leftJoint>left_wheel_hinge</leftJoint>
            <rightJoint>right_wheel_hinge</rightJoint>
            <wheelSeparation>0.39</wheelSeparation>
            <wheelDiameter>0.15</wheelDiameter>
            <torque>5</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>chassis</robotBaseFrame>
          </plugin>
          <pose>0 0 0 0 -0 0</pose>
          <static>0</static>
        </model>
        <state world_name="default">
          <sim_time>85 304000000</sim_time>
          <real_time>85 849190220</real_time>
          <wall_time>1432260579 736436496</wall_time>
          <model name="ground_plane">
            <pose>0 0 0 0 -0 0</pose>
            <link name="link">
              <pose>0 0 0 0 -0 0</pose>
              <velocity>0 0 0 0 -0 0</velocity>
              <acceleration>0 0 0 0 -0 0</acceleration>
              <wrench>0 0 0 0 -0 0</wrench>
            </link>
          </model>
          <model name="pioneer2dx">
            <pose>-0.103826 0.027961 2e-06 -2e-06 -8e-06 -0.094162</pose>
            <link name="chassis">
              <pose>-0.103827 0.027961 0.160002 -2e-06 -8e-06 -0.094162</pose>
              <velocity>-0.000797 0.002229 0.003363 -0.024657 -0.007301 0.0023</velocity>
              <acceleration>0 0 0 0 -0 0</acceleration>
              <wrench>0 0 0 0 -0 0</wrench>
            </link>
            <link name="left_wheel">
              <pose>0.011714 0.187806 0.110002 1.38258 1.57033 2.85912</pose>
              <velocity>-0.000816 0.001565 0.000275 -0.014307 -0.006987 0.002012</velocity>
              <acceleration>0 0 0 0 -0 0</acceleration>
              <wrench>0 0 0 0 -0 0</wrench>
            </link>
            <link name="right_wheel">
              <pose>-0.020254 -0.150688 0.110002 -1.56886 1.51434 -0.092322</pose>
              <velocity>-0.000347 0.001559 0.008433 -0.014387 -0.003243 0.000126</velocity>
              <acceleration>0 0 0 0 -0 0</acceleration>
              <wrench>0 0 0 0 -0 0</wrench>
            </link>
          </model>
        </state>
        <gui fullscreen="0">
          <camera name="user_camera">
            <pose>5 -5 2 0 0.275643 2.35619</pose>
            <view_controller>orbit</view_controller>
          </camera>
        </gui>
      </world>
    </sdf>

Open a terminal window and start the ROS middleware by entering:

    $ roscore

Open another terminal window, `cd` to the folder containing `pioneer2dx_ros.world` and enter:

    $ rosrun gazebo_ros gazebo -file pioneer2dx_ros.world

You should now have a Gazebo window opened with a Pioneer 2DX placed in the middle of an empty world.

![Gazebo GUI]({static}/images/articles/2015-05-21-01/snapshot.png)

Finally, open a third terminal window (it's the last one, I promise) and check if the differential drive topics have been published:

    $ rostopic list

The following topics should be visible, among others:

    :::sh
    /pioneer2dx/cmd_vel
    /pioneer2dx/odom


You should now be able to get the robot moving by publishing messages to the `/pioneer2dx/cmd_vel` topic, e.g.

    $ rostopic pub -1 /pioneer2dx/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'

Should get the robot running in a loop.

