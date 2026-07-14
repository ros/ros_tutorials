# ros_tutorials

`ros_tutorials` is a collection of packages that support the official [ROS 2
documentation](https://docs.ros.org/en/rolling/) and are used throughout the
beginner tutorials to teach the core concepts of ROS 2 — nodes, topics,
services, actions, parameters and launch files.

This branch (`rolling`) targets **ROS 2 Rolling Ridley**. For other ROS
distributions, check out the matching branch of this repository.

## What's in this repository

This repository contains two packages:

| Package | Description |
| ------- | ----------- |
| [`turtlesim`](turtlesim/) | A lightweight simulator built with Qt. It is the tool used to introduce the basics of ROS 2 in the beginner tutorials. |
| [`turtlesim_msgs`](turtlesim_msgs/) | The interface package that defines the messages, services and action used by `turtlesim`. |

### turtlesim

`turtlesim` is a tool made for teaching ROS and ROS packages. It draws one or
more turtles in a window and lets you drive them around, spawn new ones, change
pen colors and more, all through ROS 2 topics, services and actions.

It builds the following executables:

- **`turtlesim_node`** — the simulator window itself.
- **`turtle_teleop_key`** — drive a turtle from the keyboard (arrow keys), and
  send `RotateAbsolute` action goals.
- **`draw_square`** — moves a turtle in a square pattern.
- **`mimic`** — makes one turtle mimic the motion of another, used to
  demonstrate topic remapping.

The [`launch/`](turtlesim/launch/) directory contains `multisim.launch.py`,
which starts two `turtlesim_node` instances in separate namespaces (used by the
`mimic` demo).

### turtlesim_msgs

`turtlesim_msgs` contains the ROS 2 interface definitions that `turtlesim` uses:

- **Messages** ([`msg/`](turtlesim_msgs/msg/)): `Color`, `Pose`
- **Services** ([`srv/`](turtlesim_msgs/srv/)): `Kill`, `SetPen`, `Spawn`,
  `TeleportAbsolute`, `TeleportRelative`
- **Action** ([`action/`](turtlesim_msgs/action/)): `RotateAbsolute`

## Documentation and tutorials

The best place to learn how to use these packages is the official ROS 2
documentation:

- **ROS 2 Rolling documentation:** https://docs.ros.org/en/rolling/
- **Introducing turtlesim:** https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
- **Creating a workspace:** https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
- **Beginner: CLI tools:** https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html
- **Beginner: Client libraries:** https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries.html

The turtlesim-based tutorials cover, among other topics:

- [Understanding nodes](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Understanding topics](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Understanding services](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding parameters](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Understanding actions](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

## Contributing and support

- **Source & issues:** https://github.com/ros/ros_tutorials
- **ROS community forum:** https://discourse.ros.org/
- **Ask questions:** https://robotics.stackexchange.com/

## License

Both packages are released under the **BSD** license. See each package's
`package.xml` for details.
