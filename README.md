# multipanda_ros2
## A sim- and real Panda robot integration based on the `ros2_control` framework
<img src="docs/images/single_sim.png" alt="" height="250">
<img src="docs/images/dual_sim.png" alt=""   height="250">
<img src="docs/images/garmi_sim.png" alt=""  height="250">

This project implements most features from the original `franka_ros` repository in ROS 2 Jazzy, specifically for the Franka Emika Robot (Panda).
This project significantly expands upon the original `franka_ros2` from the company, who dropped the support for the Pandas.

Additionally, multi-arm mujoco simulation has been integrated, meaning you can now run the same controller on both simulated and real robots.
The simulation is designed as a plugin to the [`mujoco_ros_pkg`](https://github.com/ubi-agni/mujoco_ros_pkgs/).

This branch uses the pinned `mujoco_ros_pkgs` submodule and the tracked Jazzy patch
applied by `./tools/prepare_jazzy`. Follow [README.zh-CN.md](README.zh-CN.md) to
prepare it; replacing it with an independently cloned upstream checkout skips
the tested version and compatibility patch.

Work is ongoing to integrate FR3 into the architecture.

## Documentation

Documentation for this project is available [here](./docs/main.md).
Start with the [中文 Jazzy guide](README.zh-CN.md); the
[custom controller tutorial](franka_example_controllers/README.md) explains the
current package paths, retained plugin aliases, and Jazzy controller loading.

## Working features
More thorough information is available in the documentation.
The list below describes implemented interfaces. Actual Jazzy verification and
remaining gaps are recorded in [the validation report](docs/jazzy-validation.zh-CN.md).

* Real robot:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `/<arm_id>_error_recovery_service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* Sim robot:
    * Same as the real robot, except Cartesian command interface is not available, and there is no plan to implement this for now.
    * Gripper server with identical interface to the real gripper (i.e. action servers)
    * Example controllers for the real single-arm listed above, that correspond to those interfaces, work out of the box.
    * FrankaState implements the basics: torque, joint position/velocity, `O_T_EE` and `O_F_ext_hat`.
    * Model provides all the existing functions: `pose`, `zeroJacobian`, `bodyJacobian`, `mass`, `gravity`, `coriolis`.
        * Gravity for now just returns the corresponding `qfrc_gravcomp` force from mujoco.
        * Coriolis = `qfrc_bias - qfrc_gravcomp`
    * Camera is available as part of `mujoco_ros_pkg`'s features. You can simply add a `<camera>` object in your mujoco XML file, and the package will handle them.
    * With the forked repository's `mujoco_ros2_control_system` package, you can easily add components with additional degrees of freedom to your robot. Take a look at `garmi_packages/garmi_description/robots/*.ros2_control.xacro` for an example on how to do this.

## Known issues
* MoveIt launch files still reference the old MongoDB node when `db:=true`.
  Database support has not been migrated or validated. Keep the default `db:=false`;
  installing `warehouse_ros_sqlite` does not configure a working database backend.

## Ubuntu 24.04 / ROS 2 Jazzy

**中文安装与验证指南：[README.zh-CN.md](README.zh-CN.md)**

This branch targets ROS 2 Jazzy on Ubuntu 24.04. Build the development image with
`./tools/setup_env`, enter it with `./run`, then run `./src/tools/build_jazzy`.
Use `./run --headless` for SSH or automated simulation checks.

The MuJoCo submodule is pinned and adapted by `./tools/prepare_jazzy`; its patch is
tracked in this repository. See the Chinese guide for dependency versions, launch
commands, and the distinction between simulation tests and real-hardware validation.

## Citation
If multipanda_ros2 framework helps your research, please cite our paper:

**[Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems](https://ieeexplore.ieee.org/document/11696149)**

```bibtex
@INPROCEEDINGS{škerlj2026multipanda_ros2,
      title={Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS2 Framework for Multimanual Systems},
      author={Škerlj, Jon and Bien, Seongjin and Naceri, Abdeldjallil and Haddadin, Sami},
      booktitle={2026 IEEE International Conference on Robotics and Automation (ICRA)},
      year={2026},
      pages={9679-9686},
      doi={10.1109/ICRA57385.2026.11696149},
      url={https://ieeexplore.ieee.org/document/11696149}
}
```

## Credits
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## License

All packages of `multipanda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble
[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
