# fusion2urdf

Forked from [SpaceMaster85](https://github.com/SpaceMaster85/fusion2urdf), who forked from [syuntoku14](https://github.com/syuntoku14/fusion2urdf). Further fixes applied with [Claude Code](https://claude.ai/code) (Anthropic) to make the exporter work on macOS with Fusion 360's bundled Python 3.14.

**Tested on:** macOS Sequoia, Autodesk Fusion 360 (2025), Fusion's bundled Python 3.14, Apple Silicon (M-series).

This repo only supports Gazebo, if you are using pybullet, see: https://github.com/yanshil/Fusion2PyBullet.

## Updated!!!
* 2026/10/09: Sub-assembly support and MuJoCo viewer (this fork)
  * **Sub-assemblies become multiple links:** a top-level component that contains subcomponents is split one level down, so each direct child becomes its own link (`<parent>_<child>`, e.g. `inside_1_leg_2_1`). base_link is never split.
  * **Joints inside sub-assemblies are exported:** joints and as-built joints defined in nested components are read in the root assembly context, so they connect the correct links.
  * **Joint tree is built from base_link:** joints are walked outward from base_link, and any joint whose Component1/Component2 order is reversed is flipped automatically (the axis is negated so the motion is unchanged). You no longer have to pick parent = Component2 by hand.
  * **Skipped joints are reported:** a pop-up lists every joint that isn't exported and why (hidden, connects a hidden component, both ends in the same link, not connected to base_link, closes a loop, duplicate name), followed by a line such as `(info) 12 joints in design ..., 12 read, 12 exported`.
  * **Hidden components and joints are not exported:** turn off a light bulb to leave something out.
  * **New `scripts/view_urdf.py`:** checks an export and opens it in MuJoCo. See [Checking an export in MuJoCo](#checking-an-export-in-mujoco).

* 2021/10/13: Version 1.2
  * Made the color and material detection more robust
  * Added ROS 2 launchfile generator
  * Dialog Popup to choose between ROS 1 and ROS 2

* 2021/03/09: version 1.1
  * Fix FusionAPI Change

* 2021/01/23: Version 1.0
  * This version can handle now nested components
  * The bodies and components which should be exported can be choosen via the light bulbs in Fusion 360
  * Joints with disabled light bulb will be ignored and not exported to the urdf file
  * Hopefully final bugfix of the joint coordinates calculation
  * Reading out the color of the component and set it in the material file
  * All changes are done in a temporary file, so no backup of your design file is necessary (but it is still a good idea)
   This means that also linked components can be used
  * Added two example Fusion 360 files, which can be found in the _Example_ folder

* 2021/01/09: Fix xyz calculation. 
  * If you see that your components move arround the map center in rviz try this update 
  * More Infos see: https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767

* 2020/11/10: README fix
  * MacOS Installation command fixed in README
  * Date format unified in README to yyyy/dd/mm
  * Shifted Installation Upwards for better User Experience and easier to find
* 2020/01/04: Multiple updates:
  * no longer a need to run a bash script to convert stls
  * some cleanup around joint and transmission generation
  * defines a sample material tag instead of defining a material in each link
  * fusion2urdf now generates a self-contained ROS {robot_name}_description package
  * now launched by roslaunch {robot_name}_description display.launch
  * changed fusion2urdf output from urdf to xacro for more flexibility
  * separate out material, transmissions, gazebo elements to separate files
* 2018/20/10: Fixed functions to generate launch files
* 2018/25/09: Supports joint types "Rigid", "Slider" & Supports the joints' limit(for "Revolute" and "Slider"). 
* 2018/19/09: Fixed the bugs about the center of the mass and the inertia.


## Installation

Run the following command in your shell.

##### Windows (In PowerShell)

```powershell
cd <path to fusion2urdf>
Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

##### macOS (In bash or zsh)

```bash
cd <path to fusion2urdf>
cp -r ./URDF_Exporter "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

## What is this script?
This is a fusion 360 script to export urdf from fusion 360 directly.

This exports:
* .urdf file of your model
* .launch and .yaml files to simulate your robot on gazebo
* .stl files of your model

### Sample 

The following test model doesn't stand upright because the z axis is not upright in default fusion 360.
Make sure z axis is upright in your fusion 360 model if you want. 

#### original model
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/industrial_robot.png" alt="industrial_robot" title="industrial_robot" width="300" height="300">

#### Gazebo simulation of exported .urdf and .launch
* center of mass
<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/center_of_mass.png" alt="center_of_mass" title="center_of_mass" width="300" height="300">

* collision
<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/collision.png" alt="collision" title="collision" width="300" height="300">

* inertia
<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/inertia.png" alt="inertia" title="inertia" width="300" height="300">


## Before using this script

Before using this script, make sure that your model has all the "links" as components. You have to define the links by creating corresiponding components. For example, this model(https://grabcad.com/library/spotmini-robot-1) is not supported unless you define the "base_link". 

Every link must be connected to base_link through a chain of joints. The Component1/Component2 order no longer matters: the exporter orients the tree outward from base_link and flips reversed joints. Joints that don't connect to base_link are skipped and listed in a pop-up.

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/spot_mini.PNG" alt="spot_mini" title="spot_mini" width="300" height="300">

~~Also, make sure components of your model has only bodies. **Nested components are not supported**.~~
For example, this works:
<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/only_bodies.PNG" alt="only_bodies" title="only_bodies" width="300" height="300">

~~but this doesn't work since the "face (3):1" component contains other components. A component must contain only bodies:~~
With the new version this should also work: 

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/nest_components.PNG" alt="nest_components" title="nest_components" width="300" height="300">

Top-level components that contain subcomponents are split one level down: each direct child becomes its own link, and joints between those children (including as-built joints) are exported. Anything deeper is merged into its level-1 parent's stl, mass and inertia. base_link is always exported as one link. Only components and bodies with activated light bulb will be exported, **components and bodies with deactivated light bulb** will be ignored and **not exported!**

Joints which has a deactivated child component will be ignored. Deactivating the base_link will cause problems! 
**Joints with deactivated light bulbs will not be exported!**

Components connected through disabled joints will not be shown in RVIZ.


As-built joints are exported like normal joints.

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/asBuildJoints.png" alt="asBuildJoint" title="asBuildJoint">

Colors which are defined on the root level components will be read out and taken over into the material definition of the urdf file.

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/color.PNG" alt="color" title="color">

Sometimes this script exports abnormal urdf without any error messages. In that case, the joints should have problems. Redefine the joints and run again.

In addition to that, make sure that this script currently supports only "Rigid", "Slider" and "Revolute".




## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

### Install in Shell 

Run the [installation command](#installation) in your shell.

### Run in Fusion 360

Click ADD-INS in fusion 360, then choose ****fusion2urdf****. 

~~**This script will change your model. So before run it, copy your model to backup.**~~

Run the script and wait a few seconds(or a few minutes). Then a folder dialog will show up. Choose where you want to save the urdf (A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved by just redefining the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

**You must define the base component**. Rename the base component as "base_link". 

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/cautions.PNG" alt="cautions" title="cautions" width="300" height="300">

In the above image, base_link is gounded. Right click it and click "Unground". 

Now you can run the script. Let's run the script. Choose the folder to save and wait a few second. ~~You will see many "old_component" the components field but please ignore them.~~ 
The script will generate a folder with the name of your Fusion design in lower case to meet the ROS naming requirements concated with "description"
You have successfully exported the urdf file. Also, you got stl files in "Desktop/test_description/meshes" repository. This will be required at the next step. ~~The existing fusion CAD file is no more needed. You can delete it.~~ 

The folder "Desktop/test_description" will be required in the next step. Move them into your ros environment.


#### In your ROS environment

Place the generated _description package directory in your own ROS workspace. "catkin_ws" is used in this example.
Then, run catkin_make in catkin_ws.

```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Now you can see your robot in rviz. You can see it by the following command.

```bash
roslaunch (whatever your robot_name is (lowercases))_description display.launch
```

<img src="https://github.com/spacemaster85/fusion2urdf/blob/images/rviz_robot.png" alt="rviz" title="rviz" width="300" height="300">

If you want to simulate your robot on gazebo, just run
```bash
roslaunch (whatever your robot_name is(lowercase))_description gazebo.launch
```
Gazebo was not tested by me!

## Checking an export in MuJoCo

`scripts/view_urdf.py` validates an exported `<robot>_description` folder and opens it in the MuJoCo viewer, with a slider for each joint. Nothing in the export is modified.

```bash
pip install mujoco numpy trimesh coacd rtree
python scripts/view_urdf.py path/to/<robot>_description --zup          # report + viewer
python scripts/view_urdf.py path/to/<robot>_description --check-only   # report only
```

* Use `python`, not `mjpython`: on macOS the viewer crashes under `mjpython`.
* `--zup` rotates Fusion's Y-up frame to MuJoCo's Z-up. `--height` sets how high the base is held (in metres).
* **Report:** lists the link/joint tree and flags self-joints, missing links, links with two parents, cycles, unconnected parts and unused or missing meshes. `--check-only` exits non-zero if it finds problems.
* **Broken exports still open:** the viewer drops bad joints, attaches unconnected links and unused meshes to the root with fixed joints, and prints each change.
* **Collisions:** base_link collides with every part, while other parts collide only with the floor (neighbouring CAD parts overlap at their joints). Every collision mesh is split into convex pieces with CoACD (finer for base_link), so contacts match the CAD instead of its convex hulls. The first run per export takes a few minutes; the pieces are cached in `$TMPDIR/view_urdf_<name>/`. Delete that folder after re-exporting.
* **Servos:** joint stiffness is scaled to each joint's inertia, and the simulation uses the `implicitfast` integrator, so light links stay stable.
* Meshes over 200k faces are decimated for viewing.

**Enjoy your Fusion 360 and ROS life!**

