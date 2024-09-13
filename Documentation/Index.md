**Table of contents:**
- [General Concepts and Phrases](#general-concepts-and-phrases)
- [Force](#force)
  - [ForcePoint](#forcepoint)
  - [(Something)ForceModel](#somethingforcemodel)
- [Water](#water)
  - [SimpleWaterQueryModel](#simplewaterquerymodel)
  - [SimpleWaterCurrent](#simplewatercurrent)
- [Vehicle Components](#vehicle-components)
    - [Update rates](#update-rates)
    - [LinkAttachment](#linkattachment)
  - [Acoustics](#acoustics)
    - [Transceiver(TX)](#transceivertx)
      - [Occlusions](#occlusions)
      - [Propagation speed](#propagation-speed)
      - [Surface echoes](#surface-echoes)
      - [Bottom echoes](#bottom-echoes)
      - [Configuration](#configuration)
  - [Actuators](#actuators)
    - [Joints](#joints)
    - [VBS](#vbs)
    - [Propeller](#propeller)
  - [Sensors](#sensors)
    - [Battery](#battery)
    - [Camera Image](#camera-image)
    - [DepthPressure](#depthpressure)
    - [DVL](#dvl)
    - [GPS](#gps)
      - [GPSReferencePoint](#gpsreferencepoint)
    - [IMU](#imu)
    - [Leak](#leak)
    - [Sonar](#sonar)
      - [Sonar Concepts](#sonar-concepts)
      - [Sonar Configuration](#sonar-configuration)
      - [Sonar Visualization](#sonar-visualization)
  - [ROS](#ros)
    - [Core](#core)
      - [RosMessages](#rosmessages)
      - [Clock](#clock)
      - [ROSPublisher](#rospublisher)
    - [Publishers](#publishers)
      - [TF](#tf)
    - [Subscribers](#subscribers)
      - [Teleporter\_Sub](#teleporter_sub)
      - [Actuator Subscriber](#actuator-subscriber)
- [Utilities](#utilities)
  - [Rope](#rope)
  - [Importer](#importer)
- [Prefabs](#prefabs)
  - [SAM AUV](#sam-auv)
    - [Keyboard Controller](#keyboard-controller)
    - [ROS Controls](#ros-controls)
  - [Quadrotor](#quadrotor)
    - [Keyboard controller](#keyboard-controller-1)
    - [Geometric Tracking Controller](#geometric-tracking-controller)
  - [GameUI](#gameui)




# General Concepts and Phrases
- **Body**: Refers to either an Articulation Body(AB), or a Rigidbody(RB) in Unity. 
Since most of the time these two share the physical properties, we refer to them collectively as Bodies.

- **Standard**: The Standard Unity project found [here](https://github.com/smarc-project/SMARCUnityStandard)

- **HDRP**: The High Def. Rendering Pipeline Unity project found [here](https://github.com/smarc-project/SMARCUnityHDRP)




# Force
Simulation of water-related custom forces that are applied to Bodies. 

## ForcePoint
A generalized massless point in space where external forces are applied.
This script is usually attached to many GameObjects and made child of a Body. 
Most commonly used for approximate buoyancy forces, water currents and gravity. 
For buoyancy and water currents, queries the [Water](#water) model to check for depth and current vector.

The more of these you put on a vehicle and the denser they are, the more accurate the forces will become.

Example:

![ForcePoints](Media/ForcePoints.png)

The force points are shown as magenta spheres. 
They are children to the `base_link` object which contains a Body.

![ForcepointConfig](Media/ForcePointConfig.png)

- Connected Body: Either an articulation body or a rigidbody can be assigned here. Usually the `base_link` of a robot.
- Buoyancy
  - Volume Object: An object can be assigned to automatically assign the mesh from.
  - Volume Mesh: A mesh can be assigned to calculate the volume from.
  - Volume: Volume can be set directly.
  - Water Density: In Kg/m3, density of the fluid to calculate buoyancy forces from. 
  - Depth Before Submerged: A workaround the fixed delta time simulation steps. When set to zero, the force points tend to produce high frequency oscilations. In one update it is in water and in the next it is out. To prevent this, some depth is allowed without applying the full buoyancy force. This smooths out the oscilations.
- Gravity
  - Add Gravity: If checked, this force point will add a gravity force as well.
  - Automatic Center of Gravity: If checked, the center of gravity of the object is calculated based on the locations of the force points it has. This can be useful when the object they are attached to has complicated geometry.
  - Mass: If gravity is to be applied, the mass of the entire object. The mass will also be distributed among the forcepoints, similarly to buoyancy forces.


## (Something)ForceModel
An analytical model of a vehicle. 
Since Unity only provides a rigid body simulation, some physical modelling (like hydrodynamic forces, damping) must be done "manually". 
These "ForceModel" scripts do just that and are specific to the vehicle they model, such as SAM and BlueROV.




# Water
We do not simulate water as particles, but as vector fields and a surface.
[ForcePoint](#forcepoint)s are used to apply the appropriate forces.

## SimpleWaterQueryModel
Used to define an interface for any object within the Unity world to get the Y(height) value of the water surface at a given point.
While in Standard the water surface is a plane at a fixed Y, in HDRP water surface is simulated with waves and thus has different heights at different times and locations.

This is used for GPS fixes, pressure measurements, buoyancy forces and so on.

HDRP Water:

![Waves](Media/HDRPWaves.png)

## SimpleWaterCurrent
A simple vector field within a volume that applies forces to any ForcePoints within it.
Used to simulate currents within that volume.

Two volumes with different currents, mixing in overlapping regions.
Currents visualized with WaterCurrentVisualizer:

![Currents](Media/Currents.png)



# Vehicle Components
These are the components that can be collected within a vehicle: usually sensors and actuators.
Optionally with ROS connections.
Most of them reflect real objects and some are abstractions.

- A general idea used for vehicle components is that the structure of the vehicle and its functional parts are defined separately within the robot object.
  - The structure part only contains geometry and Bodies, essentially reflecting a URDF file.
  - The functional parts are all agnostic to their poses, like a packaged real sensor, they can be mounted anywhere.

### Update rates
Unless specified separately, vehicle components that work with the physics engine (only few exceptions) use the `FixedUpdate()` method of Unity.
This means that the update rate of all of them is limited to the physics update rate of Unity, meaning by default they can be updated at 50hz maximum.
See [the unity documentation](https://docs.unity3d.com/ScriptReference/MonoBehaviour.FixedUpdate.html) for details.

### LinkAttachment
A base class that most sensors and actuators extend. 

In general, we aim to keep the _function_ of a component and its _pose_ separete from each other: Many robots are defined with URDF files which only define a geometric structure of where everything is and connected to what. 
These URDF files tend to change over the lifetime of a robot, a camera is moved elsewhere, some sensor is turned around, but the component itself does not change, only it's _pose_. 

> Imagine the LinkAttachment mechanism as the ability to disassemble a robot on a table.
> Unlike reality, our robots can assemble themselves automatically at run-time :)

- A GameObject with a LinkAttachment component is given a string name in the editor.
- At `Awake()` this GameObject's parent is set to be the GameObject with the given name that is under the same root object.

> If you have written a new LinkAttachment-extending class, and need to use the `Awake()` method yourself, you need to call `Attach()` manually.
> See for example `Sonar.cs`.

For example, a Battery object. Before play mode, it is under `sam_auv_v1/SAMSensor`:

![before play](Media/LinkAttachment_before.png)

And at play mode, it is under `sam_auv_v1/odom/base_link/battery_link`:

![during play](Media/LinkAttachment_after.png)

- Rotate For ROS Camera: The camera origin is different in Unity and in ROS.
Checking this box ON rotates the LinkAttachment object to match the ROS standard.
- The Roll/Pitch/Yaw fields can be used to rotate the LinkAttachment object _after_ it is attached.
In general it is a better idea to modify the structure part of the robot (usually the URDF) instead of using this.


## Acoustics
Peer to peer acoustic communications.

### Transceiver(TX)
Simulates both a receiver and a transmitter of acoustic signals at the package level. 
The main way to interact with a transceiver are its `Read()` and `Write()` methods.
- `Read()`: Returns a StringStamped object that was received first. Contains the data as a string, the time it was sent and the time it was received.
- `Write()`: Takes a string and puts it in the write-queue. The string is broadcast in the next `FixedUpdate()`, first-in-first-out.

We provide different levels of realism when simulating a transceiver:

#### Occlusions
We make use of sphere-casting to determine occlusions.
When a transceiver broadcasts, it performs a sphere-cast to _every_ other transceiver in the scene.
If the cast is free of obstacles, we consider that path free.
The required obstacle-free width can be configured.

#### Propagation speed
We use coroutines to simulate the propagation speed of sound.
If the path is clear and the target is within range, a coroutine is started: The target's `Receive()` method is called after a delay depending on the distance between the source and target.
This way, the delay can be timed accurately independently of simulation time steps.

This simulates the arrival timing of any packages and can be used for triangulation purposes.

#### Surface echoes
The most common source of echoes comes from the surface.
Since the physics involved in the production of echoes is too complex to be simulated at high rates, we approximate surface echoes as follows:

- Assume the surface is a plane defined by the horizontal position of the transceiver, the vertical position of the [Water](#water) model and the vector opposite gravity.
- Create a reflection of the transceiver around this plane.
- Sphere-cast from the reflection point to the target transceiver.
- Find the intersection of this ray and the plane. This is the "bounce point".
- Sphere-cast from source TX to the "bounce point" for occlusion check.
- Sphere-cast from "bounce point" to target TX for occlusion check.
- If no occlusions, find distance of source -> "bounce point" -> target.
- Check if distance is less than maximum range reduced by a configurable echo-loss.
- Start the [receiveing coroutine](#propagation-speed) with the in-direct distance.

```
                  O' selfReflectionPos
                  |\
                  | \
 water plane ~~~~~~~~E~~~~~~~~ E=echoPoint
                  | / \
                  |/   \
          selfPos O     T targetPos 
```

> This method allows us to use only 3 raycasts per pair to create surface echoes, so it can be enabled without much concern about performance.


#### Bottom echoes
The second-most common source of echoes.
Unlike surface echoes, a flat-bottom assumption is harder to justify for the bottom, since sheer walls, crevices, rocks and other similar non-smooth objects can create such echoes.

We implement the following "shotgun" approach:
- Sphere-cast from the source TX towards equidistant points on a bottom-half-dome.
- For all non-occluded rays, sphere-cast a reflected version.
- Find intersection of reflected rays and a horizontal plane at the Y-position of the target TX.
- For all non-occluded rays, check the distance between reflection-ray-hits and the target TX.
- If any are within a configurable range, find the closest.
- Use the distance of source -> "bounce point" -> target for the [receiveing coroutine](#propagation-speed).

> This method uses many raycasts to determine a path and should be enabled with caution.
> The number and pattern of rays are configurable by passing a mesh to the transceiver. The vertices of the mesh will be used for bottom echoes.

> Note that this method uses a terrain object with a terrain collider as the source for its bottom and considers everything else as occlusions. Any bottom features that you expect echoes from should be part of the terrain and not separate objects.

#### Configuration

![TX](Media/TX.png)

- Sound Velocity: Used for propagation delays.
- Min Channel Radius: The minimum size of an opening to be considered occlusion-free between the source and target TXs. For sphere-casts, the radius of the sphere.
- Enable Echoing: Toggles bottom and surface echoes.
  - Remaining Range Ratio After Echo: Echoes absorb energy that affect the max range of a ping. After a bounce, how much of a pings energy is left to travel further.
  - Bottom Firing Opening Angle: The angle of the cone where bottom-echoes will have their rays fired within. The wider this is, the more likely far-away surfaces will produce a bounce.
  - Bottom Firing Resolution: The number of subdivisions on the half-dome. Determines the number of rays cast for bottom-echoes. The bigger, the more accurate, but also heavier on the CPU.
  - Terrain GO: The _terrain_ object bottom echoes will bounce from.
  - Bottom Echo Tolerance: On the horizontal plane of the target, how close do we allow bounced-rays to come to accept as a "hit". The larger, the more likely a bottom echo will reach targets but also less accurate to real life.
  - Single Ground Echo: Toggle bottom echoes separately.
- Draw Signal Lines: Toggle drawing debug lines for all the rays cast.


## Actuators
Scripts that control articulation bodies using drives defined within.
See [Unity Articulation Bodies](https://docs.unity3d.com/Manual/class-ArticulationBody.html) documentation for details on how these work.

The general idea:
- Actuator script exposes a public method to set all of its relevant variables.
- Some kind of controller (Keyboard, ROS, Internal, etc) calls these methods when relevant.
- In `FixedUpdate()` the actuator sets the Arti. Body properties.

> Optionally, actuators can implement `IROSPublishable` (See [IROSPublishable](#ros)) if they are expected to provide feedback to ROS.


### Joints
These are all straight interfaces to the AB's drives.
  
![Hinge](Media/Hinge.png)
  
- **Hinge**: Rotates the body around an axis with some force in both directions.
  - **Angle(Max)**: Current and maximum angles of the hinge.
  - **Reverse**: Check to reverse direction.
 
![Prismatic](Media/Prismatic.png)

- **Prismatic**: A piston. Moves the body along an axis.
  - **Percentage**: How much of the piston is extended.
  - **Reset value**: The value to set when signalled "reset". Usually the result of a homing sequence on the real thing.



### VBS
A tank that can fill up and empty itself.
Changes its AB's mass.

![VBS](Media/VBS.png)

- **Percentage**: How much of the tank is full.
- **Reset value**: The value to set when signalled "reset". Usually the result of a homing sequence on the real thing.
- **Max Volume_l**: Total fluid volume of the tank.
- **Density**: Density of the fluid stored.

### Propeller
A set of propeller blades.
Spins at a given RPM and applies torque and thrust.
Can be used underwater and in the air for a drone.

Example configs:

AUV                                 |Drone
:----------------------------------:|:---------------------------------------:
![auv propeller](Media/Prop_auv.png)|![drone propeller](Media/Prop_drone.png)

- **Hover Default**: If checked, the propeller will use `Num Propellers`, the mass of the `Base Link Articulation Body` and the `RPM To Force Multiplier` values to calculate the RPM required to keep the assigned `Base Link Articulation Body` afloat and set it when the game starts.
  - This is useful to allow a drone to start in the air already.
  - This assumes all the propellers are pointing towards the ground straight.
- **RPM To Force Multiplier**: This is a property of the propeller itself. We set it to 0.005 here arbitrarily.




## Sensors

![Sensor](Media/Sensor.png)

- **Sensor**: The base class of most sensors. Handles the update frequencies.
  - **Frequency:** Frequency of sensor updates. Limited to a period of Unity's `Time.fixedDeltaTime`. Reading the world faster than it changes produces duplicate readings. If faster sensors are desired, the physics update rate should be increased. See [Unity Simulation Time](https://docs.unity3d.com/ScriptReference/MonoBehaviour.FixedUpdate.html) for details.
  - **Has New Data:** A boolean set by individual sensors to signal an outside component that the sensor has been updated with a new reading. Useful for sensors that output something only when they detect a change.
- **Noise**: All sensors are perfect unless otherwise specified.

### Battery
A simple battery that discharges over time.
Discharge is linear between min and max voltages.

![Battery](Media/Barrey.png)



### Camera Image
A camera sensor that renders an attached Unity camera onto a texture.
This texture can be displayed within Unity or published into ROS.

![Camera Image](Media/CameraImage.png)

- **Play mode preview**: The image can be displayed within Unity game window if the checkbox is checked. The location and size of the preview is configurable.


### DepthPressure
Measures depth using water pressure.
Uses [a water query](#water) to find real depth, then converts that to KPa.

![Pressure](Media/Pressure.png)

- **Max Depth**: If the sensor is deeper than this, it will report the maximum depth it can.
- **Include Atmo. Pressure**: If checked, the reported pressure will include 1Atm constant pressure.


### DVL
Approximates a DVL raycast beams.
- Does a raycast for each beam.
- If some number of rays hit, considers this a bottom lock.
- If bottom lock
  - Reads velocity from the attached body.
  - Casts a 5th beam straight down for altitude.
  - Reads the ranges of each ray.

![DVL](Media/DVL.png)

- **Num Beams**: The number of beams on the DVL can be configured. Most use 4.
- **Min Hits To Report:** How many hitting rays do we consider to be a bottom lock.
- **Angle From Vertical:** The angle of an imaginary line in the middle of all the beams from the vertical axis of the sensor.
- **Rotation Offset**: The angle around the imaginary middle line of each beam. Use to arrange the beams. Where should the 1st beam be in relation to the sensor frame?


### GPS
A GPS antenna.
Uses [a water query](#water) to determine submersion and does not update if submersed.

Since Unity does not have a model of the globe, we must identify a position in Unity coordinates as a global position.
GPS uses this reference point to calculate its relative position in UTM coordinates, then converts the UTM coordinates into Lat/Lon as its output.

![GPS](Media/GPS.png)

- **Fix**: Filled by the sensor when it is above water.

#### GPSReferencePoint
Provides every GPS object in the Unity scene a global reference point.
We usually attach this to geo-referenced objects, like a terrain scan with known coordinates.
The global position can be given in Lat/Lon or UTM coordinates.

There must be exactly one such script in a scene.

![GPSRef](Media/GPSRef.png)

- **Origin is Lat Lon**: If checked, the Lat/Lon fields are used to calculate the UTM fields and vice-versa.
- **Draw Line To Reference Point**: If checked, each GPS object will draw a line to the origin defined here. Can be useful to see sometimes.


### IMU
A simple Inertial Measurement Unit that measures linear and agular velocities and accelerations of its attached body.
Accesses the velocity fields of its attached body.
Accelerations are calculated from ground truth velocity differences at every update.

![IMU](Media/IMU.png)

- **With Gravity**: If checked, IMU will add the gravity vector to its output.



### Leak
A very simple boolean sensor.
Does not simulate anything.
Can be "triggered" from the editor.

![Leak](Media/Leak.png)

- **Leaked**: Check manually while the game is running to trigger.
- **Count**: Number of updates (defined by Frequency field of Sensor) where "Leaked" was checked.


### Sonar
We simulate sonars using many raycasts. 
To keep the simulation running smoothly, these raycasts are done in parallel so thousands of rays can be used without slowing down the simulation.

Most of the parameters and their semantics are shared between different types of sonars:

SideScanSonar(SSS)                  |ForwardLookingSonar(FLS)
:----------------------------------:|:---------------------------------------:
![Sidescan](Media/SSS.png)          |![FLS](Media/FLS.png)

#### Sonar Concepts

- **Type**: The type of sonar, one of
  -  Multi-beam echo-sounder(**MBES**)
     -  Emits a single beam of rays with a configurable breadth and reports back distances and angles to produce a point cloud of hits.
  -  SideScanSonar (**SSS**)
     -  Emits two beams at configurable tilt and breadth and reports back only distances hit.
  -  Forward-looking sonar (**FLS**)
     -  Emits many beams forward at configurable tilt, breadth and FOV and reports back distances and angles to produce a point cloud of hits.
- **Beam**: A beam is a cone (or a slice of one) shaped volume where sound travels. Applicable to all sonar types. Red (Starboard) and blue (Port) bundles of lines are each a beam.
- **Beam Breadth**: The opening angle of the conical beam. From one straight edge to the opposite one. All sonar types use this. The neon green angle on the red beam of SideScanSonar.
- **Tilt**: The angle from the horizontal plane to the nearest part of a beam. Always positive. Usually used with MBES and SSS. The cyan curve between the black horizontal plane and the red beam in both SSS and FLS images.
- **FOV**: Field of View. The side-to-side angle around the vertical axis of many beams. Usually used with an FLS. Neon green
- **Rays**: Raycasting vectors. Approximation of sound propagation. Usually in the order of hundereds per beam. Placed equidistantly within a beam. Individual red or blue lines.


#### Sonar Configuration

![Sonar config](Media/SonarConfig.png)

- **Beam Breadth 3 Decibels Deg**: While the raycasts are done equidistantly, not all angles have the same return intensities. This is the -3dB return angle. Should be less than beam breadth.
- **SideScanSonar**: These are used only if Type == SSS.
  - **Num Buckets Per Beam**: While a SSS can use thousands of rays, it usually reports a fixed number of distance buckets. This is that fixed number. Rays will be consolidated by their hit distances into these buckets.
  - **Is Interferometric**: If checked, the SSS will also report angles in addition to distances. This allows a point cloud to be generated.
- **SSS-Noise**: These are used only if Type == SSS.
  - **Mult Gain**: The return intensities of the rays will be multiplied by this gain.
  - **Use Additive Noise**: If checked, noise will be added to the hit distances of the rays.
    - **Add Noise Std/Mean**: The additive noise is sampled from a Normal distribution with this std. and mean.
- **Load**: Ratio of time taken within one `Time.fixedDeltaTime` by the `FixedUpdate()` method of Sonar. If this value is above 1, simulation will become very sluggish.

#### Sonar Visualization

For all sonars, we can visualize the points that rays hit with colorful lines:

![RayViz](Media/RayViz.png)

Simply add the component `GameUI/RayViewer` to a sonar and check in its config what you want visualized:
- Rays
- Hits
  - Rainbow or not



## ROS
We use a the [main-ros2 branch of the ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) forked here: [ROS TCP Endpoint](https://github.com/KKalem/ROS-TCP-Endpoint) into the humble branch to match the rest of the repositories.

More info can be found [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/2_ros_tcp.md)

The main configuration from the Unity side is the IP:Port settings, found under Robotics->Ros Settings in the Unity editor.

![ROS Settings Window](Media/ROSSettings.png)

- **Connect on Startup**: Check if you want Unity to attempt to connect to the Endpoint when you press play. If the Endpoint is not running, this will keep throwing warnings.
- **Protocol**: Pick ROS2. While there is nothing stopping you from trying ROS2, all the messages we have distributed are for ROS2, meaning you would need to re-generate all the ROS messages yourself.
- **ROS IP/Port**: If you are running Unity in the same machine as your Endpoint, the defaults of 127.0.0.1 and 10000 will work. If you are running the Endpoint on a VM or a different machine on the same network, you need to put that machine's IP:Port here.
- **Show HUD**: If checked, shows the IP address and connection status in the game window. Might be obstrusive. Has no other effect.
- **Keepalive, timeout, sleep times**: Network connection settings. Defaults are good.
- **Listen for TF Messages**: Keep checked. Unless TF information in Unity is not desired.
- **Unity Z Axis Direction**: Keep the default North. Z is forward in Unity, and also North. This coincides with X forward and North of ROS well.

### Core
The core namespace includes things that require extensive knowledge of ROS itself, such as clocks, TimeStamps, Publisher class and ROS Messages.
In most cases, you should not need to think about these beyond using them if you implement new subs/pubs.

We have modified [the example found here](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example/tree/main/Nav2SLAMExampleProject/Assets/Scripts) for Clock and TF.

#### RosMessages
Where we keep all compiled C# versions of the ROS Messages in [smarc2/Messages](https://github.com/smarc-project/smarc2/tree/humble/messages).

The compilation is done from Unity by following Robotics->Generate ROS Messages.

![ROS Messages](Media/ROSMessages.png)

- **ROS messaeg path**: Browse to where your ROS messages are defined. You can select a large parent folder as well. The window will show you the discovered ROS messages/services/actions that it can compile into C#. See the next image.
- **Built message path**: This is a path within the _project_. Unfortunately you can not pick a path inside an assets package. So you will need to _move_ this folder's contents to `SMaRCUnityAssets/Runtime/Scripts/VehicleComponents/ROS/Core/RosMessages` after generating them!

![ROS Messages listed](Media/ROSMessagesListed.png)

- **Build msg**: Clicking this will compile the ROS message into C# in the selected folder. For example, for `CommsMessage.msg`, this will create `RosMessages/smarc_msgs/msg/CommsMessageMsg.cs` which can be used from a Unity script with `using RosMessageTypes.Smarc`.

#### Clock
Publishes the in-game clock since the game has started into the `/clock` topic. 

Any simulation-interacting ROS node should be setting `use_sim_time=true` in their launch and access the time with `secs, nanosecs = node.get_clock().now().seconds_nanoseconds()`.

The reason for publishing clock separately from system time is that the simulation could be faster or slower than real time, thus any node that relies on time must be aware of this discrepency.

#### ROSPublisher
Base class of most publishers.
Handling of publishing frequency, topic and namespacing is done here.

![Publisher](Media/Publisher.png)

- **Frequency**: Independent of the sensor that it is publishing and `Time.fixedDeltaTime`, the publisher can be set to an arbitrary frequency. While it won't stop you from publishing faster than the physics updates, this is probably not going to be useful.
- **Topic**: If the topic does not start with `/`, then it is treated as relative, otherwise it is treated as absolute. If relative, then the name of the _root_ object will be prepended to this topic. For example if there is an object in the Unity object hierarchy `sam0/SAMSensors/Battery` with topic `core/battery` the final topic will be `/sam0/core/battery`.
- **Ignore Sensor State**: If checked, the topic will be published at the given frequency and topic regardless of the sensor having any updated data. Uncheck if your sensor only produces a message when it has measured something new.

### Publishers
These  publishers are named as `{SensorName}_Pub` to make it clear in the editor which is which.

Publisher | Message type
:----|----:
AcoustiveReceiver_Pub | **smarc_msgs/StringStamped**
Battery_Pub | sensor_msgs/BatteryState
CameraImage_Pub | sensor_msgs/Image
CameraImageCompressed_Pub | sensor_msgs/CompressedImage
CameraInfo_Pub | sensor_msgs/CameraInfo
DepthPressure_Pub | sensor_msgs/FluidPressure
DVL_Pub | **smarc_msgs/DVL**
GeoPoint_Pub | geographic_msgs/GeoPoint
GPS_Pub | sensor_msgs/NavSatFix
IMU_Pub | sensor_msgs/Imu
Leak_Pub | **smarc_msgs/Leak**
Odometry_Pub | nav_msgs/Odometry
PercentageFeedback_Pub | **sam_msgs/PercentStamped**
PropellerFeedback_Pub | **smarc_msgs/ThrusterFeedback**
SonarPointCloud_Pub | sensor_msgs/PointCloud2
SSS_Pub | **smarc_msgs/Sidescan**

> Notice that there are some publishers that do not have a corresponding sensor, like `Odometry_Pub`. These are mostly there for convenience or ground truth information accessibility from ROS.

#### TF
While the above publishers are all used by attaching to a game object with the corresponding sensor/actuator, the TF Tree is a little more involved.
It **does not** extend the base `ROSPublisher` class.

ROS Transform Tree Publisher can be attached to an otherwise empty game object under the desired robot.
It is a `LinkAttachment` where the attached link (`odom`) in the image below will be the root of the tree published by this publisher.
The script will travel down the object hierarchy and publish the relative poses of all object with a `URDF Link` component. 
**Objects without this component are ignored by the TF Tree.**

![TF](Media/TF.png)

- **Suffix**: This string will be added to the end of _every single TF_ published by this. 
- **Global Frame Ids**: Usually "map". The map frame in Unity refers to the origin of the simulation. Since Unity uses floats for its transforms, we can not set up a simulation in a global reference frame like UTM. Thus all simulations must be in a local map frame. To get around this limitation and still produce global positioning (like for example a GPS sensor) we use a [GPS Reference Point](#gpsreferencepoint).


### Subscribers
Similar to publishers, these are all suffixed `_Sub`.

Subscriber | Message type
:----|----:
AcousticTransmitter_Sub|**smarc_msgs/StringStamped**
HingeCommand_Sub|**sam_msgs/ThrusterAngles**
PercentageCommand_Sub|**sam_msgs/PercentStamped**
PropellerCommand_Sub|**smarc_msgs/ThrusterRPM**
Teleporter_Sub|geometry_msgs/Pose
TFtoUnity_Sub|tf2_msgs/TFMessage

- TFtoUnity_Sub is explained more in the [GUI Section](#gameui).

#### Teleporter_Sub
This component will teleport the attached object according to a Pose from ROS. 
Useful when you want to move an object around from ROS, either to modify a scene from ROS, or repeat an experiment or even use a Unity object as a simple way to visualize something in ROS.

If the attached object is a body, it's velocities will be reset as well.

If the attached object is an articulation body, only the root can be teleported.

#### Actuator Subscriber
Subscribers that control an actuator usually derive from this class, they are usually named `{actuatorName}Command_Sub`.

![Subs](Media/Subscriber.png)

- **Expected/Received Freq.**: The frequency of messages expected by the acutator to work properly. Some actuators reset if no command is given.
- **Resetting**: Set by the subscriber when `ReceivedFreq < ExpectedFreq`. If resetting, the actuator that this subscriber controls is told to reset to its default value. See [actuators](#actuators).
- **Received First Message**: Set by the subscriber when it has received at least one message.




# Utilities
We have some useful things that make life easier when developing a vehicle or a scenario.
These are mostly intended to be used by scene creators to produce prefabs and objects for later use and will likely will never have a game UI parallel.


## Rope

![RopeFancy](Media/RopeFancy.png)

We simulate a rope using a chain of rigid bodies.
Using rigid bodies instead of an articulation body allows us to connect two articulation body chains together with the rope at runtime, since new joints can be added to rigid bodies but not articulation bodies.

![Rope](Media/Rope.png)

To create a rope, you can use the rope generator. 

This component
- generates a rope that is attached to the robot by default.
- can be added to any part of a robot.
- sets the specific parameters of the generated rope.
- can generate a buoy at the end of the rope.

![RopeGen](Media/RopeGen.png)

- **Rope Link Prefab**: The prefab that will be used for each link in the rope chain.
- **Vehicle Connection Name**: Similar to a [link attachment](#linkattachment), the rope will be jointed to this part of the robot.
- **Rope Diameter/Length**: Thickness and total length of the rope. The rope is generated straight.
- **Grams Per Meter**: How heavy the rope is. Each link will be assigned a mass according to this.
- **Buoy Grams**: How heavy a buoy at the end of the rope will be.
- **Rope Collision Diameter**: Small things moving fast makes collision checks very difficult. To go around this limitation, we use a separate size for just the collisions. The collision geometry will be tangent to the visual geometry at the "bottom" and grow from there. This way if the rope is hanging somewhere, it looks visually accurate despite the collisions being unnaturally large.
- **Segment Length**: Length of each rope link segment. The total length will be divided by this to determine the number of segments. Be aware that more segments usually means that the physics engine will explode when forces get bigger. 
- **Segment Mass Ratio**: Mass ratio to assign to each segment. This is a hack within the joint system to allow very-small-mass objects (like a rope link with 5 miligrams of mass) to affect larger-mass objects when connected by joints. 1% seems to be a sweet spot.
- **Rope Replacement Accuracy**: When the rope is tight, and the buoy is attached to something, the rope is replaced by exactly two links. 
  - This allows the rope to carry larger forces without breaking physics or stretching much, since there will be exactly 3 joints after replacement instead of 10s of joints. 
  - At the moment of replacement, rope can jerk the two connected objects around due to small differences in end points. This is the setting to determine how small those differences can be. 
    - The bigger it is, the quicker and easier the rope is replaced, but it will jerk the connected objects. 
    - The smaller it is, the more accurate the rope will be during replacement, but getting the rope to be that accurate while in motion is very difficult. 
    - 2cm default seems to be a happy medium.
- **(Re)Generate Rope**: This will create an object called `Rope` as a child of the object the component is attached to and place all the rope links in there. If the `Rope` object exists, it will delete it and re-create it with the currently set parameters.

A replaced rope attached to a drone, carrying a SAM:

![RopeLinks](Media/Rope2Links.png)

![RopeCarry](Media/RopeCarry.png)

A rope link prefab must contain this component:

![RopeLink](Media/RopeLink.png)

- **Spring/Damper/Max Force**: The rope links are connected by configurable joints with drives. These parameters control the spring system between each link. Playing with these parameters can make a rope stiffer or slacker.



## Importer

URDF into Unity, JSON out of Unity

![Importer](Media/Importer.png)

[URDF Importer docs](https://github.com/Unity-Technologies/URDF-Importer)






# Prefabs

## SAM AUV

![SAM](Media/SAM.png)

We have implemented the AUV that we have developed in the SMaRC Project in the simulator, available as a prefab under `SMARCUnityAssets/Runtime/Prefabs/sam_auv_v1`

### Keyboard Controller

![SAMKey](Media/SAMKeyboard.png)

- **Keys**:
  - **W,A,S,D** controls the thrust vector.
  - **Up/Down arrows** controls thrust.
  - **R,F,C** sets the VBS to 0%, 50%, 100% full respectively.
  - **T,G,V** sets the LCG to 0%, 50%, 100% forward respectively.
- **XXX Go**: No need to modify these unless you add sme new actuators that you'd like to control.

### ROS Controls

All of SAM's actuators and sensors have ROS subs/pubs implemented so SAM is completely controllable through ROS topics.
The topics can be found in the [sam messages package of the smarc2 repository here](https://github.com/smarc-project/smarc2/blob/humble/messages/sam_msgs/msg/Topics.msg)



## Quadrotor

![Quad](Media/Drone.png)

We have implemented a simple quadrotor drone, available as a prefab under `SMARCUnityAssets/Runtime/Prefabs/Quadrotor`.

### Keyboard controller

![DroneKB](Media/DroneKeyboard.png)

- **Keys**: 
  - **I,J,K,L** for horizontal motion.
  - **U,N** for up/down.
  - **Space** for "Lifting mode" that makes the other keys apply a different RPM difference.
- **XXX Prop Go**: The four propellers, probably never need to be touched.
- **Motion RPM**: The RPMs to add/remove to hovering RPMs when moving normally.
- **Lifting RPM**: Extra RPMs over the motion RPMs when pressing *Space* and a direction key.

### Geometric Tracking Controller

![DroneController](Media/DroneController.png)

- **Base Link**: Base link of the drone itself.
- **Control Freq.**: Frequency of control inputs, set to `FixedUpdate()` frequency for best results.
- **Distance Error Cap**: Limit the aggressiveness of the drone motion. The bigger this is, the more aggressively the drone will tilt to propel itself directly at the target.
- **Tracking Target TF**: An object to track. This can be literally anything. The drone will control towards this object. By default the quadrotor prefab has a `DroneTarget` object that you can use. You can move this around in the editor while the game is running or move it any other way, like from ROS using [the teleporter](#teleporter_sub).
- **Load**: This controller can also control for a suspended load.
  - **Rope**: The rope object that the load is suspended with.
  - **Attack The Buoy**: If checked, the drone will ignore Tracking Target TF above and instead go for the buoy of the rope.
  - **Load Link TF**: The transform where the rope's base is attached. Usually a part of another robot.
  

## GameUI
We have a very rudimentary UI for game mode, to change some settings easier while the sim is running.
This is provided as a prefab under `SMARCUnityAssets/Runtime/Prefabs/GUI/GameUI`.

![GUI](Media/UI.png)

- This is designed for a 1080p game window. While it will work on other resolutions, it will probably look bad. You can set your game window to 1080p at the top bar.
- **Panel**: Uncheck to hide the entire thing.
- **Cam**: Cameras in the scene are listed here. You can change which one is rendered on the game window here.
  - Cameras under `GameUI` have the same control scheme as Unity editor's cameras: *Right click* to look around. While holding *right click*, *WASD* to move around and *QE* to move up/down.
- **TF**: If checked, and there is a ROS TF Publisher/Subscriber in the scene, it will be visualized as colored arrows in the game window.
- **Robot Overlay**: If checked, objects tagged "#robot" will have their names displayed on the scene with small arrows, overlaid on their position on screen.
- **Robot**: Objects tagged "#robot" will be listed here. The chosen one will have the following controls:
  - **ROS/Unity**: Checking ROS will disable any keyboard controllers the robot might have to allow control through ROS while checking Unity will disable any [ActuatorSubscriber](#actuator-subscriber)s and enable the keyboard controllers. These two can not be checked at the same time.

![Overlay](Media/overlay.png)

- The names of the robots are displayed with yellow lines and background
- The TF Tree of SAM is displayed with colored arrows.
