# Rationale

## The 2 Facets of the Sim2real Gap

The *sim2real gap* is an euphemism for robotic projects that work well on a developer's laptop but not so much in the field.
Aerial sim2real research often focusses on modelling and simulation of complex aerodynamics effects.

Nonetheless, in robotics, an equally important component of *sim2real gap* is software engineering.

That is, the challenge of **full-stack integration** among:

- the **many frameworks** that go into drone autonomy (a physics engine to simulate drone dynamics, a rendering engine to generate realistic imagery, a GPU-accelerated machine learning runtime for perception, one or more inter-process and inter-thread communication middleware, the interface to the microcontroller and autopilot software performing state-estimation and low-level control, the SDKs of the deployed embedded systems)
- emulated **inter-robot communication** (in aerial systems, this is heavily affected by the actual flight plans and available RF hardware)

## Design Manifesto

- **Simplicity** (less is more, ["simple is better than complex"](https://peps.python.org/pep-0020/), and ["worse is better"](https://www.dreamsongs.com/RiseOfWorseIsBetter.html))
- [おまかせ](https://dhh.dk/2012/rails-is-omakase.html) **end-to-end**ness (from camera frames with YOLO bounding boxes, to uORB and MAVLink messages for the autopilot)
- **Deployment** focus
    - Clear, Dockerized split between simulation and aircraft software
    - ROS2 intra-companion board messaging
    - XRCE-DDS (PX4), MAVROS (ArduPilot) autopilot-to-companion board ROS2 UDP bridge
    - GStreamer camera-to-companion board acquisition
    - Zenoh inter-vehicle ROS2 bridge, with networking emulated by `docker network`

## Related Work

A summary of existing multi-drone flight stacks can be found in [Table II of this paper](https://arxiv.org/pdf/2303.18237). Notable ones include:

- *Universidad Politécnica de Madrid (UPM)*'s [`aerostack2`](https://github.com/aerostack2/aerostack2) (multicopter-only)
- *Czech Technical University in Prague (CTU)*'s [`mrs_uav_system`](https://github.com/ctu-mrs/mrs_uav_system) (ROS1, multicopter-only)
- *Technische Universität (TU) Berlin*'s [`crazyswarm2`](https://github.com/IMRCLab/crazyswarm2) (crazyflie-only, indoor)
- *Peking University*'s [`XTDrone`](https://github.com/robin-shaun/XTDrone) (ROS1, PX4-only)

A summary of aerial robotics simulators can be found in [Table IV of this paper](https://arxiv.org/pdf/2311.02296), including:

- *Norwegian University of Science and Technology (NTNU)*'s [`aerial_gym_simulator`](https://github.com/ntnu-arl/aerial_gym_simulator) (high-performance simulator for RL)
- *University of Pennsylvania (UPenn)*'s [`RotorPy`](https://github.com/spencerfolk/rotorpy) (high-fidelity simulator for control)
- *University of Toronto (UofT)*'s [`gym-pybullet-drones`](https://github.com/utiasDSL/gym-pybullet-drones) (simple simulator for education, control, and RL)
- *UZH*'s [`flightmare`](https://github.com/uzh-rpg/flightmare), *ETH*'s [`RotorS`](https://github.com/ethz-asl/rotors_simulator), *NYU*'s [`RotorTM`](https://github.com/arplaboratory/RotorTM), *Microsoft*'s [`AirSim`](https://github.com/microsoft/AirSim), etc.

For even more resources, also check out [`aerial_robotic_landscape`](https://github.com/ROS-Aerial/aerial_robotic_landscape).

## Desiderata/Future Work

Some additional features are highly desirable but were deemed premature for a minimum viable product:

- Support for [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics
- Support for [Betaflight SITL](https://betaflight.com/docs/development/SITL) interfaced *via* [MultiWii Serial Protocol (MSP)](https://github.com/betaflight/betaflight/tree/master/src/main/msp)
- Support for [SPARK-FAST-LIO](https://github.com/MIT-SPARK/spark-fast-lio)
