# Rationale and Related Word

## The 2 Facets of the Sim2real Gap

The *sim2real gap* is an euphemism for robotic projects that work well on a developer's laptop but not so much in the field.
In aerial robotics, sim2real research often focusses on improving modelling and simulation of complex aerodynamics effects.
Nonetheless, in practice, an equally important component of *sim2real gap* is software engineering.

That is, the challenge of *end-to-end/full-stack integration* among the many frameworks that go into drone autonomy (i.e., a physics engine to simulate the drone dynamics, a rendering engine to generate realistic imagery, a GPU-accelerated machine learning runtime for perception, one or more inter-process and inter-thread communication middleware, the interface to the microcontroller with the autopilot software performing state-estimation and low-level control, the SDK of the deployed embedded system(s), etc.) and reasonably emulated inter-robot communication (in aerial systems, this is heavily affected by the actual flight plans and available RF hardware).

## Why AAS?

- simple (see https://peps.python.org/pep-0020/)
- end-to-end (yolo bounding boxes to autopilot commands)
- deployment focussed (clear split between simulation and drone software modules with Docker)
- emulated intra- (ROS2, XRCE-DDS, MAVSDK, GStreamer) and inter- (serial, IP) robot comms with Docker

## Links to Similar and Related Work

Frameworks (from https://arxiv.org/pdf/2303.18237)

- https://github.com/aerostack2/aerostack2
- https://github.com/ctu-mrs/mrs_uav_system
- https://github.com/IMRCLab/crazyswarm2

Simulators (from https://arxiv.org/pdf/2311.02296)

- https://github.com/ntnu-arl/aerial_gym_simulator
- https://github.com/utiasDSL/gym-pybullet-drones
- https://github.com/spencerfolk/rotorpy

Also check out: https://github.com/ROS-Aerial/aerial_robotic_landscape

## Desiderata/Future Work

Some additional features are highly desirable but were deemed unnecessary for a minimum viable product (MVP):

- Support for [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics
- Support for [Betaflight SITL](https://betaflight.com/docs/development/SITL) interfaced *via* [MultiWii Serial Protocol (MSP)](https://github.com/betaflight/betaflight/tree/master/src/main/msp)
