# CS 542: Generalized Aircraft Collision Avoidance Through Deep Reinforcement Learning

This is part of our final project for <i>CS 542: An Introduction to Machine Learning</i> with Professor Kate Saenko. In this project, we extend POMDPs to aircraft collision avoidance by modifying the action space on the basis of previously inferred information about aircraft state performance, allowing us to provide highly tailored collision avoidance resolution advisories.

## Authors

Dharmesh Tarapore, dharmesh@bu.edu
Vincent Wahl, vinwah@bu.du
Shantanu Bobhate, sbobhate@bu.edu
Kasim Patel, kasimp93@bu.edu

## Installation

To install the simulator, PyPlot and HDF5 are required. Make sure you are running Julia v0.4 since other versions appear to break various things. For the simplest example, run the installation script from the root directory like so:

```
julia install.jl

```

Once this succeeds, you are ready to begin using the simulator. To run a simulation, try:

```
julia examples/simulate.jl -h
```

#!shell

$ cd examples
$ julia simulate.jl

```


## Layout

```
#!text

SISLES.jl                           Core Encounter simulation module

include/
    SISLES.jl                       Top-level include file for developers
    CommonInterfaces.jl             List common interfaces for modules

common/
    Util.jl                         Utilility module
    format_string.jl                Output "%g" format string
    ObserverImpl/                   Observer pattern implementation for modules

Encounter/
    Encounter.jl                    Encounter generation module
    AbstractEncounterModelImpl.jl   Defines the abstract type for the module
    CorAEMImpl/                     Correlated Airspace Encounter Model
    LLAEMImpl/                      Lincoln Laboratory Airspace Encounter Model

PilotResponse/
    PilotResponse.jl                Pilot response module
    AbstractPilotResponseImpl.jl    Defines the abstract type for the module
    SimplePilotResponseImpl/        Simple model for pilot response

DynamicModel/
    DynamicModel.jl                 Model for dynamics
    AbstractDynamicModelImpl.jl     Defines the abstract type for the module
    SimpleADMImpl/                  Simple model for aircraft dynamics

WorldModel/
    WorldModel.jl                   Model representing the world
    AbstractWorldModelImpl.jl       Defines the abstract type for the module
    AirSpaceImpl/                   Represents airspace

Sensor/
    Sensor.jl                       Sensor module
    AbstractSensorImpl.jl           Defines the abstract type for the module
    SimpleTCASSensorImpl/           Simple TCAS sensor

CollisionAvoidanceSystem/
    CollisionAvoidanceSystem.jl     collision avoidance system module
    AbstractCollisionAvoidanceSystemImpl.jl define abstract type for the module
    SimpleTCASImpl/                 simplified version of TCAS
    GDQNACASImpl/.                  Initial implementation of the generalized deep reinforcement learning CAS

Simulator/
    Simulator.jl                    simulator module
    AbstractSimulatorImpl.jl        define abstract type for the module
    TCASSimulatorImpl/              TCAS simulator
    GDQNACASImpl/.                  Generalized deep reinforcement learning CAS simulator.

examples/
    simulate.jl                     Entry point for most simulations
```


***

*Updated: 12/12/2017*
