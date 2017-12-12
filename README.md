This is the CS 542 fork of SISLES.jl, originally designed by Youngjun Kim.  

# SISLES

SISL + ES  
= Stanford Intelligent Systems Laboratory + Encounter Simulation


## Authors

Youngjun Kim, Ph.D. Candidate, youngjun@stanford.edu

Mykel Kochenderfer, Assistant Professor, mykel@stanford.edu

## Example

To run example code, PyPlot and HDF5 are required.

```
#!shell

$ cd examples
$ julia simulate.jl

```


## Layout

```
#!text

SISLES.jl                           encounter simulation module

include/
    SISLES.jl                       top-level include file for developers
    CommonInterfaces.jl             list common interfaces for modules

common/
    Util.jl                         utilility module
    format_string.jl                output "%g" format string
    ObserverImpl/                   observer pattern implementation for modules

Encounter/
    Encounter.jl                    encounter generation module
    AbstractEncounterModelImpl.jl   define abstract type for the module
    CorAEMImpl/                     Correlated Airspace Encounter Model
    LLAEMImpl/                      Lincoln Laboratory Airspace Encounter Model

PilotResponse/
    PilotResponse.jl                pilot response module
    AbstractPilotResponseImpl.jl    define abstract type for the module
    SimplePilotResponseImpl/        simple model for pilot response

DynamicModel/
    DynamicModel.jl                 model for dynamics
    AbstractDynamicModelImpl.jl     define abstract type for the module
    SimpleADMImpl/                  simple model for aircraft dynamics

WorldModel/
    WorldModel.jl                   model representing world
    AbstractWorldModelImpl.jl       define abstract type for the module
    AirSpaceImpl/                   represent airspace

Sensor/
    Sensor.jl                       sensor module
    AbstractSensorImpl.jl           define abstract type for the module
    SimpleTCASSensorImpl/           simple TCAS sensor

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
