# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module TCASSimulatorImpl

export
    addObserver,

    simulate,
    TCASSimulator


using AbstractSimulatorImpl
using CommonInterfaces

using Base.Test

using Encounter
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CollisionAvoidanceSystem
using RLESUtils, Observers

import CommonInterfaces.addObserver


type SimulationParameters

    em::AbstractEncounterModel
    pr::Vector{AbstractPilotResponse}
    dm::Vector{AbstractDynamicModel}
    wm::AbstractWorldModel
    sr::Vector{AbstractSensor}
    cas::Vector{AbstractCollisionAvoidanceSystem}

    number_of_aircraft::Int


    SimulationParameters() = new()
end

type TCASSimulator <: AbstractSimulator

    parameters::SimulationParameters

    time_step::Float64


    observer::Observer


    function TCASSimulator()

        obj = new()

        obj.parameters = SimulationParameters()
        obj.time_step = 1.

        obj.observer = Observer()

        return obj
    end
end


addObserver(sim::TCASSimulator, f::Function) = add_observer(sim.observer, f)
addObserver(sim::TCASSimulator, tag::AbstractString, f::Function) = add_observer(sim.observer, tag, f)


import Base.convert

convert(::Type{SimpleADMInitialState}, state::Union{CorrAEMInitialState, LLAEMInitialState}) = SimpleADMInitialState(state.t, state.x, state.y, state.h, state.v, state.psi, state.h_d)

convert(::Type{ASWMState}, state::SimpleADMOutputState) = ASWMState(state.t, state.x, state.y, state.h, state.vx, state.vy, state.vh)

convert(::Type{SimplePRCommand}, command::Union{CorrAEMCommand, LLAEMCommand}) = SimplePRCommand(command.t, command.v_d, command.h_d, command.psi_d, 0.0)

convert(::Type{SimpleADMCommand}, command::SimplePRCommand) = SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)

function convert(::Type{SimpleTCASSensorInput}, states::Vector{ASWMState})

    states_ = Array(SimpleTCASSRState, length(states))

    for i = 1:length(states)
        states_[i] = SimpleTCASSRState(states[i].t, states[i].x, states[i].y, states[i].h, states[i].vx, states[i].vy, states[i].vh)
    end

    return SimpleTCASSensorInput(states_)
end

convert(::Type{SimpleTCASInput}, output::SimpleTCASSensorOutput) = SimpleTCASInput(output.t, output.r, output.r_d, output.a, output.a_d, output.h, output.h_d)

convert(::Type{SimplePRResolutionAdvisory}, RA::SimpleTCASResolutionAdvisory) = SimplePRResolutionAdvisory(RA.h_d)
convert(::Type{SimplePRResolutionAdvisory}, RA::Void) = nothing


function simulate(sim::AbstractSimulator; bTCAS = false, sample_number = 0)

    aem = sim.parameters.em
    pr = sim.parameters.pr
    adm = sim.parameters.dm
    as = sim.parameters.wm
    sr = sim.parameters.sr
    cas = sim.parameters.cas

    number_of_aircraft = sim.parameters.number_of_aircraft
    @test number_of_aircraft == 2


    if sample_number == 0
        Encounter.generateEncounter(aem)
    else
        Encounter.generateEncounter(aem, sample_number = sample_number)
    end

    initial_coords = zeros(number_of_aircraft, 3)

    for i = 1:number_of_aircraft
        initial = Encounter.getInitialState(aem, i)
        initial_coords[i, :] = [initial.x, initial.y, initial.h]
    end

    # filter out the encounter within TCAS thresholds
#    for i = 1:number_of_aircraft
#        dmod, zthr = TCAS_thresholds(initial_coords[i, 3])
#
#        for j = (i + 1):number_of_aircraft
#            if norm(initial_coords[i, 1] - initial_coords[j, 1], initial_coords[i, 2] - initial_coords[j, 2]) <= dmod || abs(initial_coords[i, 3] - initial_coords[j, 3]) <= zthr
#                return -1
#            end
#        end
#    end

    for i = 1:number_of_aircraft
        initial = Encounter.getInitialState(aem, i)
        state = DynamicModel.initialize(adm[i], convert(SimpleADMInitialState, initial))
        WorldModel.initialize(as, i, convert(ASWMState, state))

        Sensor.initialize(sr[i])
        CollisionAvoidanceSystem.initialize(cas[i])
        PilotResponse.initialize(pr[i])
    end

    command = nothing

    while true
        for i = 1:number_of_aircraft
            command = Encounter.update(aem, i)

            if command == nothing
                break
            end

            if bTCAS
                states = WorldModel.getAll(as)
                output = Sensor.update(sr[i], convert(SimpleTCASSensorInput, states))
                RA = CollisionAvoidanceSystem.update(cas[i], convert(SimpleTCASInput, output))

                if RA != nothing
                    @notify_observer(sim.observer, "RA", [i, states[i].t, RA.h_d])
                end
            else
                RA = nothing
            end

            response = PilotResponse.update(pr[i], convert(SimplePRCommand, command), convert(SimplePRResolutionAdvisory, RA))
            state = DynamicModel.update(adm[i], convert(SimpleADMCommand, response))
            WorldModel.update(as, i, convert(ASWMState, state))
        end

        if command == nothing
            break
        end

        WorldModel.updateAll(as)
    end

    return 0
end

function TCAS_thresholds(h)

    if h < 1000
        dmod = 0
        zthr = 0
    elseif h < 2350
        dmod = 0.2 * 6076.12
        zthr = 600
    elseif h < 5000
        dmod = 0.35 * 6076.12
        zthr = 600
    elseif h < 10000
        dmod = 0.55 * 6076.12
        zthr = 600
    elseif h < 20000
        dmod = 0.80 * 6076.12
        zthr = 600
    elseif h < 42000
        dmod = 1.10 * 6076.12
        zthr = 700
    else
        dmod = 1.10 * 6076.12
        zthr = 800
    end

    return dmod, zthr
end

end


