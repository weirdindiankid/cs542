# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014


# Lincoln Laboratory Airspace Encounter Model

module LLAEMImpl

export
    AddObserver,

    update,

    generateEncounter,
    getInitialState,
    getNextCommand,
    getTrajectory,

    LLAEM,
    LLAEMInitialState,
    LLAEMCommand


import Compat.ASCIIString

using AbstractEncounterModelImpl
using CommonInterfaces
using RLESUtils, Observers

import CommonInterfaces.addObserver
import CommonInterfaces.update
import AbstractEncounterModelInterfaces.generateEncounter
import AbstractEncounterModelInterfaces.getInitialState
import AbstractEncounterModelInterfaces.getNextCommand
import AbstractEncounterModelInterfaces.getTrajectory


type LLAEMInitialState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64
    h_d::Float64
end

type LLAEMCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64
end

type LLAEM <: AbstractEncounterModel

    initial_sample_filename::ASCIIString
    number_of_initial_samples::Int
    f_init::Union{IOStream, Void}

    transition_sample_filenames::Vector{ASCIIString}
    number_of_transition_samples::Vector{Int}
    f_tran::Vector{Union{IOStream, Void}}

    number_of_aircraft::Int

    A::Int
    L::Int
    C::Vector{Int}
    v_init::Vector{Float64}

    geometry_at_TCA::Vector{Float64}

    states::Array{Float64, 3}
    state_index::Vector{Int}

    dynamic_states::Array{Float64, 3}
    dn_state_index::Vector{Int}


    observer::Observer


    function LLAEM{T <: AbstractString}(
                    number_of_aircraft::Int,
                    initial_sample_filename::AbstractString,
                    number_of_initial_samples::Int,
                    transition_sample_filenames::Vector{T},
                    number_of_transition_samples::Vector{Int})

        obj = new()

        obj.initial_sample_filename = initial_sample_filename
        obj.number_of_initial_samples = number_of_initial_samples
        obj.f_init = nothing

        obj.transition_sample_filenames = transition_sample_filenames
        obj.number_of_transition_samples = number_of_transition_samples
        obj.f_tran = [nothing, nothing]

        obj.number_of_aircraft = number_of_aircraft

        obj.C = zeros(obj.number_of_aircraft)
        obj.v_init = zeros(obj.number_of_aircraft)

        obj.state_index = zeros(obj.number_of_aircraft)
        obj.states = zeros(obj.number_of_aircraft, maximum(obj.number_of_transition_samples), 4)

        obj.dn_state_index = zeros(obj.number_of_aircraft)
        obj.dynamic_states = zeros(obj.number_of_aircraft, maximum(obj.number_of_transition_samples) + 1, 6)

        obj.observer = Observer()

        return obj
    end
end


addObserver(aem::LLAEM, f::Function) = add_observer(aem.observer, f)
addObserver(aem::LLAEM, tag::AbstractString, f::Function) = add_observer(aem.observer, tag, f)


function generateEncounter(aem::LLAEM, sample_number::Int64)

    if sample_number > 0
        reset_sample_from_file(aem)

        for i = 1:sample_number
            initial, transitions = read_sample_from_file(aem, aem.number_of_initial_samples, aem.number_of_transition_samples)

            if initial == nothing || transitions == nothing
                return -1
            end
        end
    else
        initial, transitions = read_sample_from_file(aem, aem.number_of_initial_samples, aem.number_of_transition_samples)

        if initial == nothing || transitions == nothing
            return -1
        end
    end

    # A, L, chi, beta, C1, C2, v1, v2, v1d, v2d, h1d, h2d, psi1d, psi2d, hmd, vmd, h
    convert_units_for_initial(initial)

    # t, h_d, psi_d, v_d, v, x, y, h, psi
    convert_units_for_transitions(transitions)


    A, L, chi, beta_, C1, C2, v1, v2, v1d, v2d, h1d, h2d, psi1d, psi2d, hmd, vmd, h = initial

    aem.A = A
    aem.L = L
    aem.C[1] = C1
    aem.C[2] = C2
    aem.v_init[1] = v1
    aem.v_init[2] = v2

    aem.geometry_at_TCA = [chi, beta_, hmd, vmd, h]

    for i = 1:aem.number_of_aircraft
        aem.states[i, :, 1] = transitions[i, 1:end-1, 1]  # time
        aem.states[i, :, 2] = transitions[i, 1:end-1, 4]  # v_d
        aem.states[i, :, 3] = transitions[i, 1:end-1, 2]  # h_d
        aem.states[i, :, 4] = transitions[i, 1:end-1, 3]  # psi_d
        aem.state_index[i] = 0
    end


    for i = 1:aem.number_of_aircraft
        aem.dn_state_index[i] = 0

        for j = 1:(maximum(aem.number_of_transition_samples) + 1)
            t, h_d, psi_d, v_d, v, x, y, h, psi = transitions[i, j, :]

            aem.dn_state_index[i] += 1
            aem.dynamic_states[i, j, :] = [t, x, y, h, v, psi]
        end
    end


    #for i = 1:(maximum(aem.number_of_transition_samples) + 1)
    #    print(reshape(aem.dynamic_states[1, i, :], 6)')
    #end

    #for i = 1:maximum(aem.number_of_transition_samples)
    #    print(reshape(aem.states[1, i, :], 4)')
    #end
end

generateEncounter(aem::LLAEM) = generateEncounter(aem, 0)

function getInitialState(aem::LLAEM, aircraft_number::Int)

    state = aem.dynamic_states[aircraft_number, 1, :]   # t, x, y, h, v, psi
    command = aem.states[aircraft_number, 1, :]         # t, v_d, h_d, psi_d

    return LLAEMInitialState(state[1], state[2], state[3], state[4], state[5], state[6], command[3])
end

getNextCommand(aem::LLAEM, aircraft_number::Int) = get_next_state(aem, aircraft_number)

function update(aem::LLAEM, aircraft_number::Int)

    command = getNextCommand(aem, aircraft_number)

    if command != nothing
        return LLAEMCommand(command...)
    else
        return nothing
    end
end

function getTrajectory(aem::LLAEM, aircraft_number::Int)

    return reshape(aem.dynamic_states[aircraft_number, :, 1:4], aem.dn_state_index[aircraft_number], 4)
end


function convert_units_for_initial(values)

    # A, L, chi, beta, C1, C2, v1, v2, v1d, v2d, h1d, h2d, psi1d, psi2d, hmd, vmd, h
    values[7] *= 1.68780
    values[8] *= 1.68780
    values[9] *= 1.68780
    values[10] *= 1.68780
    values[11] /= 60
    values[12] /= 60
    values[15] *= 6076.12

    return values
end

function convert_units_for_transitions(values)

    # t, h_d, psi_d, v_d, v, x, y, h, psi
    values[:, :, 2] /= 60
    values[:, :, 4] *= 1.68780
    values[:, :, 5] *= 1.68780

    return values
end

function get_next_state(aem, aircraft_number)

    aem.state_index[aircraft_number] += 1

    if aem.state_index[aircraft_number] > aem.number_of_transition_samples[aircraft_number]
        return nothing
    else
        return reshape(aem.states[aircraft_number, aem.state_index[aircraft_number], :], 4)
    end
end

# TODO support to read number of samples
# returns a sample at a time, currently

function read_sample_from_file(aem, number_of_initial_samples, number_of_transition_samples)

    # 1       id
    # 2       A = [1:4]
    # 3       L = [1:5]
    # 4       chi = [90, 270]   : deg
    # 5       beta = [0:30:360] : deg
    # 6       C1 = [1, 2]
    # 7       C2 = [1, 2]
    # 8,9     v1, v2 = [50, 100, 200, 300, 400, 500, 600]   : kt
    # 10, 11  v1d, v2d = [-5, -2, -0.25, 0.25, 2, 5]        : kt/s
    # 12, 13  h1d, h2d = [-5000, -3000, -2000, -1000, -400, 400, 1000, 2000, 3000, 5000]    : ft/min
    # 14, 15  psi1d, psi2d = [-8, -6, -3.5, -1, -0.25, 0.25, 1, 3.5, 6, 8]  : deg/s
    # 16      hmd = [0, 0.0822896, 0.5, 1, 3]   : nm
    # 17      vmd = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 6000]  : ft
    # 18      h = altitude  : ft

    if aem.f_init == nothing
        aem.f_init = open(aem.initial_sample_filename, "r")
    end

    line = readline(aem.f_init)

    if line == ""
        close(aem.f_init)
        aem.f_init = nothing

        initial = nothing
    else
        initial = split(chomp(line))

        A_ = copy(initial[2])

        if A_ == "b"
            A = 1
        elseif A_ == "c"
            A = 2
        elseif A_ == "d"
            A = 3
        elseif A_ == "o"
            A = 4
        end

        initial = [A, float(initial[3:end])]
    end


    # 1     id
    # 2     time step
    # 3     h1d         : ft/min
    # 4     psi_d       : deg/s
    # 5     v_d         : kt/s
    # 6     v           : kt
    # 7     x           : ft
    # 8     y           : ft
    # 9     h           : ft
    # 10    psi         : deg

    transitions = nothing

    for i = 1:aem.number_of_aircraft
        if aem.f_tran[i] == nothing
            aem.f_tran[i] = open(aem.transition_sample_filenames[i], "r")
        end

        line = readline(aem.f_tran[i])

        if line == ""
            close(aem.f_tran[i])
            aem.f_tran[i] = nothing
        else
            values = float(split(chomp(line)))

            if transitions == nothing
                transitions = Array(Float64, aem.number_of_aircraft, maximum(aem.number_of_transition_samples) + 1, length(values) - 1)
            end

            transitions[i, 1, :] = values[2:end]

            for j = 2:(aem.number_of_transition_samples[i] + 1)
                values = float(split(chomp(readline(aem.f_tran[i]))))

                transitions[i, j, :] = values[2:end]
            end
        end
    end

    return initial, transitions
end

function reset_sample_from_file(aem)

    if aem.f_init != nothing
        close(aem.f_init)
        aem.f_init = nothing
    end

    for i = 1:aem.number_of_aircraft
        if aem.f_tran[i] != nothing
            close(aem.f_tran[i])
            aem.f_tran[i] = nothing
        end
    end
end

end


