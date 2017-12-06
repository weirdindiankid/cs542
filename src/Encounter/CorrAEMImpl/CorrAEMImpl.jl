# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


# Correlated Encounter Model for Cooperative Aircraft in the National Airspace
# System Version 1.0

module CorrAEMImpl

export
    AddObserver,

    initialize,
    update,

    generateEncounter,
    getInitialState,
    getNextCommand,
    getTrajectory,

    CorrAEM,
    CorrAEMInitialState,
    CorrAEMCommand,

    generateEncountersToFile,
    setInitialDistributions,
    validate,
    getInitialSample

import Compat.ASCIIString

using AbstractEncounterModelImpl
using AbstractEncounterModelInterfaces
using CommonInterfaces
using Util
using RLESUtils, Observers
using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractEncounterModelInterfaces.generateEncounter
import AbstractEncounterModelInterfaces.getInitialState
import AbstractEncounterModelInterfaces.getNextCommand
import AbstractEncounterModelInterfaces.getTrajectory

import AbstractEncounterModelInterfaces.generateEncountersToFile
import AbstractEncounterModelInterfaces.setInitialDistributions
import AbstractEncounterModelInterfaces.validate
import AbstractEncounterModelInterfaces.getInitialSample

include("corr_aem_load_params.jl")
include("corr_aem_sample.jl")
include("corr_aem_validate.jl")
include("corr_aem_save_scripts.jl")
include("corr_aem_load_scripts.jl")


type CorrAEMInitialState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64 #degrees, x-axis is zero, increasing toward positive y
    h_d::Float64
end

type CorrAEMCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64

end

type CorrAEMParameters

    labels_initial
    n_initial

    G_initial
    r_initial
    N_initial

    labels_transition
    n_transition

    G_transition
    r_transition
    N_transition

    boundaries
    resample_rates

    temporal_map
    zero_bins


    CorrAEMParameters() = new()
end

type CorrAEM <: AbstractEncounterModel

    parameter_filename::ASCIIString
    parameters::CorrAEMParameters

    initial_sample_filename::ASCIIString
    number_of_initial_samples::Int
    f_init::Union{IOStream,Void}

    transition_sample_filename::ASCIIString
    number_of_transition_samples::Int
    f_tran::Union{IOStream, Void}

    b_read_from_file::Bool
    b_write_to_file::Bool
    file_format::ASCIIString

    number_of_encounters_generated::Int
    print_header::Bool

    number_of_aircraft::Int

    initial::Vector{Float64}

    A::Int
    L::Int
    C::Vector{Int}
    v_init::Vector{Float64}

    geometry_at_TCA::Vector{Float64}
    state_tca::Array{Float64, 2}

    states::Array{Float64, 3}
    state_index::Vector{Int}

    dynamic_states::Array{Float64, 3}
    dn_state_index::Vector{Int}

    initial_distributions::Union{Vector{Union{Function, Void}}, Void}
    ISInfo::Array{Float64, 2}


    const default_number_of_initial_samples = 1
    const default_number_of_transition_samples = 50


    observer::Observer


    function CorrAEM(
                    parameter_filename::AbstractString,
                    initial_sample_filename::AbstractString,
                    number_of_initial_samples::Int,
                    transition_sample_filename::AbstractString,
                    number_of_transition_samples::Int;
                    b_read_from_file = false,
                    b_write_to_file = false,
                    file_format = "text")

        obj = new()

        obj.parameter_filename = parameter_filename
        obj.parameters = CorrAEMParameters()
        em_read(obj.parameters, obj.parameter_filename)

        obj.initial_sample_filename = initial_sample_filename
        obj.number_of_initial_samples = number_of_initial_samples
        obj.f_init = nothing

        obj.transition_sample_filename = transition_sample_filename
        obj.number_of_transition_samples = number_of_transition_samples
        obj.f_tran = nothing

        obj.b_read_from_file = b_read_from_file
        obj.b_write_to_file = b_write_to_file
        obj.file_format = file_format

        obj.number_of_encounters_generated = 0
        obj.print_header = true

        @test (b_read_from_file && b_write_to_file) == false

        if b_read_from_file || b_write_to_file
            @test initial_sample_filename != ""
            @test transition_sample_filename != ""
        end

        if b_read_from_file
            @test file_format == "text"
        end

        obj.number_of_aircraft = 2

        obj.C = zeros(obj.number_of_aircraft)
        obj.v_init = zeros(obj.number_of_aircraft)

        obj.state_tca = zeros(obj.number_of_aircraft, 5)

        obj.state_index = zeros(obj.number_of_aircraft)
        obj.states = zeros(obj.number_of_aircraft, obj.number_of_transition_samples, 4)

        obj.dn_state_index = zeros(obj.number_of_aircraft)
        obj.dynamic_states = zeros(obj.number_of_aircraft, obj.number_of_transition_samples + 1, 6)

        obj.initial_distributions = nothing

        obj.observer = Observer()

        return obj
    end

    function CorrAEM(
                    parameter_filename::AbstractString,
                    initial_sample_filename::AbstractString,
                    transition_sample_filename::AbstractString;
                    b_write_to_file::Bool = false,
                    file_format::AbstractString = "text")

        if b_write_to_file
            CorrAEM(parameter_filename,
                    initial_sample_filename,
                    default_number_of_initial_samples,
                    transition_sample_filename,
                    default_number_of_transition_samples,
                    b_write_to_file = true,
                    file_format = file_format)
        else
            CorrAEM(parameter_filename,
                    initial_sample_filename,
                    default_number_of_initial_samples,
                    transition_sample_filename,
                    default_number_of_transition_samples,
                    b_read_from_file = true)
        end
    end

    CorrAEM(
        parameter_filename::AbstractString
        ) = CorrAEM(
                parameter_filename,
                "",
                default_number_of_initial_samples,
                "",
                default_number_of_transition_samples)

    CorrAEM() = new()
end


addObserver(aem::CorrAEM, f::Function) = add_observer(aem.observer, f)
addObserver(aem::CorrAEM, tag::AbstractString, f::Function) = add_observer(aem.observer, tag, f)

function generateEncounter(aem::CorrAEM; sample_number = 0, b_simulate = true)
    params = aem.parameters

    if aem.b_write_to_file && aem.file_format == "text"
        f_initial = open(aem.initial_sample_filename, "a+")
        #f_initial_cmp = open(aem.initial_sample_filename * "_cmp", "w")
        f_transition = open(aem.transition_sample_filename, "a+")

        if aem.print_header
            # print initial headers
            @printf(f_initial, "id ")
            #@printf(f_initial_cmp, "id ")
            for i = 1:params.n_initial
                @printf(f_initial, "%s ", params.labels_initial[i])
                #@printf(f_initial_cmp, "%s ", params.labels_initial[i])
            end
            @printf(f_initial, "\n")
            #@printf(f_initial_cmp, "\n")

            # print transition headers
            @printf(f_transition, "initial_id t ")
            for i = 1:(params.n_transition - params.n_initial)
                @printf(f_transition, "%s ", params.labels_transition[params.temporal_map[i, 2]])
            end
            @printf(f_transition, "\n")

            aem.print_header = false
        end
    end

    if aem.b_read_from_file
        println("read_from_file is true")
        if sample_number > 0
            reset_sample_from_file(aem)

            for i = 1:sample_number
                states = read_sample_from_file(aem, aem.number_of_transition_samples)

                if states == nothing
                    return -1
                end
            end
        else
            states = read_sample_from_file(aem, aem.number_of_transition_samples)
        end
    else
        if aem.initial_distributions == nothing
            states = em_sample(aem, aem.number_of_transition_samples)
        else
            states, aem.ISInfo = em_sample(aem, aem.number_of_transition_samples, initial_dist = aem.initial_distributions)
        end

        aem.number_of_encounters_generated += 1

        states = states[:, 2:end]

        if aem.b_write_to_file && aem.file_format == "text"
            # print initial sample
            @printf(f_initial, "%d ", aem.number_of_encounters_generated)
            #@printf(f_initial_cmp, "%d ", aem.number_of_encounters_generated)
            for j = 2:size(states, 2)
                @printf(f_initial, "%s ", outputGFormatString(states[1, j]))
                #@printf(f_initial_cmp, "%s ", states[1, j])
            end
            @printf(f_initial, "\n")
            #@printf(f_initial_cmp, "\n")

            # print transition samples
            for j = 1:aem.number_of_transition_samples
                @printf(f_transition, "%d %d ", aem.number_of_encounters_generated, j - 1)
                for k = params.temporal_map[:, 1]
                    @printf(f_transition, "%s ", outputGFormatString(states[j, k + 1]))
                end
                @printf(f_transition, "\n")
            end
        end
    end

    if b_simulate
        convert_units(states)

        t, A, L, chi, beta_, C1, C2, v1, v2, v1d, v2d, h1d, h2d, psi1d, psi2d, hmd, vmd = states[1, :]

        aem.initial = vec(states[1, 2:end])

        aem.A = A
        aem.L = L
        aem.C[1] = C1
        aem.C[2] = C2
        aem.v_init[1] = v1
        aem.v_init[2] = v2

        aem.geometry_at_TCA = [chi, beta_, hmd, vmd]

        aem.states[1, :, 1] = states[:, 1]  # time
        aem.states[1, :, 2] = states[:, 10] # v1d
        aem.states[1, :, 3] = states[:, 12] # h1d
        aem.states[1, :, 4] = states[:, 14] # psi1d
        aem.state_index[1] = 0

        aem.states[2, :, 1] = states[:, 1]  # time
        aem.states[2, :, 2] = states[:, 11] # v2d
        aem.states[2, :, 3] = states[:, 13] # h2d
        aem.states[2, :, 4] = states[:, 15] # psi2d
        aem.state_index[2] = 0

        aem.dn_state_index[1] = 0
        aem.dn_state_index[2] = 0

        simulate_tracks(aem)

        transform_regarding_TCA(aem, aem.L, aem.geometry_at_TCA)

        for i = 1:aem.number_of_aircraft
            aem.state_index[i] = 0
        end

        #for i = 1:(aem.number_of_transition_samples + 1)
        #    print(reshape(aem.dynamic_states[1, i, :], 6)')
        #end

        #for i = 1:aem.number_of_transition_samples
        #    print(reshape(aem.states[1, i, :], 4)')
        #end
    end

    if aem.b_write_to_file && aem.file_format == "binary"
        encounters = Array(Any, aem.number_of_aircraft)

        for i = 1:aem.number_of_aircraft
            encounters[i] = Dict()
        end

        # initial
        # airspeed, ft/s, double
        # north position, ft, double
        # east position, ft, double
        # altitude, ft, double
        # heading angle, radians, double
        # flight path angle, radians, double
        # roll angle, radians, double
        # airspeed acceleration, ft/s^2, double

        for i = 1:aem.number_of_aircraft
            t, x, y, h, v, psi = aem.dynamic_states[i, 1, :]
            t_n, x_n, y_n, h_n, v_n, psi_n, = aem.dynamic_states[i, 2, :]

            t, v_d, h_d, psi_d = aem.states[i, 1, :]

            theta = atand((h_n - h) / norm(x_n - x, y_n - y)) |> to_plusminus_180
            phi = 0.0

            encounters[i]["initial"] = Float64[v, x, y, h, deg2rad(psi), deg2rad(theta), deg2rad(phi), v_d]
        end

        # time, s, double
        # vertical rate, ft/s, double
        # turn rate, rad/s, double
        # airspeed acceleration, ft/s^2, double

        for i = 1:aem.number_of_aircraft
            update = Array(Float64, 4, aem.number_of_transition_samples)

            k = 0

            t, v_d, h_d, psi_d = aem.states[i, 1, :]

            k += 1
            update[:, k] = [t, h_d, psi_d * pi / 180, v_d]

            v_d_p = v_d
            h_d_p = h_d
            psi_d_p = psi_d

            for j = 2:aem.number_of_transition_samples
                t, v_d, h_d, psi_d = aem.states[i, j, :]

                if v_d != v_d_p || h_d != h_d_p || psi_d != psi_d_p
                    k += 1
                    update[:, k] = [t, h_d, psi_d * pi / 180, v_d]
                end

                v_d_p = v_d
                h_d_p = h_d
                psi_d_p = psi_d
            end

            encounters[i]["update"] = update[:, 1:k]
        end

        save_scripts(aem.initial_sample_filename, encounters, append = true)
    end

    if aem.b_write_to_file && aem.file_format == "text"
        close(f_initial)
        #close(f_initial_cmp)
        close(f_transition)
    end
end


function generateEncountersToFile(aem::CorrAEM; file_format = "")

    if isfile(aem.initial_sample_filename)
        println("Removing ", aem.initial_sample_filename)
        rm(aem.initial_sample_filename)
    end

    if isfile(aem.transition_sample_filename)
        println("Removing ", aem.transition_sample_filename)
        rm(aem.transition_sample_filename)
    end

    println("write_to_file is ", aem.b_write_to_file, " file_format_orig is ", aem.file_format)
    b_write_to_file_orig = aem.b_write_to_file
    file_format_orig = aem.file_format

    aem.b_write_to_file = true
    if file_format != ""
        aem.file_format = file_format
    end

    for i = 1:aem.number_of_initial_samples
        if aem.file_format == "text"
            generateEncounter(aem, b_simulate = false)
        elseif aem.file_format == "binary"
            generateEncounter(aem)
        end
    end

    aem.b_write_to_file = b_write_to_file_orig
    aem.file_format = file_format_orig
end

function getInitialState(aem::CorrAEM, aircraft_number::Int)

    state = aem.dynamic_states[aircraft_number, 1, :]   # t, x, y, h, v, psi
    command = aem.states[aircraft_number, 1, :]         # t, v_d, h_d, psi_d

    return CorrAEMInitialState(state[1], state[2], state[3], state[4], state[5], state[6], command[3])
end

getNextCommand(aem::CorrAEM, aircraft_number::Int) = get_next_state(aem, aircraft_number)

function initialize(aem::CorrAEM)

  for i = 1:aem.number_of_aircraft
    aem.state_index[i] = 0
  end

end

function update(aem::CorrAEM, aircraft_number::Int)

    command = getNextCommand(aem, aircraft_number)

    if command != nothing
        return CorrAEMCommand(command...)
    else
        return nothing
    end
end

function getTrajectory(aem::CorrAEM, aircraft_number::Int)

    return reshape(aem.dynamic_states[aircraft_number, :, 1:4], aem.dn_state_index[aircraft_number], 4)
end

function setInitialDistributions(aem::CorrAEM, n::Int, f::Function)

    if aem.initial_distributions == nothing
        aem.initial_distributions = Array(Union{Function,Void}, 16)

        for i = 1:16
            aem.initial_distributions[i] = nothing
        end
    end

    aem.initial_distributions[n] = f
end

function getInitialSample(aem::CorrAEM, index::Int)

    return aem.initial[index]
end

function getInitialSample(aem::CorrAEM, var::Symbol=:all)

    if var == :all
        return aem.initial
    elseif var == :A
        return aem.initial[1]
    elseif var == :L
        return aem.initial[2]
    elseif var == :chi
        return aem.initial[3]
    elseif var == :beta
        return aem.initial[4]
    elseif var == :C1
        return aem.initial[5]
    elseif var == :C2
        return aem.initial[6]
    elseif var == :v1
        return aem.initial[7]
    elseif var == :v2
        return aem.initial[8]
    elseif var == :v1d
        return aem.initial[9]
    elseif var == :v2d
        return aem.initial[10]
    elseif var == :h1d
        return aem.initial[11]
    elseif var == :h2d
        return aem.initial[12]
    elseif var == :psi1d
        return aem.initial[13]
    elseif var == :psi2d
        return aem.initial[14]
    elseif var == :hmd
        return aem.initial[15]
    elseif var == :vmd
        return aem.initial[16]
    else
        return nothing
    end
end


function transform_regarding_TCA(aem, L, geometry_at_TCA)

    chi, beta_, hmd, vmd = geometry_at_TCA

    x1_tca, y1_tca, h1_tca, v1_tca, psi1_tca = aem.state_tca[1, :]
    x2_tca, y2_tca, h2_tca, v2_tca, psi2_tca = aem.state_tca[2, :]

    AC1_state = reshape(aem.dynamic_states[1, 1:aem.dn_state_index[1], :], aem.dn_state_index[1], 6)
    AC2_state = reshape(aem.dynamic_states[2, 1:aem.dn_state_index[2], :], aem.dn_state_index[2], 6)


    AC1_state[:, 2] -= x1_tca
    AC1_state[:, 3] -= y1_tca

    AC1_state[:, 2:3] = transpose([cosd(-psi1_tca) -sind(-psi1_tca); sind(-psi1_tca) cosd(-psi1_tca)] * AC1_state[:, 2:3]')

    if L == 1
        h_trans = 1000 + (3000 - 1000) * rand()
    elseif L == 2
        h_trans = 3000 + (10000 - 3000) * rand()
    elseif L == 3
        h_trans = 10000 + (18000 - 10000) * rand()
    elseif L == 4
        h_trans = 18000 + (29000 - 18000) * rand()
    elseif L == 5
        h_trans = 29000 + (50000 - 29000) * rand()
    end

    AC1_state[:, 4] += (-h1_tca + h_trans)

    psi = update_psi(AC1_state[1, 2:3], AC1_state[2, 2:3])
    AC1_state[:, 6] = map(to_plusminus_180, AC1_state[:, 6] + psi) #make sure we're in [-180, 180]

    AC2_state[:, 2] -= x2_tca
    AC2_state[:, 3] -= y2_tca

    psi2_0 = beta_ - psi2_tca
    AC2_state[:, 2:3] = transpose([cosd(psi2_0) -sind(psi2_0); sind(psi2_0) cosd(psi2_0)] * AC2_state[:, 2:3]')

    v_r_tca = [v2_tca * cosd(beta_) - v1_tca, v2_tca * sind(beta_)]

    x_trans = hmd / norm(v_r_tca) * [-v_r_tca[2], v_r_tca[1]]

    if chi == 1     # front
        if x_trans[1] < 0
            x_trans = -x_trans
        end
    else
        if x_trans[1] >= 0
            x_trans = -x_trans
        end
    end

    AC2_state[:, 2] += x_trans[1]
    AC2_state[:, 3] += x_trans[2]

    AC2_state[:, 4] += (-h2_tca + h_trans - vmd)

    psi = update_psi(AC2_state[1, 2:3], AC2_state[2, 2:3])
    AC2_state[:, 6] = map(to_plusminus_180, AC2_state[:, 6] + psi) #make sure we're in [-180, 180]

    aem.dynamic_states[1, 1:aem.dn_state_index[1], :] = reshape(AC1_state, 1, 
        size(AC1_state, 1), 6)
    aem.dynamic_states[2, 1:aem.dn_state_index[2], :] = reshape(AC2_state, 1, 
        size(AC2_state, 1), 6)
end

function update_psi(state1, state2)

    x1, y1 = state1
    x2, y2 = state2

    x = x2 - x1
    y = y2 - y1

    if x >= 0 && y >= 0
        psi = atand(y / x)
    elseif x <= 0 && y >= 0
        psi = atand(y / x) + 180
    elseif x <= 0 && y <= 0
        psi = atand(y / x) + 180
    elseif x >= 0 && y <= 0
        psi = atand(y / x) + 360
    end

    return psi
end

function simulate_tracks(aem)

    for i = 1:aem.number_of_aircraft
        t_curr, v_d_curr, h_d_curr, psi_d_curr = get_next_state(aem, i)
        v_curr = aem.v_init[i]

        aem.dn_state_index[i] += 1
        aem.dynamic_states[i, aem.dn_state_index[i], :] = [t_curr, 0., 0., 0, v_curr, 0.]

        while true
            state_next = get_next_state(aem, i)

            if state_next != nothing
                t_next, v_d_next, h_d_next, psi_d_next = state_next

                sim_time = t_next
            else    # last second
                sim_time += 1
            end

            #if i == 1
            #    print([sim_time, v_d_curr, h_d_curr, psi_d_curr]')
            #end

            state = update_state(aem, i, sim_time, v_d_curr, h_d_curr, psi_d_curr)

            if abs(sim_time - 40) < 0.001
                aem.state_tca[i, :] = [state...]
            end

            if state_next == nothing
                break
            else
                v_d_curr = v_d_next
                h_d_curr = h_d_next
                psi_d_curr = psi_d_next
            end
        end
    end
end

function update_state(aem, aircraft_number, t, v_d, h_d, psi_d)

    theta_regulated = 45    # degree

    t_, x, y, h, v, psi = aem.dynamic_states[aircraft_number, aem.dn_state_index[aircraft_number], :]

    if abs(h_d) > abs(v) * sind(theta_regulated)
        h_d = sign(h_d) * abs(v) * sind(theta_regulated)
    end

    dt = t - t_

    x_n = x + sqrt(v^2 - h_d^2) * cosd(psi) * dt
    y_n = y + sqrt(v^2 - h_d^2) * sind(psi) * dt

    v_n = v + v_d * dt  # ft/s
    h_n = h + h_d * dt  # ft
    psi_n = psi + psi_d * dt    # deg

    aem.dn_state_index[aircraft_number] += 1
    aem.dynamic_states[aircraft_number, aem.dn_state_index[aircraft_number], :] = [t, x_n, y_n, h_n, v_n, psi_n]

    return x_n, y_n, h_n, v_n, psi_n
end

function convert_units(values)

    values[:, 8] *= 1.68780
    values[:, 9] *= 1.68780
    values[:, 10] *= 1.68780
    values[:, 11] *= 1.68780
    values[:, 12] /= 60
    values[:, 13] /= 60
    values[:, 16] *= 6076.12

    return values
end

function get_next_state(aem, aircraft_number)

    aem.state_index[aircraft_number] += 1

    if aem.state_index[aircraft_number] > aem.number_of_transition_samples
        return nothing
    else
        return reshape(aem.states[aircraft_number, aem.state_index[aircraft_number], :], 4)
    end
end

# TODO support to read number of samples
# returns a sample at a time, currently

function read_sample_from_file(aem, number_of_transition_samples)

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

    if aem.f_init == nothing
        aem.f_init = open(aem.initial_sample_filename, "r")
        readline(aem.f_init)
    end

    line = readline(aem.f_init)

    if line == ""
        close(aem.f_init)
        aem.f_init = nothing

        initial = nothing
    else
        initial = float(split(chomp(line)))
    end


    # 1     id
    # 2     time step
    # 3, 4  h1d, h2d        : ft/min
    # 5, 6  psi1d, psi2d    : deg/s

    if aem.f_tran == nothing
        aem.f_tran = open(aem.transition_sample_filename, "r")
        readline(aem.f_tran)
    end

    line = readline(aem.f_tran)

    if line == ""
        close(aem.f_tran)
        aem.f_tran = nothing

        transitions = nothing
    else
        values = float(split(chomp(line)))

        transitions = Array(Float64, aem.number_of_transition_samples, length(values))
        transitions[1, :] = values

        for i = 2:aem.number_of_transition_samples
            transitions[i, :] = float(split(chomp(readline(aem.f_tran))))
        end
    end

    if initial != nothing && transitions != nothing
        states = repmat(initial', size(transitions, 1))
        states[:, 1] = transitions[:, 2]
        states[:, aem.parameters.temporal_map[:, 1] + 1] = transitions[:, 3:(3 + length(aem.parameters.temporal_map[:, 1]) - 1)]

        return states
    else
        return nothing
    end
end

function reset_sample_from_file(aem)

    if aem.f_init != nothing
        close(aem.f_init)
        aem.f_init = nothing
    end

    if aem.f_tran != nothing
        close(aem.f_tran)
        aem.f_tran = nothing
    end
end

#mods x to the range [-b, b]
function to_plusminus_b(x::AbstractFloat, b::AbstractFloat)
  z = mod(x, 2 * b)
  return (z > b) ? (z - 2 * b) : z
end

to_plusminus_180(x::AbstractFloat) = to_plusminus_b(x, 180.0)

end


