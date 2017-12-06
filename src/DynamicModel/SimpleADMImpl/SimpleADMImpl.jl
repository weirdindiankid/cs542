# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module SimpleADMImpl

export
    addObserver,

    initialize,
    update,

    initializeDynamicModel,
    simulateDynamicModel,

    SimpleADM,
    SimpleADMInitialState,
    SimpleADMOutputState,
    SimpleADMCommand


using AbstractDynamicModelImpl
using AbstractDynamicModelInterfaces
using CommonInterfaces
using RLESUtils, Observers

using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractDynamicModelInterfaces.initializeDynamicModel
import AbstractDynamicModelInterfaces.simulateDynamicModel


type SimpleADMInitialState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64
    h_d::Float64
end

type SimpleADMState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    v::Float64
    psi::Float64
end

type SimpleADMOutputState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    vx::Float64
    vy::Float64
    vh::Float64
end

type SimpleADMCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64
end

type SimpleADM <: AbstractDynamicModel

    state::SimpleADMState
    command::Union{SimpleADMCommand, Void}

    timestep::Float64
    number_of_substeps::Int

    theta_regulated::Int

    v_d_max::Float64 #feet per second squared
    h_dd_max::Float64 #feet per second squared
    psi_d_max::Float64 #degrees per second

    observer::Observer


    function SimpleADM(;timestep = 1., number_of_substeps = 10,
                       v_d_max::Float64=typemax(Float64),
                       h_dd_max::Float64=typemax(Float64),
                       psi_d_max::Float64=typemax(Float64))

        obj = new()

        obj.command = nothing
        obj.timestep = timestep
        obj.number_of_substeps = number_of_substeps
        obj.theta_regulated = 45    # degree

        obj.v_d_max = v_d_max
        obj.h_dd_max = h_dd_max
        obj.psi_d_max = psi_d_max

        obj.observer = Observer()

        return obj
    end
end


addObserver(adm::SimpleADM, f::Function) = add_observer(adm.observer, f)
addObserver(adm::SimpleADM, tag::AbstractString, f::Function) = add_observer(adm.observer, tag, f)


function initializeDynamicModel(adm::SimpleADM, state::SimpleADMInitialState)

    adm.state = SimpleADMState(state.t, state.x, state.y, state.h, state.v, state.psi)
    #print([state.t, state.x, state.y, state.h, state.v, state.psi]')

    @notify_observer(adm.observer, "adm_init", [adm.state.t, adm.state.x, adm.state.y, adm.state.h])

    vh = state.h_d
    vx = sqrt(adm.state.v^2 - vh^2) * cosd(adm.state.psi)
    vy = sqrt(adm.state.v^2 - vh^2) * sind(adm.state.psi)

    adm.command = nothing

    return SimpleADMOutputState(adm.state.t, adm.state.x, adm.state.y, adm.state.h, vx, vy, vh)
end

initialize(adm::SimpleADM, state) = initialize(adm,convert(SimpleADMInitialState, state))

initialize(adm::SimpleADM, state::SimpleADMInitialState) = initializeDynamicModel(adm, state)

function simulateDynamicModel(adm::SimpleADM, command::SimpleADMCommand)

    t, x, y, h, v, psi = adm.state.t, adm.state.x, adm.state.y, adm.state.h, adm.state.v, adm.state.psi

    if adm.command == nothing    # first second
        t_prev, v_d_prev, h_d_prev, psi_d_prev = t - 1, command.v_d, command.h_d, command.psi_d
    else
        t_prev, v_d_prev, h_d_prev, psi_d_prev = adm.command.t, adm.command.v_d, adm.command.h_d, adm.command.psi_d
    end
    t_curr, v_d_curr, h_d_curr, psi_d_curr = command.t, command.v_d, command.h_d, command.psi_d

    #limit acclerations and rates
    v_d_curr = min(max(v_d_curr,-adm.v_d_max),adm.v_d_max)
    psi_d_curr = min(max(psi_d_curr,-adm.psi_d_max),adm.psi_d_max)

    h_dd = (h_d_curr - h_d_prev) / adm.timestep
    h_dd = min(max(h_dd,-adm.h_dd_max),adm.h_dd_max)
    h_d_curr = h_d_prev + h_dd * adm.timestep
    command.h_d = h_d_curr #propagated to the next time step as prev

    #@test t == t_curr
    #@test t_prev + 1 == t_curr

    dt = adm.timestep / adm.number_of_substeps

    t_sim = t

    x_n = x
    y_n = y
    h_n = h
    v_n = v
    psi_n = psi

    v_d = v_d_prev
    h_d = h_d_prev
    psi_d = psi_d_prev

    #print([t_curr, v_d_curr, h_d_curr, psi_d_curr]')

    for i = 1:adm.number_of_substeps
        # update simulation time
        t_sim += dt


        # update control

        # linear control
        dv_d = (v_d_curr - v_d_prev) * (t_sim - t_curr)
        dh_d = (h_d_curr - h_d_prev) * (t_sim - t_curr)
        dpsi_d = (psi_d_curr - psi_d_prev) * (t_sim - t_curr)

        # sigmoid control
        #CMD_DIST = Normal(0.3, 0.07)
        #dv_d = (v_d_curr - v_d_prev) * cdf(CMD_DIST, (t_sim - t_curr))
        #dh_d = (h_d_curr - h_d_prev) * cdf(CMD_DIST, (t_sim - t_curr))
        #dpsi_d = (psi_d_curr - psi_d_prev) * cdf(CMD_DIST, (t_sim - t_curr))

        v_d = v_d_prev + dv_d
        h_d = h_d_prev + dh_d
        psi_d = psi_d_prev + dpsi_d

        #println(v_d, " ", h_d, " ", psi_d)

        if abs(h_d) > abs(v_n) * sind(adm.theta_regulated)
            h_d = sign(h_d) * abs(v_n) * sind(adm.theta_regulated)
            #println("regulated")
        end


        # update state

        x_n += sqrt(v_n^2 - h_d^2) * cosd(psi) * dt
        y_n += sqrt(v_n^2 - h_d^2) * sind(psi) * dt

        v_n += v_d * dt     # ft/s
        h_n += h_d * dt     # ft
        psi_n += psi_d * dt # deg

        #print([t_sim, x_n, y_n, h_n, v_n, psi_n]')

        @notify_observer(adm.observer, "adm_simulate", [t_sim, x_n, y_n, h_n])
    end

    @test_approx_eq_eps t_sim (t_curr + adm.timestep) 0.001
    @test_approx_eq_eps v_d v_d_curr 0.001
    #@test_approx_eq_eps h_d h_d_curr 0.001
    @test_approx_eq_eps psi_d psi_d_curr 0.001

    adm.state = SimpleADMState(t_curr + adm.timestep, x_n, y_n, h_n, v_n, psi_n)
    adm.command = deepcopy(command)
end

update(adm::SimpleADM, command) = update(adm,convert(SimpleADMCommand, command))

function update(adm::SimpleADM, command::SimpleADMCommand)

    t =  copy(adm.state.t)
    x =  copy(adm.state.x)
    y =  copy(adm.state.y)
    h =  copy(adm.state.h)

    simulateDynamicModel(adm, command)

    t_n =  adm.state.t
    x_n =  adm.state.x
    y_n =  adm.state.y
    h_n =  adm.state.h

    vx_n = (x_n - x) / (t_n - t)
    vy_n = (y_n - y) / (t_n - t)
    vh_n = (h_n - h) / (t_n - t)

    # instantaneous velocities
    #vh_n = adm.command.h_d
    #vx_n = sqrt(adm.state.v^2 - vh_n^2) * cosd(adm.state.psi)
    #vy_n = sqrt(adm.state.v^2 - vh_n^2) * sind(adm.state.psi)

    return SimpleADMOutputState(t_n, x_n, y_n, h_n, vx_n, vy_n, vh_n)
end

end


