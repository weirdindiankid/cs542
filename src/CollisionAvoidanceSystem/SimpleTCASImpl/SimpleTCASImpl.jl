# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


# + References
# Kochenderfer, M., Espindle, L., Kuchar, J., and Griffith, J. D., "Correlated encounter model for cooperative aircraft in the national airspace system version 1.0," Project Report ATC-344, Lincoln Laboratory, 2008. Appendix D.
# "Introduction to TCAS II version 7.1," Tech. rep., Federal Aviation Administration, 2011.
# Kochenderfer, M. J., Chryssanthacopoulos, J. P., Kaelbling, L. P., and Lozano-Perez, T., "Model-based optimization of airborne collision avoidance logic," Project Report ATC-360, Lincoln Laboratory, 2010. Appendix I.


module SimpleTCASImpl

export
    AddObserver,

    initialize,
    update,

    testThreat,
    selectRA,

    SimpleTCAS,
    SimpleTCASInput,
    SimpleTCASResolutionAdvisory


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces

using Util
using RLESUtils, Observers

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractCollisionAvoidanceSystemInterfaces.testThreat
import AbstractCollisionAvoidanceSystemInterfaces.selectRA


type SimpleTCASInput

    t::Float64

    r::Float64
    r_d::Float64

    a::Float64
    a_d::Float64

    h::Vector{Float64}
    h_d::Vector{Float64}
end

type SimpleTCASResolutionAdvisory

    h_d::Float64
end

type SimpleTCAS <: AbstractCollisionAvoidanceSystem

    b_TCAS_activated::Bool
    RA::Union{SimpleTCASResolutionAdvisory,Void}

    observer::Observer


    function SimpleTCAS()

        obj = new()

        obj.b_TCAS_activated = false
        obj.RA = nothing

        obj.observer = Observer()

        return obj
    end
end


addObserver(cas::SimpleTCAS, f::Function) = add_observer(cas.observer, f)
addObserver(cas::SimpleTCAS, tag::AbstractString, f::Function) = add_observer(cas.observer, tag, f)


function testThreat(cas::SimpleTCAS, input::SimpleTCASInput)

    b_TCAS_activated = false

    r = input.r
    r_dot = input.r_d

    a = input.a
    a_dot = input.a_d

    sl, tau, dmod, zthr, alim = simple_TCAS_thresholds(input.h[1])

    if (r_dot < 0 && (-(r - dmod) / r_dot <= tau || r < dmod)) && ((a_dot < 0 && -a / a_dot <= tau) || a <= zthr)
        b_TCAS_activated = true
    end

    return b_TCAS_activated
end

function selectRA(cas::SimpleTCAS, input::SimpleTCASInput)

    r = input.r
    r_dot = input.r_d

    a = input.a
    a_dot = input.a_d

    h1 = input.h[1]
    h2 = input.h[2]

    h1_dot = input.h_d[1]
    h2_dot = input.h_d[2]

    sl, tau, dmod, zthr, alim = simple_TCAS_thresholds(input.h[1])

    ascend_cross = false
    ascend_dist = 0
    ascend_alim = false

    descend_cross = false
    descend_dist = 0
    descend_alim = false

    t_ = -r / r_dot

    # debug
    if false && t_ < 0
        println("t, r, r_dot, a, a_dot, h1, h2, h1_dot, h2_dot")
        println([input.t, input.r, input.r_d, input.a, input.a_d, input.h[1], input.h[2], input.h_d[1], input.h_d[2]]')

        println("sl, tau, dmod, zthr, alim")
        println([sl, tau, dmod, zthr, alim]')

        println("tests")
        println([r_dot < 0, -(r - dmod) / r_dot <= tau, r < dmod, a_dot < 0, -a / a_dot <= tau, a <= zthr]')

        println("selects")
        println(t_)
        println()

        @notify_observer(cas.observer, "debug", [false])

        return 0.
    end
    @assert t_ > 0

    h_dot_ascend = 1500 / 60

    h1_cpa = h1 + h_dot_ascend * t_
    h2_cpa = h2 + h2_dot * t_

    if (h1 <= h2 && h1_cpa >= h2_cpa) || (h1 >= h2 && h1_cpa <= h2_cpa)
        ascend_cross = true
    end

    ascend_dist = abs(h1_cpa - h2_cpa)
    if ascend_dist >= alim
        ascend_alim = true
    end

    h1_dot_descend = - 1500 / 60

    h1_cpa = h1 + h1_dot_descend * t_
    h2_cpa = h2 + h2_dot * t_

    if (h1 <= h2 && h1_cpa >= h2_cpa) || (h1 >= h2 && h1_cpa <= h2_cpa)
        descend_cross = true
    end

    descend_dist = abs(h1_cpa - h2_cpa)
    if descend_dist >= alim
        descend_alim = true
    end

    if ascend_cross
        if descend_alim
            resolution_advisory = - 1500 / 60
        elseif ascend_alim
            resolution_advisory = 1500 / 60
        else
            resolution_advisory = - 1500 / 60
        end
    elseif descend_cross
        if ascend_alim
            resolution_advisory = 1500 / 60
        elseif descend_alim
            resolution_advisory = - 1500 / 60
        else
            resolution_advisory = 1500 / 60
        end
    else
        if descend_alim && ascend_alim
            if descend_dist < ascend_dist
                resolution_advisory = - 1500 / 60
            else
                resolution_advisory = 1500 / 60
            end
        elseif descend_alim
            resolution_advisory = - 1500 / 60
        elseif ascend_alim
            resolution_advisory = 1500 / 60
        else
            if descend_dist > ascend_dist
                resolution_advisory = - 1500 / 60
            else
                resolution_advisory = 1500 / 60
            end
        end
    end

    # debug
    if false
        println("t, r, r_dot, a, a_dot, h1, h2, h1_dot, h2_dot")
        println([input.t, input.r, input.r_d, input.a, input.a_d, input.h[1], input.h[2], input.h_d[1], input.h_d[2]]')

        println("sl, tau, dmod, zthr, alim")
        println([sl, tau, dmod, zthr, alim]')

        println("tests")
        println([r_dot < 0, -(r - dmod) / r_dot <= tau, r < dmod, a_dot < 0, -a / a_dot <= tau, a <= zthr]')

        println("selects")
        println(Any[t_, ascend_cross, ascend_dist, ascend_alim, descend_cross, descend_dist, descend_alim, resolution_advisory]')
    end

    @notify_observer(cas.observer, "debug", Any[Any[input.t, input.r, input.r_d, input.a, input.a_d, input.h, input.h_d], Any[sl, tau, dmod, zthr, alim], Any[r_dot < 0, -(r - dmod) / r_dot <= tau, r < dmod, a_dot < 0, -a / a_dot <= tau, a <= zthr], Any[t_, ascend_cross, ascend_dist, ascend_alim, descend_cross, descend_dist, descend_alim, resolution_advisory]])

    return resolution_advisory
end

update(cas::SimpleTCAS, input) = update(cas,convert(SimpleTCASInput, input))

function update(cas::SimpleTCAS, input::SimpleTCASInput)

    if cas.b_TCAS_activated == false
        cas.b_TCAS_activated = testThreat(cas, input)

        if cas.b_TCAS_activated
            cas.RA = SimpleTCASResolutionAdvisory(selectRA(cas, input))
        end
    end

    if cas.RA == nothing
        return nothing
    else
        return cas.RA
    end
end

function initialize(cas::SimpleTCAS)
    cas.b_TCAS_activated = false
    cas.RA = nothing
end


function simple_TCAS_thresholds(h)

    if h < 1000
        sl = 2
        tau = 0
        dmod = 0
        zthr = 0
        alim = 0
    elseif h < 2350
        sl = 3
        tau = 15
        dmod = 0.2 * 6076.12
        zthr = 600
        alim = 300
    elseif h < 5000
        sl = 4
        tau = 20
        dmod = 0.35 * 6076.12
        zthr = 600
        alim = 300
    elseif h < 10000
        sl = 5
        tau = 25
        dmod = 0.55 * 6076.12
        zthr = 600
        alim = 350
    elseif h < 20000
        sl = 6
        tau = 30
        dmod = 0.80 * 6076.12
        zthr = 600
        alim = 400
    elseif h < 42000
        sl = 7
        tau = 35
        dmod = 1.10 * 6076.12
        zthr = 700
        alim = 600
    else
        sl = 7
        tau = 35
        dmod = 1.10 * 6076.12
        zthr = 800
        alim = 700
    end

    return sl, tau, dmod, zthr, alim
end

end


