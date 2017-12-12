# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 12/11/2014

module ACASX_GM_Impl

import Compat.ASCIIString

using AbstractGenerativeModelImpl
using AbstractGenerativeModelInterfaces
using CommonInterfaces

using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator
using RLESUtils, Observers

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractGenerativeModelInterfaces.get
import AbstractGenerativeModelInterfaces.isterminal

import CommonInterfaces.addObserver
import CommonInterfaces.clearObservers!

export ACASX_GM_params, ACASX_GM, initialize, update, isterminal, addObserver, clearObservers!

using CASInterface

#TODO: Not all variables are used by all models, provide a system for 
#managing parameters, e.g., dict + kwargs passing
type ACASX_GM_params
    #global params: remains constant per sim
    encounter_number::Int64 #encounter number in file (LLCEMDEBN only)
    encounter_seed::UInt64 #Seed for generating encounters
    nmac_r::Float64 #NMAC radius in feet
    nmac_h::Float64 #NMAC vertical separation, in feet
    max_steps::Int64 #maximum number of steps in sim
    num_aircraft::Int64 #number of aircraft  #must be 2 for now...
    encounter_model::Symbol #{:LLCEMDBN, :StarDBN, :SideOnDBN}
    encounter_equipage::Symbol #{:EvE, :EvU}
    response_model::Symbol #{:ICAO, :ICAOVsNone, :StochasticLinear}
    cas_model::Symbol #{:CCAS, :ADD}
    dynamics_model::Symbol #{:LLADM}
    end_on_nmac::Bool #end scenario on nmac

    #transition DBN
    encounter_file::ASCIIString #Path to encounter file

    #LLCEMDEBN only
    command_method::Symbol #:DBN=sampled from DBN or :ENC=from encounter file
    initial_sample_file::ASCIIString #Path to initial sample file
    transition_sample_file::ASCIIString #Path to transition sample file

    #CCAS libcas config (CCAS only)
    libcas::ASCIIString #Path to libcas library
    libcas_config::ASCIIString #Path to libcas config file
end
ACASX_GM_params() = ACASX_GM_params(1, UInt64(12), 500.0, 100.0, 50, 2,
                                        :LLCEMDBN, :EvE, :ICAO, :ADD, :LLADM,
                                        true, "/acas/CASSATT/src/Encounter/CorrAEMImpl/params/cor.txt", 
                                        :DBN, "/acas/CASSATT/src/examples/initial.txt",
                                        "/acas/CASSATT/src/examples/transition.txt", "", "")

type ACASX_GM <: AbstractGenerativeModel
    params::ACASX_GM_params

    #sim objects: contains state that changes throughout sim run
    em::AbstractEncounterDBN
    pr::Vector{AbstractPilotResponse}
    dm::Vector{AbstractDynamicModel}
    wm::AbstractWorldModel
    coord::AbstractCASCoord
    sr::Vector{AbstractSensor}
    cas::Vector{AbstractCollisionAvoidanceSystem}

    observer::Observer
    string_id::ASCIIString

    #sim states: changes throughout simulation run
    t_index::Int64 #current time index in the simulation. Starts at 0 and increments by 1.
    #This is different from t which starts at 0 and could increment in the reals.

    vmd::Float64 #minimum vertical distance so far
    hmd::Float64 #minimum horizontal distance so far
    md::Float64 #combined miss distance metric
    md_time::Int64 #time index at which vmd and hmd are taken
    label_as_nmac::Bool #label this trajectory as nmac (different from NMAC_occurred that only looks at md,
                      #this one also needs to pass label filters)

    step_logProb::Float64 #cumulative probability of this update()

    #empty constructor
    function ACASX_GM(p::ACASX_GM_params)
        sim = new()
        sim.params = p

        if p.encounter_model == :LLCEMDBN
            @assert p.num_aircraft == 2
            sim.em = CorrAEMDBN(p.num_aircraft, p.encounter_file, p.initial_sample_file,
                          p.transition_sample_file,
                          p.encounter_number, p.encounter_seed, p.command_method)
        elseif p.encounter_model == :StarDBN
            @assert p.num_aircraft > 1
            sim.em = StarDBN(p.num_aircraft, p.encounter_file, p.encounter_seed)
        elseif p.encounter_model == :SideOnDBN
            sim.em = SideOnDBN(p.num_aircraft, p.encounter_file, p.encounter_seed)
        end

        if p.response_model == :ICAO
            sim.pr = [LLDetPR(5, 3) for i = 1 : p.num_aircraft]
        else
            error("ACASX_GM_Impl: Response model not supported ($(p.response_model))")
        end

        if p.dynamics_model == :LLADM
            g = 32.2 #ft/s^2
            min_vertical_accel = -g/3
            max_vertical_accel = g/3
            sim.dm = [ LLADM(; 
                        _min_vertical_accel=min_vertical_accel, 
                        _max_vertical_accel=max_vertical_accel) 
                    for i = 1 : p.num_aircraft]
        else
            error("ACASX_GM_Impl: Dynamics model not supported ($(p.dynamics_model))")
        end

        sim.wm = AirSpace(p.num_aircraft)
        sim.coord = GenericCoord(p.num_aircraft)

        max_intruders = p.num_aircraft - 1
        sim.sr = [ACASXSensor(i, max_intruders) for i = 1:p.num_aircraft]

        if p.cas_model == :CCAS
            sim.cas = Array(AbstractCollisionAvoidanceSystem, p.num_aircraft)
            sim.cas[1] = ACASX_CCAS(1, p.libcas, p.libcas_config, p.num_aircraft, sim.coord,
                           EQUIPAGE_TCAS)
            if p.encounter_equipage == :EvE
                equip = EQUIPAGE_TCAS
            elseif p.encounter_equipage == :EvU
                equip = EQUIPAGE_MODES
            else
                error("ACASX_GM_Impl: Encounter equipage not supported ($(p.encounter_equipage))")
            end
            for i = 2:p.num_aircraft
                sim.cas[i] = ACASX_CCAS(i, p.libcas, p.libcas_config, p.num_aircraft,
                                sim.coord, equip)
            end
        elseif p.cas_model == :ADD
            sim.cas = Array(AbstractCollisionAvoidanceSystem, p.num_aircraft)
            sim.cas[1] = CollisionAvoidanceSystem.ACASX_ADD(1, p.num_aircraft, sim.coord, EQUIPAGE_TCAS)
            if p.encounter_equipage == :EvE
                equip = EQUIPAGE_TCAS
            elseif p.encounter_equipage == :EvU
                equip = EQUIPAGE_MODES
            else
                error("ACASX_GM_Impl: Encounter equipage not supported ($(p.encounter_equipage))")
            end
            for i = 2:p.num_aircraft
                sim.cas[i] = CollisionAvoidanceSystem.ACASX_ADD(i, p.num_aircraft,
                                sim.coord, equip)
            end
        elseif p.cas_model == :SimpleTCAS
            sim.cas = Array(AbstractCollisionAvoidanceSystem, p.num_aircraft)
            sim.cas[1] = SimpleTCAS()
            for i = 2:p.num_aircraft
                sim.cas[i] = SimpleTCAS()
            end
        else
            error("ACASX_GM_Impl: No such CAS model ($(p.cas_model))")
        end

        sim.vmd = realmax(Float64)
        sim.hmd = realmax(Float64)
        sim.md = realmax(Float64)
        sim.md_time = 0
        label_as_nmac = false

        sim.step_logProb = 0.0
    
        sim.observer = Observer()
        sim.string_id = "ACASX_GM_$(p.encounter_number)"

        sim.t_index = 0

        return sim
    end
end

#TODO: combine some more.  Perhaps use an abstract type for all ACASX GMs.
import ACASX_Common

addObserver(sim::ACASX_GM, f::Function) = ACASX_Common.addObserver(sim, f)
function addObserver(sim::ACASX_GM, tag::AbstractString, f::Function) 
    ACASX_Common.addObserver(sim, tag, f)
end
clearObservers!(sim::ACASX_GM) = ACASX_Common.clearObservers!(sim) 

initialize(sim::ACASX_GM) = ACASX_Common.initialize(sim)
update(sim::ACASX_GM) = ACASX_Common.update(sim)
isterminal(sim::ACASX_GM) = ACASX_Common.isterminal(sim)

end #module

