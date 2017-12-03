# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014

module SimpleTCAS_EvU_Impl

import Compat.ASCIIString

using AbstractGenerativeModelImpl
using AbstractGenerativeModelInterfaces
using CommonInterfaces

using Base.Test
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CollisionAvoidanceSystem
using Simulator
using RLESUtils, Observers

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractGenerativeModelInterfaces.get
import AbstractGenerativeModelInterfaces.isterminal

import CommonInterfaces.addObserver

export SimpleTCAS_EvU_params, SimpleTCAS_EvU, initialize, update, isterminal, addObserver

type SimpleTCAS_EvU_params
  #global params: remains constant per sim
  encounter_number::Int64 #encounter number in file
  nmac_r::Float64 #NMAC radius in feet
  nmac_h::Float64 #NMAC vertical separation, in feet
  maxSteps::Int64 #maximum number of steps in sim
  number_of_aircraft::Int64 #number of aircraft  #must be 2 for now...
  encounter_seed::UInt64 #Seed for generating encounters
  pilotResponseModel::Symbol #{:SimplePR, :StochasticLinear}

  #Defines behavior of CorrAEMDBN.  Read from file or generate samples on-the-fly
  command_method::Symbol #:DBN=sampled from DBN or :ENC=from encounter file

  #these are to define CorrAEM:
  encounter_file::ASCIIString #Path to encounter file
  initial_sample_file::ASCIIString #Path to initial sample file
  transition_sample_file::ASCIIString #Path to transition sample file

  string_id::ASCIIString

  SimpleTCAS_EvU_params() = new()
end

type SimpleTCAS_EvU <: AbstractGenerativeModel
  params::SimpleTCAS_EvU_params

  #sim objects: contains state that changes throughout sim run
  em::CorrAEMDBN
  pr::Vector{Union{SimplePilotResponse,StochasticLinearPR}}
  dm::Vector{SimpleADM}
  wm::AirSpace
  sr::Vector{Union{SimpleTCASSensor,Void}}
  cas::Vector{Union{SimpleTCAS,Void}}

  observer::Observer

  #sim states: changes throughout simulation run
  t_index::Int64 #current time index in the simulation. Starts at 1 and increments by 1.
  #This is different from t which starts at 0 and could increment in the reals.

  #empty constructor
  function SimpleTCAS_EvU(p::SimpleTCAS_EvU_params)
    @test p.number_of_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    sim.em = CorrAEMDBN(p.number_of_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number,p.encounter_seed,p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.number_of_aircraft ]
    else
      error("SimpleTCAS_EvU_Impl: No such pilot response model")
    end

    sim.dm = SimpleADM[ SimpleADM(number_of_substeps=1) for i=1:p.number_of_aircraft ]
    sim.wm = AirSpace(p.number_of_aircraft)

    sim.sr = Union{SimpleTCASSensor,Void}[ SimpleTCASSensor(1), nothing ]
    sim.cas = Union{SimpleTCAS,Void}[ SimpleTCAS(), nothing ]

    sim.observer = Observer()

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

addObserver(sim::SimpleTCAS_EvU, f::Function) = add_observer(sim.observer, f)
addObserver(sim::SimpleTCAS_EvU, tag::AbstractString, f::Function) = add_observer(sim.observer, tag, f)

function initialize(sim::SimpleTCAS_EvU)

  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  #Start time at 1 for easier indexing into arrays according to time
  sim.t_index = 1

  EncounterDBN.initialize(aem)

  for i = 1:sim.params.number_of_aircraft
    initial = EncounterDBN.getInitialState(aem, i)
    @notify_observer(sim.observer,"Initial",[i, sim.t_index, initial])

    state = DynamicModel.initialize(adm[i], initial)
    WorldModel.initialize(wm, i, state)

    # If aircraft has a CAS
    if sr[i] != nothing && cas[i] != nothing
      Sensor.initialize(sr[i])
      CollisionAvoidanceSystem.initialize(cas[i])
    end

    @notify_observer(sim.observer,"Sensor",[i, sim.t_index, sr[i]])
    @notify_observer(sim.observer,"CAS", [i, sim.t_index, cas[i]])

    PilotResponse.initialize(pr[i])
    @notify_observer(sim.observer,"Response",[i, sim.t_index, pr[i]])
  end

  @notify_observer(sim.observer,"WorldModel", [sim.t_index, wm])

  return
end

function update(sim::SimpleTCAS_EvU)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1

  logProb = 0.0 #track the probabilities in this update

  cmdLogProb = EncounterDBN.update(aem)
  logProb += cmdLogProb #TODO: decompose this by aircraft?

  states = WorldModel.getAll(wm)

  for i = 1:sim.params.number_of_aircraft
    #intended command
    command = EncounterDBN.get(aem,i)
    @notify_observer(sim.observer,"Command",[i, sim.t_index, command])

    #If aircraft is equipped with a CAS
    if sr[i] != nothing && cas[i] != nothing
      output = Sensor.update(sr[i], states)
      RA = CollisionAvoidanceSystem.update(cas[i], output)
    else
      RA = nothing
    end

    @notify_observer(sim.observer,"Sensor",[i, sim.t_index, sr[i]])
    @notify_observer(sim.observer,"CAS", [i, sim.t_index, cas[i]])

    response = PilotResponse.update(pr[i], command, RA)
    logProb += response.logProb #this will break if response is not SimplePRCommand
    @notify_observer(sim.observer,"Response",[i, sim.t_index, pr[i]])

    state = DynamicModel.update(adm[i], response)
    WorldModel.update(wm, i, state)

  end

  WorldModel.updateAll(wm)
  @notify_observer(sim.observer,"WorldModel", [sim.t_index, wm])

  return logProb
end

function getvhdist(wm::AbstractWorldModel)
  states = WorldModel.getAll(wm) #states::Vector{ASWMState}

  #[(vdist,hdist)]
  vhdist = [(abs(s2.h-s1.h),norm([(s2.x-s1.x),(s2.y-s1.y)])) for s1 = states, s2 = states]
  for i = 1:length(states)
    vhdist[i,i] = (typemax(Float64),typemax(Float64))
  end

  return vhdist
end

function isNMAC(sim::SimpleTCAS_EvU)
  vhdist = getvhdist(sim.wm)
  nmac_test = map((vhd)->vhd[2] <= sim.params.nmac_r && vhd[1] <= sim.params.nmac_h,vhdist)
  return any(nmac_test)
end

isterminal(sim::SimpleTCAS_EvU) = isNMAC(sim) || sim.t_index > sim.params.maxSteps

end #module



