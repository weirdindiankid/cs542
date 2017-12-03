# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014

module SimpleTCAS_EvE_Impl

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
using CASCoordination
using CollisionAvoidanceSystem
using Simulator
using RLESUtils, Observers

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractGenerativeModelInterfaces.get
import AbstractGenerativeModelInterfaces.isterminal

import CommonInterfaces.addObserver

export SimpleTCAS_EvE_params, SimpleTCAS_EvE, initialize, update, isterminal, addObserver

type SimpleTCAS_EvE_params
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

  SimpleTCAS_EvE_params() = new()
end

type SimpleTCAS_EvE <: AbstractGenerativeModel
  params::SimpleTCAS_EvE_params

  #sim objects: contains state that changes throughout sim run
  em::CorrAEMDBN
  pr::Vector{Union{SimplePilotResponse,StochasticLinearPR}}
  dm::Vector{SimpleADM}
  wm::AirSpace
  coord::GenericCoord
  sr::Vector{SimpleTCASSensor}
  cas::Vector{CoordSimpleTCAS}

  observer::Observer

  #sim states: changes throughout simulation run
  t_index::Int64 #current time index in the simulation. Starts at 1 and increments by 1.
  #This is different from t which starts at 0 and could increment in the reals.

  #empty constructor
  function SimpleTCAS_EvE(p::SimpleTCAS_EvE_params)
    @test p.number_of_aircraft == 2 #need to revisit the code if this is not true

    sim = new()
    sim.params = p

    srand(p.encounter_seed) #There's a rand inside generateEncounter, need to control it
    sim.em = CorrAEMDBN(p.number_of_aircraft, p.encounter_file, p.initial_sample_file,
                    p.transition_sample_file,
                    p.encounter_number,p.encounter_seed,p.command_method)

    if p.pilotResponseModel == :SimplePR
      sim.pr = SimplePilotResponse[ SimplePilotResponse() for i=1:p.number_of_aircraft ]
    elseif p.pilotResponseModel == :StochasticLinear
      sim.pr = StochasticLinearPR[ StochasticLinearPR() for i=1:p.number_of_aircraft ]
    else
      error("SimpleTCAS_EvE_Impl: No such pilot response model")
    end

    sim.dm = SimpleADM[ SimpleADM(number_of_substeps=1) for i=1:p.number_of_aircraft ]
    sim.wm = AirSpace(p.number_of_aircraft)

    sim.coord = GenericCoord(p.number_of_aircraft)
    sim.sr = SimpleTCASSensor[ SimpleTCASSensor(1), SimpleTCASSensor(2) ]
    sim.cas = CoordSimpleTCAS[ CoordSimpleTCAS(1,sim.coord), CoordSimpleTCAS(2,sim.coord) ]

    sim.observer = Observer()

    #Start time at 1 for easier indexing into arrays according to time
    sim.t_index = 1

    return sim
  end
end

addObserver(sim::SimpleTCAS_EvE, f::Function) = add_observer(sim.observer, f)
addObserver(sim::SimpleTCAS_EvE, tag::AbstractString, f::Function) = add_observer(sim.observer, tag, f)

function initialize(sim::SimpleTCAS_EvE)

  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  #Start time at 1 for easier indexing into arrays according to time
  sim.t_index = 1

  EncounterDBN.initialize(aem)

  for i = 1:sim.params.number_of_aircraft
    initial = EncounterDBN.getInitialState(aem, i)
    @notify_observer(sim.observer,"Initial",[i, sim.t_index, initial])

    state = DynamicModel.initialize(adm[i], initial)
    WorldModel.initialize(wm, i, state)

    Sensor.initialize(sr[i])
    @notify_observer(sim.observer,"Sensor",[i, sim.t_index, sr[i]])

    CollisionAvoidanceSystem.initialize(cas[i])
    @notify_observer(sim.observer,"CAS", [i, sim.t_index, cas[i]])

    PilotResponse.initialize(pr[i])
    @notify_observer(sim.observer,"Response",[i, sim.t_index, pr[i]])
  end

  @notify_observer(sim.observer,"WorldModel", [sim.t_index, wm])

  return
end

function update(sim::SimpleTCAS_EvE)
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

    output = Sensor.update(sr[i], states)
    @notify_observer(sim.observer,"Sensor",[i, sim.t_index, sr[i]])

    RA = CollisionAvoidanceSystem.update(cas[i], output)
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

function isNMAC(sim::SimpleTCAS_EvE)
  vhdist = getvhdist(sim.wm)
  nmac_test = map((vhd)->vhd[2] <= sim.params.nmac_r && vhd[1] <= sim.params.nmac_h,vhdist)
  return any(nmac_test)
end

isterminal(sim::SimpleTCAS_EvE) = isNMAC(sim) || sim.t_index > sim.params.maxSteps

end #module



