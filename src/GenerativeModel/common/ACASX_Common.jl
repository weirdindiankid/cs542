module ACASX_Common
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator
using RLESUtils, Observers

addObserver(sim, f::Function) = add_observer(sim.observer, f)
addObserver(sim, tag::AbstractString, f::Function) = add_observer(sim.observer, tag, f)
clearObservers!(sim) = empty!(sim.observer)

function initialize(sim)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr
  sim.t_index = 0
  EncounterDBN.initialize(aem)

  for i = 1:sim.params.num_aircraft
    #TODO: clean up this structure
    initial = EncounterDBN.getInitialState(aem, i)
    @notify_observer(sim.observer,"Initial", Any[i, sim.t_index, aem])

    Sensor.initialize(sr[i])
    CollisionAvoidanceSystem.initialize(cas[i])
    PilotResponse.initialize(pr[i])
    state = DynamicModel.initialize(adm[i], initial)
    WorldModel.initialize(wm, i, state)

    @notify_observer(sim.observer,"CAS_info", Any[i, sim.cas[i]])
  end

  # reset miss distances
  sim.vmd = typemax(Float64)
  sim.hmd = typemax(Float64)
  sim.md = typemax(Float64)
  sim.md_time = 0

  #reset the probability
  sim.step_logProb = 0.0
  @notify_observer(sim.observer, "initialize", Any[sim])
  return
end

function update(sim)
  wm, aem, pr, adm, cas, sr = sim.wm, sim.em, sim.pr, sim.dm, sim.cas, sim.sr

  sim.t_index += 1
  sim.step_logProb = 0.0 #track the log probabilities in this update

  cmdLogProb = EncounterDBN.update(aem)
  sim.step_logProb += cmdLogProb #TODO: decompose this by aircraft?
  states = WorldModel.getAll(wm)
  @notify_observer(sim.observer,"WorldModel", Any[sim.t_index, wm])

  #check and update miss distances
  vhdist = getvhdist(sim.wm)
  mds = getMissDistance(sim.params.nmac_h, sim.params.nmac_r, vhdist)
  md, index = findmin(mds)
  if md < sim.md
    sim.vmd, sim.hmd = vhdist[index]
    sim.md = md
    sim.md_time = states[1].t #report time instead of index
  end

  for i = 1:sim.params.num_aircraft
    @notify_observer(sim.observer, "Dynamics", Any[i, sim.t_index, adm[i]])

    #intended command
    command = EncounterDBN.get(aem, i)
    @notify_observer(sim.observer, "Command", Any[i, sim.t_index, command])

    output = Sensor.update(sr[i], states)
    @notify_observer(sim.observer, "Sensor",Any[i, sim.t_index, sr[i]])

    RA = CollisionAvoidanceSystem.update(cas[i], output)
    @notify_observer(sim.observer, "CAS", Any[i, sim.t_index, cas[i]])

    response = PilotResponse.update(pr[i], command, RA, states[i])
    sim.step_logProb += response.logProb
    @notify_observer(sim.observer, "Response", Any[i, sim.t_index, pr[i]])

    state = DynamicModel.update(adm[i], response)
    WorldModel.update(wm, i, state)
  end
  WorldModel.updateAll(wm)

  @notify_observer(sim.observer, "logProb", Any[sim.t_index, sim.step_logProb])

  sim.label_as_nmac = NMAC_occurred(sim) #the same for now... no filters
  @notify_observer(sim.observer, "update", Any[sim])
  return (exp(sim.step_logProb), NMAC_occurred(sim), sim.md)
end

function getvhdist(wm::AbstractWorldModel)
  states = WorldModel.getAll(wm) #states::Vector{ASWMState}
  #[(vdist,hdist)]
  vhdist = [(abs(s2.h - s1.h), norm([(s2.x - s1.x), (s2.y - s1.y)])) for s1 in states, s2 in states]
  for i = 1:length(states)
    vhdist[i, i] = (typemax(Float64), typemax(Float64)) #make selfs big to ease finding min dist
  end
  return vhdist
end

isterminal(sim) = (sim.params.end_on_nmac && NMAC_occurred(sim)) || sim.t_index >= sim.params.max_steps
NMAC_occurred(sim) = sim.hmd <= sim.params.nmac_r && sim.vmd <= sim.params.nmac_h
getMissDistance(nmac_h::Float64, nmac_r::Float64, vhmd) = map((vh) -> max(vh[2] * (nmac_h / nmac_r), vh[1]), vhmd)

end #module
