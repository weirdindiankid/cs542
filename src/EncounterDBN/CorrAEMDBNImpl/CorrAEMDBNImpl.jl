# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 12/15/2014


# Correlated Encounter Model for Cooperative Aircraft in the National Airspace
# exposed as DBN.  Samples are generated at runtime at each step.

module CorrAEMDBNImpl

export
    AddObserver,

    getInitialState,
    initialize,
    update,
    get,

    CorrAEMDBN

import Compat.ASCIIString

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using Util
using Encounter
using CorrAEMImpl
using RLESUtils, Observers

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractEncounterDBNInterfaces.get
import AbstractEncounterDBNInterfaces.getInitialState

import Base.convert

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))

type CorrAEMDBN <: AbstractEncounterDBN

  number_of_aircraft::Int64

  encounter_file::ASCIIString
  initial_sample_file::ASCIIString
  transition_sample_file::ASCIIString

  encounter_number::Int64
  command_method::Symbol #:ENC = read from encounter file, :DBN = generate from DBN on-the-fly

  aem::CorrAEM
  dirichlet_transition

  #initial states
  init_aem_dstate::Vector{Int64} #discrete current state
  init_aem_dyn_cstate::Vector{Float64} #continuous of current dynamic variables

  #current states
  t::Int64
  aem_dstate::Vector{Int64} #discrete current state
  aem_dyn_cstate::Vector{Float64} #continuous of current dynamic variables

  #caching and reuse
  dynamic_variables0::Vector{Int64}
  dynamic_variables1::Vector{Int64}
  parents_cache::Dict{Int64, Vector{Bool}}
  weights_cache::Dict{Tuple{Int64, Int64}, Vector{Float64}}
  cumweights_cache::Dict{Tuple{Int64, Int64}, Vector{Float64}}

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function CorrAEMDBN(number_of_aircraft::Int, encounter_file::AbstractString, initial_sample_file::AbstractString,
                      transition_sample_file::AbstractString,
                      encounter_number::Int, encounter_seed::UInt64, command_method::Symbol)
    dbn = new()

    @assert number_of_aircraft == 2 #need to revisit the code if this is not true
    dbn.number_of_aircraft     = number_of_aircraft

    dbn.encounter_file         = encounter_file
    dbn.initial_sample_file    = initial_sample_file
    dbn.transition_sample_file = transition_sample_file
    dbn.encounter_number       = encounter_number
    dbn.command_method         = command_method

    dbn.aem = CorrAEM(encounter_file, initial_sample_file, transition_sample_file)

    dbn.t = 0

    srand(encounter_seed) #There's a rand inside generateEncounter
    generateEncounter(dbn.aem, sample_number=encounter_number) #To optimize: This allocates a lot of memory

    #compute initial states of variables
    p = dbn.aem.parameters
    dbn.dynamic_variables0 = p.temporal_map[:,1]
    dbn.dynamic_variables1 = p.temporal_map[:,2]

    aem_initial_unconverted = unconvertUnitsAemState(dbn.aem.initial)
    aem_initial_dstate = Int64[ val2ind(p.boundaries[i], p.r_transition[i], val)
                               for (i, val) in enumerate(aem_initial_unconverted)]
    dbn.init_aem_dstate = [aem_initial_dstate; aem_initial_dstate[dbn.dynamic_variables0]] #bins, [11:14] are updated with time, append space for t+1 variables
    dbn.init_aem_dyn_cstate = dbn.aem.initial[dbn.dynamic_variables0] #continuous variables.
    dbn.dirichlet_transition = bn_dirichlet_prior(p.N_transition)

    dbn.aem_dstate = deepcopy(dbn.init_aem_dstate)
    dbn.aem_dyn_cstate = deepcopy(dbn.init_aem_dyn_cstate)

    #precompute and cache these quantities
    dbn.parents_cache = Dict{Int64,Vector{Bool}}()
    dbn.weights_cache = Dict{Tuple{Int64,Int64}, Vector{Float64}}()
    dbn.cumweights_cache = Dict{Tuple{Int64,Int64}, Vector{Float64}}()

    for i = 1:length(p.N_transition)
      dbn.parents_cache[i] = p.G_transition[:, i]

      for j = 1:1:size(dbn.dirichlet_transition[i], 2)
        dbn.weights_cache[(i, j)] = p.N_transition[i][:, j] + dbn.dirichlet_transition[i][:, j]
        dbn.weights_cache[(i, j)] /= sum(dbn.weights_cache[(i, j)])
        dbn.cumweights_cache[(i, j)] = cumsum(dbn.weights_cache[(i, j)])
      end
    end

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.0, 0.0, 0.0, 0.0) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end
end

addObserver(dbn::CorrAEMDBN, f::Function) = add_observer(aem.observer, f)
addObserver(dbn::CorrAEMDBN, tag::AbstractString, f::Function) = add_observer(aem.observer, tag, f)

function initialize(dbn::CorrAEMDBN)
  #reset to initial state
  copy!(dbn.aem_dstate, dbn.init_aem_dstate)
  copy!(dbn.aem_dyn_cstate, dbn.init_aem_dyn_cstate)
  dbn.t = 0

  #reset aem indices
  initialize(dbn.aem)
end

function getInitialState(dbn::CorrAEMDBN, index::Int)
  return Encounter.getInitialState(dbn.aem, index)
end

function update(dbn::CorrAEMDBN)

  if dbn.command_method == :DBN
    logProb = step_dbn(dbn)
  elseif dbn.command_method == :ENC
    logProb = step_enc(dbn)
  else
    error("CorrAEMDBNImpl::Step: No such command method")
  end

  dbn.t += 1

  return logProb
end

function step_dbn(dbn::CorrAEMDBN)
  p = dbn.aem.parameters
  aem_dstate = dbn.aem_dstate #entire state, discrete bins
  aem_dyn_cstate = dbn.aem_dyn_cstate #dynamic states, continuous

  logProb = 0.0

  for (o,i) in enumerate(dbn.dynamic_variables1)
    if !isempty(find(dbn.parents_cache[i]))
      dims = tuple(p.r_transition[dbn.parents_cache[i]]...)
      j = sub2ind(dims, aem_dstate[dbn.parents_cache[i]]...)
      aem_dstate[i] = select_random_cumweights(dbn.cumweights_cache[(i,j)])
      logProb += log(dbn.weights_cache[(i,j)][aem_dstate[i]])
      #Resampling and dediscretizing process
      i_t = dbn.dynamic_variables0[o]
      if (aem_dstate[i] != aem_dstate[i_t]) || #compare to state at last time step, #Different bin, do resample
        (aem_dstate[i] == aem_dstate[i_t] && rand() < p.resample_rates[i_t]) #Same bin but meets resample rate
        aem_dyn_cstate[o],_ = dediscretize(aem_dstate[i],p.boundaries[i_t],p.zero_bins[i_t])
        if in(i,[17,18]) #these need unit conversion
          aem_dyn_cstate[o] /= 60 #convert units
        end
      end
      #Else same bin and does not meet rate, just set equal to previous (no update)
    end
  end

  # copy over x(t+1) to x(t)
  aem_dstate[dbn.dynamic_variables0] = aem_dstate[dbn.dynamic_variables1]

  #Just a reminder, this will break if number_of_aircraft != 2
  @assert dbn.number_of_aircraft == 2

  dbn.output_commands[1].t = dbn.t
  dbn.output_commands[1].v_d = getInitialSample(dbn.aem, :v1d)
  dbn.output_commands[1].h_d = dbn.aem_dyn_cstate[1]
  dbn.output_commands[1].psi_d = dbn.aem_dyn_cstate[3]

  dbn.output_commands[2].t = dbn.t
  dbn.output_commands[2].v_d = getInitialSample(dbn.aem, :v2d)
  dbn.output_commands[2].h_d = dbn.aem_dyn_cstate[2]
  dbn.output_commands[2].psi_d = dbn.aem_dyn_cstate[4]

  return dbn.logProb = logProb
end

#TODO: remove hardcoding
convert_units(x::AbstractFloat, i::Int) = in(i, [17,18]) ? x / 60 : x

convert(::Type{Vector{Float64}}, command_1::CorrAEMCommand, command_2::CorrAEMCommand) =
  [ command_1.h_d, command_2.h_d, command_1.psi_d, command_2.psi_d ]

function step_enc(dbn::CorrAEMDBN)
    aem = dbn.aem
    p = aem.parameters
    
    for i = 1:dbn.number_of_aircraft
        cmd = Encounter.update(aem, i)
        if cmd != nothing
            dbn.output_commands[i] = cmd
        end
    end

  #Just a reminder, this will break if number_of_aircraft != 2
  #@assert dbn.number_of_aircraft == 2

  #= FIXME: skip probability calc for now...
  #prepare t+1 from encounter commands
  aem_dyn_cstate = convert(Vector{Float64}, dbn.output_commands[1], dbn.output_commands[2])
  aem_dstate = dbn.aem_dstate #only copies pointer
  #load into the (t+1) slots
  #boundaries are specified at t
  aem_dyn_cstate_unconverted = unconvertUnitsDynVars(aem_dyn_cstate) #need to unconvert units
  aem_dstate[dbn.dynamic_variables1] = Int64[ val2ind(p.boundaries[i], p.r_transition[i], aem_dyn_cstate_unconverted[o])
                                                for (o, i) in enumerate(dbn.dynamic_variables0) ]

  # compute the probability of this transition
  logProb = 0.0
  for (o, i) in enumerate(dbn.dynamic_variables1)
    j = 1
    parents = dbn.parents_cache[i]
    if !isempty(find(parents))
      dims = tuple(p.r_transition[parents]...)
      indices = aem_dstate[parents]
      j = sub2ind(dims, indices...)
    end
    weights = dbn.weights_cache[(i, j)]
    logProb += log(weights[aem_dstate[i]])

    #probability from continuous sampling process
    #two components: resample prob, dediscretize prob
    i_prev = dbn.dynamic_variables0[o]
    if aem_dstate[i] != aem_dstate[i_prev] #not the same bin, resample wp 1
      logProb += dediscretize_prob(aem_dyn_cstate[o], aem_dstate[i], p.boundaries[i_prev], p.zero_bins[i_prev])
    elseif isapprox(aem_dyn_cstate[o], dbn.aem_dyn_cstate[o], atol=0.0001) #same bin same value, did not resample
      logProb += log(1.0 - p.resample_rates[i_prev])
    else #Same bin different value, got resampled
      logProb += log(p.resample_rates[i_prev])
      logProb += dediscretize_prob(aem_dyn_cstate[o], aem_dstate[i], p.boundaries[i_prev], p.zero_bins[i_prev])
    end
  end

  # copy over x(t+1) to x(t)
  aem_dstate[dbn.dynamic_variables0] = aem_dstate[dbn.dynamic_variables1]

  #push to sim
  dbn.aem_dstate = aem_dstate
  dbn.aem_dyn_cstate = aem_dyn_cstate

  #return
  dbn.logProb = logProb
  =#
  dbn.logProb = 0.0 #for now... FIXME
end

function get(dbn::CorrAEMDBN, aircraft_number::Int)
  return dbn.output_commands[aircraft_number]
end

function val2ind(boundariesi, ri, value)
  if !isempty(boundariesi)
    index = findfirst(x -> (x > value), boundariesi) - 1
    if index == -1
      index = ri
    end
  else
    index = value
  end
  return index
end

function dediscretize(dval::Int64, boundaries::Vector{Float64}, zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval + 1]

  if dval == zero_bin
    val = 0.0
    prob = 1.0
  elseif val_max == val_min
    val = val_min
    prob = 1.0
  else
    val = val_min +  rand() * (val_max - val_min)
    prob = 1.0 / (val_max - val_min)
    #this is a density so it won't be normalized to [0,1]
  end

  return (val, prob)
end

function dediscretize_prob(val::Float64, dval::Int64, boundaries::Vector{Float64}, zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval + 1]

  if dval == zero_bin
    @assert val == 0.0
    prob = 1.0
  elseif val_max == val_min
    @assert val == val_min
    prob = 1.0
  else
    @assert val_min <= val <= val_max
    prob = 1.0 / (val_max - val_min)
    #this is a density so it won't be normalized to [0,1]
  end
  return prob
end

function unconvertUnitsDynVars(v)
  return [v[1:2] * 60.0, v[3:end]]
end

function unconvertUnitsAemState(state_) #from CorrAEM
  state = deepcopy(state_)

  state[7] /= 1.68780
  state[8] /= 1.68780
  state[9] /= 1.68780
  state[10] /= 1.68780
  state[11] *= 60
  state[12] *= 60
  state[15] /= 6076.12

  return state
end

function select_random_cumweights(cweights::Vector{Float64})
#select randomly according to cumulative weights vector

  r = cweights[end] * rand()

  return findfirst(x -> (x >= r), cweights)
end

end #module


