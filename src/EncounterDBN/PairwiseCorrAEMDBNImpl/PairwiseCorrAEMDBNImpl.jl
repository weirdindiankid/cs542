# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 2/20/2015

# Multiple aircraft encounter model based on generating encounters pairwise using CorrAEM
# DBN samples are generated at runtime at each step.

module PairwiseCorrAEMDBNImpl

export
    AddObserver,

    getInitialState,
    initialize,
    update,
    get,

    PairwiseCorrAEMDBN

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using Util
using Encounter
using RLESUtils, Observers
using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractEncounterDBNInterfaces.get
import AbstractEncounterDBNInterfaces.getInitialState

import CorrAEMImpl
AEM_CorrAEMParameters = CorrAEMImpl.CorrAEMParameters
AEM_getInitialState = CorrAEMImpl.getInitialState
AEM_generateEncounter = CorrAEMImpl.generateEncounter

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))

type PairwiseCorrAEMDBN <: AbstractEncounterDBN

  number_of_aircraft::Int64

  parameter_file::ASCIIString
  parameters::AEM_CorrAEMParameters

  dirichlet_initial
  dirichlet_transition

  encounter_seed::UInt64

  aem::CorrAEM
  ac1_initial_dist::Union{Vector{Union{Function, Void}}, Void}
  ac1_dynamic_vars::Union{Array{Float64,2},Void}

  t::Float64

  #initial state
  initial_states::Vector{CorrAEMInitialState}

  #initial command
  initial_commands_d::Vector{Vector{Int64}} #discrete current state
  initial_commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #current command
  commands_d::Vector{Vector{Int64}} #discrete current state
  commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function PairwiseCorrAEMDBN(number_of_aircraft::Int, parameter_file::AbstractString,
                              encounter_seed::UInt64;
                              initial_file::AbstractString="",
                              transition_file::AbstractString="")

    dbn = new()

    @test number_of_aircraft >= 2 #multi-aircraft sim
    dbn.number_of_aircraft     = number_of_aircraft

    dbn.parameter_file = parameter_file
    dbn.encounter_seed = encounter_seed

    const default_number_of_initial_samples = 1
    const default_number_of_transition_samples = 50

    #Based on CorrAEM (call it repeatedly for pairwise encounters)
    dbn.aem = CorrAEM(parameter_file, initial_file, default_number_of_initial_samples,
                      transition_file, default_number_of_transition_samples,
                      b_read_from_file=false, b_write_to_file=false)

    dbn.parameters = dbn.aem.parameters
    dbn.dirichlet_initial = bn_dirichlet_prior(dbn.parameters.N_initial)
    dbn.dirichlet_transition = bn_dirichlet_prior(dbn.parameters.N_transition)

    dbn.ac1_initial_dist = nothing
    dbn.ac1_dynamic_vars = nothing

    dbn.t = 0.0

    srand(encounter_seed) #There's a rand inside generateEncounter
    generate_encounter(dbn) #sets initial_states, initial_commands_d, initial_commands

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.,0.,0.,0.) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end
end

const AC1_MASK = [true,true,false,false,true,false,true,false,true,false,true,false,true,false,false,false]
const SAVE_INDICES_AC1 = [1, 10, 12, 14]

const map_G2L = Dict(2 => 1, 9 => 2, 11 => 3, 13 => 4, 17 => 5, 19 => 6) #global to local
const map_L2G = Dict(1 => 2, 2 => 9, 3 => 11, 4 => 13, 5=> 17, 6 => 19) #local to global
const map_var2ind_L = Dict(:L => 1, :v_d => 2, :h_d0 => 3, :psi_d0 => 4, :h_d1 => 5, :psi_d1 => 6) #variable names to local
const map_ind2var_L = Dict(1 => :L, 2 => :v_d, 3 => :h_d0, 4 => :psi_d0, 5 => :h_d1, 6 => :psi_d1) #local to variable names
const temporal_map = [3 5; 4 6] #[dynamic_variables0; dynamic_variables1]

function generate_encounter(dbn::PairwiseCorrAEMDBN; sample_number::Int64 = 0)

  @test dbn.number_of_aircraft >= 2

  p = dbn.parameters
  dynamic_variables1 = temporal_map[:,2]

  #initial aircraft states and commands
  dbn.initial_states = Array(CorrAEMInitialState,dbn.number_of_aircraft)
  dbn.initial_commands = Array(Vector{Float64},dbn.number_of_aircraft) #[L,v_d,h_d,psi_d]
  dbn.initial_commands_d = Array(Vector{Int64},dbn.number_of_aircraft) #[L,v_d,h_d,psi_d,[hd_tp1,psid_tp1]]

  #Handle the first 2 aircraft
  AEM_generateEncounter(dbn.aem)

  for i = 1:2 #aircraft_number
    dbn.initial_states[i] = AEM_getInitialState(dbn.aem,i)

    t, v_d, h_d, psi_d = getNextCommand(dbn.aem, i)
    dbn.initial_commands[i] = Float64[dbn.aem.L, v_d, h_d, psi_d]
    initial_commands_d = discretize(dbn.parameters,unconvert_units(dbn.initial_commands[i]))

    dbn.initial_commands_d[i] = [ initial_commands_d, Int64(zeros(dynamic_variables1)) ]
  end

  #Save aircraft 1 initial vars
  dbn.ac1_initial_dist = masked_initial_dist(dbn.aem.initial,AC1_MASK)

  #Save aircraft 1 dynamic vars
  dbn.ac1_dynamic_vars = save_ac1_dyn_vars(squeeze(dbn.aem.states[1,:,:],1))

  #Handle aircraft 3+
  for i = 3:dbn.number_of_aircraft

    #Load aircraft 1 initial vars into initial distribution
    dbn.aem.initial_distributions = dbn.ac1_initial_dist

    states = get_em_sample(dbn.aem)

    #load aircraft 1 dynamic vars into states (replacing this one)
    load_dynamic_vars!(states,dbn.ac1_dynamic_vars)

    sim_and_transform(dbn.aem,states)

    #aircraft 1 is globally common, aircraft 2 is newly generated and stacked on
    dbn.initial_states[i] = AEM_getInitialState(dbn.aem,2)

    t, v_d, h_d, psi_d = getNextCommand(dbn.aem, 2)
    dbn.initial_commands[i] = Float64[dbn.aem.L, v_d, h_d, psi_d]
    initial_commands_d = discretize(dbn.parameters,unconvert_units(dbn.initial_commands[i]))

    dbn.initial_commands_d[i] = [ initial_commands_d, Int64(zeros(dynamic_variables1)) ]
  end
end

#Excerpted from CorrAEMImpl.jl and made into its own function
function get_em_sample(aem::CorrAEM)

  if aem.initial_distributions == nothing
    states = em_sample(aem, aem.number_of_transition_samples)
  else
    states, aem.ISInfo = em_sample(aem, aem.number_of_transition_samples, initial_dist = aem.initial_distributions)
  end

  aem.number_of_encounters_generated += 1

  states = states[:, 2:end]
  convert_units(states) #moved from sim_and_transform

  return states
end

function load_ac1_dyn_vars!(states::Array{Float64,3},dyn_vars)
  for (l,g) in enumerate(SAVE_INDICES_AC1)
    states[1,:, g] = dyn_vars[:, l]
  end
end

#Excerpted from CorrAEMImpl.jl and made into its own function
function sim_and_transform(aem::CorrAEM,states)
  #convert_units(states) #moved to sim_and_transform

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

function masked_initial_dist(v::Vector{Float64}, mask::Vector{Bool})
  return [m ? ()->(x,0.0) : nothing for (m,x) = zip(mask,v)]
end

addObserver(dbn::PairwiseCorrAEMDBN, f::Function) = add_observer(aem.observer, f)
addObserver(dbn::PairwiseCorrAEMDBN, tag::AbstractString, f::Function) = add_observer(aem.observer, tag, f)

function initialize(dbn::PairwiseCorrAEMDBN)
  #reset to initial state
  dbn.commands_d = deepcopy(dbn.initial_commands_d)
  dbn.commands = deepcopy(dbn.initial_commands)
  dbn.t = 0.0

end

function getInitialState(dbn::PairwiseCorrAEMDBN, index::Int)
  return dbn.initial_states[index]
end

function update(dbn::PairwiseCorrAEMDBN)
  logProb = 0.0 #to accumulate over each aircraft

  for i = 1:dbn.number_of_aircraft
    logProb += step_dbn(dbn.parameters,dbn.dirichlet_transition,
                        dbn.commands_d[i],dbn.commands[i])

    dbn.output_commands[i].t = dbn.t
    dbn.output_commands[i].v_d = dbn.commands[i][map_var2ind_L[:v_d]]
    dbn.output_commands[i].h_d = dbn.commands[i][map_var2ind_L[:h_d0]]
    dbn.output_commands[i].psi_d = dbn.commands[i][map_var2ind_L[:psi_d0]]
  end

  dbn.t += 1.0

  return logProb
end

function step_dbn(p::AEM_CorrAEMParameters, dirichlet_transition,
                         command_d::Vector{Int64}, command::Vector{Float64})
  logProb = 0.0

  dynamic_variables0 = temporal_map[:,1] #[hdot_1(t), psidot_1(t)]
  dynamic_variables1 = temporal_map[:,2] #[hdot_1(t+1), psidot_1(t+1)]

  for (o,i_L) in enumerate(dynamic_variables1)
    i_G = map_L2G[i_L]
    parents_G = p.G_transition[:, i_G]
    if !isempty(find(parents_G))
      parents_L = Int64[map_G2L[iparents] for iparents in find(parents_G)]
      j_G = asub2ind(p.r_transition[parents_G], command_d[parents_L]')
      weights = p.N_transition[i_G][:, j_G] + dirichlet_transition[i_G][:, j_G]
      weights /= sum(weights)
      command_d[i_L] = select_random(weights)
      logProb += log(weights[command_d[i_L]])
      #Resampling and dediscretizing process
      i0_L = dynamic_variables0[o]
      i0_G = map_L2G[i0_L]
      if (command_d[i_L] != command_d[i0_L]) || #compare to state at last time step, #Different bin, do resample
        (command_d[i_L] == command_d[i0_L] && rand() < p.resample_rates[i0_G]) #Same bin but meets resample rate
        command[i0_L] = dediscretize(command_d[i_L],p.boundaries[i0_G],p.zero_bins[i0_G])
        command[i0_L] = convert_units(command[i0_L],map_ind2var_L[i0_L])
      end
      #Else same bin and does not meet rate, just set equal to previous (no update)
    end
  end

  # update x(t) with x(t+1)
  command_d[dynamic_variables0] = command_d[dynamic_variables1]

  #return
  return logProb
end

function get(dbn::PairwiseCorrAEMDBN, aircraft_number::Int)
  return dbn.output_commands[aircraft_number]
end

function val2ind(boundariesi,ri,value)
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

function discretize(p::AEM_CorrAEMParameters,v::Vector{Float64})
  return Int64[ val2ind(p.boundaries[map_L2G[i]],
                        p.r_transition[map_L2G[i]],val)
               for (i,val) in enumerate(v) ]
end

function dediscretize(dval::Int64,boundaries::Vector{Float64},zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval+1]

  return dval == zero_bin ? 0.0 : val_min +  rand() * (val_max - val_min)
end

function asub2ind(siz, x)
# ASUB2IND Linear index from multiple subscripts.
#   Returns a linear index from multiple subscripts assuming a matrix of a
#   specified size.
#
#   NDX = ASUB2IND(SIZ,X) returns the linear index NDX of the element in a
#   matrix of dimension SIZ associated with subscripts specified in X.

    k = [1, cumprod(siz[1:end-1])]
    ndx = k' * (x' - 1) + 1

    return ndx[1]
end

convert_units(v::Vector{Float64}) = Float64[convert_units(v[i],map_ind2var_L[i]) for i=1:endof(v)]
unconvert_units(v::Vector{Float64}) = Float64[unconvert_units(v[i],map_ind2var_L[i]) for i=1:endof(v)]

function convert_units(x::Float64,var::Symbol)
  if var == :v_d0 || var == :v_d1
    return x * 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x / 60
  else
    return x
  end
end

function unconvert_units(x::Float64,var::Symbol)
  if var == :v_d0 || var == :v_d1
    return x / 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x * 60
  else
    return x
  end
end

end #module


