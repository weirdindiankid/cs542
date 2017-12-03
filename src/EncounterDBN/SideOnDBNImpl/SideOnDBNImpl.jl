# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 8/26/2016

# Aircraft encounter model 90 degree "side-on" initial aircraft states 
# DBN samples are generated at runtime at each step.

#TODO: This module is copied from StarDBNImpl
#Both modules need to be cleaned up and unified
#Performance is probably an issue as well
module SideOnDBNImpl

export
    AddObserver,

    getInitialState,
    initialize,
    update,
    get,

    SideOnDBNParams,
    SideOnDBN

import Compat.ASCIIString

using AbstractEncounterDBNImpl
using AbstractEncounterDBNInterfaces
using CommonInterfaces
using Util
using RLESUtils, Observers
using Encounter
import CorrAEMImpl: CorrAEMParameters, CorrAEMInitialState, CorrAEMCommand

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractEncounterDBNInterfaces.get
import AbstractEncounterDBNInterfaces.getInitialState

include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_sample.jl"))
include(Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/corr_aem_load_params.jl"))
include(Pkg.dir("SISLES/src/EncounterDBN/DBNShared/DBNShared.jl"))

const MAP_G2L = Dict(2 => 1, 9 => 2, 11 => 3, 13 => 4, 17 => 5, 
    19 => 6) #global to local
const MAP_L2G = Dict(1 => 2, 2 => 9, 3 => 11, 4 => 13, 5=> 17, 
    6 => 19) #local to global
const MAP_VAR2IND_L = Dict(:L => 1, :v_d => 2, :h_d0 => 3, :psi_d0 => 4, 
    :h_d1 => 5, :psi_d1 => 6) #variable names to local
const MAP_IND2VAR_L = Dict(1 => :L, 2 => :v_d, 3 => :h_d0, 4 => :psi_d0, 
    5 => :h_d1, 6 => :psi_d1) #local to variable names
const TEMPORAL_MAP = [3 5; 4 6] #[dynamic_variables0; dynamic_variables1]

immutable SideOnDBNParams
    tca::Float64 #time of closest approach in seconds
    v_min::Float64
    v_max::Float64
    vdot_min::Float64
    vdot_max::Float64
    dh_min::Float64 #feet
    dh_max::Float64 #feet
    hdot_min::Float64 #feet per second
    hdot_max::Float64 #feet per second
    psidot_min::Float64 #degrees
    psidot_max::Float64 #degrees
    l_min::Int64
    l_max::Int64
end

function SideOnDBNParams(; 
    tca::Float64=40.0, 
    v_min::Float64=400.0, 
    v_max::Float64=600.0, 
    vdot_min::Float64=-2.0, 
    vdot_max::Float64=2.0, 
    dh_min::Float64=-1000.0, 
    dh_max::Float64=1000.0, 
    hdot_min::Float64=-10.0, 
    hdot_max::Float64=10.0, 
    psidot_min::Float64=0.0, 
    psidot_max::Float64=0.0, 
    l_min::Int64=2,
    l_max::Int64=5) 

    SideOnDBNParams(tca, v_min, v_max, vdot_min, vdot_max, dh_min, dh_max, hdot_min,
        hdot_max, psidot_min, psidot_max, l_min, l_max)
end

type SideOnDBN <: AbstractEncounterDBN
  number_of_aircraft::Int64

  parameters::SideOnDBNParams

  parameter_file::ASCIIString
  aem_parameters::CorrAEMParameters

  encounter_seed::UInt64

  dirichlet_transition

  t::Int64

  #initial state
  initial_states::Vector{CorrAEMInitialState}

  #initial command
  initial_commands_d::Vector{Vector{Int64}} #discrete current state
  initial_commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #current command
  commands_d::Vector{Vector{Int64}} #discrete current state
  commands::Vector{Vector{Float64}} #continuous of current dynamic variables

  #caching and reuse
  dynamic_variables0::Vector{Int64}
  dynamic_variables1::Vector{Int64}
  parents_cache::Dict{Int64, Vector{Bool}}
  weights_cache::Dict{Tuple{Int64,Int64}, Vector{Float64}}
  cumweights_cache::Dict{Tuple{Int64,Int64}, Vector{Float64}}

  #pre-allocated output to avoid repeating reallocations
  output_commands::Vector{CorrAEMCommand}

  logProb::Float64 #log probability of output

  function SideOnDBN(number_of_aircraft::Int,
                   parameter_file::AbstractString,
                   encounter_seed::UInt64,
                   p::SideOnDBNParams = SideOnDBNParams())

    dbn = new()

    dbn.number_of_aircraft = number_of_aircraft
    dbn.parameters = p
    dbn.parameter_file = parameter_file
    dbn.aem_parameters = CorrAEMParameters()
    em_read(dbn.aem_parameters, dbn.parameter_file)

    dbn.encounter_seed = encounter_seed

    dbn.dirichlet_transition = bn_dirichlet_prior(dbn.aem_parameters.N_transition)

    dbn.t = 0

    #compute initial states of variables
    dbn.dynamic_variables0 = TEMPORAL_MAP[:, 1]
    dbn.dynamic_variables1 = TEMPORAL_MAP[:, 2]

    #FIXME: don't use srand here
    srand(encounter_seed) #There's a rand inside generateEncounter
    generateEncounter(dbn) #sets initial_states, initial_commands_d, initial_commands

    dbn.commands_d = deepcopy(dbn.initial_commands_d)
    dbn.commands = deepcopy(dbn.initial_commands)

    #precompute and cache these quantities
    dbn.parents_cache = Dict{Int64, Vector{Bool}}()
    dbn.weights_cache = Dict{Tuple{Int64,Int64}, Vector{Float64}}()
    dbn.cumweights_cache = Dict{Tuple{Int64,Int64}, Vector{Float64}}()

    for i = 1:length(dbn.aem_parameters.N_transition)
      dbn.parents_cache[i] = dbn.aem_parameters.G_transition[:, i]

      for j = 1:1:size(dbn.dirichlet_transition[i], 2)
        dbn.weights_cache[(i, j)] = dbn.aem_parameters.N_transition[i][:, j] + dbn.dirichlet_transition[i][:, j]
        dbn.weights_cache[(i, j)] /= sum(dbn.weights_cache[(i, j)])
        dbn.cumweights_cache[(i, j)] = cumsum(dbn.weights_cache[(i, j)])
      end
    end

    dbn.output_commands = CorrAEMCommand[ CorrAEMCommand(0.0, 0.0, 0.0, 0.0) for i = 1:number_of_aircraft ]
    dbn.logProb = 0.0

    return dbn
  end

end

#this is where most of the initialization differences are
function generateEncounter(dbn::SideOnDBN)
    p = dbn.parameters
    L = rand(p.l_min:p.l_max) #randint
    h_min, h_max = alt_bounds(L) 
    h_mid = (h_min + h_max) / 2

    #initial aircraft states - place in star pattern heading towards origin
    dbn.initial_states = Array(CorrAEMInitialState, dbn.number_of_aircraft)

    @assert dbn.number_of_aircraft == 2
    for i = 1:dbn.number_of_aircraft
        t = 0
        v = randfloat(p.v_min, p.v_max)
        h = i==1 ? h_mid : h_mid + randfloat(p.dh_min, p.dh_max)
        h_d = randfloat(p.hdot_min, p.hdot_max)
        psi = i == 1 ? 0.0 : 90.0 
        x = v * p.tca * cosd(psi + 180)
        y = v * p.tca * sind(psi + 180)

        dbn.initial_states[i] = CorrAEMInitialState(t, x, y, h, v, psi, h_d)
    end

    #initial aircraft commands
    #TODO: Change the data structure here
    #[L,v_d,h_d,psi_d]
    dbn.initial_commands = Array(Vector{Float64}, dbn.number_of_aircraft) 
    #[L,v_d,h_d,psi_d,[hd_tp1,psid_tp1]]
    dbn.initial_commands_d = Array(Vector{Int64}, dbn.number_of_aircraft) 
    
    for i = 1:dbn.number_of_aircraft
        h_d = dbn.initial_states[i].h_d  #set commanded same as current h_d
        psi_d = randfloat(p.psidot_min, p.psidot_max)
        v_d = randfloat(p.vdot_min, p.vdot_max)
    
        #TODO: These should be typed rather than as Vectors...
        dbn.initial_commands[i] = Float64[L, v_d, h_d, psi_d]
        initial_commands_d = discretize(dbn.aem_parameters, 
            unconvert_units(dbn.initial_commands[i]))

        dbn.initial_commands_d[i] = [initial_commands_d; 
            zeros(Int64, length(dbn.dynamic_variables1))]
    end
end

addObserver(dbn::SideOnDBN, f::Function) = add_observer(aem.observer, f)
addObserver(dbn::SideOnDBN, tag::AbstractString, f::Function) = add_observer(aem.observer, tag, f)

function initialize(dbn::SideOnDBN)
  #reset to initial state
  for i = 1:dbn.number_of_aircraft
    copy!(dbn.commands_d[i], dbn.initial_commands_d[i])
    copy!(dbn.commands[i], dbn.initial_commands[i])
  end
  dbn.t = 0
end

getInitialState(dbn::SideOnDBN, index::Int) = dbn.initial_states[index]
get(dbn::SideOnDBN, aircraft_number::Int) = dbn.output_commands[aircraft_number]

function update(dbn::SideOnDBN)
  logProb = 0.0 #to accumulate over each aircraft

  for i = 1:dbn.number_of_aircraft
    logProb += step_dbn(dbn, dbn.commands_d[i], dbn.commands[i])

    dbn.output_commands[i].t = dbn.t
    dbn.output_commands[i].v_d = dbn.commands[i][MAP_VAR2IND_L[:v_d]]
    dbn.output_commands[i].h_d = dbn.commands[i][MAP_VAR2IND_L[:h_d0]]
    dbn.output_commands[i].psi_d = dbn.commands[i][MAP_VAR2IND_L[:psi_d0]]
  end
  dbn.t += 1

  return logProb
end

#TODO: This function needs a cleanup
function step_dbn(dbn::SideOnDBN, command_d::Vector{Int64}, command::Vector{Float64})
    #_G is global (to the file), _L is local (to this module)

    p = dbn.aem_parameters
    logProb = 0.0
    for (o,i_L) in enumerate(dbn.dynamic_variables1)
        i_G = MAP_L2G[i_L]
        j_G = 1
        if !isempty(find(dbn.parents_cache[i_G]))
            parents_L = map(i -> MAP_G2L[i], find(dbn.parents_cache[i_G]))
            dims = tuple(p.r_transition[dbn.parents_cache[i_G]]...)
            j_G = sub2ind(dims, command_d[parents_L]...)
        end

        command_d[i_L] = select_random_cumweights(dbn.cumweights_cache[(i_G, j_G)])
        logProb += log(dbn.weights_cache[(i_G, j_G)][command_d[i_L]]) #prob of bin

        #Resampling and dediscretizing process
        i0_L = dbn.dynamic_variables0[o]
        i0_G = MAP_L2G[i0_L]

        if command_d[i_L] != command_d[i0_L] #Changed bins, resample
            command[i0_L], prob = dediscretize(command_d[i_L], p.boundaries[i0_G], 
                p.zero_bins[i0_G])
            command[i0_L] = convert_units(command[i0_L], MAP_IND2VAR_L[i0_L])
            logProb += log(prob)
        elseif rand() < p.resample_rates[i0_G] #same bin and meets rate
            logProb += log(p.resample_rates[i0_G]) #prob of meeting resample rate
            command[i0_L], prob = dediscretize(command_d[i_L], 
            p.boundaries[i0_G], p.zero_bins[i0_G])
            command[i0_L] = convert_units(command[i0_L], MAP_IND2VAR_L[i0_L])
            logProb += log(prob)
        else #command_d[i_L] == command_d[i0_L] same bin, but does not meet rate
            logProb += log(1.0 - p.resample_rates[i0_G]) #prob of not meeting 
                #resample rate
            #don't resample state here
        end
    end

    # update x(t) with x(t+1)
    command_d[dbn.dynamic_variables0] = command_d[dbn.dynamic_variables1]

    return logProb
end


end #module

