# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 10/13/2014

# Reference: Chryssanthacopoulos, Kochenderfer - Collision Avoidance System Optimization with
# Probabilistic Pilot Response Models. American Control Conference. 2011.


module StochasticLinearPRImpl

export
    initialize,
    update,

    updatePilotResponse,

    StochasticLinearPR,
    StochasticLinearPRCommand,
    StochasticLinearPRRA

using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractPilotResponseInterfaces.updatePilotResponse

const statemap = Dict(:coc => 1, :first => 2, :multi => 3)
const RAmap = Dict(:none => 1, :climb => 2, :level => 3, :descend => 4)
const responsemap = Dict(:none => 1, :stay => 2, :follow => 3)
const strength_change_map = Dict(true => 1, false => 2)

function generateProbabilityTable()

  #key = (state,displayRA,response,currentRA,diffStrength)
  #value = (probability vector, output values vector)
  # where output values vector is an array of tuples (nextState,response) corresponding
  # to each probability given in probability vector
  #state = {:coc = clear of conflict, :first = first RA, :multi = change in RA}
  #displayRA = {:none = no RA, :climb, :level, :descend}
  #response = {:none = not following, :stay = following last displayRA, :follow = following latest displayRA}
  #currentRA = incoming RA, values are same as displayRA
  #diffStrength = {true=currentRA has same symbol but different target_rate, false=same target_Rate}
  A = Array(Tuple{Vector{Float64},Vector{Tuple{Symbol,Symbol}}}, length(statemap), length(RAmap), length(responsemap),
            length(RAmap), length(strength_change_map))

  # all entries going to :none go there with prob 1
  A[:, :, :, RAmap[:none], :] = ([1.0],[(:coc,:none)])

  # First RA
  for RA in map(x -> RAmap[x], [:climb, :level, :descend])
    #transition from coc to first
    A[statemap[:coc], RAmap[:none], responsemap[:none],:, strength_change_map[false]] = ([1/6,5/6],[(:first,:follow),(:first,:none)]) #first time presenting RA

    #staying in first
    A[statemap[:first], RA, responsemap[:none], RA, strength_change_map[false]] = ([1/6,5/6],[(:first,:follow),(:first,:none)]) #already active, but not following yet
    A[statemap[:first], RA, responsemap[:follow], RA, strength_change_map[false]] = ([1.0],[(:first,:follow)]) #already following
  end

  # Transition into multi
  for displayRA in map(x -> RAmap[x], [:climb, :level, :descend])
    for currentRA in map(x -> RAmap[x], [:climb, :level, :descend])
      for state in map(x -> statemap[x], [:first, :multi])
        b_same = displayRA == currentRA #same sense?
        A[state, displayRA, responsemap[:none], currentRA, strength_change_map[b_same]] = ([1/4,3/4],[(:multi,:follow),(:multi,:none)]) #change already active, but not following yet
        A[state, displayRA, responsemap[:stay], currentRA, strength_change_map[b_same]] = ([1/4,3/4],[(:multi,:follow),(:multi,:stay)]) #change already active, but not following yet
        A[state, displayRA, responsemap[:follow], currentRA, strength_change_map[b_same]] = ([1/4,3/4],[(:multi,:follow),(:multi,:stay)]) #change already active, but not following yet
      end
    end
  end

  # Staying in multi
  for RA in map(x -> RAmap[x], [:climb, :level, :descend])
    A[statemap[:multi], RA, responsemap[:none], RA, strength_change_map[false]] = ([1/4,3/4],[(:multi,:follow),(:multi,:none)]) #already active, but not following yet
    A[statemap[:multi], RA, responsemap[:stay], RA, strength_change_map[false]] = ([1/4,3/4],[(:multi,:follow),(:multi,:stay)]) #already active, but not following yet
    A[statemap[:multi], RA, responsemap[:follow], RA, strength_change_map[false]] = ([1.0],[(:multi,:follow)]) #already following
  end

  return A
end

const probabilityTable = generateProbabilityTable() #generate it once and store it

type StochasticLinearPRCommand

    t::Float64
    v_d::Float64
    h_d::Float64
    psi_d::Float64

    logProb::Float64 #log probability of generating this command
end

type StochasticLinearPRRA
  ra_active::Bool #true if RA is on
  target_rate::Float64
  dh_min::Float64 #min bound on h_d
  dh_max::Float64 #max bound on h_d
end

type StochasticLinearPR <: AbstractPilotResponse

  state::Symbol #[:coc,:first,:multi]
  displayRA::Symbol #RA currently displaying = [:none, :climb, :descend]
  target_rate::Union{Void,Float64} #target_rate of currently following, used for :stay
  response::Symbol #current pilot response = [:none, :stay, :follow]
  response_time::Int64 #for first RA
  output::StochasticLinearPRCommand

  probTable::Array{Tuple{Vector{Float64},Vector{Tuple{Symbol,Symbol}}}} #transition prob table

  function StochasticLinearPR()

    obj = new()
    obj.state = :coc
    obj.displayRA = :none
    obj.target_rate = nothing
    obj.response = :none
    obj.output = StochasticLinearPRCommand(0.0, 0.0, 0.0, 0.0, 0.0)
    obj.response_time = 0
    obj.probTable = probabilityTable

    return obj
  end
end

function updatePilotResponse(pr::StochasticLinearPR, update::StochasticLinearPRCommand, RA::StochasticLinearPRRA)

  t, v_d, h_d, psi_d = update.t, update.v_d, update.h_d, update.psi_d

  if RA.ra_active
    ra = RA.target_rate == 0 ? :level : RA.target_rate > 0 ? :climb : :descend
  else
    ra = :none
  end

  strengthening = (pr.displayRA == ra != :coc) && (nothing != pr.target_rate != RA.target_rate)
  probabilities, values = pr.probTable[statemap[pr.state], RAmap[pr.displayRA], responsemap[pr.response],
                                      RAmap[ra], strength_change_map[strengthening]]
  index = select_random(probabilities)
  pr.state,pr.response = values[index]
  pr.displayRA = ra

  if pr.displayRA != :none && pr.response == :none
    pr.response_time += 1
  end

  if pr.response == :stay
    h_d = pr.target_rate
  elseif pr.response == :follow
    h_d = pr.target_rate = RA.target_rate
  elseif pr.response == :none
    pr.target_rate = nothing
  else
    error("StochasticLinearPRImpl::updatePilotResponse: No such response!")
  end

  #Assign to output
  pr.output.t = t
  pr.output.v_d = v_d
  pr.output.h_d = h_d
  pr.output.psi_d = psi_d
  pr.output.logProb = log(probabilities[index])

  return pr.output
end

update(pr::StochasticLinearPR, command, RA) = update(pr,
                                                convert(StochasticLinearPRCommand, command),
                                                convert(StochasticLinearPRRA,RA))

update(pr::StochasticLinearPR,
     command::StochasticLinearPRCommand,
     RA::StochasticLinearPRRA) = updatePilotResponse(pr, command, RA)

function initialize(pr::StochasticLinearPR)
  pr.state = :coc
  pr.displayRA = :none
  pr.response = :none
  pr.response_time = 0
end

function select_random(weights)
# SELECT_RANDOM Randomly selects an index according to specified weights.
#   Returns a randomly selected index according to the distribution
#   specified by a vector of weights.
#
#   INDEX = SELECT_RANDOM(WEIGHTS) returns a scalar index INDEX selected
#   randomly according to the specified weights WEIGHTS represented as an
#   array.

    s = cumsum(weights)
    r = s[end] * rand()
    index = findfirst(x -> (x >= r), s)

    return index
end

end

