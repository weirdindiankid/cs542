# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AirSpaceImpl

export
    initialize,
    update,

    updateObjectState,
    getAllObjectStates,
    getAll,
    updateAll,

    AirSpace,
    ASWMState


using AbstractWorldModelImpl
using AbstractWorldModelInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractWorldModelInterfaces.updateObjectState
import AbstractWorldModelInterfaces.getAllObjectStates
import AbstractWorldModelInterfaces.getAll
import AbstractWorldModelInterfaces.updateAll


type ASWMState

    t::Float64
    x::Float64
    y::Float64
    h::Float64
    vx::Float64
    vy::Float64
    vh::Float64
end

type AirSpace <: AbstractWorldModel

    number_of_aircraft::Int

    states::Vector{ASWMState}
    states_update::Vector{ASWMState}


    function AirSpace(number_of_aircraft::Int)

        obj = new()

        obj.number_of_aircraft = number_of_aircraft

        obj.states = Array(ASWMState, number_of_aircraft)
        obj.states_update = Array(ASWMState, number_of_aircraft)

        return obj
    end
end

function initialize(as::AirSpace, aircraft_number::Int, state)
  initialize(as,aircraft_number,convert(ASWMState, state))
end

function initialize(as::AirSpace, aircraft_number::Int, state::ASWMState)

    as.states[aircraft_number] = state
end

function updateObjectState(as::AirSpace, aircraft_number::Int, state::ASWMState)

    as.states_update[aircraft_number] = state
end

update(as::AirSpace, aircraft_number::Int, state) = update(as, aircraft_number, convert(ASWMState,state))

update(as::AirSpace, aircraft_number::Int, state::ASWMState) = updateObjectState(as, aircraft_number, state)

function getAllObjectStates(as::AirSpace)

    return as.states
end

getAll(as::AirSpace) = getAllObjectStates(as)

function updateAll(as::AirSpace)

    as.states = copy(as.states_update)
end

end


