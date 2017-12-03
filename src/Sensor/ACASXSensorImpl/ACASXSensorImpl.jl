# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/23/2014

module ACASXSensorImpl

export
    initialize,
    update,

    updateSensor,

    ACASXSensor,
    ACASXSensorState,
    ACASXSensorInput,
    ACASXSensorOutput


using AbstractSensorImpl
using AbstractSensorInterfaces
using CommonInterfaces

using CASInterface

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractSensorInterfaces.updateSensor

typealias ACASXSensorOutput CASInterface.Input #STM Input

type ACASXSensor <: AbstractSensor
  my_id::Int64 #my id
  max_intruders::Int64
  output::ACASXSensorOutput
end

function ACASXSensor(my_id::Int, max_intruders::Int)
  return ACASXSensor(my_id, max_intruders, ACASXSensorOutput(max_intruders))
end

type ACASXSensorState
  x::Float64 #East
  y::Float64 #North
  z::Float64 #Up
  vx::Float64
  vy::Float64
  vz::Float64
end

type ACASXSensorInput
  states::Vector{ACASXSensorState}
end

getListId(list_owner_id::Integer,id::Integer) = id < list_owner_id ? id : id - 1

function updateSensor(sensor::ACASXSensor, input::ACASXSensorInput)

  own_state = input.states[sensor.my_id]
  ownInput = sensor.output.ownInput

  ownInput.dz = own_state.vz
  ownInput.z = own_state.z #baro alt
  ownInput.psi = atan2(own_state.vx, own_state.vy) #zero when aligned with y-axis / north
  ownInput.h = own_state.z #agl alt
  ownInput.modes = UInt32(sensor.my_id)

  intruders = sensor.output.intruders

  for i = 1:endof(input.states)
    if i != sensor.my_id
      intr_i = getListId(sensor.my_id, i)
      intr_state = input.states[i]
      intruders[intr_i].valid = true
      intruders[intr_i].id = UInt32(i)
      intruders[intr_i].modes = UInt32(i)
      intruders[intr_i].sr = norm([own_state.x, own_state.y, own_state.z]-
                                    [intr_state.x, intr_state.y, intr_state.z]) #slant range (feet)
      intr_psi = atan2(intr_state.x - own_state.x, intr_state.y - own_state.y)
      intruders[intr_i].chi = to_plusminus_pi(intr_psi - ownInput.psi) #bearing (radians, nose is 0, clockwise is positive)
      intruders[intr_i].z = intr_state.z #altitude (feet)  #quantized

      ## These are the defaults.  Modified through coordination
      intruders[intr_i].cvc = 0x0
      intruders[intr_i].vrc = 0x0
      intruders[intr_i].vsb = 0x0
      intruders[intr_i].equipage = EQUIPAGE_TCAS
      intruders[intr_i].quant = intruders[intr_i].equipage == EQUIPAGE_TCAS ? 25 : 100
      intruders[intr_i].sensitivity_index = 0x0
      intruders[intr_i].protection_mode = 0x0
    end
  end

  return sensor.output
end

update(sensor::ACASXSensor, input) = update(sensor, convert(ACASXSensorInput, input))
update(sensor::ACASXSensor, input::ACASXSensorInput) = updateSensor(sensor, input)

function initialize(sensor::ACASXSensor)
  reset!(sensor.output)
end

#mods x to the range [-b, b]
function to_plusminus_b(x::AbstractFloat, b::AbstractFloat)
  z = mod(x, 2 * b)
  return (z > b) ? (z - 2 * b) : z
end

to_plusminus_pi(x::AbstractFloat) = to_plusminus_b(x, Float64(pi))

end


