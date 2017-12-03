# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

#Piggyback and add coordination to SimpleTCAS

module CoordSimpleTCASImpl

export
    addObserver,

    initialize,
    update,

    testThreat,
    selectRA,

    CoordSimpleTCAS,
    CoordSimpleTCASInput,
    CoordSimpleTCASResolutionAdvisory


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces

using Util
using RLESUtils, Observers

using AbstractCASCoordImpl

using CASCoordination
using SimpleTCASImpl

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractCollisionAvoidanceSystemInterfaces.testThreat
import AbstractCollisionAvoidanceSystemInterfaces.selectRA


type CoordSimpleTCAS <: AbstractCollisionAvoidanceSystem
  aircraft_number::Int64
  simpleTCAS::SimpleTCAS
  coordination::AbstractCASCoord

  CoordSimpleTCAS(aircraft_number::Int,coordination::AbstractCASCoord) = new(aircraft_number,SimpleTCAS(),coordination)
end

addObserver(cas::CoordSimpleTCAS, f::Function) = add_observer(cas.observer, f)
addObserver(cas::CoordSimpleTCAS, tag::AbstractString, f::Function) = add_observer(cas.observer, tag, f)

testThreat(cas::CoordSimpleTCAS, input::SimpleTCASInput) = testThreat(cas.simpleTCAS, input)

function selectRA(cas::CoordSimpleTCAS, input::SimpleTCASInput)

  #Make a two aircraft assumption for now
  if cas.aircraft_number == 1
    intruder_number = 2
  else
    intruder_number = 1
  end

  intruderRA = getRecord(cas.coordination,intruder_number) #The coordination object is ownship's RA

  if intruderRA != nothing
    h_d = intruderRA.h_d < 0 ? 1500 / 60 : -1500 / 60  #Do opposite sense, default strength
  else
    h_d = selectRA(cas.simpleTCAS, input)
  end

  return h_d
end

update(cas::CoordSimpleTCAS, input) = update(cas,convert(SimpleTCASInput, input))

function update(cas::CoordSimpleTCAS, input::SimpleTCASInput)

  if cas.simpleTCAS.b_TCAS_activated == false
    cas.simpleTCAS.b_TCAS_activated = testThreat(cas, input)

    if cas.simpleTCAS.b_TCAS_activated
      cas.simpleTCAS.RA = SimpleTCASResolutionAdvisory(selectRA(cas, input))
      setRecord(cas.coordination,cas.aircraft_number,cas.simpleTCAS.RA)
    end
  end

  return cas.simpleTCAS.RA
end

function initialize(cas::CoordSimpleTCAS)
    initialize(cas.simpleTCAS)
end

end


