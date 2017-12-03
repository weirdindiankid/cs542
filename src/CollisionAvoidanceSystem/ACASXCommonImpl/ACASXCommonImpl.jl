# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/21/2014

#ACASX implementation: based on CCAS interface to libCAS

module ACASXCommonImpl

export
    addObserver,

    ACASXInput,
    ACASXOutput,
    ACASXCoordRecord

    #use explicit form for these
    #initialize
    #step
    #update_from_coord!
    #update_to_coord!


using AbstractCollisionAvoidanceSystemImpl
using AbstractCollisionAvoidanceSystemInterfaces
using CommonInterfaces

using AbstractCASCoordImpl

using CASCoordination
using CASInterface
using RLESUtils, Observers

import CASInterface.reset! #extend

typealias ACASXInput CASInterface.Input
typealias ACASXOutput CASInterface.Output

type ACASXCoordRecordIntruder
  id::UInt32
  cvc::UInt8
  vrc::UInt8
  vsb::UInt8

  ACASXCoordRecordIntruder(id::Int) = new(id, 0x0, 0x0, 0x0)
end

type ACASXCoordRecord
  my_id::UInt32
  modes::UInt32
  equipage::EQUIPAGE
  quant::UInt8
  sensitivity_index::UInt8
  protection_mode::UInt8
  intruders::Vector{ACASXCoordRecordIntruder}
end
function ACASXCoordRecord(my_id::Int64, equipage::EQUIPAGE, quant::Int64, max_intruders::Int64)
  ACASXCoordRecord(UInt32(my_id), equipage, UInt8(quant), max_intruders)
end
function ACASXCoordRecord(my_id::UInt32, equipage::EQUIPAGE, quant::UInt8, max_intruders::Int64)
  ACASXCoordRecord(my_id, my_id, equipage, quant, 0x0, 0x0,
                   [ACASXCoordRecordIntruder(i) for i = 1:max_intruders])
end

getListId(list_owner_id::Integer,id::Integer) = id < list_owner_id ? id : id - 1

function update_from_coord!(input::ACASXInput, coord::AbstractCASCoord, my_id::Int64)
  #Grab from coordination object
  coord_records = CASCoordination.getAll(coord)

  #Augment the input with info from coord
  #note: looping over intruders skips self
  for intruder in input.intruders

    my_list_id = getListId(intruder.id, my_id)
    record = coord_records[intruder.id]

    #intruder-shared
    intruder.equipage             = record.equipage
    intruder.quant                = record.quant
    intruder.sensitivity_index    = record.sensitivity_index
    intruder.protection_mode      = record.protection_mode

    #quantize sensor reading depending on equipage
    #TODO: should these be moved to Sensor?
    intruder.z = quantize(intruder.z, Float64(intruder.quant))

    #intruder-specific
    intruder_self = record.intruders[my_list_id] #self in intruders' record

    @assert my_id == intruder_self.id #sanity check the record

    intruder.cvc = intruder_self.cvc
    intruder.vrc = intruder_self.vrc
    intruder.vsb = intruder_self.vsb
  end
end

#update coordination records with output
function update_to_coord!(coord::AbstractCASCoord, my_id::Int64, output::ACASXOutput)
  my_record = getRecord(coord, my_id)

  #update my record in coordination object
  #intruder-shared
  my_record.sensitivity_index = output.sensitivity_index
  #others (id,modes,equipage,quant) don't need to be updated...

  #intruder-specific
  for i = 1:endof(my_record.intruders)
    my_record.intruders[i].id           = output.intruders[i].id
    my_record.intruders[i].cvc          = output.intruders[i].cvc
    my_record.intruders[i].vrc          = output.intruders[i].vrc
    my_record.intruders[i].vsb          = output.intruders[i].vsb
  end
  setRecord(coord, my_id, my_record) #push back to coord

  return output
end

function initialize(cas)
  reset!(cas.input)
  reset!(cas.output)
  reset!(cas, getRecord(cas.coord, cas.my_id))
end

function reset!(cas, rec::ACASXCoordRecord)
  rec.my_id = cas.my_id
  rec.modes = cas.my_id
  rec.equipage = cas.equipage
  rec.quant = cas.quant
  rec.sensitivity_index = 0x0
  rec.protection_mode = 0x0

  for i = 1:endof(rec.intruders)
    rec.intruders[i].id = UInt32(i)
    rec.intruders[i].cvc = 0x0
    rec.intruders[i].vrc = 0x0
    rec.intruders[i].vsb = 0x0
  end
end

function quantize(x::AbstractFloat, b::AbstractFloat)
  # quantize x to the nearest multiple of b
  d, r = divrem(x, b)
  return b * (d + round(r / b))
end

end #module


