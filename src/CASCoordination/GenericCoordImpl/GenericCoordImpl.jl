# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 11/11/2014

module GenericCoordImpl

export
    initialize,
    update,

    setRecord,
    getRecord,
    getAll,

    GenericCoord,
    GenericCoordRecord


using AbstractCASCoordImpl
using AbstractCASCoordInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractCASCoordInterfaces.setRecord
import AbstractCASCoordInterfaces.getRecord
import AbstractCASCoordInterfaces.getAll

type GenericCoord <: AbstractCASCoord

  num_records::Int64
  records::Vector{Any}

  function GenericCoord(num_records::Int)
    obj = new()

    obj.num_records = num_records
    obj.records = [nothing for i=1:num_records]

    return obj
  end
end

function setRecord(coord::GenericCoord, record_number::Int, record)

    coord.records[record_number] = record
end

update(coord::GenericCoord, record_number::Int, data) = setRecord(coord, record_number, record)

function initialize(coord::GenericCoord)

  fill!(coord.records, nothing)
end

getRecord(coord::GenericCoord,record_number::Int) = coord.records[record_number]

getAll(coord::GenericCoord) = coord.records

end


