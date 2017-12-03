# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module SimplePilotResponseImpl

export
    initialize,
    update,

    updatePilotResponse,

    SimplePilotResponse,
    SimplePRResolutionAdvisory,
    SimplePRCommand


using AbstractPilotResponseImpl
using AbstractPilotResponseInterfaces
using CommonInterfaces

import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractPilotResponseInterfaces.updatePilotResponse


type SimplePRResolutionAdvisory
  h_d::Float64
end

type SimplePRCommand

  t::Float64
  v_d::Float64
  h_d::Float64
  psi_d::Float64

  logProb::Float64
end

type SimplePilotResponse <: AbstractPilotResponse

  b_CAS_activated::Bool
  RA::SimplePRResolutionAdvisory

  function SimplePilotResponse()

    obj = new()

    obj.b_CAS_activated = false

    return obj
  end
end

function updatePilotResponse(pr::SimplePilotResponse, command::SimplePRCommand, RA::Union{SimplePRResolutionAdvisory,Void})

  t, v_d, h_d, psi_d = command.t, command.v_d, command.h_d, command.psi_d

  if RA != nothing && pr.b_CAS_activated == false
    pr.RA = RA
    pr.b_CAS_activated = true
  end

  if pr.b_CAS_activated
    h_d = pr.RA.h_d
  end

  return SimplePRCommand(t, v_d, h_d, psi_d, 0.0)
end

update(pr::SimplePilotResponse, command, RA) = update(pr,convert(SimplePRCommand, command), convert(SimplePRResolutionAdvisory,RA))

update(pr::SimplePilotResponse, command::SimplePRCommand, RA::Union{SimplePRResolutionAdvisory,Void}) = updatePilotResponse(pr, command, RA)

function initialize(pr::SimplePilotResponse)

  pr.b_CAS_activated = false
end

end


