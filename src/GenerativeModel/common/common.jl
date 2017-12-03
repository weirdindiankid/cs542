using Encounter
using Sensor
using CollisionAvoidanceSystem
using PilotResponse
using DynamicModel
using WorldModel

import Base.convert

#The fact that these are distributed between the simulators (i.e., TCASSimulatorImpl.jl),
#is a bit messy.
function convert(::Type{StochasticLinearPRCommand}, command::Union{CorrAEMCommand, LLAEMCommand})
    StochasticLinearPRCommand(command.t, command.v_d, command.h_d, command.psi_d, 0.0)
end

function convert(::Type{LLDetPRCommand}, command::Union{CorrAEMCommand, LLAEMCommand}) 
    LLDetPRCommand(command.t, command.v_d, command.h_d, command.psi_d)
end
function convert(::Type{LLDetPRAircraft}, state::ASWMState) 
    LLDetPRAircraft(state.vh)
end

function convert(::Type{SimpleADMCommand}, command::StochasticLinearPRCommand)
    SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)
end

function convert(::Type{SimpleADMCommand}, command::LLDetPRCommand) 
    SimpleADMCommand(command.t, command.v_d, command.h_d, command.psi_d)
end

function convert(::Type{ACASXSensorInput}, states::Vector{ASWMState})
    sr_states = ACASXSensorState[ ACASXSensorState(state.x,state.y,state.h,
        state.vx,state.vy,state.vh) for state in states]
  ACASXSensorInput(sr_states)
end

function convert(::Type{StochasticLinearPRRA},RA::ACASXOutput)
    ra_active = (RA.dh_min > -9999.0 || RA.dh_max < 9999.0)
    StochasticLinearPRRA(ra_active,RA.target_rate,RA.dh_min,RA.dh_max)
end

function convert(::Type{StochasticLinearPRRA},RA::Union{SimpleTCASResolutionAdvisory,Void})
    ra_active = RA != nothing
    StochasticLinearPRRA(ra_active,ra_active ? RA.h_d : 0.0,-9999.0,9999.0)
end

function convert(::Type{LLDetPRRA},RA::ACASXOutput)
    LLDetPRRA(RA.dh_min,RA.dh_max,RA.target_rate,RA.ddh)
end

function convert(::Type{LLDetPRRA},RA::Union{SimpleTCASResolutionAdvisory,Void})
    ra_active = RA != nothing
    if !ra_active
        #not active
        return LLDetPRRA(-9999.0,9999.0,0.0)  #TODO: put these hardcodings in a central location
    else
        #RA
        if RA.h_d > 0
            #climb
            return LLDetPRRA(RA.h_d, 9999.0, RA.h_d)
        elseif RA.h_d < 0
            #descend
            return LLDetPRRA(-9999.0, RA.h_d, RA.h_d)
        else
            #level
            return LLDetPRRA(RA.h_d, RA.h_d, RA.h_d)
        end
    end
    error("Shouldn't have gotten here")
end

function convert(::Type{LLADMCommand}, out::LLDetPROutput) 
    LLADMCommand(out.t, out.h_d, deg2rad(out.psi_d), out.v_d, out.dh_min, 
        out.dh_max, out.target_rate, out.ddh)
end

function convert(::Type{LLADMState}, s::CorrAEMInitialState) 
    LLADMState(s.t, s.v, s.x, s.y, s.h, deg2rad(s.psi), asin(s.h_d / s.v), 0.0, 0.0)
end

convert(::Type{ASWMState}, s::LLADMOutputState) = ASWMState(s.t, s.x, s.y, s.h, s.vx, s.vy, s.vh)

