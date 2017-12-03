# Author: Ritchie Lee, ritchie.lee@sv.cmu.edu
# Date: 04/14/2015
# Implements Lincoln Lab scripted dynamics model

module LLADMImpl

export
addObserver,

initialize,
update,

initializeDynamicModel,
simulateDynamicModel,

LLADM,
LLADMState,
LLADMOutputState,
LLADMCommand


using AbstractDynamicModelImpl
using AbstractDynamicModelInterfaces
using CommonInterfaces
using RLESUtils, Observers

using Base.Test

import CommonInterfaces.addObserver
import CommonInterfaces.initialize
import CommonInterfaces.update
import AbstractDynamicModelInterfaces.initializeDynamicModel
import AbstractDynamicModelInterfaces.simulateDynamicModel

# Note: Angles are all in radians in this module unless otherwise indicated
# CASSATT_TIMING is false

type LLADMState

  t::Float64       #time (s)
  v::Float64       #airspeed (ft/s)
  N::Float64       #position north (ft)
  E::Float64       #position east (ft)
  h::Float64       #position altitude (ft)
  psi::Float64     #heading angle (rad), N is zero, clockwise is positive
  theta::Float64   #pitch angle (rad)
  phi::Float64     #bank angle (rad)
  a::Float64       #airspeed acceleration (ft/s^2)
end

type LLADMOutputState #mitcas state in scripted_dynamics.h

  t::Float64
  x::Float64
  y::Float64
  h::Float64
  vx::Float64   #DX in scripted_dynamics.h
  vy::Float64   #DY in scripted_dynamics.h
  vh::Float64   #DH in scripted_dynamics.h
  psi::Float64
  theta::Float64
  phi::Float64
end

LLADMOutputState() = LLADMOutputState(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)

type LLADMCommand

  t::Float64        #time of command (s)
  hdot::Float64     #commanded vertical rate (ft/s)
  psidot::Float64   #commanded turn rate (rad/s)
  a::Float64        #commanded acceleration (ft/s^2)
  dh_min::Float64          #dh_min of advisory being followed (ft/s)
  dh_max::Float64          #dh_max of advisory being followed (ft/s)
  target_rate::Float64     #target_rate of advisory being followed (ft/s)
  ddh::Float64             #ddh of advisory being followed (ft/s^2)

end

immutable LLADMConsts

  _dt::Float64           # minor time step
  _K::Float64            # integrator gain
  _g::Float64            # acceleration of gravity (ft/s^2)
  _qmax::Float64         # as in CASSATT (rad/s)
  _rmax::Float64         # as in CASSATT GA_psidotMAX = 1e6
  _phi_max::Float64
  _phidotmax::Float64
  _v_high::Float64       # airspeed limits
  _v_low::Float64
  LIMIT_DDH::Bool
  _min_vertical_rate::Float64
  _max_vertical_rate::Float64
  _min_vertical_accel::Float64
  _max_vertical_accel::Float64

end

type LLADM <: AbstractDynamicModel

  state::LLADMState
  update::Union{LLADMCommand, Void}
  timestep::Float64
  params::LLADMConsts
  output_state::LLADMOutputState

  observer::Observer

  function LLADM(; timestep::Float64 = 1.0, #major time step
                 _dt::Float64 = 0.1, #minor time step
                 _K::Float64 = 1.0,
                 _g::Float64 = 32.2,
                 _qmax::Float64 = deg2rad(3.0),
                 _rmax::Float64 = 1000000.0,
                 _phi_max::Float64 = deg2rad(75.0),
                 _phidotmax::Float64 = 0.524,
                 _v_high::Float64 = 100000.0,
                 _v_low::Float64 = 1.7,
                 LIMIT_DDH::Bool = true,
                 _min_vertical_rate::Float64 = -9999.0,
                 _max_vertical_rate::Float64 = 9999.0,
                 _min_vertical_accel::Float64 = -9.66,
                 _max_vertical_accel::Float64 = 9.66,
                 )

    obj = new()

    obj.update = nothing
    obj.timestep = timestep

    obj.params = LLADMConsts(_dt,
                               _K,
                               _g,
                               _qmax,
                               _rmax,
                               _phi_max,
                               _phidotmax,
                               _v_high,
                               _v_low,
                               LIMIT_DDH,
                               _min_vertical_rate,
                               _max_vertical_rate,
                               _min_vertical_accel,
                               _max_vertical_accel
                               )

    obj.output_state = LLADMOutputState()

    obj.observer = Observer()

    return obj
  end
end

addObserver(adm::LLADM, f::Function) = add_observer(adm.observer, f)
addObserver(adm::LLADM, tag::AbstractString, f::Function) = add_observer(adm.observer, tag, f)

initialize(adm::LLADM, state) = initialize(adm,convert(LLADMState, state))

function initialize(adm::LLADM, state::LLADMState)
  adm.state = state

  # Intialize the mitcas state
  # Note: x is E and y is N in scripted_dynamics.h
  adm.output_state.t     = state.t
  adm.output_state.h     = state.h
  adm.output_state.vh    = sin(state.theta) * state.v
  #adm.output_state.x     = state.E
  #adm.output_state.vx    = sin(state.psi) * cos(state.theta) * state.v
  #adm.output_state.y     = state.N
  #adm.output_state.vy    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.psi   = state.psi
  adm.output_state.theta = state.theta
  adm.output_state.phi   = state.phi

  # RLEE: Override x and y to match LLCEM.  X is north
  adm.output_state.x     = state.N
  adm.output_state.vx    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.y     = state.E
  adm.output_state.vy    = sin(state.psi) * cos(state.theta) * state.v

  adm.update = nothing

  return adm.output_state
end

# Mirrors state_update() from scripted_dynamics.h
function state_update(C::LLADMConsts, state::LLADMState, ctrl::LLADMCommand, hddcmd::Float64, dt::Float64)
  # Inputs:
  # C = constants
  # state = current state (adm.state)
  # ctrl_update = current command (update::LLADMCommand)
  # hddcmd = vertical acceleration to apply
  # dt = time step

  const s_theta = sin(state.theta)
  const c_theta = cos(state.theta)
  const t_theta = tan(state.theta)
  const s_phi = sin(state.phi)
  const c_phi = cos(state.phi)
  const s_psi = sin(state.psi)
  const c_psi = cos(state.psi)

  # save v and saturated v
  const v = state.v
  const v_saturated = max(v,1.0)

  # compute and saturate q
  q = saturate(1.0 / (v_saturated * c_phi) * (hddcmd / c_theta + C._g * c_theta * s_phi * s_phi - ctrl.a * t_theta), -C._qmax, C._qmax)

  # compute and saturate r
  r = saturate(C._g * s_phi * c_theta / v_saturated, -C._rmax, C._rmax)

  # sometimes the aircraft bank angle is so large that it would prevent reaching desired h_dot,
  # so we dynamically saturate the bank angle so that h_dot is always acheived
  # compute phimax (in the Plopt/compute q block)
  hdd_cmd_phi = min(hddcmd, v_saturated * C._qmax * c_phi * c_theta)

  # calculate discriminant
  sqrt_arg = v_saturated * v_saturated * C._qmax * C._qmax - 4 * C._g * ctrl.a * s_theta + 4 * C._g * hdd_cmd_phi + 4 * C._g * C._g * c_theta * c_theta
  phi_max_2 = 0

  if sqrt_arg < 0
    phi_max_2 = 10000
  else
    # calculate cos(phi)
    cphi1 = (sqrt(sqrt_arg) - v_saturated * C._qmax) / (2 * C._g * c_theta)

    if abs(cphi1) < 1
      phi_max_2 = acos(cphi1) * 0.98 # add a small buffer to prevent jittering
    end
  end

  phimax = min(C._phi_max, phi_max_2)

  # compute and saturate p
  psidot_if_no_change = (q * s_phi + r * c_phi) / c_theta
  dpsidot = ctrl.psidot - psidot_if_no_change
  p = saturate(20.0 * dpsidot, -C._phidotmax, C._phidotmax) # 20 is user tuned parameter from CASSATT

  # limit max bank angle
  if (state.phi + p * dt > phimax)

    p = (phimax - state.phi) / dt
  end

  if (state.phi + p * dt < -phimax)

    p = (-phimax - state.phi) / dt
  end

  state.v += dt * (ctrl.a)
  state.N += dt * (v * c_theta * c_psi)
  state.E += dt * (v * c_theta * s_psi)
  state.h += dt * (v * s_theta)
  state.psi += dt * (q * s_phi / c_theta + r * c_phi / c_theta)
  state.psi = to_plusminus_pi(state.psi) #RLEE added: mod back to [-pi, pi]
  state.theta += dt * (q * c_phi - r * s_phi)
  state.theta = to_plusminus_pi(state.theta) #RLEE added: mod back to [-pi, pi]
  state.phi += dt * (p + q * s_phi * t_theta + r * c_phi * t_theta)
  state.phi = to_plusminus_pi(state.phi) #RLEE added: mode back to [-pi, pi]

  state.v = saturate(state.v, 1.7, C._v_high - 0.000001)
  state.t += dt

end

update(adm::LLADM, ctrl) = update(adm,convert(LLADMCommand, ctrl))

function update(adm::LLADM, ctrl::LLADMCommand)

  state = adm.state
  C = adm.params #constants

  t_start = state.t

  for i = 1:Int64(round(adm.timestep / C._dt))
    substep(state, ctrl, C)
  end

  #snap so small errors don't accumulate
  state.t = snap(state.t, t_start + adm.timestep)

  #make sure timing is in sync
  @test state.t == t_start + adm.timestep

  # calculate the new mitcas state vars
  # Note: x is E and y is N in scripted_dynamics.h
  adm.output_state.t     = state.t
  adm.output_state.h     = state.h
  adm.output_state.vh    = sin(state.theta) * state.v
  #adm.output_state.x     = state.E
  #adm.output_state.vx    = sin(state.psi) * cos(state.theta) * state.v
  #adm.output_state.y     = state.N
  #adm.output_state.vy    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.psi   = state.psi
  adm.output_state.theta = state.theta
  adm.output_state.phi   = state.phi

  # RLEE: Override x and y to match LLCEM.  X is north
  adm.output_state.x     = state.N
  adm.output_state.vx    = cos(state.psi) * cos(state.theta) * state.v
  adm.output_state.y     = state.E
  adm.output_state.vy    = sin(state.psi) * cos(state.theta) * state.v

  return adm.output_state
end

function substep(state::LLADMState, ctrl::LLADMCommand, C::LLADMConsts)

  dh = state.v * sin(state.theta)

  dummy, ddh_script = response(dh, (ctrl.hdot - dh) / C._dt,
                                       ctrl.dh_min, ctrl.dh_max,
                                       C._min_vertical_rate, C._max_vertical_rate, C._dt)

  dummy, ddh_cmd = response(dh, ctrl.dh_min, ctrl.dh_max, 0.0, 0.0, ctrl.ddh,
                                 C._min_vertical_rate, C._max_vertical_rate, C._dt)

  ddh2apply = resolve_TCAS_and_script(ddh_cmd,ddh_script)

  #debug
  #println("ddh_script=", ddh_script, ", ddh_cmd=", ddh_cmd, ", ddh2apply=", ddh2apply)

  if C.LIMIT_DDH
    ddh2apply = saturate(ddh2apply, C._min_vertical_accel, C._max_vertical_accel)
  end

  state_update(C, state, ctrl, ddh2apply, C._dt)
end

function saturate(x::Float64, limit_min::Float64, limit_max::Float64)
  @test limit_min <= limit_max

  return min(max(x,limit_min),limit_max)
end

function response(dh::Float64, ddh_in::Float64, dh_cmd_min::Float64, dh_cmd_max::Float64,
                  min_dh::Float64, max_dh::Float64, dt::Float64)

  # default value for ddh_out
	ddh_out = ddh_in

	# new projected vertical rate
	dh_proj = dh + ddh_out * dt

	# whether or not inside green zone
	dh_inside_green = saturate(dh, dh_cmd_min, dh_cmd_max) == dh

	if (dh > dh_cmd_max && dh_proj < dh_cmd_min) || (dh_inside_green && dh_proj > dh_cmd_max)

		# OUTSIDE GREEN ZONE
		# cross max edge from above and go below min edge => hit max edge
		# OR
		# INSIDE GREEN ZONE
		# cross max edge from below and go above max edge => hit max edge

		dh_proj = dh_cmd_max

	elseif (dh < dh_cmd_min && dh_proj > dh_cmd_max) || (dh_inside_green && dh_proj < dh_cmd_min)

		# OUTSIDE GREEN ZONE
		# cross min edge from below and go above max edge => hit min edge
		# OR
		# INSIDE GREEN ZONE
		# cross min edge from above and go below min edge => hit min edge

		dh_proj = dh_cmd_min

	elseif dh_proj < min_dh

		# cross max descent rate => hit max descent rate
		dh_proj = min_dh

	elseif dh_proj > max_dh

		# cross max climb rate => hit max climb rate
		dh_proj = max_dh

  end

	# recompute ddh
	ddh_out = dt != 0.0 ? (dh_proj - dh) / dt : 0.0

	# compute next dh
	dh_next = snap(dh + ddh_out * dt, dh_cmd_min)
	dh_next = snap(dh_next, dh_cmd_max)
	dh_next = snap(dh_next, min_dh)
	dh_next = snap(dh_next, max_dh)

  return dh_next, ddh_out
end

function response(dh::Float64, dh_cmd_min::Float64, dh_cmd_max::Float64, ddh_no_response::Float64,
                  ddh_response::Float64, cmd_ddh::Float64, min_dh::Float64, max_dh::Float64, dt::Float64)

  ddh_in = acceleration_to_apply(dh, ddh_no_response, ddh_response, dh_cmd_min, dh_cmd_max, cmd_ddh)

  return response(dh, ddh_in, dh_cmd_min, dh_cmd_max, min_dh, max_dh, dt)
end

function acceleration_to_apply(dh::Float64, ddh_no_response::Float64, ddh_response::Float64,
                               dh_cmd_min::Float64, dh_cmd_max::Float64, cmd_ddh::Float64)


  ddh = ddh_no_response

  # resolution advisory handling
	if (saturate(dh, dh_cmd_min, dh_cmd_max) != dh)

		# commanded acceleration
		ddh = cmd_ddh

		# determine direction
		if (dh > dh_cmd_max)
      ddh = -ddh
    end

		# add response noise
		ddh += ddh_response;
  end

	return ddh
end

function resolve_TCAS_and_script(ddh_TCAS::Float64, ddh_script::Float64)

  opposite_senses = ddh_script * ddh_TCAS < 0
  same_senses = !opposite_senses

  cmd_greater = abs(ddh_TCAS) > abs(ddh_script)

  return (ddh_TCAS != 0.0 && ((same_senses && cmd_greater) || opposite_senses)) ? ddh_TCAS : ddh_script
end

snap(x, y) = approxeq(x, y, 1e-8) ? y : x

approxeq(x, y, eps) = (x == y) ? true : abs(x-y) < eps

#mods x to the range [-b, b]
function to_plusminus_b(x::AbstractFloat, b::AbstractFloat)
  z = mod(x, 2 * b)
  return (z > b) ? (z - 2 * b) : z
end

to_plusminus_pi(x::AbstractFloat) = to_plusminus_b(x, Float64(pi))

end
