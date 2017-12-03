# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014


if !isdefined(:PACKAGE_PATH)
    PACKAGE_PATH = Pkg.dir("SISLES", "src")
end

push!(LOAD_PATH, "$PACKAGE_PATH/include")
push!(LOAD_PATH, "$PACKAGE_PATH/common")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/CorrAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/LLAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/SimplePilotResponseImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel/SimpleADMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel/AirspaceImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor/SimpleTCASSensorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/SimpleTCASImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator/TCASSimulatorImpl")


using Util
using Encounter
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CollisionAvoidanceSystem
using Simulator


