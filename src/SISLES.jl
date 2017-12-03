# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014


module SISLES

export
    CorrAEM,
    LLAEM,
    CorrAEMDBN,
    StarDBN,
    SideOnDBN,
    PairwiseCorrAEMDBN,
    SimplePilotResponse,
    StochasticLinearPR,
    LLDetPR,
    SimpleADM,
    LLADM,
    AirSpace,
    SimpleTCASSensor,
    ACASXSensor,
    GenericCoord,
    SimpleTCAS,
    CoordSimpleTCAS,
    ACASX_CCAS,
    ACASX_ADD,
    TCASSimulator,
    SimpleTCAS_EvU, #deprecated
    SimpleTCAS_EvE, #deprecated
    ACASX_GM,

    addObserver,
    clearObservers!,
    initialize,
    update,
    outputGFormatString,

    # Encounter
    getTrajectory,
    setInitialDistributions,
    validate,
    getInitialSample,

    # Simulator
    simulate


PACKAGE_PATH = Pkg.dir("SISLES", "src")

push!(LOAD_PATH, "$PACKAGE_PATH/include")
push!(LOAD_PATH, "$PACKAGE_PATH/common")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/CorrAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Encounter/LLAEMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN/CorrAEMDBNImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN/StarDBNImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN/SideOnDBNImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/EncounterDBN/PairwiseCorrAEMDBNImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/SimplePilotResponseImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/StochasticLinearPRImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/PilotResponse/LLDetPRImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel/SimpleADMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/DynamicModel/LLADMImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel")
push!(LOAD_PATH, "$PACKAGE_PATH/WorldModel/AirSpaceImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor/SimpleTCASSensorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/Sensor/ACASXSensorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CASCoordination")
push!(LOAD_PATH, "$PACKAGE_PATH/CASCoordination/GenericCoordImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/SimpleTCASImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/CoordSimpleTCASImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/ACASXCommonImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/ACASX_CCAS_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/CollisionAvoidanceSystem/ACASX_ADD_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator")
push!(LOAD_PATH, "$PACKAGE_PATH/Simulator/TCASSimulatorImpl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/SimpleTCAS_EvU_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/SimpleTCAS_EvE_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/ACASX_GM_Impl")
push!(LOAD_PATH, "$PACKAGE_PATH/GenerativeModel/common")

using CommonInterfaces
using Util
using Encounter
using EncounterDBN
using PilotResponse
using DynamicModel
using WorldModel
using Sensor
using CASCoordination
using CollisionAvoidanceSystem
using Simulator
using GenerativeModel

end


