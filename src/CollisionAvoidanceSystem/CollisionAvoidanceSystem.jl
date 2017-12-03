# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module CollisionAvoidanceSystem

export
    AbstractCollisionAvoidanceSystem,

    initialize,
    update,

    testThreat,
    selectRA,

    SimpleTCAS,
    SimpleTCASInput,
    SimpleTCASResolutionAdvisory,

    CoordSimpleTCAS,

    ACASX_CCAS,
    ACASX_ADD,
    ACASXInput,
    ACASXOutput

import CommonInterfaces: initialize, update

using AbstractCollisionAvoidanceSystemImpl

using SimpleTCASImpl

using CoordSimpleTCASImpl

using ACASXCommonImpl

pkgs = keys(Pkg.installed())
if "CCAS" in pkgs && "CASInterface" in pkgs
    @eval using ACASX_CCAS_Impl
end

using ACASX_ADD_Impl

end


