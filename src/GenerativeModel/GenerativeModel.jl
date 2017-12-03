# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014


module GenerativeModel

export
    AbstractGenerativeModel,
    initialize,
    update,
    get,
    isterminal,

    SimpleTCAS_EvU_params,
    SimpleTCAS_EvU,

    SimpleTCAS_EvE_params,
    SimpleTCAS_EvE,

    ACASX_GM_params,
    ACASX_GM

import CommonInterfaces: initialize, update

using AbstractGenerativeModelImpl

include("common/common.jl")

using SimpleTCAS_EvU_Impl

using SimpleTCAS_EvE_Impl

using ACASX_GM_Impl

end


