# Author: Ritchie Lee, ritchie.lee@sv.cmu.reduce
# Date: 12/15/2014


module EncounterDBN

export
    AbstractEncounterDBN,
    AddObserver,

    getInitialState,
    initialize,
    update,
    get,

    CorrAEMDBN,
    StarDBN,
    SideOnDBN,
    PairwiseCorrAEMDBN

import CommonInterfaces: initialize, update

using AbstractEncounterDBNImpl
import AbstractEncounterDBNInterfaces: getInitialState, get

using CorrAEMDBNImpl
using StarDBNImpl
#using PairwiseCorrAEMDBNImpl
using SideOnDBNImpl

end


