# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/05/2014


module WorldModel

export
    AbstractWorldModel,

    initialize,
    update,

    updateObjectState,
    getAllObjectStates,
    getAll,
    updateAll,

    AirSpace,
    ASWMState

import CommonInterfaces: initialize, update

using AbstractWorldModelImpl

using AirSpaceImpl

end


