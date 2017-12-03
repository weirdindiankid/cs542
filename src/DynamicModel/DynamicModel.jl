# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module DynamicModel

export
    AbstractDynamicModel,
    addObserver,

    initialize,
    update,

    initializeDynamicModel,
    simulateDynamicModel,

    SimpleADM,
    SimpleADMInitialState,
    SimpleADMOutputState,
    SimpleADMCommand,

    LLADM,
    LLADMState,
    LLADMOutputState,
    LLADMCommand

import CommonInterfaces: initialize, update

using AbstractDynamicModelImpl

using SimpleADMImpl

using LLADMImpl

end


