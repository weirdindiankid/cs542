# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AbstractDynamicModelImpl

export AbstractDynamicModel

abstract AbstractDynamicModel

end


module AbstractDynamicModelInterfaces

export initializeDynamicModel,
       simulateDynamicModel

function initializeDynamicModel()
end

function simulateDynamicModel()
end

end


