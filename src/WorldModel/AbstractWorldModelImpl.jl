# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AbstractWorldModelImpl

export AbstractWorldModel

abstract AbstractWorldModel

end


module AbstractWorldModelInterfaces

export updateObjectState,
       getAllObjectStates,
       getAll,
       updateAll

function updateObjectState()
end

function getAllObjectStates()
end

function getAll()
end

function updateAll()
end

end


