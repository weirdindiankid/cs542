# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AbstractCollisionAvoidanceSystemImpl

export AbstractCollisionAvoidanceSystem

abstract AbstractCollisionAvoidanceSystem

end


module AbstractCollisionAvoidanceSystemInterfaces

export testThreat,
       selectRA

function testThreat()
end

function selectRA()
end

end


