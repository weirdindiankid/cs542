# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/16/2014


module Sensor

export
    AbstractSensor,

    initialize,
    update,

    updateSensor,

    SimpleTCASSensor,
    SimpleTCASSRState,
    SimpleTCASSensorInput,
    SimpleTCASSensorOutput,

    ACASXSensor,
    ACASXSensorState,
    ACASXSensorInput,
    ACASXSensorOutput

import CommonInterfaces: initialize, update

using AbstractSensorImpl

using SimpleTCASSensorImpl

using ACASXSensorImpl

end


