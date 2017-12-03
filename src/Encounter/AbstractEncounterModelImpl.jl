# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AbstractEncounterModelImpl

export AbstractEncounterModel

abstract AbstractEncounterModel

end


module AbstractEncounterModelInterfaces

export
       # common
       generateEncounter,
       getInitialState,
       getNextCommand,
       getTrajectory,

       # CorrAEM
       generateEncountersToFile,
       setInitialDistributions,
       validate,
       getInitialSample


# common

function generateEncounter()
end

function getInitialState()
end

function getNextCommand()
end

function getTrajectory()
end


# CorrAEM

function generateEncountersToFile()
end

function setInitialDistributions()
end

function validate()
end

function getInitialSample()
end

end


