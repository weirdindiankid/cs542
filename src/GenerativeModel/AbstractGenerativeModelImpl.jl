# Author: Ritchie Lee, ritchie.lee@sv.cmu.@schedule
# Date: 12/11/2014


module AbstractGenerativeModelImpl

export AbstractGenerativeModel

abstract AbstractGenerativeModel

end #module

module AbstractGenerativeModelInterfaces

export get,
       isterminal

function get() end

function isterminal() end

end #module


