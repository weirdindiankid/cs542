# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 08/19/2014
#
# Originally written in MATLAB by Mykel Kochenderfer, mykel@stanford.edu
# Converted by Youngjun Kim, youngjun@stanford.edu

#   LOAD_SCRIPTS(FILENAME) returns a structure holding the encounter
#   scripts stored in the specified file.
#
#   SCRIPTS FILE:
#   The scripts file contains a set of scripted encounters. Each
#   encounter is defined by a set of encounter scripts associated with a
#   fixed number of aircraft. The file is organized as follows:
#
#   [Header]
#   uint32 (number of encounters)
#   uint32 (number of aircraft)
#       [Encounter 1]
#           [Initial]
#               [Aircraft 1]
#               double (initial airspeed in ft/s)
#               double (initial north position in ft)
#               double (initial east position in ft)
#               double (initial altitude in ft)
#               double (initial heading angle in radians)
#               double (initial flight path angle in radians)
#               double (initial roll angle in radians)
#               double (initial airspeed acceleration in ft/s^2)
#               ...
#               [Aircraft n]
#               double (initial airspeed in ft/s)
#               double (initial north position in ft)
#               double (initial east position in ft)
#               double (initial altitude in ft)
#               double (initial heading angle in radians)
#               double (initial flight path angle in radians)
#               double (initial roll angle in radians)
#               double (initial airspeed acceleration in ft/s^2)
#           [Updates]
#               [Aircraft 1]
#               uint8 (number of updates)
#                   [Update 1]
#                   double (time in s)
#                   double (commanded vertical rate in ft/s)
#                   double (commanded turn rate in rad/s)
#                   double (commanded airspeed acceleration in ft/s^2)
#                   ...
#                   [Update m]
#                   double (time in s)
#                   double (commanded vertical rate in ft/s)
#                   double (commanded turn rate in rad/s)
#                   double (commanded airspeed acceleration in ft/s^2)
#               ...
#               [Aircraft n]
#                   ...
#       ...
#       [Encounter k]
#           ...
#
#   SCRIPTS STRUCTURE:
#   The waypoints structure is an m x n structure matrix, where m is the
#   number of aircraft and n is the number of encounters. This structure
#   matrix has two fields: initial and update. The initial field is an 8
#   element array specifying the airspeed, north position, east position,
#   altitude, heading, flight path angle, roll angle, and airspeed
#   acceleration. The update field is a 4 x n matrix, where n is the number
#   of updates. The rows correspond to the time, vertical rate, turn rate
#   and airspeed acceleration.
#   The script also takes an additional parameter through varargin:  the
#   number of encounters to load, 'limit'.  load_encounters will load the
#   minimum of this value and the number of encounters specified in the
#   encounter file.  If specified, this impacts 'n' in the above paragraph.


function load_encounters(filename, initial_dim, update_dim, num_update_type; limit = 0)

    fio = open(filename, "r")

    num_encounters = read(fio, UInt32)
    num_ac = read(fio, UInt32)

    if limit != 0
        num_encounters = min(num_encounters, limit)
    end

    encounters = Array(Any, num_ac, num_encounters)

    for i = 1:num_encounters
        for j = 1:num_ac
            encounters[j, i] = Dict()
        end

        for j = 1:num_ac
            encounters[j, i]["initial"] = read(fio, Float64, initial_dim)
        end

        for j = 1:num_ac
            num_update = read(fio, num_update_type)

            encounters[j, i]["update"] = reshape(read(fio, Float64, update_dim * num_update), Int64(update_dim), Int64(num_update))
        end
    end

    close(fio)

    return encounters, num_ac, num_encounters
end

function load_scripts(filename; limit = 0)

    scripts, num_ac, num_enc = load_encounters(filename, 8, 4, UInt8; limit = limit)
end


if false
    scripts, num_ac, num_enc = load_scripts("encounters_0.dat", limit = 2)

    println(num_ac)
    println(num_enc)
    dump(scripts)
end


