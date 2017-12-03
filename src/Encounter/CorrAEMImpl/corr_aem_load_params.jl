# Converted from the MATLAB code for airspace encounter model

# Originally written in MATLAB by Mykel Kochenderfer, mykel@stanford.edu
# Converted by Youngjun Kim, youngjun@stanford.edu
# Date: 4/23/2014


function em_read(p, filename)

# EM_READ  Reads an encounter model parameters file.
# Reads an encounter model parameters file and returns a structure
# containing the parsed data.
#
#   EM_READ(FILENAME) reads the parameters contained in the specified file
#   and returns the parameters in a structure. Included in this structure
#   are the following fields:
#        labels_initial
#             n_initial
#             G_initial
#             r_initial
#             N_initial
#     labels_transition
#          n_transition
#          G_transition
#          r_transition
#          N_transition
#            boundaries
#        resample_rates
#          temporal_map
#             zero_bins

    f = open(filename, "r")

    validate_label(f, "# labels_initial")
    p.labels_initial = scanline(f, "string", ",")
    p.n_initial = length(p.labels_initial)

    validate_label(f, "# G_initial")
    p.G_initial = logical(scanmatrix(f, p.n_initial))

    validate_label(f, "# r_initial")
    p.r_initial = round(Int64, scanmatrix(f))

    validate_label(f, "# N_initial")
    dims_initial = getdims(p.G_initial, p.r_initial, 1:p.n_initial)
    p.N_initial = array2cells(scanmatrix(f), dims_initial)

    validate_label(f, "# labels_transition")
    p.labels_transition = scanline(f, "string", ",")
    p.n_transition = length(p.labels_transition)

    validate_label(f, "# G_transition")
    p.G_transition = logical(scanmatrix(f, p.n_transition))

    validate_label(f, "# r_transition")
    p.r_transition = round(Int64, scanmatrix(f))

    validate_label(f, "# N_transition")
    dims_transition = getdims(p.G_transition, p.r_transition, (p.n_initial + 1):p.n_transition)
    p.N_transition = array2cells(scanmatrix(f), dims_transition)

    validate_label(f, "# boundaries")
    p.boundaries = Array{Any}(1, p.n_initial)
    for i = 1:p.n_initial
        p.boundaries[i] = scanmatrix(f)
    end

    validate_label(f, "# resample_rates")
    p.resample_rates = scanmatrix(f)

    close(f)

    p.temporal_map = extract_temporal_map(p.labels_transition)
    p.zero_bins = extract_zero_bins(p.boundaries)
end


function extract_zero_bins(boundaries)

    zero_bins = Array{Any}(1, length(boundaries))

    for i = 1:length(boundaries)
        b = boundaries[i]
        z = []

        if length(b) > 2
            for j = 2:length(b)
                if b[j - 1] < 0 && b[j] > 0
                    z = j - 1
                end
            end
        end

        zero_bins[i] = z
    end

    return zero_bins
end


function extract_temporal_map(labels_transition)

    tt = []
    tt1 = []

    for i = 1:length(labels_transition)
        t = searchindex(labels_transition[i], "(t)")

        if t == 0
            t = searchindex(labels_transition[i], "(t+1)")

            if t != 0
                tt1 = [tt1; i]
            end
        else
            tt = [tt; i]
        end
    end

    return [tt tt1]
end


function array2cells(x, dims)

    c = Array{Any}(size(dims, 1))

    index = 1
    for i = 1:length(c)
        c[i] = zeros(dims[i,1], dims[i,2])
        c[i][:] = x[index:(index - 1 + length(c[i]))]
        index = index + length(c[i])
    end

    return c
end


function getdims(G, r, vars)

    n = size(G, 1)
    dims = zeros(Int, (n, 2))

    for i = vars
        q = prod(r[G[:,i]])
        dims[i,:] = [r[i] q]
    end

    return dims
end


function logical(x)

    return map((x) -> x == 0 ? false : true, x)
end


function scanmatrix(fid, num_rows = -1)

    if num_rows == -1
        x = scanline(fid, "float", " ")
    else
        x = zeros(num_rows, num_rows)
    end

    for i = 1:num_rows
        x[i,:] = scanline(fid, "float", " ")
    end

    return x
end


function scanline(fid, typename, delimiter)

    s = map((x) -> strip(x), split(rstrip(readline(fid), collect(" \r\n")), delimiter))

    if typename == "float"

        if s[1] == "*"
            return []
        else
            return float(s)
        end
    else
        return s
    end
end


function validate_label(fid, s)

    t = rstrip(readline(fid), collect(" \r\n"))

    if t != s
        error("Invalid parameters file")
    end
end


