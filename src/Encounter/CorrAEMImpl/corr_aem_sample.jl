# Converted the code for airspace encounter model from MATLAB to Julia

# Originally written in MATLAB by Mykel Kochenderfer, mykel@stanford.edu
# Converted by Youngjun Kim, youngjun@stanford.edu
# Date: 4/23/2014

function em_sample(aem, num_transition_samples; initial_dist = nothing)

# EM_SAMPLE Outputs samples from an encounter model to files.
#   Outputs samples into two specified files from an encounter model
#   described in a file.
#
#   EM_SAMPLES takes as input the following arguments:
#   PARAMETERS_FILENAME: a string specifying the name of the parameters
#   file
#   INITIAL_OUTPUT_FILENAME: a string specifying the the name of the file
#   to store transition samples
#   NUM_INITIAL_SAMPLES: the number of samples to generate
#   NUM_TRANSITION_SAMPLES: the number of steps to sample from the
#   transition network

    p = aem.parameters

    # read parameters
    #p = Parameters()
    #em_read(p, parameters_filename)

    # create priors
    dirichlet_initial = bn_dirichlet_prior(p.N_initial)
    dirichlet_transition = bn_dirichlet_prior(p.N_transition)

    num_transition_samples  = Int(num_transition_samples)

    output = Array(AbstractFloat, (num_transition_samples, p.n_initial + 2))

    if initial_dist == nothing
        x = create_sample(p, dirichlet_initial, dirichlet_transition, num_transition_samples)

        for j = 1:p.n_initial
            if !isempty(p.boundaries[j])
                if x[j, 1] < p.boundaries[j][1] || x[j, 1] > p.boundaries[j][end]
                    print("sample: ", x[:, 1]')
                    println("i: ", j, ", value: ", x[j, 1], ", boundary: ", p.boundaries[j][end])
                    error("The value violates the boundary.")
                end
            elseif !(Int(x[j, 1]) in collect(1:p.r_initial[j]))
                print("sample: ", x[:, 1]')
                println("i: ", j, ", value: ", x[j, 1], ", boundary: ", p.r_initial[j])
                error("The value violates the boundary.")
            end
        end
    else
        x, IS = create_sample(p, dirichlet_initial, dirichlet_transition, num_transition_samples, initial_dist = initial_dist)
    end

    output[1:num_transition_samples, 1] = ones(num_transition_samples)
    output[1:num_transition_samples, 2] = collect(0:num_transition_samples - 1)
    output[1:num_transition_samples, 3:end] = x'

    if initial_dist == nothing
        return output
    else
        return output, IS
    end
end


function em_sample_n(aem, num_initial_samples, num_transition_samples)

# EM_SAMPLE Outputs samples from an encounter model to files.
#   Outputs samples into two specified files from an encounter model
#   described in a file.
#
#   EM_SAMPLES takes as input the following arguments:
#   PARAMETERS_FILENAME: a string specifying the name of the parameters
#   file
#   INITIAL_OUTPUT_FILENAME: a string specifying the the name of the file
#   to store transition samples
#   NUM_INITIAL_SAMPLES: the number of samples to generate
#   NUM_TRANSITION_SAMPLES: the number of steps to sample from the
#   transition network

    p = aem.parameters

    # read parameters
    #p = Parameters()
    #em_read(p, parameters_filename)

    # create priors
    dirichlet_initial = bn_dirichlet_prior(p.N_initial)
    dirichlet_transition = bn_dirichlet_prior(p.N_transition)

    num_initial_samples = Int(num_initial_samples)
    num_transition_samples  = Int(num_transition_samples)

    output_array = Array(AbstractFloat, (num_initial_samples * num_transition_samples, p.n_initial + 2))

    for i = 1:num_initial_samples
        x = create_sample(p, dirichlet_initial, dirichlet_transition, num_transition_samples)

        for j = 1:p.n_initial
            if !isempty(p.boundaries[j])
                if x[j, 1] < p.boundaries[j][1] || x[j, 1] > p.boundaries[j][end]
                    print("sample: ", i, " ", x[:, 1]')
                    println("i: ", j, ", value: ", x[j, 1], ", boundary: ", p.boundaries[j][end])
                    error("The value violates the boundary.")
                end
            elseif !(Int(x[j, 1]) in [1:p.r_initial[j]])
                print("sample: ", i, " ", x[:, 1]')
                println("i: ", j, ", value: ", x[j, 1], ", boundary: ", p.r_initial[j])
                error("The value violates the boundary.")
            end
        end

        output_array[((i - 1) * num_transition_samples + 1):(i * num_transition_samples), 1] = i * ones(num_transition_samples)
        output_array[((i - 1) * num_transition_samples + 1):(i * num_transition_samples), 2] = [0:num_transition_samples - 1]
        output_array[((i - 1) * num_transition_samples + 1):(i * num_transition_samples), 3:end] = x'
    end

    return output_array
end


function create_sample(p, dirichlet_initial, dirichlet_transition, sample_time; initial_dist = nothing)

    if initial_dist == nothing
        initial, events = dbn_sample(p.G_initial, p.G_transition, p.temporal_map, p.r_transition, p.N_initial, p.N_transition, dirichlet_initial, dirichlet_transition, p.boundaries, sample_time)
    else
        initial, events, IS = dbn_sample(p.G_initial, p.G_transition, p.temporal_map, p.r_transition, p.N_initial, p.N_transition, dirichlet_initial, dirichlet_transition, p.boundaries, sample_time, initial_dist = initial_dist)
    end

    if isempty(events)
        events = [sample_time 0 0]
    else
        events = [events; (sample_time - sum(events[:,1])) 0 0]
    end

    # Within-bin resampling
    events = resample_events(initial, events, p.resample_rates)

    # Dediscretize
    for i = 1:length(initial)
        if !isempty(p.boundaries[i])
            if initial_dist == nothing
                initial[i] = dediscretize(initial[i], p.boundaries[i], p.zero_bins[i])[1]
            else
                if IS[i, 1] == 0
                    initial[i] = dediscretize(initial[i], p.boundaries[i], p.zero_bins[i])[1]
                else
                    initial[i] = IS[i, 2]
                end
            end
        end
    end

    if !isempty(events)
        for i = 1:(size(events, 1) - 1)
            e2 = convert(Int, events[i, 2])
            events[i, 3] = dediscretize(events[i, 3], p.boundaries[e2], p.zero_bins[e2])[1]
        end
    end

    if initial_dist == nothing
        return events2samples(initial, events)
    else
        return events2samples(initial, events), IS
    end
end


function events2samples(initial, events)

    n = length(initial)
    D = zeros(n, Int(sum(events[:, 1])))

    x = vec(copy(initial))
    t = 0

    for i = 1:size(events, 1)
        event = events[i, :]

        delta_t = Int(event[1])

        if event[2] == 0
            if delta_t > 0
                t = t + 1
                D[:, t:(t + delta_t - 1)] = repmat(x, 1, delta_t)
            end
        else
            if delta_t > 0
                D[:, (t + 1):(t + delta_t)] = repmat(x, 1, delta_t)
                t = t + delta_t
            end

            e2 = convert(Int, event[2])
            x[e2] = event[3]
        end
    end

    return D
end


function dediscretize(d, parameters, zero_bins)

    if isempty(parameters)
        return d
    end

    x = zeros(length(d))
    n = length(d)

    for i = 1:n
        # generate uniform in interval [a, b]
        if d[i] == zero_bins
            x[i] = 0
        else
            dd = convert(Int, d[i])
            a = parameters[dd]
            b = parameters[dd + 1]
            x[i] = a + (b - a) * rand()
        end
    end

    return x
end


function resample_events(initial, events, rates)

    n = size(events, 1)

    newevents = Float64[]

    x = vec(copy(initial))

    for i = 1:n
        holdtime = events[i, 1]

        if holdtime == 0
            append!(newevents, vec(events[i, :]))
        else
            delta_t = 0

            for j = 1:holdtime
                changes = find(rand(size(rates)) .< rates)
                delta_t = delta_t + 1

                if !isempty(changes)
                    T = [[delta_t; zeros(length(changes) - 1)] changes x[changes]]
                    for k = 1:size(T, 1)
                        append!(newevents, vec(T[k, :]))
                    end

                    delta_t = 0
                end
            end

            append!(newevents, [delta_t, events[i,2], events[i,3]])
        end

        if events[i, 2] > 0
            e2 = convert(Int, events[i,2])
            x[e2] = events[i, 3]
        end
    end

    newevents = reshape(newevents, (3, Int(length(newevents)/3)))'

    return newevents
end


function dbn_sample(G_initial, G_transition, temporal_map, r, N_initial, N_transition, dirichlet_initial, dirichlet_transition, boundaries, t_max; initial_dist = nothing)

# DBN_SAMPLE Samples from a dynamic Bayesian network.
# Returns a sample of the intitial variables and a series of events from
# the dynamic Bayesian network with specified graphical structure,
# sufficient statistics, and prior.
#
#   DBN_SAMPLE takes as input the following
#   G_initial - initial distribution graph structure
#   G_transition - continuous transition model graph structure
#   r - a column vector specifying the number of bins for each variable
#   N_initial - sufficient statistics for initial distribution
#   N_transition - sufficient statistics for transition model
#   dirichlet_initial - Dirichlet prior
#   dirichlet_transition - Dirichlet prior
#   t_max - the maximum amount of time to run the simulation
#
#   DBN_SAMPLE returns the following
#   initial
#   events - matrix of (t, variable_index, new_value) (NOTE: t is the time
#   since the last event)

    if initial_dist == nothing
        initial = bn_sample(G_initial, r, N_initial, dirichlet_initial, boundaries, 1)
    else
        initial, IS = bn_sample(G_initial, r, N_initial, dirichlet_initial, boundaries, 1, initial_dist = initial_dist)
    end

    events = Float64[]
    n_initial = size(G_initial, 1)

    dynamic_variables = temporal_map[:, 2]

    order = bn_sort(G_transition)

    x = [initial zeros(1, length(dynamic_variables))]

    delta_t = 0

    for t = 2:t_max
        delta_t = delta_t + 1
        x_old = copy(x)

        for i = order
            if in(i, dynamic_variables)
                # only dynamic variables change
                parents = G_transition[:, i]

                j = 1
                if !isempty(find(parents))
                    j = asub2ind(r[parents], x[parents'])
                end

                x[i] = select_random(N_transition[i][:, j] + dirichlet_transition[i][:, j])
            end
        end

        # map back
        x[temporal_map[:, 1]] = x[temporal_map[:, 2]]

        if !isequal(x[1:n_initial], x_old[1:n_initial])
            # change (i.e. new event)
            for i = 1:n_initial
                if x[i] != x_old[i]
                    append!(events, vec([delta_t i x[i]]))
                    delta_t = 0
                end
            end
        end
    end

    events = reshape(events, (3, Int(length(events)/3)))'

    if initial_dist == nothing
        return initial, events
    else
        return initial, events, IS
    end
end

function bn_sample(G, r, N, alpha, boundaries, num_samples; initial_dist = nothing)

# BN_SAMPLE Produces a sample from a Bayesian network.
#   Returns a matrix whose rows consist of n-dimensional samples from the
#   specified Bayesian network.
#
#   S = BN_SAMPLE(G, R, N, ALPHA, NUM_SAMPLES) returns a matrix S whose
#   rows consist of n-dimensional samples from a Bayesian network with
#   graphical structure G, sufficient statistics N, and prior ALPHA. The
#   number of samples is specified by NUM_SAMPLES. The array R specifies
#   the number of bins associated with each variable.

    order = bn_sort(G)

    n = length(N)
    S = zeros((num_samples, n))

    if initial_dist != nothing
        IS = zeros(n, 4)
    end

    for sample_index = 1:num_samples
        # generate each sample
        for i = order
            if initial_dist == nothing
                parents = G[:, i]

                j = 1
                if !isempty(find(parents))
                    j = asub2ind(r[[parents; falses(length(r) - size(G, 1))]], S[sample_index, parents])
                end

                S[sample_index, i] = select_random(N[i][:, j] + alpha[i][:, j])
            else
                f = initial_dist[i]

                if f == nothing
                    parents = G[:, i]

                    j = 1
                    if !isempty(find(parents))
                        j = asub2ind(r[[parents; falses(length(r) - size(G, 1))]], S[sample_index, parents])
                    end

                    S[sample_index, i] = select_random(N[i][:, j] + alpha[i][:, j])
                else
                    value, prob_new = f()

                    if !isempty(boundaries[i])
                        index = findfirst(x -> (x > value), boundaries[i]) - 1

                        if index == -1
                            index = r[i]
                        end
                    else
                        index = value
                    end


                    parents = G[:, i]

                    j = 1
                    if !isempty(find(parents))
                        j = asub2ind(r[[parents, falses(length(r) - size(G, 1))]], S[sample_index, parents])
                    end

                    A = N[i][:, j] + alpha[i][:, j]

                    if !isempty(boundaries[i])
                        prob = 1 / (boundaries[i][index + 1] - boundaries[i][index]) * A[index] / sum(A)
                    else
                        prob = A[index] / sum(A)
                    end


                    IS[i, :] = [index, value, prob_new, prob]

                    S[sample_index, i] = index
                end
            end
        end
    end

    if initial_dist == nothing
        return S
    else
        return S, IS
    end
end


function select_random(weights)
# SELECT_RANDOM Randomly selects an index according to specified weights.
#   Returns a randomly selected index according to the distribution
#   specified by a vector of weights.
#
#   INDEX = SELECT_RANDOM(WEIGHTS) returns a scalar index INDEX selected
#   randomly according to the specified weights WEIGHTS represented as an
#   array.

    s = cumsum(weights)
    r = s[end] * rand()
    index = findfirst(x -> (x >= r), s)

    return index
end

#can't replace this with built-in sub2ind...
function asub2ind(siz, x)
# ASUB2IND Linear index from multiple subscripts.
#   Returns a linder index from multiple subscripts assuming a matrix of a
#   specified size.
#
#   NDX = ASUB2IND(SIZ,X) returns the linear index NDX of the element in a
#   matrix of dimension SIZ associated with subscripts specified in X.

    k = [1; cumprod(siz[1:end-1])]
    ndx = k' * (x - 1) + 1
    lindex = convert(Int, ndx[1])

    #@assert lindex == sub2ind(siz, map(Int64, x)...) #different answer than sub2ind

    lindex 
end

function bn_sort(G)
# BN_SORT Produces a topological sort of a Bayesian network.
#   Returns an array specifying the indices of variables in topological
#   order according to a graphical structure.
#
#   ORDER = BN_SORT(G) topologically sorts the variables in the specified
#   graph G and returns an array ORDER that contains the indices of the
#   variables in order. The matrix G is a square adjacency matrix.
#
# This code is based on the topological sort routine by Kevin Murphy. The
# original source code is available from:
# http://bnt.sourceforge.net/

# INPUT:
# G - a square adjacency matrix
#
# OUTPUT:
# order - an array of indices indicating order
#
# This code is based on the topological sort routine by Kevin Murphy. The
# original source code is available from:
# http://bnt.sourceforge.net/

    n = size(G)[1]
    indeg = zeros(Int, (1, n))
    zero_indeg = Int[]    # a stack of nodes with no parents

    for i = 1:n
        indeg[i] = sum(G[:, i])

        if indeg[i] == 0
            push!(zero_indeg, i)
        end
    end

    t = 1
    order = zeros(Int, (1, n))

    while !isempty(zero_indeg)
        v = pop!(zero_indeg)

        order[t] = v
        t = t + 1

        cs = find(G[v, :])

        for j = 1:length(cs)
            c = cs[j]
            indeg[c] = indeg[c] - 1

            if indeg[c] == 0
                push!(zero_indeg, c)
            end
        end
    end

    return order
end


function bn_dirichlet_prior(N)

    n = length(N)

    alpha = Array{Any}(n, 1)

    for i = 1:n
        r, q = size(N[i])
        alpha[i] = ones(r, q)
    end

    return alpha
end


