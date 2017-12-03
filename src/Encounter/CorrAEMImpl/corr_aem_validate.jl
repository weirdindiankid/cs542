# Author: Youngjun Kim, youngjun@stanford.edu
# Data: 05/02/2014


using Util


function validate(aem)

    validate_initial(aem)
    #validate_transition(aem)
end


function validate_initial(aem)

    params = aem.parameters

    marginal_dist = Array{Any}(params.n_initial)
    marginal_prob = Array{Any}(params.n_initial)

    for i = 1:params.n_initial
        marginal_dist[i] = vec(sum(params.N_initial[i], 2))
        marginal_prob[i] = marginal_dist[i] / sum(marginal_dist[i])
    end

    f = open(aem.initial_sample_filename, "r")

    readline(f)

    n_nmac = 0
    n_lines = 0

    sample_dist = Array{Any}(params.n_initial)
    sample_prob = Array{Any}(params.n_initial)

    for i = 1:params.n_initial
        sample_dist[i] = zeros(Int, params.r_initial[i])
    end

    for line = eachline(f)
        values = float(split(chomp(line)))

        # 1       id
        # 2       A = [1:4]
        # 3       L = [1:5]
        # 4       chi = [90, 270]   : deg
        # 5       beta = [0:30:360] : deg
        # 6       C1 = [1, 2]
        # 7       C2 = [1, 2]
        # 8,9     v1, v2 = [50, 100, 200, 300, 400, 500, 600]   : kt
        # 10, 11  v1d, v2d = [-5, -2, -0.25, 0.25, 2, 5]        : kt/s
        # 12, 13  h1d, h2d = [-5000, -3000, -2000, -1000, -400, 400, 1000, 2000, 3000, 5000]    : ft/min
        # 14, 15  psi1d, psi2d = [-8, -6, -3.5, -1, -0.25, 0.25, 1, 3.5, 6, 8]  : deg/s
        # 16      hmd = [0, 0.0822896, 0.5, 1, 3]   : nm
        # 17      vmd = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 6000]  : ft

        for i = [2, 3, 4, 6, 7]
            if !(Int(values[i]) in 1:params.r_initial[i-1])
                println("sample: ", chomp(line))
                println("i: ", i, ", value: ", Int(values[i]), ", boundary: ", params.r_initial[i-1])
                error("The value violates the boundary.")
            end

            sample_dist[i - 1][Int(values[i])] += 1
        end

        for i = [5, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
            if !isempty(params.boundaries[i-1]) && (values[i] < params.boundaries[i-1][1] || values[i] > params.boundaries[i-1][end])
                println("sample: ", chomp(line))
                println("i: ", i, ", value: ", values[i], ", boundary: ", params.boundaries[i-1][end])
                error("The value violates the boundary.")
            end

            j = findfirst(x -> (x > values[i]), params.boundaries[i - 1])

            if j == 0 && values[i] == params.boundaries[i - 1][end]
                sample_dist[i - 1][params.r_initial[i - 1]] += 1
            else
                sample_dist[i - 1][j - 1] += 1
            end
        end

        hmd_ft = 6076.12 * values[16]
        vmd = values[17]

        if hmd_ft <= 500 && vmd <= 100
            n_nmac += 1
        end

        n_lines += 1
    end

    close(f)

    for i = 1:params.n_initial
        sample_prob[i] = sample_dist[i] / sum(sample_dist[i])
    end

#    for i = 1:params.n_initial
#        if !isempty(find(x -> (x > 5), abs((sample_prob[i] - marginal_prob[i]) ./ marginal_prob[i] * 100)))
#            println(i)
#            println()
#            println(marginal_prob[i])
#            println(sample_prob[i])
#        end
#    end

    for i = 1:params.n_initial
        println("variable #: ", i)

        for j = 1:params.r_initial[i]
            print(marginal_dist[i][j], " ")
            print(outputGFormatString(marginal_prob[i][j]), " ")
            print(sample_dist[i][j], " ")
            print(outputGFormatString(sample_prob[i][j]), " ")
            @printf("%.2f%%\n", abs((sample_prob[i][j] - marginal_prob[i][j]) / marginal_prob[i][j] * 100))
        end

        println()
    end

    println("P(NMAC | enc, no TCAS) = ", n_nmac / n_lines, " ($n_nmac / $n_lines)")
    println()
end


function validate_transition(aem)

    # should validate conditional probabilities rather than marginal probability
    # check marginal probability here for convenience

    params = aem.parameters

    marginal_dist = Array{Any}(params.n_transition - params.n_initial)
    marginal_prob = Array{Any}(params.n_transition - params.n_initial)

    for i = 1:(params.n_transition - params.n_initial)
        marginal_dist[i] = vec(sum(params.N_transition[params.n_initial + i], 2))
        marginal_prob[i] = marginal_dist[i] / sum(marginal_dist[i])
    end


    f = open(aem.transition_sample_filename, "r")

    readline(f)

    sample_dist = Array{Any}(params.n_transition - params.n_initial)
    sample_prob = Array{Any}(params.n_transition - params.n_initial)

    for i = 1:(params.n_transition - params.n_initial)
        sample_dist[i] = zeros(Int, params.r_transition[params.n_initial + i])
    end

    bin = Array(Int, params.n_transition - params.n_initial)
    value = Array(AbstractFloat, params.n_transition - params.n_initial)

    n_sample = 0
    n_resample = zeros(Int, params.n_transition - params.n_initial)

    for line = eachline(f)
        values = float(split(chomp(line)))

        # 1     id
        # 2     time step
        # 3, 4  h1d, h2d        : ft/min
        # 5, 6  psi1d, psi2d    : deg/s

        # resampling_rate = 0 0 0 0 0 0 0 0 0 0 0.0487462 0.0505306 0.0794427 0.0827686 0 0

        id = Int(values[1])
        ts = Int(values[2])

        if ts != 0
            n_sample += 1
        end

        for i = [3 4 5 6]
            if !isempty(params.boundaries[i + 8]) && (values[i] < params.boundaries[i + 8][1] || values[i] > params.boundaries[i + 8][end])
                println("sample: ", chomp(line))
                println("i: ", i, ", value: ", values[i], ", boundary: ", params.boundaries[i + 8][end])
                error("The value violates the boundary.")
            end

            j = findfirst(x -> (x > values[i]), params.boundaries[i + 8])

            if ts != 0
                if j == 0 && values[i] == params.boundaries[i + 8][end]
                    sample_dist[i - 2][params.r_transition[params.n_initial + i - 2]] += 1
                else
                    sample_dist[i - 2][j - 1] += 1
                end

                if (j - 1) == bin[i - 2] && values[i] != value[i - 2]
                    n_resample[i - 2] += 1
                end
            end

            bin[i - 2] = j - 1
            value[i - 2] = values[i]
        end
    end

    close(f)

    for i = 1:(params.n_transition - params.n_initial)
        sample_prob[i] = sample_dist[i] / sum(sample_dist[i])
    end

    for i = 1:(params.n_transition - params.n_initial)
        println("variable #: ", params.n_initial + i)

        for j = 1:params.r_transition[params.n_initial + i]
            print(marginal_dist[i][j], " ")
            print(outputGFormatString(marginal_prob[i][j]), " ")
            print(sample_dist[i][j], " ")
            print(outputGFormatString(sample_prob[i][j]), " ")
            @printf("%.2f%%\n", abs((sample_prob[i][j] - marginal_prob[i][j]) / marginal_prob[i][j] * 100))
        end
        println()

        print(outputGFormatString(params.resample_rates[i + 10]), " ", outputGFormatString(n_resample[i] / n_sample), " (", n_resample[i], " / ", n_sample, ") ")
        @printf("%.2f%%\n", abs(((n_resample[i] / n_sample) - params.resample_rates[i + 10]) / params.resample_rates[i + 10] * 100))
        println()
    end
end


