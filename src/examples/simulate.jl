# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014

using SISLES
using ArgParse
using HDF5, JLD
using PyPlot


# Examples
#
# to get help
# $ julia simulate.jl -h
# $ julia simulate.jl run -h
# $ julia simulate.jl compare -h
# $ julia simulate.jl validate -h
# $ julia simulate.jl generate -h
# $ julia simulate.jl plot -h
#
# run/plot simulation. sample is generated randomly.
# $ julia simulate.jl run
# $ julia simulate.jl plot
#
# read 11th sample from sample files. run/plot simulation with TCAS
# $ julia simulate.jl run -t -r -n 11
# $ julia simulate.jl plot
#
# read 12th sample from sample files. run/plot simulation with/out TCAS
# $ julia simulate.jl compare -n 12
# $ julia simulate.jl plot -c
#
# read 23th sample from Lincoln Lab sample files. run/plot simulation
# $ julia simulate.jl validate -n 23
# $ julia simulate.jl plot -d
#
# generate 100 samples with 50 seconds transitions for each sample to file. "initial.txt" and "transition.txt" files are generated under current directory.
# $ julia simulate.jl generate -n 100
#
# generate 10 samples with 20 seconds transitions for each sample to file. "init.txt" and "tran.txt" files are generated under current directory.
# $ julia simulate.jl generate --init_file "init.txt" --ninit 10 --tran_file "tran.txt" --ntran 20 --param_file "../Encounter/CorrAEMImpl/params/cor.txt"


function initialize_simulation(; bValidate = false, bReadSampleFromFile = false, initial_sample_filename = "initial.txt", transition_sample_filename = "transition.txt")

    SISLES_PATH = Pkg.dir("SISLES", "src")
    number_of_aircraft = 2

    if bValidate
        aem = LLAEM(number_of_aircraft, "$SISLES_PATH/Encounter/LLAEMImpl/data/cor_enc.txt", 50, ["$SISLES_PATH/Encounter/LLAEMImpl/data/cor_ac1.txt", "$SISLES_PATH/Encounter/LLAEMImpl/data/cor_ac2.txt"], [50, 50])
    else
        if bReadSampleFromFile
            aem = CorrAEM("$SISLES_PATH/Encounter/CorrAEMImpl/params/cor.txt", initial_sample_filename, transition_sample_filename)
            #validate(aem)
        else
            aem = CorrAEM("$SISLES_PATH/Encounter/CorrAEMImpl/params/cor.txt")
        end
    end

    pr_1 = SimplePilotResponse()
    pr_2 = SimplePilotResponse()

    #adm_1 = SimpleADM()
    #adm_2 = SimpleADM()
    adm_1 = SimpleADM(number_of_substeps = 1)
    adm_2 = SimpleADM(number_of_substeps = 1)

    as = AirSpace(number_of_aircraft)

    sr_1 = SimpleTCASSensor(1)
    sr_2 = SimpleTCASSensor(2)

    cas_1 = SimpleTCAS()
    cas_2 = SimpleTCAS()

    sim = TCASSimulator()

    sim.parameters.em = aem
    sim.parameters.pr = [pr_1, pr_2]
    sim.parameters.dm = [adm_1, adm_2]
    sim.parameters.wm = as
    sim.parameters.sr = [sr_1, sr_2]
    sim.parameters.cas = [cas_1, cas_2]
    sim.parameters.number_of_aircraft = number_of_aircraft

    return sim
end


function run_simulation(; bTCAS = false, bReadSampleFromFile = false, initial_sample_filename = "initial.txt", transition_sample_filename = "transition.txt", sample_number = 1)

    sim = initialize_simulation(bReadSampleFromFile = bReadSampleFromFile, initial_sample_filename = initial_sample_filename, transition_sample_filename = transition_sample_filename)

    aem = sim.parameters.em

    AC1_trajectory_ = Vector{Float64}[]
    AC2_trajectory_ = Vector{Float64}[]

    adm_1, adm_2 = sim.parameters.dm

    addObserver(adm_1, x -> push!(AC1_trajectory_, x))
    addObserver(adm_2, x -> push!(AC2_trajectory_, x))


    if bReadSampleFromFile
        simulate(sim, bTCAS = bTCAS, sample_number = sample_number)
    else
        simulate(sim, bTCAS = bTCAS)
    end


    labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

    initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

    AC1_trajectory = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory[i, :] = AC1_trajectory_[i]
        AC2_trajectory[i, :] = AC2_trajectory_[i]
    end


    SimulationResult = Any[labels, initial, AC1_trajectory, AC2_trajectory]

    save("result.jld", "data", SimulationResult)
end


function run_simulation_for_comparison(; initial_sample_filename = "initial.txt", transition_sample_filename = "transition.txt", sample_number = 1)

    sim = initialize_simulation(bReadSampleFromFile = true, initial_sample_filename = initial_sample_filename, transition_sample_filename = transition_sample_filename)

    aem = sim.parameters.em

    AC1_trajectory_ = Vector{Float64}[]
    AC2_trajectory_ = Vector{Float64}[]

    adm_1, adm_2 = sim.parameters.dm

    addObserver(adm_1, x -> push!(AC1_trajectory_, x))
    addObserver(adm_2, x -> push!(AC2_trajectory_, x))


    simulate(sim, bTCAS = false, sample_number = sample_number)


    AC1_trajectory = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory[i, :] = AC1_trajectory_[i]
        AC2_trajectory[i, :] = AC2_trajectory_[i]
    end


    AC1_trajectory_ = Vector{Float64}[]
    AC2_trajectory_ = Vector{Float64}[]

    simulate(sim, bTCAS = true, sample_number = sample_number)


    AC1_trajectory_tcas = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory_tcas = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory_tcas[i, :] = AC1_trajectory_[i]
        AC2_trajectory_tcas[i, :] = AC2_trajectory_[i]
    end


    labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

    initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

    SimulationResult = Any[labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_tcas, AC2_trajectory_tcas]

    save("result.jld", "data", SimulationResult)
end


function run_simulation_for_validation(; sample_number = 1)

    sim = initialize_simulation(bValidate = true)

    aem = sim.parameters.em

    AC1_trajectory_ = Vector{Float64}[]
    AC2_trajectory_ = Vector{Float64}[]

    adm_1, adm_2 = sim.parameters.dm

    addObserver(adm_1, x -> push!(AC1_trajectory_, x))
    addObserver(adm_2, x -> push!(AC2_trajectory_, x))


    simulate(sim, sample_number = sample_number)


    AC1_trajectory = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory[i, :] = AC1_trajectory_[i]
        AC2_trajectory[i, :] = AC2_trajectory_[i]
    end

    AC1_trajectory_ll = getTrajectory(aem, 1)
    AC2_trajectory_ll = getTrajectory(aem, 2)


    labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

    initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

    SimulationResult = Any[labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_ll, AC2_trajectory_ll]

    save("result.jld", "data", SimulationResult)
end


function generate_samples_to_file(initial_sample_filename = "initial.txt", 
    number_of_initial_samples = 1, transition_sample_filename = "transition.txt", 
    number_of_transition_samples = 50, 
    parameter_file = Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/params/cor.txt"))

    aem = CorrAEM(parameter_file, initial_sample_filename, number_of_initial_samples, 
        transition_sample_filename, number_of_transition_samples)

    Encounter.generateEncountersToFile(aem)
end

function plot_result()

    SimulationResult = load("result.jld", "data")

    labels, initial, AC1_trajectory, AC2_trajectory = SimulationResult

    println(labels[1])
    println(initial')

    index_tca = find(x -> abs(x - 40) < 0.001, AC1_trajectory[:, 1])

    figure(1)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    axis("equal")
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2"], loc = 0)

    figure(2)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    axis("equal")
    l = initial[7] * 1.2
    axis([-l, l, -l, l])
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2"], loc = 0)

    figure(3)
    plot(AC1_trajectory[:, 1] , AC1_trajectory[:, 4], AC2_trajectory[:, 1], AC2_trajectory[:, 4])
    plot(AC1_trajectory[index_tca, 1], AC1_trajectory[index_tca, 4], "*", AC2_trajectory[index_tca, 1], AC2_trajectory[index_tca, 4], "*")
    grid(true)
    xlabel("time(sec)")
    ylabel("h(ft)")
    legend(["AC1", "AC2"], loc = 0)
end


function plot_result_for_comparison()

    SimulationResult = load("result.jld", "data")

    labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_tcas, AC2_trajectory_tcas = SimulationResult

    println(labels[1])
    println(initial')

    index_tca = find(x -> abs(x - 40) < 0.001, AC1_trajectory[:, 1])
    index_tca_tcas = find(x -> abs(x - 40) < 0.001, AC1_trajectory_tcas[:, 1])

    figure(1)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory_tcas[:, 3], AC1_trajectory_tcas[:, 2], "--", AC2_trajectory_tcas[:, 3], AC2_trajectory_tcas[:, 2], "--")
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    plot(AC1_trajectory_tcas[index_tca_tcas, 3], AC1_trajectory_tcas[index_tca_tcas, 2], "x", AC2_trajectory_tcas[index_tca_tcas, 3], AC2_trajectory_tcas[index_tca_tcas, 2], "x")
    axis("equal")
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2", "AC1_TCAS", "AC2_TCAS"], loc = 0)

    figure(2)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory_tcas[:, 3], AC1_trajectory_tcas[:, 2], "--", AC2_trajectory_tcas[:, 3], AC2_trajectory_tcas[:, 2], "--")
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    plot(AC1_trajectory_tcas[index_tca_tcas, 3], AC1_trajectory_tcas[index_tca_tcas, 2], "x", AC2_trajectory_tcas[index_tca_tcas, 3], AC2_trajectory_tcas[index_tca_tcas, 2], "x")
    axis("equal")
    l = initial[7] * 1.2
    axis([-l, l, -l, l])
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2", "AC1_TCAS", "AC2_TCAS"], loc = 0)

    figure(3)
    h1 = AC1_trajectory_tcas[1, 4] - AC1_trajectory[1, 4]
    h2 = AC2_trajectory_tcas[1, 4] - AC2_trajectory[1, 4]
    plot(AC1_trajectory[:, 1] , AC1_trajectory[:, 4] + h1, AC2_trajectory[:, 1], AC2_trajectory[:, 4] + h2)
    plot(AC1_trajectory_tcas[:, 1], AC1_trajectory_tcas[:, 4], "--", AC2_trajectory_tcas[:, 1], AC2_trajectory_tcas[:, 4], "--")
    plot(AC1_trajectory[index_tca, 1], AC1_trajectory[index_tca, 4] + h1, "*", AC2_trajectory[index_tca, 1], AC2_trajectory[index_tca, 4] + h2, "*")
    plot(AC1_trajectory_tcas[index_tca_tcas, 1], AC1_trajectory_tcas[index_tca_tcas, 4], "x", AC2_trajectory_tcas[index_tca_tcas, 1], AC2_trajectory_tcas[index_tca_tcas, 4], "x")
    grid(true)
    xlabel("time(sec)")
    ylabel("h(ft)")
    legend(["AC1", "AC2", "AC1_TCAS", "AC2_TCAS"], loc = 0)
end


function plot_result_for_validation()

    SimulationResult = load("result.jld", "data")

    labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_ll, AC2_trajectory_ll = SimulationResult

    println(labels[1])
    println(initial')

    index_tca = find(x -> abs(x - 40) < 0.001, AC1_trajectory[:, 1])
    index_tca_ll = find(x -> abs(x - 40) < 0.001, AC1_trajectory_ll[:, 1])

    figure(1)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory_ll[:, 3], AC1_trajectory_ll[:, 2], "--", AC2_trajectory_ll[:, 3], AC2_trajectory_ll[:, 2], "--")
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    plot(AC1_trajectory_ll[index_tca_ll, 3], AC1_trajectory_ll[index_tca_ll, 2], "x", AC2_trajectory_ll[index_tca_ll, 3], AC2_trajectory_ll[index_tca_ll, 2], "x")
    axis("equal")
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2", "AC1_LL", "AC2_LL"], loc = 0)

    figure(2)
    plot(AC1_trajectory[:, 3] , AC1_trajectory[:, 2], AC2_trajectory[:, 3], AC2_trajectory[:, 2])
    plot(AC1_trajectory_ll[:, 3], AC1_trajectory_ll[:, 2], "--", AC2_trajectory_ll[:, 3], AC2_trajectory_ll[:, 2], "--")
    plot(AC1_trajectory[index_tca, 3], AC1_trajectory[index_tca, 2], "*", AC2_trajectory[index_tca, 3], AC2_trajectory[index_tca, 2], "*")
    plot(AC1_trajectory_ll[index_tca_ll, 3], AC1_trajectory_ll[index_tca_ll, 2], "x", AC2_trajectory_ll[index_tca_ll, 3], AC2_trajectory_ll[index_tca_ll, 2], "x")
    axis("equal")
    l = initial[7] * 1.2
    axis([-l, l, -l, l])
    grid(true)
    xlabel("y(ft)")
    ylabel("x(ft)")
    legend(["AC1", "AC2", "AC1_LL", "AC2_LL"], loc = 0)

    figure(3)
    plot(AC1_trajectory[:, 1] , AC1_trajectory[:, 4], AC2_trajectory[:, 1], AC2_trajectory[:, 4])
    plot(AC1_trajectory_ll[:, 1], AC1_trajectory_ll[:, 4], "--", AC2_trajectory_ll[:, 1], AC2_trajectory_ll[:, 4], "--")
    plot(AC1_trajectory[index_tca, 1], AC1_trajectory[index_tca, 4], "*", AC2_trajectory[index_tca, 1], AC2_trajectory[index_tca, 4], "*")
    plot(AC1_trajectory_ll[index_tca_ll, 1], AC1_trajectory_ll[index_tca_ll, 4], "x", AC2_trajectory_ll[index_tca_ll, 1], AC2_trajectory_ll[index_tca_ll, 4], "x")
    grid(true)
    xlabel("time(sec)")
    ylabel("h(ft)")
    legend(["AC1", "AC2", "AC1_LL", "AC2_LL"], loc = 0)
end


function parse_commandline()

    settings = ArgParseSettings()

    settings.description = "Airspace Encounter Simulation"

    @add_arg_table settings begin
        "run"
            help = "simulate trajectories"
            action = :command
        "compare"
            help = "compare simulated trajectories with/out TCAS"
            action = :command
        "validate"
            help = "compare simulated trajectories to those from Lincoln Lab"
            action = :command
        "generate"
            help = "generate samples to file"
            action = :command
        "plot"
            help = "draw trajectories"
            action = :command
    end

    @add_arg_table settings["run"] begin
        "--tcas", "-t"
            help = "simulate with TCAS"
            action = :store_true
        "--read_from_file", "-r"
            help = "read samples from file"
            action = :store_true
        "--init_file"
            help = "initial sample file"
            arg_type = AbstractString
            default = "initial.txt"
        "--tran_file"
            help = "transition sample file"
            arg_type = AbstractString
            default = "transition.txt"
        "--sample", "-n"
            help = "sample number"
            arg_type = Int
            default = 1
    end

    @add_arg_table settings["compare"] begin
        "--init_file"
            help = "initial sample file"
            arg_type = AbstractString
            default = "initial.txt"
        "--tran_file"
            help = "transition sample file"
            arg_type = AbstractString
            default = "transition.txt"
        "--sample", "-n"
            help = "sample number"
            arg_type = Int
            default = 1
    end

    @add_arg_table settings["validate"] begin
        "--sample", "-n"
            help = "sample number"
            arg_type = Int
            default = 1
    end

    @add_arg_table settings["generate"] begin
        "--init_file"
            help = "initial sample file"
            arg_type = AbstractString
            default = "initial.txt"
        "--ninit", "-n"
            help = "number of initial samples"
            arg_type = Int
            default = 1
        "--tran_file"
            help = "transition sample file"
            arg_type = AbstractString
            default = "transition.txt"
        "--ntran"
            help = "number of transition samples"
            arg_type = Int
            default = 50
        "--param_file"
            help = "encounter parameter file"
            arg_type = AbstractString
            default = Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/params/cor.txt")
    end

    @add_arg_table settings["plot"] begin
        "--compare", "-c"
            help = "compare simulated trajectories with/out TCAS"
            action = :store_true
        "--validate", "-d"
            help = "compare simulated trajectories to those from Lincoln Lab"
            action = :store_true
    end

    return parse_args(settings)
end


function main()

    parsed_args = parse_commandline()

    if parsed_args["%COMMAND%"] == "run"
        run_simulation(bTCAS = parsed_args["run"]["tcas"], bReadSampleFromFile = parsed_args["run"]["read_from_file"], initial_sample_filename = parsed_args["run"]["init_file"], transition_sample_filename = parsed_args["run"]["tran_file"], sample_number = parsed_args["run"]["sample"])

    elseif parsed_args["%COMMAND%"] == "compare"
        run_simulation_for_comparison(initial_sample_filename = parsed_args["compare"]["init_file"], transition_sample_filename = parsed_args["compare"]["tran_file"], sample_number = parsed_args["compare"]["sample"])

    elseif parsed_args["%COMMAND%"] == "validate"
        run_simulation_for_validation(sample_number = parsed_args["validate"]["sample"])

    elseif parsed_args["%COMMAND%"] == "generate"
        generate_samples_to_file(parsed_args["generate"]["init_file"], parsed_args["generate"]["ninit"], parsed_args["generate"]["tran_file"], parsed_args["generate"]["ntran"], parsed_args["generate"]["param_file"])

    elseif parsed_args["%COMMAND%"] == "plot"
        if parsed_args["plot"]["compare"]
            plot_result_for_comparison()
        elseif parsed_args["plot"]["validate"]
            plot_result_for_validation()
        else
            plot_result()
        end

        readline()
    end
end


main()


