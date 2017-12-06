using SISLES
using ArgParse
using HDF5, JLD
using PyPlot

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

initial_sample_filename = "./initial.txt"
transition_sample_filename = "./transition.txt"
sample_number = 1

sim = initialize_simulation(bReadSampleFromFile = true, initial_sample_filename = initial_sample_filename, transition_sample_filename = transition_sample_filename)

aem = sim.parameters.em

AC1_trajectory_ = Vector{Float64}[]
AC2_trajectory_ = Vector{Float64}[]

adm_1, adm_2 = sim.parameters.dm

addObserver(adm_1, x -> push!(AC1_trajectory_, x))
addObserver(adm_2, x -> push!(AC2_trajectory_, x))


simulate(sim, bTCAS = false, sample_number = sample_number)