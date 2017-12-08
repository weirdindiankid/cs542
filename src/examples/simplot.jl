using SISLES
using ArgParse
using HDF5, JLD
using PyPlot

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