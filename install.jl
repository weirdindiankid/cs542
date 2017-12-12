# Filename: install.jl
# Author: Dharmesh Tarapore <dharmesh@bu.edu>

Pkg.add("Compat")
Pkg.add("Distributions")
Pkg.add("JLD")
Pkg.add("HDF5")
Pkg.add("PyPlot")
Pkg.add("ArgParse")

Pkg.clone("https://github.com/weirdindiankid/CASInterface.jl", "CASInterface")
Pkg.clone("https://github.com/weirdindiankid/RLESUtils.jl", "RLESUtils")
Pkg.clone("https://github.com/weirdindiankid/TensorFlow.jl", "TensorFlow")

Pkg.init(); run(`ln -s $(pwd()) $(Pkg.dir("SISLES"))`); Pkg.pin("SISLES"); Pkg.resolve()
using SISLES; @assert isdefined(:SISLES); @assert typeof(SISLES) === Module