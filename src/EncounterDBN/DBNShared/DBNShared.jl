
function convert_units(v::Vector{Float64}) 
    Float64[convert_units(v[i], MAP_IND2VAR_L[i]) for i=1:endof(v)]
end

function unconvert_units(v::Vector{Float64}) 
    Float64[unconvert_units(v[i], MAP_IND2VAR_L[i]) for i=1:endof(v)]
end

function convert_units(x::Float64, var::Symbol)
  if var == :v_d0 || var == :v_d1
    return x * 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x / 60
  else
    return x
  end
end

function unconvert_units(x::Float64,var::Symbol)
  if var == :v_d0 || var == :v_d1
    return x / 1.68780
  elseif var == :h_d0 || var == :h_d1
    return x * 60
  else
    return x
  end
end

function val2ind(boundariesi, ri, value)
  if !isempty(boundariesi)
    index = findfirst(x -> (x > value), boundariesi) - 1

    if index == -1
      index = ri
    end
  else
    index = value
  end
  return index
end

function discretize(p::CorrAEMParameters, v::Vector{Float64})
  Int64[val2ind(p.boundaries[MAP_L2G[i]], p.r_transition[MAP_L2G[i]], val) for (i, val) in enumerate(v)]
end

function dediscretize(dval::Int64, boundaries::Vector{Float64}, zero_bin::Int64)
  val_min = boundaries[dval]
  val_max = boundaries[dval + 1]

  if dval == zero_bin
    val = 0.0
    prob = 1.0
  elseif val_max == val_min
    val = val_min
    prob = 1.0
  else
    val = randfloat(val_min, val_max)
    prob = 1.0 / (val_max - val_min)
    #this is a density so it won't be normalized to [0,1]
  end

  return (val, prob)
end

function select_random_cumweights(cweights::Vector{Float64})
  r = cweights[end] * rand()

  return findfirst(x -> (x >= r), cweights)
end

function alt_bounds(L::Int64)
    if L == 1
        return (1000.0, 3000.0)
    elseif L == 2
        return (3000.0, 10000.0)
    elseif L == 3
        return (10000.0, 18000.0)
    elseif L == 4
        return (18000.0, 29000.0)
    elseif L == 5
        return (29000.0, 40000.0)
    else
        error("invalid L")
    end
end

#mods x to the range [-b, b]
function to_plusminus_b(x::AbstractFloat, b::AbstractFloat)
  z = mod(x, 2 * b)

  return (z > b) ? (z - 2 * b) : z
end

to_plusminus_180(x::AbstractFloat) = to_plusminus_b(x, 180.0)
randfloat(xmin::Float64, xmax::Float64) = xmin + rand() * (xmax-xmin)

