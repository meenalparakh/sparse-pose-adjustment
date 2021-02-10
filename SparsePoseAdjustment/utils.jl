cd(@__DIR__)
using Pkg
Pkg.activate(".")
# Pkg.add("OrderedCollections")
# Pkg.add("LinearAlgebra")
# Pkg.add("DataStructures")
# Pkg.add("SharedArrays")
#
#
# using Distributed
# addprocs(2)

import Pkg
using DataStructures
using OrderedCollections
using SparseArrays
using BenchmarkTools
using SuiteSparse
using LinearAlgebra
using Plots
using StaticArrays
using SharedArrays
using Base.Threads

include("readFile.jl")
include("plot.jl")
#
# z, numPoses = readG2oFile("./../city.g2o")
# X = initializeX(z, numPoses, false)


# @inline function offset_ij(offset::Array{Float64,1}, i::Int64, j::Int64, x::Array)
#
#   offset .= @view(x[:,j]) .- @view(x[:,i])
#   # offset[1:2] = reshape([cos(x[3,i]), -sin(x[3,i]), sin(x[3,i]), cos(x[3,i])], 2, 2) * @view(offset[1:2])
#   offset[1:3] = [cos(x[3,i])*offset[1]+sin(x[3,i])*offset[2], -sin(x[3,i])*offset[1]+cos(x[3,i])*offset[2],mod(offset[3],2*π)]
#   # offset[2] = -sin(x[3,i])*offset[1]+cos(x[3,i])*offset[2]
#   # offset[3] = mod(offset[3],2*π)
#   # if (offset[3] > π)
#   #   offset[3] = offset[3]-2*π
#   # end
# end
@inline function offset_ij(offset::Array{Float64, 1}, i::Int64, j::Int64, x::Array)

  offset .= @view(x[:,j]) .- @view(x[:,i])
  # offset[1:3] = [cos(x[3,i])*offset[1]+sin(x[3,i])*offset[2], -sin(x[3,i])*offset[1]+cos(x[3,i])*offset[2],mod(offset[3],2*π)]
  offset[1], offset[2], offset[3] = cos(x[3,i])*offset[1]+sin(x[3,i])*offset[2],
                                    -sin(x[3,i])*offset[1]+cos(x[3,i])*offset[2],
                                    mod(offset[3],2*π)
  nothing
end

##### time testing #########

# offseto = Array{Float64, 1}(undef, 3)
# @btime offset_ij(offseto, 1, 2, X)
###########################

function initializeError(errorVector, Z)
  i = one(Int64)
  for edge in Z
    errorVector[i] = [edge.first[1], edge.first[2], Any]
    i += 1
  end
end

# errorVector = Array{Array}(undef, numDataPoints)
# sets errorVec and returns average error
function Error(errorVec::Array{Array,1},
               x::Array{Float64,2},
               edges::OrderedDict{Tuple, Edge})
  # i = one(Int64)
  currentError = SharedArray{Float64}(length(edges))
  # currentOffset = Array{Float64, 1}(undef, 3)
  Threads.@threads for i in 1:length(edges)
    # println((edge.first[1], edge.first[2]))
    currentOffset = Array{Float64, 1}(undef, 3)
    offset_ij(currentOffset, edges.keys[i][1], edges.keys[i][2], x)
    dataOffset = edges[edges.keys[i]].T .- currentOffset
    dataOffset[3] = mod(dataOffset[3],2*π)
    if (dataOffset[3] > π)
      dataOffset[3] = dataOffset[3]-2*π
    end
    # errorVec[i] = [edge.first[1], edge.first[2], dataOffset]
    errorVec[i][3] = dataOffset
    currentError[i] = transpose(dataOffset)*edges[edges.keys[i]].Λ_ij*dataOffset
    # i += 1
  end
  sum(currentError)
end

# @everywhere errorVec = SharedArray{Array}(length(z))
# initializeError(errorVec, z)
# # initError = Error(errorVec, X, z)
# @btime Error(errorVec, X, z)
