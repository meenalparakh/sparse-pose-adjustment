cd(@__DIR__)
using Pkg
Pkg.activate(".")

include("Optimize.jl")
include("SpanningTree.jl")

function example1()
  # z = OrderedDict{Tuple, Edge}()
  z, numPoses = readG2oFile("./../MIT_Kilian.g2o")
  ## for intel  - start at 1e-6, factor = 0.9

  # length(z)
  ###############################
  # Spanning tree intialization #
  ###############################
  X = initializeX(z, numPoses, true)
  # plotPose(X)
  X = optimizePose(X, z, 1e-7, numPoses, 10, 0.6)
  plotPose(X, numPoses, false, 2)
end

example1()

###### odom intialization ########
function example2()
  # z = OrderedDict{Tuple, Edge}()
  z, numPoses = readG2oFile("./../intel.g2o")
  X_odom = initializeX_Odom(z, numPoses)
  X_odom = optimizePose(X_odom, z, 1e-6, numPoses, 10, 0.7)
  plotPose(X_odom, numPoses)
end

example2()

###### chordal intialization ########
function example3()
  z, numPoses = readG2oFile("./../MIT_Kilian.g2o")
  X = initializeChordal(z, numPoses)
  X = optimizePose(X, z, 1e-6, numPoses, 10, 0.9)
  plotPose(X, numPoses, false, 2)
end

example3()
################################  time testing  ########

v = OrderedDict{Int32, Array}()
vertices = readG2oFileVertices("./../intel.g2o", v)

init_X = zeros(3, numPoses)
if (vertices!=0)
  for i in v.keys
    init_X[:,i] = v[i]
  end
end

plotPose(init_X)

optimizePose(init_X, z, 1e-6, numPoses, 30, 0.9,1e10)
plotPose(init_X, scatter = true)




########### create sparse

# @btime CreateSparse()
# SharedArray{Float64}(150*length(r))
# errorVec = SharedArray{Array,1}(undef, length(z))
z, numPoses = readG2oFile("./../city.g2o")
X = initializeX(z, numPoses)
errorVec = Array{Array,1}(undef, length(z))
initializeError(errorVec, z)
initError = Error(errorVec, X, z)
@btime Error(errorVec, X, z)

@btime CreateSparse(errorVec, X, z, numPoses, 1e-4)
