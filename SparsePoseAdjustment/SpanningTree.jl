using JuMP
using PolyJuMP
import Ipopt

function initializeX_Odom(Z,numPoses)
  X = zeros(3,numPoses)
  for edge in Z
    i = edge.first[1]
    j = edge.first[2]
    if (j==i+1)
      transform(j,i,X,edge.second)
    end
  end
  X
end

function initializeRotations(Z, numPoses)
    # solve the linear least squares optimization problem ##
  model = Model(Ipopt.Optimizer)
  @variable(model, rotations[1:2, 1:2*numPoses])
  ex = QuadExpr()
  for edge_dict in Z
    c = cos(edge_dict.second.T[3])
    s = sin(edge_dict.second.T[3])
    i = edge_dict.first[1]
    j = edge_dict.first[2]
    for d in 1:2
      add_to_expression!(ex, (c*rotations[1,2*i-2+d] + s*rotations[2,2*i-2+d] - rotations[1,2*j-2+d])^2)
      add_to_expression!(ex, (-s*rotations[1,2*i-2+d] + c*rotations[2,2*i-2+d] - rotations[2,2*j-2+d])^2)
    end
  end
  @objective(model, Min, ex)
  @constraint(model, rotations[1,1] == 1)
  @constraint(model, rotations[2,1] == 0)
  @constraint(model, rotations[1,2] == 0)
  @constraint(model, rotations[2,2] == 1)

  optimize!(model)
  chordalRotations = value.(rotations)
  chordalRotations
end

function initializeChordal(Z, numPoses)
  chordalRotations = initializeRotations(Z, numPoses)
  X = zeros(3,numPoses)
  for k in 1:numPoses
    F = svd(chordalRotations[:, 2*k-1:2*k])
    Rt = transpose(F.U*F.Vt)
    X[3,k] = atan(Rt[2,1], Rt[1,1])
  end

  model = Model(Ipopt.Optimizer)
  @variable(model, translations[1:2, 1:numPoses])
  ex = QuadExpr()

  for edge_dict in Z
    i = edge_dict.first[1]
    j = edge_dict.first[2]
    c = cos(X[3,j])
    s = sin(X[3,j])
    RT = [[c,s] [-s,c]]*edge_dict.second.T[1:2]
    add_to_expression!(ex, (- translations[1,j] + translations[1,i] + RT[1])^2)
    add_to_expression!(ex, (- translations[2,j] + translations[2,i] + RT[2])^2)
  end

  @objective(model, Min, ex)
  @constraint(model, translations[1,1] == 0.0)
  @constraint(model, translations[2,1] == 0.0)

  optimize!(model)
  Translations = value.(translations)

  for k in 1:numPoses
    X[1:2,k] .= Translations[:,k]
  end
  # for edge in z
  #   i = edge.first[1]
  #   j = edge.first[2]
  #   if (j==i+1)
  #     s = sin(X[3,j])
  #     c = cos(X[3,j])
  #     X[1:2,j] .= [[c,s] [-s,c]]*@view(edge.second.T[1:2]) .+ @view(X[1:2,i])
  #   end
  # end
  X
end

z, numPoses = readG2oFile("./../intel.g2o")
X = initializeChordal(z, numPoses)
plotPose(X, numPoses)
X = optimizePose(X, z, 1e-6, numPoses, 10, 0.9)
plotPose(X, numPoses, false, 2)

X = initializeX_Odom(z, numPoses)
plotPose(X, numPoses, false, 2)

X = initializeX(z, numPoses, true)
plotPose(X, numPoses, false, 2)

function transform(j,i,X,edge, reverse = false)
  if reverse
    # X[3,i] = mod(X[3,j] - edge.T[3],2*π)
    s = sin(X[3,j])
    c = cos(X[3,j])
    X[1:2,i] .= -[[c,s] [-s,c]]*@view(edge.T[1:2]) .+ @view(X[1:2,j])
    X[3,i] = mod(X[3,j] - edge.T[3],2*π)
  else
    X[3,j] = mod(X[3,i]+edge.T[3],2*π)
    s = sin(X[3,j])
    c = cos(X[3,j])
    X[1:2,j] .= [[c,s] [-s,c]]*@view(edge.T[1:2]) .+ @view(X[1:2,i])
  end
  nothing
end

function initializeX(Z,numPoses, reverse = false)

  X = zeros(3,numPoses)

  chDict = Dict{Int64, Array}()
  revChDict = Dict{Int64, Array}()

  for i in 1:numPoses
    chDict[i] = Array{Int64,1}()
    revChDict[i] = Array{Int64,1}()
  end
  queue = Queue{Int}()

  if reverse
    for edge in Z
      push!(chDict[edge.first[1]], edge.first[2])
      push!(revChDict[edge.first[2]], edge.first[1])
    end
    enqueue!(queue, one(Int64))
    visited_nodes = Set([1])
  else
    for edge in Z
      push!(chDict[edge.first[1]], edge.first[2])
    end
    enqueue!(queue, one(Int64))
    visited_nodes = Set([1])
  end
  # for i in sort(chDict)
  #   println(i.first," | ",i.second," | ", revChDict[i.first])
  # end
  while (length(visited_nodes)!=numPoses)
    node =  dequeue!(queue)
    for childNode in chDict[node]
      if !(childNode in visited_nodes)
        # println(childNode)
        transform(childNode, node, X, Z[(node, childNode)])
        enqueue!(queue, childNode)
        union!(visited_nodes, childNode)
      end
    end
    for childNode in revChDict[node]
      if !(childNode in visited_nodes)
        # println(childNode)
        transform(node, childNode, X, Z[(childNode, node)], true)
        enqueue!(queue, childNode)
        union!(visited_nodes, childNode)
      end
    end
  end
  X
end

# z, numPoses = readG2oFile("./../CSAIL.g2o")
# X1 = initializeX(z, numPoses, true)
# plotPose(X1, numPoses)
# X2 = initializeX(z, numPoses, false)
# plotPose(X2, numPoses)
# X = 0.8*X1+0.2*X2
# plotPose(X, numPoses, false)
# # X
# #
# X = optimizePose(X2, z, 1e-5, numPoses, 30, 0.9)
