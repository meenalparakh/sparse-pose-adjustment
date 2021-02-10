
include("utils.jl")

function CreateSparse(#RHS::Array{Float64,2},
                      Error::Array{Array,1},
                      X::Array{Float64,2},
                      Z::OrderedDict{Tuple,Edge},
                      numPoses::Int64,
                      λ::Float64)
  mapVector = Vector{Union{Nothing, Dict}}(undef, numPoses)
  RHS = zeros(3,numPoses)
  @inbounds for j in 1:numPoses
    mapVector[j] = Dict{Int64, Array{Float64,2}}()
  end
  Λ_ij = Matrix{Float64}(undef,3,3)
  Ji = Matrix{Float64}(undef,3,3)
  Jj = Matrix{Float64}(undef,3,3)
  @inbounds for eij in Error
    i = eij[1]
    j = eij[2]
    Λ_ij .= Z[(i,j)].Λ_ij
    cos_i = cos(X[3,i])
    sin_i = sin(X[3,i])
    Ji .= reshape([-cos_i, sin_i, 0, -sin_i, -cos_i, 0, sin_i * (X[1,j] - X[1,i]) - cos_i * (X[2,j] - X[2,i]),
          cos_i * (X[1,j] - X[1,i]) + sin_i * (X[2,j] - X[2,i]), -1],3,3)
    Jj .= reshape([cos_i, -sin_i, 0, sin_i, cos_i, 0, 0, 0, 1],3,3)
    # currentError .= currentError + [transpose(eij.second)*Λ_ij*eij.second]
    RHS[:,i] .= RHS[:,i] .+ transpose(Ji)*Λ_ij*eij[3]
    RHS[:,j] .= RHS[:,j] .+ transpose(Jj)*Λ_ij*eij[3]
#  get (collection, key, default)
    # @assert(j > i)
    if haskey(mapVector[i], i)
      mapVector[i][i] += transpose(Ji)*Λ_ij*Ji
    else
      mapVector[i][i] = transpose(Ji)*Λ_ij*Ji
    end

    if haskey(mapVector[j], j)
      mapVector[j][j] += transpose(Jj)*Λ_ij*Jj
    else
      mapVector[j][j] = transpose(Jj)*Λ_ij*Jj
    end

    if (j > i)
      if haskey(mapVector[j], i)
        mapVector[j][i] += transpose(Ji)*Λ_ij*Jj
      else
        mapVector[j][i] = transpose(Ji)*Λ_ij*Jj
      end
    else
      if haskey(mapVector[j], i)
        mapVector[i][j] += transpose(transpose(Ji)*Λ_ij*Jj)
      else
        mapVector[i][j] = transpose(transpose(Ji)*Λ_ij*Jj)
      end
    end
  end

  col_indices = Array{Int64,1}(undef, 6*numPoses+9*length(Z))
  row_indices = Array{Int64,1}(undef, 6*numPoses+9*length(Z))
  values = Array{Float64,1}(undef, 6*numPoses+9*length(Z))

  i = 1
  matrix_ij = Array{Float64,2}
  @inbounds for col in 1:numPoses
    for row in sort(collect(keys(mapVector[col])))
      # println(col,' ',row)
      matrix_ij = mapVector[col][row]
      if col==row
        # only assign upper half elements (6)
        col_indices[i:i+5] .= [3*col-2,
                              3*col-1, 3*col-1,
                              3*col, 3*col, 3*col]
        row_indices[i:i+5] .= [3*row-2,
                              3*row-2, 3*row-1,
                              3*row-2, 3*row-1, 3*row]
        values[i:i+5] .= [matrix_ij[1]*(1.0+λ),
                         matrix_ij[4], matrix_ij[5]*(1.0+λ),
                         matrix_ij[7], matrix_ij[8], matrix_ij[9]*(1.0+λ)]
        i = i+6
      else
        # assign 9 elements
        col_indices[i:i+8] .= [3*col-2, 3*col-2, 3*col-2,
                              3*col-1, 3*col-1, 3*col-1,
                              3*col,   3*col,   3*col]
        row_indices[i:i+8] .= [3*row-2, 3*row-1, 3*row,
                              3*row-2, 3*row-1, 3*row,
                              3*row-2, 3*row-1, 3*row]
        values[i:i+8] .= @view(matrix_ij[1:9])
        i = i+9
      end
    end
  end
  H = sparse(row_indices, col_indices, values)
  (H, reshape(RHS, :, 1))
end
