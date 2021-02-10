include("CreateSparse.jl")

function optimizePose(X::Array{Float64,2},
                      Z::OrderedDict{Tuple,Edge},
                      λ::Float64,
                      numPoses::Int64,
                      numIterations::Int64 = 500,
                      d::Float64 = 0.8,
                      λ_max::Float64 = 1e7)
# function ContinuableLM(X::Array, Z::Dict, λ::Float64, numPoses::Int64)
  errorVec = Array{Array,1}(undef, length(Z))
  newerrorVec = Array{Array,1}(undef, length(Z))
  initializeError(errorVec, Z)
  initializeError(newerrorVec, Z)
  initError = Error(errorVec, X, Z)
  α = one(Float64)

  println("Iteration: 0 | Cost: ", initError)

  @time begin

  @inbounds for iter in 1:numIterations
    print("Iteration: ", iter)
    LHS, RHS = CreateSparse(errorVec, X, Z, numPoses, λ)
    Factor = cholesky(Hermitian(LHS, :U))
    y = Factor.PtL\RHS
    Δx = Factor.UP\y

    X_new = reshape(Δx, 3, numPoses)
    X_new .= X .+ α*X_new
    X_new[3,:] .= mod.(@view(X_new[3,:]),2*π)

    plotPose(X, numPoses, true)
    # sleep(2)
    # update error
    newError = Error(newerrorVec, X_new, Z)
    if (newError <= initError)
      λ = λ*d
      α = d*α
      # to try:
      # X .= X_new
      X = X_new
      # for g in 1:length(Z)
      #   errorVec[g][3] .= newerrorVec[g][3]
      # end
      errorVec, newerrorVec = newerrorVec, errorVec
      initError = newError
      print(" | New Cost: ", newError)
    else
      λ = 2*λ
      # print("| KeptCost: ", initError, " | ObtCost: ", newError)
      if (λ > λ_max)
        print("\n Stopping! \n")
        return X
      end
    end
    println(" | λ: ", λ)
  end
  end
  X
end
