# creating z matrix and invariance matrix  - loaded from g2o files

struct Edge
  # i::Int32
  # j::Int32
  T::Array{Float64}
  # θ::Float64
  # R::Array{Float64}
  # τ::Float64    # translational precision
  # κ::Float64    # rotational precision
  Λ_ij::Matrix{Float64}
end

 # The g2o format specifies a 2D relative pose measurement in the
 # following form:
 #
 # EDGE_SE2 id1 id2 dx dy dtheta, I11, I12, I13, I22, I23, I33
function readG2oFile(filename)
  edges = OrderedDict{Tuple, Edge}()
  f = open(filename, "r")
  numPoses = 0
  for line in readlines(f)
    # global numPoses
    # numPoses = 0
    wordArray = split(line, " ")
      if (wordArray[1] == "EDGE_SE2")
        i = parse(Int32, wordArray[2])+1
        j = parse(Int32, wordArray[3])+1
        if true#(j>i)
          # println(i," ", j)
          x = parse(Float64, wordArray[4])
          y = parse(Float64, wordArray[5])
          θ = parse(Float64, wordArray[6])
          I11 = parse(Float64, wordArray[7])
          I12 = parse(Float64, wordArray[8])
          I13 = parse(Float64, wordArray[9])
          I22 = parse(Float64, wordArray[10])
          I23 = parse(Float64, wordArray[11])
          I33 = parse(Float64, wordArray[12])
          Λ_ij = [[I11, I12, I13] [I12, I22, I23] [I13, I23, I33]]
          edges[(i,j)] = Edge([x,y,θ],Λ_ij)
          # else
          #   edges[(j,i)] = Edge([y*sin(θ)-x*cos(θ), -y*cos(θ)-x*sin(θ)], -θ, transpose(Λ_ij))
          numPoses = max(i, j, numPoses)
        end
        # println(numPoses)
      end
  end
  close(f)
  edges, numPoses
end

# Read vertices
function readG2oFileVertices(filename, vertices)
  f = open(filename, "r")
  numPoses = 0
  for line in readlines(f)
    # global numPoses
    # numPoses = 0
    wordArray = split(line, " ")
      if (wordArray[1] == "VERTEX_SE2")
        i = parse(Int32, wordArray[2])+1
        x = parse(Float64, wordArray[3])
        y = parse(Float64, wordArray[4])
        θ = parse(Float64, wordArray[5])
        vertices[i] = [x, y, θ]
        numPoses += 1
      end
  end
  close(f)
  numPoses
end
