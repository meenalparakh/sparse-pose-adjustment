using Plots

function plotPose(x, numPoses, scatter_plot = false, s = 1)
    x_plot = Array{Float64,1}(undef, numPoses)
    y_plot = Array{Float64,1}(undef, numPoses)
    for i in 1:numPoses
        x_plot[i] = x[1,i]
        y_plot[i] = x[2,i]
    end
    if (scatter_plot)
        a = scatter(x_plot,y_plot, m=s)
    else
        a = plot(x_plot,y_plot)
    end
    display(a)
end
