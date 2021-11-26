module contact_walk

using JuMP
using Ipopt 
# using LCPsolve
using Plots

include("types.jl")
include("simulator.jl")
include("visualize.jl")
include("lcp_solve.jl")

export solve_lcp, 
       form_lcp, 
       step, 
       render_block_trajectory,
       render_ball_trajectory,
       simulate,
       Object,
       Physics 

end
