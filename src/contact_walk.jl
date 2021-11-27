module contact_walk

using JuMP
using Ipopt 
using Plots
using WGLMakie
using DifferentialEquations
using Interpolations
# using LCPsolve

include("types.jl")
include("simulator.jl")
include("visualize.jl")
include("lcp_solve.jl")
include("foot_collision.jl")
include("controller.jl")
include("walking_dynamics.jl")

export solve_lcp, 
       form_lcp, 
       step, 
       render_block_trajectory,
       render_ball_trajectory,
       render_walking_trajectory,
       render_kick_trajectory!,
       simulate,
       Object,
       Physics,
       Walker,
       collision_cb,
       controller,
       simulate_walking,
       single_stance!,
       footstrike


end
