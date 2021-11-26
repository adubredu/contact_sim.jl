module contact_walk

using JuMP
using Ipopt 
using LCPsolve

include("lcp_solve.jl")
export solve_lcp

end
