module contact_walk

using JuMP
using Ipopt

include("lcp_solve.jl")
export solve_lcp

end
