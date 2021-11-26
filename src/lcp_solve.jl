function solve_lcp(M, q)  
    lcp = Model(Ipopt.Optimizer)
    N = length(q) 
    @variable(lcp, x[1:N])
    @constraint(lcp,  M*x + q .>= 0.)
    @constraint(lcp, x .>= 0.)
    @objective(lcp, Min, x'*(M*x + q))
    optimize!(lcp) 
    return value.(x)

    # result = solve!(LCP(M, q))
    # return result.sol
end