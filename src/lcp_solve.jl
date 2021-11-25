function solve_lcp(M, q)
    lcp = Model(Ipopt.Optimizer)
    N = length(q)
    init = zeros(N) 
    @variable(lcp, z[i=1:N] >= init[i])
    @constraint(lcp,  M*z + q .>= 0.)
    @constraint(lcp, z'*(M*z + q) .== 0.)
    @objective(lcp, Min, 1.0)
    optimize!(lcp)

    return value.(z)
end