function form_lcp(J, v, body, physics)
    μ = body.μ
    M = body.M
    Mi = body.M_inv
    ϵ = body.ϵ
    dt = physics.dt
    g = physics.g

    Jₙ = J[:,2]
    Jₜ = J[:,1]

    V = zeros((4,4))
    V[1,1] = Jₙ'* Mi * (dt*Jₙ)
    V[1,2] = -Jₙ'* Mi * (dt*Jₜ)
    V[1,3] = Jₙ'* Mi * (dt*Jₜ)
    V[2,1] = -Jₜ'* Mi * (dt*Jₙ)
    V[2,2] = Jₜ'* Mi * (dt*Jₜ)
    V[2,3] = -Jₜ'* Mi * (dt*Jₜ)
    V[2,4] = 1.
    V[3,1] = Jₜ'* Mi * (dt*Jₙ)
    V[3,2] = -Jₜ'* Mi * (dt*Jₜ)
    V[3,3] = Jₜ'* Mi * (dt*Jₜ)
    V[3,4] = 1.
    V[4,1] = μ
    V[4,2] = -1.
    V[4,3] = -1.
    fₑ = M*g 
    p = zeros(4) 
    p[1] = Jₙ' * ((1+ϵ)*v + dt*Mi*fₑ)
    p[2] = -Jₜ' * (v + dt*Mi*fₑ)
    p[3] = Jₜ' * (v + dt*Mi*fₑ)

    return V, p
end


function solve_lcp(V, p)  
    lcp = Model(Ipopt.Optimizer)
    N = length(p) 
    @variable(lcp, x[1:N])
    @constraint(lcp,  V*x + p .>= 0.)
    @constraint(lcp, x .>= 0.)
    @objective(lcp, Min, x'*(V*x + p))
    optimize!(lcp) 
    
    return value.(x)

    # result = solve!(LCP(M, q))
    # return result.sol
end