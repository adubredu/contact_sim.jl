function step(q, v, body, physics, get_contacts)
    M = body.M 
    Mi = body.M_inv
    g = physics.g
    Δ = physics.Δ 
    dt = physics.dt

    J, ϕ = get_contacts(q, body)
    fₑ = M*g 
    Jₙ = J[:,2]
    Jₜ = J[:,1]
    qₚ = [0., Δ, 0.] 

    if ϕ >= Δ
        v¹ = v + dt*Mi*fₑ
        q¹ = q + dt*v¹
    else 
        V, p = form_lcp(J, v, body, physics) 
        fᵧ = solve_lcp(V, p)
        fₙ = fᵧ[1]; fₜ₁ = fᵧ[2]; fₜ₂ = fᵧ[3]
        v¹ = v + dt*Mi*(fₑ + fₙ*Jₙ - fₜ₁*Jₜ + fₜ₂*Jₜ)
        q¹ = q + dt*v¹+qₚ
    end
    return q¹, v¹
end


function simulate(q₀, v₀, body, physics, get_contacts)
    q = [q₀]
    v = [v₀]
    T = physics.T

    for i=1:T-1
        q¹, v¹ = step(q₀, v₀, body, physics, get_contacts)
        q₀ = q¹
        v₀ = v¹
        push!(q, q¹)
        push!(v, v¹)
    end 
    q = hcat(q...)
    v = hcat(v...)

    return q, v 
end 