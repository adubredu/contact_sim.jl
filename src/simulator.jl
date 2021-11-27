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

function simulate_walking(z0, walker, steps...)
    l = walker.l
    flag = 1
    if length(steps) == 0
        flag = 0
        steps = [1]
    end
    θ₁ = z0[1]
    xh = -1.2
    yh = l*cos(θ₁)
    xh_start = xh

    t0 = 0.0
    dt = 4.0
    t_ode = t0
    z_ode = reshape(cat(z0, xh, yh, dims=2), (1, 6))
    zplus = zeros(4)
    z_ref = [z0[3] z0[4] 0]

    for i=1:steps[1]
        if walker.use_controller
            walker.t₀ = t0 
            walker.Θ₂ = z0[3]
            walker.Θ₂̇  = z0[4]
        end 
        tspan = (t0, t0+dt)
        prob = ODEProblem(single_stance!, z0, tspan, walker)
        sol = DifferentialEquations.solve(prob, Tsit5(), callback=collision_cb)

        if walker.use_controller
            n = length(sol.t)
            θ₂ᵣ, θ₂̇ᵣ, θ₂̈ᵣ = zeros(n), zeros(n), zeros(n)
            for j = 1:length(n)
                (u, θ₂ᵣ[j], θ₂̇ᵣ[j], θ₂̈ᵣ[j]) = controller(sol.t[j], sol.u[j][1:4], walker)
            end
        end

        zplus = footstrike(sol.t[end], sol.u[end, :], walker)

        z0 = zplus
        t0 = sol.t[end]
        z_temp = reduce(vcat, sol.u)
        t_temp = sol.t
        xh_temp = xh_start .+ l*sin(z_temp[1,1]).-l*sin.(z_temp[:,1])
        yh_temp = l*cos.(z_temp[:,1])
        t_ode = vcat(t_ode, t_temp[2:end])
        z_ode = vcat(z_ode, hcat(vcat(z_temp[2:end-1, :],zplus), xh_temp[2:end], yh_temp[2:end]))

        if walker.use_controller 
            z_ref = vcat(z_ref, transpose(vcat(transpose(θ₂ᵣ[2:end]), transpose(θ₂̇ᵣ[2:end]), transpose(θ₂̈ᵣ[2:end]))))
            θ₂ᵣ, θ₂̇ᵣ, θ₂̈ᵣ = [],[],[]
        end

        xh_start = xh_temp[end]

    end
    z = zplus[1:4]
    if flag == 1
        return (z_ode, t_ode)
    else
        return z
    end
end