function collision_condition(u, t, integrator)
    θ₁ = u[1]; θ₂ = u[3]
    if θ₁>-0.05
        return 1.0
    else
        return θ₂ + 2*θ₁

    end
end

function collision_affect!(integrator)
    terminate!(integrator)
end

collision_cb = ContinuousCallback(collision_condition, collision_affect!)