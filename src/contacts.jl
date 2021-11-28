function get_ball_contact_jacobian(q, body)
    xₜ = q[1]
    yₜ = q[2]
    θ  = q[3]
    D = body.l
    rad = D/2.

    xyc = [xₜ, yₜ-rad]
    ϕ = xyc[2]

    n = [0, 1]
    r = xyc - [xₜ, yₜ]
    rₓnᵤ = r[1]*n[2]; rᵤnₓ = r[2]*n[1]
    rₓnₓ = r[1]*n[1]; rᵤnᵤ = r[2]*n[2]

    J = [[n[2] n[1]];
         [-n[1] n[2]];
         [-rₓnₓ-rᵤnᵤ rₓnᵤ-rᵤnₓ]]
    
    return J, ϕ
end

function get_block_contact_jacobian(q, body)
    xₜ = q[1]
    yₜ = q[2]
    θ  = q[3]
    L = body.l

    l = sqrt(2* (0.5*L)^2)
    a = [xₜ+l*cos(θ+3π/4), yₜ+l*sin(θ+3π/4)]
    b = [xₜ+l*cos(θ+5π/4), yₜ+l*sin(θ+5π/4)]
    c = [xₜ+l*cos(θ+π/4), yₜ+l*sin(θ+π/4)]
    d = [xₜ+l*cos(θ+7π/4), yₜ+l*sin(θ+7π/4)]

    corners = [a, b, c, d]
    min_index = argmin([a[2], b[2], c[2], d[2]])
    xyc = corners[min_index]
    ϕ = xyc[2]

    n = [0, 1]
    r = xyc - [xₜ, yₜ]
    rₓnᵤ = r[1]*n[2]; rᵤnₓ = r[2]*n[1]
    rₓnₓ = r[1]*n[1]; rᵤnᵤ = r[2]*n[2]

    J = [[n[2] n[1]];
         [-n[1] n[2]];
         [-rₓnₓ-rᵤnᵤ rₓnᵤ-rᵤnₓ]]
    
    return J, ϕ
end