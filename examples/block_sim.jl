using Revise
using contact_walk 
 
function get_contacts(q, body)
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

 

function main()
    #system vars
    l = 0.4 
    μ = 0.3
    ϵ = 0.5
    dt = 0.01
    m = 0.3
    g = [0., -9.81, 0.]
    R = (1.0/12.) * (2 * l * l)

    M = [[m 0. 0.]; 
        [0. m 0.];
        [0. 0. m * R]]

    Mi = [[1.0/m  0.  0.];
        [0. 1.0/m 0.];
        [0. 0. 1.0/(m*R)]]
    Δ = 0.001
    T = 150

    #init conditions
    q₀ = [0.0, 1.5, π/180.0*30.]
    v₀ = [0.0, -0.2, 0.0] 

    body = Object(l, μ, ϵ, m, R, M)
    physics = Physics(dt, g, Δ, T)

    q, v = simulate(q₀, v₀, body, physics, get_contacts)
    render_block_trajectory(q, physics) 
end
 
main()