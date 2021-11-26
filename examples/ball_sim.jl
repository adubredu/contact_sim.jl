using Revise
using contact_walk 
 
function get_ball_contacts(q, body)
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

    q, v = simulate(q₀, v₀, body, physics, get_ball_contacts)
    render_ball_trajectory(q, body, physics) 
end
 
main()