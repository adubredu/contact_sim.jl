using contact_sim 
  

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
    q₀ = [-1.5, 0.2, π/180.0*30.]
    v₀ = [2.2, 3.2, 0.0] 

    body = Object(l, μ, ϵ, m, R, M)
    physics = Physics(dt, g, Δ, T)

    q, v = simulate(q₀, v₀, body, physics, get_ball_contact_jacobian)
    render_ball_trajectory(q, body, physics, true) 
end
 
main()