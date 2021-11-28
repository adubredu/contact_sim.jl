using contact_sim  

function main()
    #walker inits 
    q1 = 0.2; u1 = -0.25;
    q2 = -0.4; u2 = 0.2;
    z0 = [q1 u1 q2 u2]
    steps = 4
    fps = 20
    walker = Walker(1.0, 0.5, 0.02, 1.0, 0.5, 1.0, 0.01, true, 1.0, 100.0, 0.28564, z0[3], z0[4], 0.0)
    z_init = [ 0.142819999999995  -0.326813112785275  -0.285640000000000   0.068243824367047]
    zpert = z_init + [0. 0.05 -0.1 0.2]

    #ball inits 
    l = 0.4 
    μ = 0.3
    ϵ = 0.5
    dt = 0.01
    m = 0.04
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
    q₀ = [0.0, 0.2, π/180.0*30.] 

    Torque = 27.9 #hinge motor torque (Nm)
    Fe = Torque*walker.l #kick force (N)
    th = 0.24 #kick angle (rad)
    v₀ = dt*Mi*[Fe*cos(th); Fe*sin(th); 0.0] 


    body = Object(l, μ, ϵ, m, R, M)
    physics = Physics(dt, g, Δ, T)

    zs, ts = simulate_walking(zpert,walker,steps)
    qs, v = simulate(q₀, v₀, body, physics, get_ball_contact_jacobian) 
    render_kick_trajectory(ts, zs, qs, walker, steps, fps, body, physics, true)
end

main()
 