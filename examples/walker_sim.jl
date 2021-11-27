using contact_walk




q1 = 0.2; u1 = -0.25;
q2 = -0.4; u2 = 0.2;

z0 = [q1 u1 q2 u2]
steps = 4
fps = 20

#passive: no controller
#=
walker = Walker(1.0, 0.5, 0.02, 1.0, 0.5, 1.0, 0.01, false, 1.0, 100.0, 0.28564, z0[3], z0[4], 0.0)
zstar = [0.162597833780035  -0.231869638058927  -0.325195667560070   0.03797846807373]
zpert = zstar
=#

#active: use controller
walker = Walker(1.0, 0.5, 0.02, 1.0, 0.5, 1.0, 0.01, true, 1.0, 100.0, 0.28564, z0[3], z0[4], 0.0)
zstar = [ 0.142819999999995  -0.326813112785275  -0.285640000000000   0.068243824367047]
zpert = zstar + [0. 0.05 -0.1 0.2]

z, t = simulate_walking(zpert,walker,steps)
render_walking_trajectory(t,z,walker, steps, fps)