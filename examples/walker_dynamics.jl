using DifferentialEquations
using Interpolations
using Plots

struct Walker{T}
    M::T
    m::T
    I::T
    l::T
    c::T
    g::T
    γ::T
end

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


function single_stance!(du, u, p, t)
    z = u
    walker = p
    θ₁ = z[1]
    θ₂ = z[3]
    ω₁ = z[2]
    ω₂ = z[4]
    M = walker.M; m = walker.m; I=walker.I; l = walker.l; c = walker.c;
    g = walker.g; γ = walker.γ

    A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(θ₂);
    A12 = I + c^2*m - c*l*m*cos(θ₂);
    A21 = I + c^2*m - c*l*m*cos(θ₂);
    A22 = I + c^2*m;
    A_ss = [A11 A12; A21 A22];

    b1 = c*g*m*sin(γ - θ₁) - M*g*l*sin(γ - θ₁) - c*g*m*sin(θ₁ - γ + θ₂) - 2*g*l*m*sin(γ - θ₁) - c*l*m*ω₂^2*sin(θ₂) - 2*c*l*m*ω₁*ω₂*sin(θ₂);
    b2 = -c*m*(g*sin(θ₁ - γ + θ₂) - l*ω₁^2*sin(θ₂));
    b_ss = [b1; b2];

    α = A_ss\b_ss;
    du[1] = ω₁
    du[2] = α[1]
    du[3] = ω₂
    du[4] = α[2]

end

function footstrike(t, z, walker)
    z=z[1]
    θ₁_n = z[1]
    θ₂_n = z[3]
    ω₁_n = z[2]
    ω₂_n = z[4]

    θ₁ = θ₁_n + θ₂_n
    θ₂ = -θ₂_n

    M = walker.M; m = walker.m; I = walker.I
    l = walker.l; c=walker.c

    J11 = 1;
    J12 = 0;
    J13 = l*(cos(θ₁_n + θ₂_n) - cos(θ₁_n));
    J14 = l*cos(θ₁_n + θ₂_n);
    J21 = 0;
    J22 = 1;
    J23 = l*(sin(θ₁_n + θ₂_n) - sin(θ₁_n));
    J24 = l*sin(θ₁_n + θ₂_n);
    J = [J11 J12 J13 J14; J21 J22 J23 J24];

    A11 = M + 2*m;
    A12 = 0;
    A13 = (m*(2*c*cos(θ₁_n + θ₂_n) - 2*l*cos(θ₁_n)))/2 + m*cos(θ₁_n)*(c - l) - M*l*cos(θ₁_n);
    A14 = c*m*cos(θ₁_n + θ₂_n);
    A21 = 0;
    A22 = M + 2*m;
    A23 = (m*(2*c*sin(θ₁_n + θ₂_n) - 2*l*sin(θ₁_n)))/2 - M*l*sin(θ₁_n) + m*sin(θ₁_n)*(c - l);
    A24 = c*m*sin(θ₁_n + θ₂_n);
    A31 = (m*(2*c*cos(θ₁_n + θ₂_n) - 2*l*cos(θ₁_n)))/2 + m*cos(θ₁_n)*(c - l) - M*l*cos(θ₁_n);
    A32 = (m*(2*c*sin(θ₁_n + θ₂_n) - 2*l*sin(θ₁_n)))/2 - M*l*sin(θ₁_n) + m*sin(θ₁_n)*(c - l);
    A33 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(θ₂_n);
    A34 = I + c^2*m - c*l*m*cos(θ₂_n);
    A41 = c*m*cos(θ₁_n + θ₂_n);
    A42 = c*m*sin(θ₁_n + θ₂_n);
    A43 = I + c^2*m - c*l*m*cos(θ₂_n);
    A44 = I + c^2*m;
    A_n_hs = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];

    X_n_hs = [0 0 ω₁_n ω₂_n]';
    b_hs = [A_n_hs*X_n_hs; 0; 0];
    A_hs = [A_n_hs -J' ; J zeros((2,2))];
    X_hs = A_hs\b_hs;
    ω = zeros(2)
    ω[1] = X_hs[3]+X_hs[4]; ω[2] = -X_hs[4];

    zplus = [θ₁ ω[1] θ₂ ω[2]]
end



function runsteps(z0, walker, steps...)
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

    for i=1:steps[1]
        tspan = (t0, t0+dt)
        prob = ODEProblem(single_stance!, z0, tspan, walker)
        sol = DifferentialEquations.solve(prob, Tsit5(), callback=collision_cb)
        zplus = footstrike(sol.t[end], sol.u[end, :], walker)

        z0 = zplus
        t0 = sol.t[end]
        z_temp = reduce(vcat, sol.u)
        t_temp = sol.t
        xh_temp = xh_start .+ l*sin(z_temp[1,1]).-l*sin.(z_temp[:,1])
        yh_temp = l*cos.(z_temp[:,1])
        t_ode = vcat(t_ode, t_temp[2:end])
        z_ode = vcat(z_ode, hcat(vcat(z_temp[2:end-1, :],zplus), xh_temp[2:end], yh_temp[2:end]))

        xh_start = xh_temp[end]

    end
    z = zplus[1:4]
    if flag == 1
        return (z_ode, t_ode)
    else
        return z
    end
end


function animate(ts, zs, walker, steps, fps)
    zs_tp = hcat(zs[:,1], zs[:,3], zs[:,5], zs[:,6])
    num_var = 4
    total_frames = Int(round(ts[end]*fps))
    t = 0:ts[end]/total_frames:ts[end]
    z = zeros((total_frames+1,num_var))
    for i = 1:num_var
        interp = LinearInterpolation(ts, zs_tp[:,i])
        z[:,i] = interp.(t)
    end

    l = walker.l
    c = walker.c

    mm = size(z, 1)
    min_xh = min(z[:,3]...); max_xh = max(z[:,3]...) 

    window_xmin = -1*l; window_xmax = 1*l
    window_ymin = -0.1; window_ymax = 1.1*l
    anim = Plots.Animation()
    rampref = [min_xh-1 max_xh+1; 0 0]

    plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

    for i=1:mm
        plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

        θ₁ = z[i,1]; θ₂ = z[i,2]
        xh = z[i,3]; yh = z[i,4]

        hinge = [xh; yh]
        stance_foot = [xh+l*sin(θ₁); yh-l*cos(θ₁)]
        stance_leg_com = [xh+c*sin(θ₁); yh-c*cos(θ₁)]
        swing_foot = [xh+l*sin(θ₁ +θ₂); yh-l*cos(θ₁+θ₂)]
        swing_leg_com = [xh+c*sin(θ₁+θ₂); yh-c*cos(θ₁+θ₂)]

        plot!([hinge[1]], [hinge[2]], markershape=:circle, markercolor=:black, markersize=15)
        plot!([stance_leg_com[1]], [stance_leg_com[2]],  markershape=:circle, markercolor=:black, markersize=10)
        plot!([swing_leg_com[1]], [swing_leg_com[2]],  markershape=:circle, markercolor=:black, markersize=10)

        plot!([hinge[1], stance_foot[1]], [hinge[2], stance_foot[2]], color=:red, linewidth=2)
        plot!([hinge[1], swing_foot[1]], [hinge[2], swing_foot[2]], color=:blue, linewidth=2)

        sleep(0.01)
        frame(anim)
    end
    gif(anim, "media/walker.gif", fps=15) 

end


walker = Walker(1.0, 0.5, 0.02, 1.0, 0.5, 1.0, 0.01)
q1 = 0.2; u1 = -0.25;
q2 = -0.4; u2 = 0.2;03797846807373

z0 = [q1 u1 q2 u2];

steps = 8;
fps = 20;
zstar = [0.162597833780035  -0.231869638058927  -0.325195667560070   0.03797846807373]
z, t = runsteps(zstar,walker,steps)
animate(t,z,walker, steps, fps)