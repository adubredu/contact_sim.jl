function render_block_trajectory(traj, body, physics) 
    
    anim = Animation()
    T = physics.T
    for t = 1:T
        plot([-2, 2], [-0.015, -0.015], aspect_ratio=:equal, legend=false, linecolor=:black,
            linewidth=4, xlims=[-2,2], ylims=[-0.1,2])
        local_corners = [[-0.2 -0.2 0.2 0.2 -0.2 ];
                         [-0.2 0.2 0.2 -0.2 -0.2];
                         [1. 1. 1. 1. 1.]]

        H = [[cos(traj[3, t]) -sin(traj[3, t]) traj[1, t]];
             [sin(traj[3, t]) cos(traj[3,t]) traj[2, t]];
             [0. 0. 1]]
        world_corners = H * local_corners

        plot!([world_corners[1,:]], [world_corners[2,:]], color=:blue, linewidth=2)
        sleep(0.01)
        frame(anim)
    end 
    gif(anim, "media/block_sim.gif", fps=15)

end

function render_block_trajectory(traj, body, physics, use_makie::Bool) 
    T = physics.T
    lc = [  [-0.2 -0.2 0.2 0.2 -0.2 ];
            [-0.2 0.2 0.2 -0.2 -0.2];
            [1. 1. 1. 1. 1.]]
    world_corners = Observable([Point2f0(-0.2, -0.2), Point2f0(-0.2, 0.2), 
                                Point2f0(0.2, 0.2), Point2f0(0.2, -0.2),
                                Point2f0(-0.2, -0.2)])
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-2,2,-0.1,2)) 
    lines!(ax, [-2,2], [-0.015, -0.015], color=:black, linewidth=4) 
    lines!(ax, world_corners, color=:blue, linewidth=2)

    function animstep!(t) 
            H = [[cos(traj[3, t]) -sin(traj[3, t]) traj[1, t]];
                    [sin(traj[3, t]) cos(traj[3,t]) traj[2, t]];
                    [0. 0. 1]]
            wc = H * lc
            world_corners[] = [Point2f0(x[1],x[2]) for x in [wc[:,i] for i in 1:size(wc,2)]]
            sleep(0.01) 
    end
    record(fig, "media/block_sim_makie.gif", 1:T; framerate=60) do t
        animstep!(t)
    end   
end 

function render_ball_trajectory(traj, body, physics) 
    
    anim = Animation()
    T = physics.T
    r = body.l/2.
    circle(x,y,r) = (x .+ r*sin.(LinRange(0, 2*π, 500)), y .+ r*cos.(LinRange(0, 2*π, 500)))
    for t = 1:T
        plot([-2, 2], [-0.015, -0.015], aspect_ratio=:equal,
            legend=false, linecolor=:black,
            linewidth=4, xlims=[-2,2], ylims=[-0.1,2])
        
        local_center = [0.0; 0.0; 1.0]

        H = [[cos(traj[3, t]) -sin(traj[3, t]) traj[1, t]];
             [sin(traj[3, t]) cos(traj[3,t]) traj[2, t]];
             [0. 0. 1]]
        world_center = H * local_center
        x = world_center[1]
        y = world_center[2]

        # plot!(circle(x,y,r), seriestype=[:shape], color=:blue, linewidth=2)
        plot!(circle(x,y,r), color=:blue, linewidth=2)
        sleep(0.01)
        frame(anim)
    end 
    gif(anim, "media/ball_sim.gif", fps=15)

end 

function render_ball_trajectory(traj, body, physics, use_makie::Bool) 
    T = physics.T
    r = body.l/2.
    lc = [0.0; 0.0; 1.0]
    center = Observable(Point2f0(0.,0.)) 

    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-2,2,-0.1,2)) 
    lines!(ax, [-2,2], [-0.015, -0.015], color=:black, linewidth=4) 
    Makie.scatter!(ax, center; marker=:circle,  linewidth=4, strokecolor=:blue, markersize= 80)

    function animstep!(t) 
            H = [[cos(traj[3, t]) -sin(traj[3, t]) traj[1, t]];
                    [sin(traj[3, t]) cos(traj[3,t]) traj[2, t]];
                    [0. 0. 1]]
            wc = H * lc
            center[] = Point2f0(wc[1], wc[2])
            sleep(0.01) 
    end
    record(fig, "media/ball_sim_makie.gif", 1:T; framerate=15) do t
        animstep!(t)
    end   
end 


function render_walking_trajectory(ts, zs, walker, steps, fps)
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
    rampref = [min_xh-1 max_xh+1; -0.015 -0.015]

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

        plot!([hinge[1]], [hinge[2]], markershape=:square, markercolor=:black, markersize=15) 

        plot!([hinge[1], stance_foot[1]], [hinge[2], stance_foot[2]], color=:red, linewidth=2)
        plot!([hinge[1], swing_foot[1]], [hinge[2], swing_foot[2]], color=:blue, linewidth=2)
        
        fl = 0.05
        stance_feet = [[stance_foot[1] - (fl/2.)*cos(θ₁), stance_foot[2] - (fl/2.)*sin(θ₁)], [stance_foot[1] + (fl/2.)*cos(θ₁), stance_foot[2] + (fl/2.)*sin(θ₁)]]

        swing_feet = [[swing_foot[1] - (fl/2.)*cos(θ₁+θ₂), swing_foot[2] - (fl/2.)*sin(θ₁+θ₂)], [swing_foot[1] + (fl/2.)*cos(θ₁+θ₂), swing_foot[2] + (fl/2.)*sin(θ₁+θ₂)]]

        plot!([stance_feet[1][1], stance_feet[2][1]], [stance_feet[1][2], stance_feet[2][2]], color=:red, linewidth=4)

        plot!([swing_feet[1][1], swing_feet[2][1]], [swing_feet[1][2], swing_feet[2][2]], color=:blue, linewidth=4) 

        sleep(0.01)
        frame(anim)
    end
    if walker.use_controller
        gif(anim, "media/walker_sim_ctrl.gif", fps=15)
    else
        gif(anim, "media/walker_sim_no_ctrl.gif", fps=15)
    end 
end

function render_walking_trajectory(ts, zs, walker, steps, fps, use_makie::Bool)
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
    fl = 0.05

    T = size(z, 1)
    min_xh = min(z[:,3]...); max_xh = max(z[:,3]...) 

    window_xmin = -1*l; window_xmax = 1*l
    window_ymin = -0.1; window_ymax = 1.1*l 
    rampref = [min_xh-1 max_xh+1; -0.015 -0.015]

    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(window_xmin,window_xmax,window_ymin,window_ymax)) 
    lines!(ax, rampref[1,:], rampref[2,:], color=:black, linewidth=10) 
    
    hinge = Observable(Point2f0(0.0, 0.0))
    stance_foot = Observable([Point2f0(0.0, 0.0),Point2f0(0.0, 0.0)])
    swing_foot = Observable([Point2f0(0.0, 0.0),Point2f0(0.0, 0.0)])
    stance_feet = Observable([Point2f0(0.0, 0.0),Point2f0(0.0, 0.0)])
    swing_feet = Observable([Point2f0(0.0, 0.0),Point2f0(0.0, 0.0)])

    function animstep!(i) 
        θ₁ = z[i,1]; θ₂ = z[i,2]
        xh = z[i,3]; yh = z[i,4]
        hinge[] = Point2f0(xh, yh)
        stance_foot[] = [Point2f0(xh, yh), Point2f0(xh+l*sin(θ₁), yh-l*cos(θ₁))]
        swing_foot[] = [Point2f0(xh, yh), Point2f0(xh+l*sin(θ₁ +θ₂), yh-l*cos(θ₁+θ₂))] 

        stance_feet[] = [Point2f0(xh+l*sin(θ₁) - (fl/2.)*cos(θ₁), yh-l*cos(θ₁) - (fl/2.)*sin(θ₁)), Point2f0(xh+l*sin(θ₁) + (fl/2.)*cos(θ₁), yh-l*cos(θ₁) + (fl/2.)*sin(θ₁))]
        swing_feet[] = [Point2f0(xh+l*sin(θ₁ +θ₂) - (fl/2.)*cos(θ₁+θ₂), yh-l*cos(θ₁+θ₂) - (fl/2.)*sin(θ₁+θ₂)), Point2f0(xh+l*sin(θ₁ +θ₂) + (fl/2.)*cos(θ₁+θ₂), yh-l*cos(θ₁+θ₂) + (fl/2.)*sin(θ₁+θ₂))] 

        Makie.scatter!(ax, hinge; marker=:rect,  linewidth=4, strokecolor=:black, color=:black, markersize= 80)
        lines!(ax, stance_foot, color=:red, linewidth=6)
        lines!(ax, swing_foot, color=:blue, linewidth=6)
        lines!(ax, stance_feet, color=:red, linewidth=6)
        lines!(ax, swing_feet, color=:blue, linewidth=6)
        sleep(0.001) 
    end

    record(fig, "media/walker_sim_makie.gif", 1:T; framerate=60) do t
        for j=1:2
            animstep!(t)
        end
    end


end 


function render_kick_trajectory!(ts, zs, walker, steps, fps;
    gif_name="kick_ball.gif")
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
    rampref = [min_xh-1 max_xh+1; -0.015 -0.015]

    # plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

    for i=1:mm
        # plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

        θ₁ = z[i,1]; θ₂ = z[i,2]
        xh = z[i,3]; yh = z[i,4]

        hinge = [xh; yh]
        stance_foot = [xh+l*sin(θ₁); yh-l*cos(θ₁)]
        stance_leg_com = [xh+c*sin(θ₁); yh-c*cos(θ₁)]
        swing_foot = [xh+l*sin(θ₁ +θ₂); yh-l*cos(θ₁+θ₂)]
        swing_leg_com = [xh+c*sin(θ₁+θ₂); yh-c*cos(θ₁+θ₂)]

        plot!([hinge[1]], [hinge[2]], markershape=:square, markercolor=:black, markersize=15) 

        plot!([hinge[1], stance_foot[1]], [hinge[2], stance_foot[2]], color=:red, linewidth=2)
        plot!([hinge[1], swing_foot[1]], [hinge[2], swing_foot[2]], color=:blue, linewidth=2)

        fl = 0.05
        stance_feet = [[stance_foot[1] - (fl/2.)*cos(θ₁), stance_foot[2] - (fl/2.)*sin(θ₁)], [stance_foot[1] + (fl/2.)*cos(θ₁), stance_foot[2] + (fl/2.)*sin(θ₁)]]

        swing_feet = [[swing_foot[1] - (fl/2.)*cos(θ₁+θ₂), swing_foot[2] - (fl/2.)*sin(θ₁+θ₂)], [swing_foot[1] + (fl/2.)*cos(θ₁+θ₂), swing_foot[2] + (fl/2.)*sin(θ₁+θ₂)]]

        plot!([stance_feet[1][1], stance_feet[2][1]], [stance_feet[1][2], stance_feet[2][2]], color=:red, linewidth=4)

        plot!([swing_feet[1][1], swing_feet[2][1]], [swing_feet[1][2], swing_feet[2][2]], color=:blue, linewidth=4) 

        sleep(0.01)
        frame(anim)
    end 
    gif(anim, gif_name, fps=15)
end