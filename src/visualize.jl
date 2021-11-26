function render_block_trajectory(traj, physics) 
    
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