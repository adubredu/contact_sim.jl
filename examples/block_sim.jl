using Revise
using contact_walk
using Plots

L = 0.4 
MU = 0.3
EP = 0.5
dt = 0.01
m = 0.3
g = [0., -9.81, 0.]
rg = (1.0/12.) * (2 * L * L)

M = [[m 0. 0.]; 
     [0. m 0.];
     [0. 0. m * rg]]

Mi = [[1.0/m  0.  0.];
      [0. 1.0/m 0.];
      [0. 0. 1.0/(m*rg)]]
Δ = 0.001
T = 150

function get_contacts(q)
    xₜ = q[1]
    yₜ = q[2]
    θ  = q[3]

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


function form_lcp(J, v)
    Jₙ = J[:,2]
    Jₜ = J[:,1]

    V = zeros((4,4))
    V[1,1] = Jₙ'* Mi * (dt*Jₙ)
    V[1,2] = -Jₙ'* Mi * (dt*Jₜ)
    V[1,3] = Jₙ'* Mi * (dt*Jₜ)
    V[2,1] = -Jₜ'* Mi * (dt*Jₙ)
    V[2,2] = Jₜ'* Mi * (dt*Jₜ)
    V[2,3] = -Jₜ'* Mi * (dt*Jₜ)
    V[2,4] = 1.
    V[3,1] = Jₜ'* Mi * (dt*Jₙ)
    V[3,2] = -Jₜ'* Mi * (dt*Jₜ)
    V[3,3] = Jₜ'* Mi * (dt*Jₜ)
    V[3,4] = 1.
    V[4,1] = MU
    V[4,2] = -1.
    V[4,3] = -1.
    fₑ = M*g 
    p = zeros(4) 
    p[1] = Jₙ' * ((1+EP)*v + dt*Mi*fₑ)
    p[2] = -Jₜ' * (v + dt*Mi*fₑ)
    p[3] = Jₜ' * (v + dt*Mi*fₑ)

    return V, p
end


function step(q, v)
    J, ϕ = get_contacts(q)
    fₑ = M*g 
    Jₙ = J[:,2]
    Jₜ = J[:,1]
    qₚ = [0., Δ, 0.] 

    if ϕ >= Δ
        v¹ = v + dt*Mi*fₑ
        q¹ = q + dt*v¹
    else 
        V, p = form_lcp(J, v) 
        fᵧ = solve_lcp(V, p)
        fₙ = fᵧ[1]; fₜ₁ = fᵧ[2]; fₜ₂ = fᵧ[3]
        v¹ = v + dt*Mi*(fₑ + fₙ*Jₙ - fₜ₁*Jₜ + fₜ₂*Jₜ)
        q¹ = q + dt*v¹+qₚ
    end
    return q¹, v¹
end

function render(traj) 
    
    anim = Animation()

    for t = 1:T
        plot([-2, 2], [-0.05, -0.05], aspect_ratio=:equal, legend=false, linecolor=:black,
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
    gif(anim, "block_sim.gif", fps=15)

end 

function simulate(q₀, v₀)
    q = [q₀]
    v = [v₀]

    for i=1:T-1
        q¹, v¹ = step(q₀, v₀)
        q₀ = q¹
        v₀ = v¹
        push!(q, q¹)
        push!(v, v¹)
    end

    # q = q'
    # v = v'
    # q = reshape(q, (3,T))
    # v = reshape(v, (3,T))
    q = hcat(q...)
    v = hcat(v...)

    return q, v 
end 

function main()
    q₀ = [0.0, 1.5, π/180.0*30.]
    v₀ = [0.0, -0.2, 0.0]

    q, v = simulate(q₀, v₀)
    
    return q, v
end

q, v = main()
render(q)