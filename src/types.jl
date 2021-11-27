mutable struct Object 
l::Float64
μ::Float64
ϵ::Float64 
m::Float64
R::Float64
M::Matrix{Float64}
M_inv::Matrix{Float64}

    function Object(l,μ,ϵ,m,R,M)
        new(l,μ,ϵ,m,R,M,inv(M))
    end
end

mutable struct Physics 
    dt::Float64
    g::Vector{Float64}
    Δ::Float64
    T::Int 
    
    function Physics(dt,g,Δ,T)
        new(dt,g,Δ,T)
    end
end

mutable struct Walker{T}
    M::T
    m::T
    I::T
    l::T
    c::T
    g::T
    γ::T
    use_controller::Bool
    tf::T 
    Kp::T 
    Θf::T
    Θ₂::T
    Θ₂̇ ::T 
    t₀::T
end