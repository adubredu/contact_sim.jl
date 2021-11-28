function controller(t, z, walker)
    tt = t - walker.t₀
    θ₁ = z[1]
    θ₂ = z[3]
    ω₁ = z[2]
    ω₂ = z[4]
    M = walker.M; m = walker.m; I=walker.I; l = walker.l; c = walker.c;
    g = walker.g; γ = walker.γ

    t₀ = 0 
    tf = walker.tf
    q₀ = walker.Θ₂
    qf = walker.Θf 
    q₀̇ = walker.Θ₂̇
    qḟ = 0.0

    AA = [  1 t₀ t₀^2   t₀^3    t₀^4    t₀^5;
            1 tf tf^2   tf^3    tf^4    tf^5; 
            0  1 2*t₀ 3*t₀^2  4*t₀^3  5*t₀^4;
            0  1 2*tf 3*tf^2  4*tf^3  5*tf^4;
            0  0    2   6*t₀ 12*t₀^2 20*t₀^3;
            0  0    2   6*tf 12*tf^2 20*tf^3]
    bb = [q₀; qf; q₀̇ ; qḟ; 0.; 0.]
    xx = AA\bb
    a0 = xx[1]; a1 = xx[2]; a2 = xx[3]; 
    a3 = xx[4]; a4 = xx[5]; a5 = xx[6];
    if (tt>tf)
        tt = tf
    end 

    θ₂ᵣ = a5*tt^5 + a4*tt^4 + a3*tt^3 + a2*tt^2 + a1*tt + a0
    θ₂̇ᵣ = 5*a5*tt^4 + 4*a4*tt^3 + 3*a3*tt^2 + 2*a2*tt + a1
    θ₂̈ᵣ =   20*a5*tt^3 + 12*a4*tt^2 + 6*a3*tt + 2*a2

    if t >= 4.5
        # println("t = ",t)
        θ₂ᵣ = 2*θ₂ᵣ
    end

    #partial feedback linearization
    M11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(θ₂)
    M12 = I + c^2*m - c*l*m*cos(θ₂)
    M21 = I + c^2*m - c*l*m*cos(θ₂)
    M22 = I + c^2*m
    Ms = [M11 M12; M21 M22]
    
    N1 = c*g*m*sin(θ₁ - γ + θ₂) + M*g*l*sin(γ - θ₁) - c*g*m*sin(γ - θ₁) + 2*g*l*m*sin(γ - θ₁) + c*l*m*ω₂^2*sin(θ₂) + 2*c*l*m*ω₁*ω₂*sin(θ₂)
    N2 = c*m*(g*sin(θ₁ - γ + θ₂) - l*ω₁^2*sin(θ₂))
    Ns = [N1; N2]

    Kp = walker.Kp
    Kd = 2*sqrt(Kp)
    B = [0; 1]
    Sc = [0 1]
    e = θ₂-θ₂ᵣ
    ė = ω₂ - θ₂̇ᵣ;
    v = [θ₂̈ᵣ - Kp*e - Kd*ė]
    Minv = Ms\[1. 0.; 0. 1.] 
    u = (Sc*Minv*B)\(v+Sc*Minv*Ns)

    

    return (u, θ₂ᵣ, θ₂̇ᵣ, θ₂̈ᵣ)

end
