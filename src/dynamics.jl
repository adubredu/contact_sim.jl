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

    if walker.use_controller
        (U, θ₂ᵣ, θ₂̇ᵣ, θ₂̈ᵣ) = controller(t, z, walker)
    else
        U = 0.0
    end

    B = [0; 1] 
    α = A_ss\(b_ss+B*U);
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

    return zplus
end