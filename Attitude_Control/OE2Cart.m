function Sat_vec_Cart = OE2Cart(Sat_vec_OE,u)
    %Orbital Elements
    a = Sat_vec_OE(1,:);
    ecc = Sat_vec_OE(2,:);
    inc = Sat_vec_OE(3,:);
    w = Sat_vec_OE(4,:);
    Omega = Sat_vec_OE(5,:);
    f = Sat_vec_OE(6,:);
    
    % Helping calculations
    p = a.*(1 - ecc.^2);
    r = p./(1 + ecc.*cos(f));
    x_1 = r.*cos(f);
    y_1 = r.*sin(f);
    Vx_1 = -(u./p).^(1/2) .* sin(f);
    Vy_1 = (u./p).^(1/2) .* (ecc + cos(f));

    % Cartesian
    x = (cos(Omega) .* cos(w) - sin(Omega) .* sin(w) .* cos(inc)) .* x_1 + (-cos(Omega) .* sin(w) - sin(Omega) .* cos(w) .* cos(inc)) .* y_1;
    y = (sin(Omega) .* cos(w) + cos(Omega) .* sin(w) .* cos(inc)) .* x_1 + (-sin(Omega) .* sin(w) + cos(Omega) .* cos(w) .* cos(inc)) .* y_1;
    z = (sin(w) .* sin(inc)) .* x_1 + (cos(w) .* sin(inc)) .* y_1;
    X = (cos(Omega) .* cos(w) - sin(Omega) .* sin(w) .* cos(inc)) .* Vx_1 + (-cos(Omega) .* sin(w) - sin(Omega) .* cos(w) .* cos(inc)) .* Vy_1;
    Y = (sin(Omega) .* cos(w) + cos(Omega) .* sin(w) .* cos(inc)) .* Vx_1 + (-sin(Omega) .* sin(w) + cos(Omega) .* cos(w) .* cos(inc)) .* Vy_1;
    Z = (sin(w) .* sin(inc)) .* Vx_1 + (cos(w) .* sin(inc)) .* Vy_1;
    
    Sat_vec_Cart = [x; y; z; X; Y; Z]; 
end