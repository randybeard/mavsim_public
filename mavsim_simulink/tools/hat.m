function omega_hat = hat(omega)
    %vector to skew symmetric matrix associated with cross product
    a = omega(1)
    b = omega(2)
    c = omega(3)

    omega_hat = [0, -c, b;
                 c, 0, -a;
                 -b, a, 0];
end

