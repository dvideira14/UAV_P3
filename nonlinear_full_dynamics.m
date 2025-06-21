function dx = nonlinear_full_dynamics(~, x, K, m, D, g, J)
    % Estados
    p = x(1:3);           % posição
    v = x(4:6);           % velocidade
    lambda = x(7:9);      % ângulos de Euler: [phi; theta; psi]
    omega  = x(10:12);    % velocidades angulares: [p; q; r]

    phi = lambda(1);
    theta = lambda(2);
    psi = lambda(3);

    % Matriz de rotação (zYX)
    R = rotation_matrix(phi, theta, psi);

    % Controlador LQR: calcula aceleração desejada (referencial inercial)
    ua = -K * [p; v];

    % Saturar aceleração vertical mínima para garantir thrust positivo
    ua(3) = max(ua(3), -g + 0.1);

    % Entradas físicas correspondentes
    theta_cmd = max(min(-ua(1)/g, pi/6), -pi/6);   % pitch
    phi_cmd   = max(min( ua(2)/g, pi/6), -pi/6);   % roll
    T_cmd     = m * ua(3);                  % thrust

    % Força de propulsão (projetada para o referencial inercial)
    fp = R * [0; 0; T_cmd];

    % Força gravitacional (projetada no corpo)
    fg = m * g * R' * [0; 0; 1];

    % Arrasto linear
    fa = -D * v;

    % Força total
    total_force = fa + fg + fp;

    % Torque proporcional (para seguir [phi_cmd; theta_cmd])
    kp_att = 10;
    kd_att = 0.5;
    torque = kp_att * ([phi_cmd; theta_cmd; 0] - lambda) - kd_att * omega;

    % Matrizes auxiliares
    Q_lambda = euler_Q_matrix(phi, theta);

    % Dinâmicas
    p_dot = R * v;
    v_dot = (1/m) * total_force;
    lambda_dot = Q_lambda * omega;
    omega_dot = -inv(J) * cross(omega, J * omega) + inv(J) * torque;

    % Estado derivado completo
    dx = [p_dot; v_dot; lambda_dot; omega_dot];
end
