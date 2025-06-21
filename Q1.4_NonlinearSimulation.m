% Parâmetros
D = 0.108;
m = 0.035;
g = 9.81;
J = diag([8.06e-6, 9.71e-6, 1.41e-6]);

% Modelo linear + LQR
A = [zeros(3), eye(3); zeros(3), -D/m * eye(3)];
B = [zeros(3); eye(3)];
Q = diag([10, 10, 10, 1, 1, 1]);
R = eye(3) * 0.1;
K = lqr(A, B, Q, R);

% Condição inicial: posição + velocidade + ângulos + omega
x0 = [0.5; -0.5; 0.3; 0; 0; 0; 0; 0; 0; 0; 0; 0];
tspan = [0 10];

% Simulação
[t, x] = ode45(@(t, x) nonlinear_full_dynamics(t, x, K, m, D, g, J), tspan, x0);

% Calcular entradas de controlo ao longo do tempo
u = zeros(length(t), 3); % [T, φ, θ]
for i = 1:length(t)
    p = x(i,1:3).';
    v = x(i,4:6).';
    ua = -K * [p; v];
    theta = -ua(1)/g;
    phi   =  ua(2)/g;
    T     = m * ua(3);
    u(i,:) = [T, phi, theta];
end

% Gráficos
figure;
subplot(3,1,1); plot(t, x(:,1), 'r', t, x(:,2), 'g', t, x(:,3), 'b');
ylabel('Position (m)'); legend('x','y','z'); title('Nonlinear Model LQR Response'); grid on;

subplot(3,1,2); plot(t, x(:,4), 'r--', t, x(:,5), 'g--', t, x(:,6), 'b--');
ylabel('Velocity (m/s)'); legend('v_x','v_y','v_z'); grid on;

subplot(3,1,3); plot(t, u(:,1), 'r', t, u(:,2), 'g', t, u(:,3), 'b');
xlabel('Tempo (s)'); ylabel('Control Inputs'); legend('T','phi','theta'); grid on;
