% Parâmetros físicos
D = 0.108;
m = 0.035;

% Modelo linear
A = [zeros(3), eye(3); zeros(3), -D/m * eye(3)];
B = [zeros(3); eye(3)];

% Pesos LQR (com controlo eficaz em z)
Q = diag([10, 10, 10, 1, 1, 1]);
R = eye(3)*0.1;

% Ganho LQR
K = lqr(A, B, Q, R);

% Condições iniciais
p0 = [0.5; -0.5; 0.3];    % posição inicial (m)
v0 = [0; 0; 0];           % velocidade inicial (m/s)
x0 = [p0; v0];

% Intervalo de simulação
tspan = [0 10];

% Simulação do modelo linear com feedback LQR
f = @(t, x) A*x + B*(-K*x);
[t, x] = ode45(f, tspan, x0);

% Cálculo da entrada de controlo aplicada
u = zeros(length(t), 3);
for i = 1:length(t)
    u(i,:) = (-K * x(i,:).').';
end

% Gráficos em subplots numa única figura
figure;

% Subplot 1 - Posição
subplot(3,1,1)
plot(t, x(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'g', 'LineWidth', 1.5);
plot(t, x(:,3), 'b', 'LineWidth', 1.5);
ylabel('Position (m)');
title('LQR Closed-Loop Response');
legend('x','y','z'); grid on;

% Subplot 2 - Velocidade
subplot(3,1,2)
plot(t, x(:,4), 'r--', 'LineWidth', 1.5); hold on;
plot(t, x(:,5), 'g--', 'LineWidth', 1.5);
plot(t, x(:,6), 'b--', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
legend('v_x','v_y','v_z'); grid on;

% Subplot 3 - Entrada de controlo
subplot(3,1,3)
plot(t, u(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, u(:,2), 'g', 'LineWidth', 1.5);
plot(t, u(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Control Input (m/s^2)');
legend('u_x','u_y','u_z'); grid on;
