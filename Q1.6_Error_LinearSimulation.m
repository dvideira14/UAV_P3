% Parâmetros físicos
D = 0.108;     
m = 0.035;      
g = 9.81;

% Modelo linear
A = [zeros(3), eye(3); zeros(3), -D/m * eye(3)];
B = [zeros(3); eye(3)];

% Ganho LQR
Q = diag([10 10 10 1 1 1]);
R = eye(3);
K = lqr(A, B, Q, R);

% Referência desejada
xref = @(t) [0.5; -0.5; 0.3; 0; 0; 0];  % posição desejada fixa
xdot_ref = @(t) zeros(6,1);

% Correção: não inverter B, usa blocos diretos
uref = @(t, xbar) - A(4:6,:) * xbar;  

%% Simulação
p0 = [0; 0; 0];
v0 = [0; 0; 0];
x0 = [p0; v0];
tspan = [0 10];

f = @(t,x) A*x + B*( uref(t,xref(t)) - K*(x - xref(t)) );

[t, x] = ode45(f, tspan, x0);

xref_vec = repmat(xref(0)', length(t), 1);

u = zeros(length(t), 3);
for i = 1:length(t)
    uref_i = uref(t(i), xref(t(i)));
    u(i,:) = ( uref_i - K*( x(i,:)' - xref(t(i)) ) )';
end

%% Gráficos
figure;

% Subplot 1 - Posição real vs referência
subplot(4,1,1);
plot(t, x(:,1), 'r', t, x(:,2), 'g', t, x(:,3), 'b', 'LineWidth', 1.5); hold on;
plot(t, xref_vec(:,1), 'r--', t, xref_vec(:,2), 'g--', t, xref_vec(:,3), 'b--', 'LineWidth', 1.2);
ylabel('Position (m)');
title('LQR Tracking - Position vs Reference');
legend('x','y','z','x_{ref}','y_{ref}','z_{ref}'); grid on;

% Subplot 2 - Velocidade real
subplot(4,1,2);
plot(t, x(:,4), 'r--', t, x(:,5), 'g--', t, x(:,6), 'b--', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
legend('v_x','v_y','v_z'); grid on;

% Subplot 3 - Erro de posição
subplot(4,1,3);
plot(t, x(:,1)-xref_vec(:,1), 'r', t, x(:,2)-xref_vec(:,2), 'g', t, x(:,3)-xref_vec(:,3), 'b', 'LineWidth', 1.5);
ylabel('Position Error (m)');
legend('e_x','e_y','e_z'); grid on;

% Subplot 4 - Entrada de controlo
subplot(4,1,4);
plot(t, u(:,1), 'r', t, u(:,2), 'g', t, u(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (m/s^2)');
legend('u_x','u_y','u_z'); grid on;
