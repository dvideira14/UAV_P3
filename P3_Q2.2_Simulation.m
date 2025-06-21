%% Parâmetros físicos e de controlo
m = 0.035;                      % massa [kg]
g = 9.81;                       % gravidade [m/s^2]
D = diag([0.108, 0.108, 0.108]);   % matriz de arrasto
Kp = diag([6, 6, 6]);           % ganhos proporcionais ajustados
Kv = diag([6, 6, 8]);           % ganhos derivativos ajustados

dt = 0.005;                     % passo de tempo [s]
T = 10;                         % tempo total [s]
N = T / dt;                     % número de passos de simulação

%% Inicialização de vetores
p = zeros(3, N);   % posição real
v = zeros(3, N);   % velocidade real
pd = zeros(3, N);  % posição desejada
vd = zeros(3, N);  % velocidade desejada
ad = zeros(3, N);  % aceleração desejada

%% Trajetória desejada: subida até 2 m, depois curva circular
t_trans = 3;          % tempo de transição entre subida e curva [s]
h_final = 2.0;        % altura final da subida [m]
r = 0.5;              % raio da curva [m]
omega = 0.6;          % velocidade angular [rad/s]

for k = 1:N
    t = (k-1)*dt;
    
    if t < t_trans
        % Fase 1: subida vertical
        pd(:,k) = [0; 0; h_final * (t / t_trans)];
        vd(:,k) = [0; 0; h_final / t_trans];
        ad(:,k) = [0; 0; 0];  % subida constante
    else
        % Fase 2: curva circular no plano XY
        t2 = t - t_trans;
        pd(:,k) = [r * sin(omega * t2);
                   r * cos(omega * t2);
                   h_final];
        
        vd(:,k) = [r * omega * cos(omega * t2);
                  -r * omega * sin(omega * t2);
                   0];
        
        ad(:,k) = [-r * omega^2 * sin(omega * t2);
                   -r * omega^2 * cos(omega * t2);
                    -g];
    end
end

%% Condições iniciais
p(:,1) = pd(:,1);
v(:,1) = [0;0;0]; % começa parado

%% Simulação com controlador
for k = 1:N-1
    e_p = pd(:,k) - p(:,k);
    e_v = vd(:,k) - v(:,k);

    ua = ad(:,k) + Kv * e_v + Kp * e_p + [0; 0; g];

    % Saturação separada por eixo
    ua(3) = max(min(ua(3), 3), -3);
    ua(1:2) = max(min(ua(1:2), 4), -4);

    a = ua - (1/m) * D * v(:,k);
    v(:,k+1) = v(:,k) + a * dt;
    p(:,k+1) = p(:,k) + v(:,k) * dt;
end

%% Gráfico da trajetória 3D
figure;
plot3(p(1,:), p(2,:), p(3,:), 'b', pd(1,:), pd(2,:), pd(3,:), 'r--', 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Trajetória real', 'Trajetória desejada');
title('Seguimento 3D: Subida + Curva Circular');
grid on; axis equal;

%% Gráfico da altura ao longo do tempo
figure;
tvec = (0:N-1) * dt;
plot(tvec, p(3,:), 'b', tvec, pd(3,:), 'r--', 'LineWidth', 1.5);
xlabel('Tempo [s]'); ylabel('Altura Z [m]');
legend('Z real', 'Z desejado');
title('Altura ao longo do tempo');

%% Erro RMSE
rmse_p = sqrt(mean(vecnorm(p - pd).^2));
rmse_v = sqrt(mean(vecnorm(v - vd).^2));
fprintf('RMSE da posição: %.4f m\n', rmse_p);
fprintf('RMSE da velocidade: %.4f m/s\n', rmse_v);
