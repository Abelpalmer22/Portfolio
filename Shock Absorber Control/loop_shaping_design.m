%% 03_loop_shaping_design.m
% Classical loop-shaping design for quarter-car active suspension
% Plant input: actuator force u
% Plant output: body displacement z_s

clear; clc; close all;

%% Load plant and parameters
load('data/quarter_car_model.mat', 'sys', 'A', 'B', 'C', 'D');
load('data/quarter_car_params.mat', 'ms', 'mu', 'ks', 'cs', 'kt');

%% Make sure figures folder exists
if ~exist('figures', 'dir')
    mkdir('figures');
end

%% Plant
P = tf(sys);

disp('Plant transfer function P(s) = z_s / u:')
P

%% ------------------------------------------------------------
% Controller choice: lead compensator
%
% K(s) = k * (tau*s + 1) / (alpha*tau*s + 1),  with 0 < alpha < 1
%
% This gives phase lead near crossover.
% Tune k, tau, alpha as needed.
%% ------------------------------------------------------------

k = 2.0e4;      % overall gain
tau = 0.03;     % lead time constant
alpha = 0.10;   % alpha < 1 gives lead

s = tf('s');
K = k * (tau*s + 1) / (alpha*tau*s + 1);

disp('Controller K(s):')
K

%% Loop transfer, sensitivity, complementary sensitivity
L = minreal(P*K);
S = minreal(1/(1 + L));
T = minreal(L/(1 + L));

%% Gain / phase margins
[Gm, Pm, Wcg, Wcp] = margin(L);

fprintf('\nLoop-shaping results:\n');
fprintf('Gain margin      = %.6g\n', Gm);
fprintf('Phase margin     = %.6f deg\n', Pm);
fprintf('Gain crossover   = %.6f rad/s\n', Wcg);
fprintf('Phase crossover  = %.6f rad/s\n', Wcp);

%% -------------------------
% Bode plot of loop transfer
%% -------------------------
figure;
margin(L);
grid on;
title('Bode Plot of Loop Transfer L(s) = P(s)K(s)');
exportgraphics(gcf, 'figures/loop_transfer_bode.png');

%% -------------------------
% Nyquist plot of loop transfer
%% -------------------------
w = logspace(-1, 4, 6000);
H = squeeze(freqresp(L, w));

figure;
plot(real(H), imag(H), 'LineWidth', 2); hold on;
plot(real(H), -imag(H), 'LineWidth', 2);
plot(-1, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % critical point
grid on;
axis equal;
xlabel('Real');
ylabel('Imaginary');
title('Nyquist Plot of Loop Transfer L(s)');
legend('L(j\omega)', 'L(-j\omega)', '-1', 'Location', 'best');
exportgraphics(gcf, 'figures/loop_transfer_nyquist.png');

%% -------------------------
% Bode plot of closed-loop transfer T
%% -------------------------
figure;
bode(T);
grid on;
title('Bode Plot of Closed-Loop Transfer T(s) = L/(1+L)');
exportgraphics(gcf, 'figures/closed_loop_bode.png');

%% -------------------------
% Bode plot of sensitivity S
%% -------------------------
figure;
bode(S);
grid on;
title('Bode Plot of Sensitivity S(s) = 1/(1+L)');
exportgraphics(gcf, 'figures/sensitivity_bode.png');

%% Optional: compare |S| and |T| on same figure
figure;
bodemag(S, T);
grid on;
legend('S(s)', 'T(s)', 'Location', 'best');
title('Magnitude of Sensitivity and Complementary Sensitivity');
exportgraphics(gcf, 'figures/sensitivity_and_complementary_sensitivity.png');

%% Save controller and closed-loop objects
save('data/loop_shaping_design.mat', 'P', 'K', 'L', 'S', 'T', ...
    'k', 'tau', 'alpha', 'Gm', 'Pm', 'Wcg', 'Wcp');