load('data/quarter_car_model.mat', 'A', 'B', 'C', 'D', 'sys');
load('data/quarter_car_params.mat', 'cs', 'ks', 'kt', 'ms', 'mu');

% Frequency grid
w = logspace(-1, 4, 5000);

% Frequency response
H = squeeze(freqresp(sys, w));

figure
plot(real(H), imag(H), 'r', 'LineWidth', 2); hold on
plot(real(H), -imag(H), 'r', 'LineWidth', 2);   % mirrored branch

axis equal
grid on
xlabel('Real')
ylabel('Imaginary')
title('Nyquist Plot of Quarter Car Model')

[Gm, Pm, Wcg, Wcp] = margin(sys);