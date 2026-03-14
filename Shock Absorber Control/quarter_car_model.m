%% Defining the quarter car model 

ms = 250;
mu = 40;
ks = 1.5e+4;
cs = 1500;
kt = 1.5e+5;

% Construct the state space realization of the suspension model
A = [0 1 0 0;
    -ks/ms -cs/ms ks/ms cs/ms;
     0 0 0 1;
     ks/mu cs/mu -(ks+kt)/mu -cs/mu];

B = [0; 1/ms; 0; -1/mu];
C = [1 0 0 0];
D = 0;

sys = ss(A, B, C, D);
% input is actuator force u and output is body displacement z_s

% Transfer function
tfSys = tf(sys)

% Bode plot
figure;
bode(tfSys);
grid on;
title('Bode Plot of the Quarter Car Model');

p = pole(sys);
z = zero(sys);

figure;
plot(real(p), imag(p), 'x', 'MarkerSize', 14, 'LineWidth', 2.5); hold on;
plot(real(z), imag(z), 'o', 'MarkerSize', 14, 'LineWidth', 2.5);
axis([-20 2 -80 80])
grid on;
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');
title('Pole-Zero Plot of Suspension System');
legend('Poles','Zeros','Location','best');

save('data/quarter_car_params.mat','ms','mu','ks','cs','kt');
save('data/quarter_car_model.mat','A','B','C','D','sys')
