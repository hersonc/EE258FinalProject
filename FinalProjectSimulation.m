clear;

%% FREE PARAMETERS TO TOY WITH
mat1 = "drySnow";          % MATERIAL 1 - top layer
mat2 = "ice";           % MATERIAL 2 - bottom layer
mat1_thickness = 10;     % Material 1 average thickness (m)
mat2_thickness = 10;    % Material 2 average thickenss (m)
avg_slope = 30;         % average ground slope in degrees
crevasse_present = true;% true = crevasse present, false = no crevasse present
droneH_init = 100;      % initial drone height
drone_slope = 0;        % drone slope of ascent in degrees



%% Setup: Generate randomized ground elevation profile, average 30º incline over 100m
% Two layers of material before rock: ice and snow, each with an average
% thickness of 10m

realDielectric = dielectricConstants(1);
imagDielectric = dielectricConstants(0);
x = 0:0.5:100;
groundH = zeros(1, length(x));
iceH = mat2_thickness * ones(1, length(x));
snowH = mat1_thickness * ones(1, length(x));
droneH = droneH_init * ones(1, length(x));

for i = 2:length(x)
    groundH(i) = groundH(i - 1) + 0.5*tan(avg_slope * pi / 180) + 0.3*randn;
    iceH(i) = max(0, iceH(i - 1) + 0.5 * randn);
    snowH(i) = max(0, snowH(i - 1) + 0.5 * randn);
    droneH(i) = droneH(i - 1) + 0.5 * tan(drone_slope * pi / 180);
end


surfaceElevation = groundH + iceH + snowH;

if crevasse_present
 for i = 50:60
    iceH(i) = 0;
 end
end
iceElevation = groundH + iceH;

figure(1);
plot(x, groundH, "DisplayName", "Ground elevation");
hold on;
plot(x, iceElevation, "DisplayName", "Interface elevation");
plot(x, surfaceElevation, "DisplayName", "Top Surface Elevation");
plot(x, droneH, "DisplayName", "Drone Elevation");
xlabel("Horizontal distance (m)"); ylabel("Elevation (m)");
ylim([-2 110]);
title("True topography");
legend;
hold off;

%% SPECIFY MATERIALS

indices = [1 2 3 4 5 6];
materials = ["air" "drySnow" "wetSnow" "hoar" "ice" "ground"];
accessIndex = dictionary(materials, indices);

% REFLECTIVITY: the (i, j)th index is the power reflectivity coefficient
% from materials(i) to materials(j)
reflectivities = zeros(6, 6);
for i = 1:6
    for j = 1:6
        reflectivities(i, j) = reflectivity(materials(i), materials(j), realDielectric);
    end
end

rho_firstInterface = reflectivities(1, accessIndex(mat1));
rho_secondInterface = reflectivities(accessIndex(mat1), accessIndex(mat2));
rho_gnd = reflectivities(accessIndex(mat2), 6);

% ABSORPTIVITY LOSSES:
L_1 = attenuation(mat1, snowH, 0.3, realDielectric, imagDielectric);
L_2 = attenuation(mat2, iceH, 0.3, realDielectric, imagDielectric);

%% Simulate radar chirp
f_sample = 2e9;
f_c = 1e9;
T = 1e-6;
B = 1.5e9;
alpha = B/T;

t = 0:1/f_sample:1.1*T;
s_chirp = chirp(t, B, f_c, T);

L = 2*floor(length(s_chirp)/2);
f = (0:L)*f_sample/(L); % frequency vector
S_chirp = (fft(s_chirp));


%% Simulate Returns
c = 3 * 10^8;
p_t = 5;
g_t = 12.426;
A = 0.089;
k = 1.380649e-23;
T_sys = 273;
p_n = k*T_sys*B;

% Range from drone to interfaces
R_surf = droneH - surfaceElevation;
R_interface = droneH - iceElevation;
R_gnd = droneH-groundH;


% Time delays
td_surf = 2 * R_surf./c;
td_interface = 2 * R_interface./c;
td_gnd = 2 * R_gnd./c;

% Return Power
p_rSurf = p_t * g_t * A * rho_firstInterface ./ (16*pi*R_surf.^2);
p_rInterface = p_t * g_t * A * rho_secondInterface * L_1.^2 ./ (16*pi*R_interface.^2);
p_rGnd = p_t * g_t * A * rho_gnd * L_1.^2 .* L_2.^2 ./ (16*pi*R_gnd.^2);

rawImage = zeros(length(t), length(x));
finalImage = zeros(length(t), length(x));

% Populate finalImage with filtered output
for i = 1:length(x)
returnSurf = p_rSurf(i) * chirp(t - td_surf(i), B, f_c, T);
returnInterface = p_rInterface(i) * chirp(t - td_interface(i), B, f_c, T);
returnGnd = p_rGnd(i) * chirp(t - td_gnd(i), B, f_c, T);

returnNoise = p_n * randn(1, length(t));
returnTot = returnSurf + returnInterface + returnGnd + returnNoise;
rawImage(:, i) = returnTot;

S_return = fft(returnTot);
S_filtered = S_return .* conj(S_chirp);
s_filtered = ifft(S_filtered);
finalImage(:, i) = s_filtered;

end

figure(2);
subplot(111);
imagesc(abs(finalImage), 'XData', [0 100], 'YData', t * c / 2, [0 1e-4]);
% MIGHT NEED TO CHANGE LIMITS IN THE ABOVE EXPRESSION
ylim([0 110]);
xlabel("Horizontal distance (m)");
ylabel("Distance from radar (m)");
title("Radar return");
colormap(flipud(gray));
a = colorbar;
a.Label.String = 'Received Power (W)';

figure(3);
subplot(111);
imagesc(abs(rawImage), 'XData', [0 100], 'YData', t * c / 2, [0 1e-6]);
% MIGHT NEED TO CHANGE LIMITS IN THE ABOVE EXPRESSION
ylim([0 110]);
xlabel("Horizontal distance (m)");
ylabel("Distance from radar (m)");
title("Radar return (unfiltered)");
colormap(flipud(gray));
a = colorbar;
a.Label.String = 'Received Power (W)';
