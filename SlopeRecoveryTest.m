%% Script for Slope Recovery Accuracy for Snow Ground
%Only 2 layers allowed
clc;
clear all;

%% Setup for multiple simulations
angles = 0:1:40; % Ground truth angles to test
n_sim = length(angles);
predicted_surf_angles = zeros(1, n_sim);
predicted_gnd_angles = zeros(1, n_sim);
true_surf_angles = zeros(1, n_sim);
true_gnd_angles = zeros(1, n_sim);

%% FREE PARAMETERS
mat1 = "drySnow";          % MATERIAL 1 - top layer
mat2 = "ice";             % MATERIAL 2 - bottom layer
mat1_thickness = 20;      % Material 1 average thickness (m)
mat2_thickness = 10;      % Material 2 average thickness (m)
crevasse_present = false;  % true = crevasse present, false = no crevasse present
droneH_init = 150;        % initial drone height
drone_slope = 0;         % drone slope of ascent in degrees

%% Material Properties
realDielectric = dielectricConstants(1);
imagDielectric = dielectricConstants(0);
indices = [1 2 3 4 5 6];
materials = ["air" "drySnow" "wetSnow" "hoar" "ice" "ground"];
accessIndex = dictionary(materials, indices);

reflectivities = zeros(6, 6);
for i = 1:6
    for j = 1:6
        reflectivities(i, j) = reflectivity(materials(i), materials(j), realDielectric);
    end
end

%% Radar Parameters
f_sample = 2e9;
f_c = 1e9;
T = 1e-6;
B = 1.5e9;
alpha = B/T;
t = 0:1/f_sample:1.1*T;
s_chirp = chirp(t, B, f_c, T);
L = 2*floor(length(s_chirp)/2);
f = (0:L)*f_sample/(L); % frequency vector
S_chirp = fft(s_chirp);
Hamm = 0.54 + 0.46*cos(pi*(f-f_c)/(B/2));
c = 3e8;
p_t = 5;
g_t = 12.426;
A = 0.089;
k = 1.380649e-23;
T_sys = 273;
p_n = k*T_sys*B;

%% Main Simulation Loop
x = 0:0.5:100;
for sim = 1:n_sim
    avg_slope = angles(sim);
    
    % Generate profiles
    groundH = zeros(1, length(x));
    snowH = mat1_thickness * ones(1, length(x));
    droneH = droneH_init * ones(1, length(x));
    
    for i = 2:length(x)
        groundH(i) = groundH(i-1) + 0.5*tan(avg_slope * pi/180) + 0.3*randn;
        snowH(i) = max(0, snowH(i-1) + 0.5 * randn);
        droneH(i) = droneH(i-1) + 0.5 * tan(drone_slope * pi/180);
    end
    %extract true slopes of the profile
    %coeff_snow = polyfit(x, snowH, 1);
    coeff_ground = polyfit(x, groundH, 1);

    %true_surf_angles(sim) = rad2deg(atan(coeff_snow(1)));
    true_gnd_angles(sim) = rad2deg(atan(coeff_ground(1)));
    
    surfaceElevation = groundH + snowH;
    
    % Material properties
    rho_firstInterface = reflectivities(1, accessIndex(mat1));
    rho_gnd = reflectivities(accessIndex(mat2), 6);
    L_1 = attenuation(mat1, snowH, 0.3, realDielectric, imagDielectric);
    
    % Ranges and returns
    R_surf = droneH - surfaceElevation;
    R_gnd = droneH - groundH;
    td_surf = 2 * R_surf./c;
    td_gnd = 2 * R_gnd./c;
    p_rSurf = p_t * g_t * A * rho_firstInterface ./ (16*pi*R_surf.^2);
    p_rGnd = p_t * g_t * A * rho_gnd * L_1.^2 ./ (16*pi*R_gnd.^2);
    
    % Simulate returns
    finalImage = zeros(length(t), length(x));
    for i = 1:length(x)
        if snowH(i) == 0
            returnSurf = 0;
        else
            returnSurf = p_rSurf(i) * chirp(t - td_surf(i), B, f_c, T);
        end
        returnGnd = p_rGnd(i) * chirp(t - td_gnd(i), B, f_c, T);
        
        returnNoise = p_n * randn(1, length(t));
        returnTot = returnSurf + returnGnd + returnNoise;
        
        S_return = fft(returnTot);
        S_filtered = Hamm.*S_return .* conj(S_chirp);
        s_filtered = ifft(S_filtered);
        finalImage(:, i) = s_filtered;
    end
    
    % Extract slopes
    peakDistancesSurf = zeros(1, length(x));
    peakDistancesGnd = zeros(1, length(x));
    for i = 1:length(x)
        columnData = abs(finalImage(:, i));
        [pks, locs] = findpeaks(columnData, 'SortStr', 'descend', 'NPeaks', 2);
        
        if length(pks) >= 2
            [sortedLocs, ~] = sort(locs);
            distances = t(sortedLocs) * c/2;
            peakDistancesSurf(i) = distances(1);
            peakDistancesGnd(i) = distances(2);
        else
            peakDistancesSurf(i) = NaN;
            peakDistancesGnd(i) = NaN;
        end
    end
    
    % Calculate slopes
    %coeffsSurf = polyfit(x(~isnan(peakDistancesSurf)), peakDistancesSurf(~isnan(peakDistancesSurf)), 1);
    coeffsGnd = polyfit(x(~isnan(peakDistancesGnd)), peakDistancesGnd(~isnan(peakDistancesGnd)), 1);
    
    %predicted_surf_angles(sim) = rad2deg(atan(-coeffsSurf(1)));
    predicted_gnd_angles(sim) = rad2deg(atan(-coeffsGnd(1)));
end

%% Results
angle_diffs_surf = predicted_surf_angles - angles;
angle_diffs_gnd = predicted_gnd_angles - angles;


% Single comparison plot
figure(1);
%plot(true_surf_angles, predicted_surf_angles, 'b-', 'DisplayName', 'Predicted Surface'); hold on;
scatter(true_gnd_angles, predicted_gnd_angles, 'DisplayName', 'Ground Slope'); hold on;
plot(linspace(0,45,1000),linspace(0,45,1000),'DisplayName', 'Perfect Prediction')
xlabel('Ground Truth Angle (degrees)');
ylabel('Predicted Angle (degrees)');
title('Angle Prediction Comparison with Hamming Function');
yscale('log')
legend;
grid on;
hold off;