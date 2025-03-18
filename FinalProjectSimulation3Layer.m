clear;
%% FREE PARAMETERS TO TOY WITH
mat1 = "drySnow";          % MATERIAL 1 - top layer
mat2 = "hoar";           % MATERIAL 2 - bottom layer
mat3 = "ice";
mat1_thickness = 10;     % Material 1 average thickness (m)
mat2_thickness = 10;    % Material 2 average thickenss (m)
mat3_thickness = 15;
avg_slope = 30;         % average ground slope in degrees
crevasse_present = false; % true = crevasse present, false = no crevasse present
droneH_init = 120;      % initial drone height
drone_slope = 0;        % drone slope of ascent in degrees


%% Setup: Generate randomized ground elevation profile, average 30º incline over 100m
% Two layers of material before rock: ice and snow, each with an average
% thickness of 10m

realDielectric = dielectricConstants(1);
imagDielectric = dielectricConstants(0);
x = 0:0.5:100;
groundH = zeros(1, length(x));
layer3H = mat3_thickness * ones(1, length(x));
iceH = mat2_thickness * ones(1, length(x));
snowH = mat1_thickness * ones(1, length(x));
droneH = droneH_init * ones(1, length(x));

for i = 2:length(x)
    groundH(i) = groundH(i - 1) + 0.5*tan(avg_slope * pi / 180) + 0.3*randn;
    layer3H(i) = max(0, layer3H(i - 1) + 0.5 * randn);
    iceH(i) = max(0, iceH(i - 1) + 0.5 * randn);
    snowH(i) = max(0, snowH(i - 1) + 0.5 * randn);
    droneH(i) = droneH(i - 1) + 0.5 * tan(drone_slope * pi / 180);
end


surfaceElevation = groundH + iceH + snowH + layer3H;

if crevasse_present
    for i = 50:60
        iceH(i) = 0;
    end
end
iceElevation = groundH + iceH + layer3H;
layer3Elevation = groundH + layer3H; 

figure(1);
plot(x, groundH, "DisplayName", "Ground elevation");
hold on;
plot(x, layer3Elevation, "DisplayName", "Ice elevation");
plot(x, iceElevation, "DisplayName", "Hoar elevation");
plot(x, surfaceElevation, "DisplayName", "Surface (Snow) Elevation");
plot(x, droneH, "DisplayName", "Drone Elevation");
xlabel("Horizontal distance (m)"); ylabel("Elevation (m)");
ylim([-2 droneH_init + 10]);
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
rho_thirdInterface = reflectivities(accessIndex(mat2), accessIndex(mat3));
rho_gnd = reflectivities(accessIndex(mat3), 6);
rho_crevasse = reflectivities(accessIndex(mat1), 6);

% ABSORPTIVITY LOSSES:
L_1 = attenuation(mat1, snowH, 0.3, realDielectric, imagDielectric);
L_2 = attenuation(mat2, iceH, 0.3, realDielectric, imagDielectric);
L_3 = attenuation(mat3, layer3H, 0.3, realDielectric, imagDielectric);

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
Hamm = 0.54 + 0.46 * cos(pi*(f-f_c)/(B/2));

%% Simulate Returns
c = 3 * 10^8;
p_t = 5;
g_t = 12.426;
A = 0.089;
k = 1.380649e-23;
T_sys = 273;
p_n = k*T_sys*B;

Range from drone to interfaces
R_surf = droneH - surfaceElevation;
R_interface = droneH - iceElevation;
R_interface2 = droneH - layer3Elevation;
R_gnd = droneH-groundH;

% Time delays
td_surf = 2 * R_surf./c;
td_interface = 2 * R_interface./c;
td_interface2 = 2 * R_interface2./c;
td_gnd = 2 * R_gnd./c;

% add error in time delay
percent_err = 5/100;
td_surf = td_surf + percent_err .* randn(1, length(td_surf)) .* td_surf;
td_interface = td_interface + percent_err .* randn(1, length(td_interface)) .* td_interface;
td_interface2 = td_interface2 + percent_err .* randn(1, length(td_interface2)) .* td_interface2;
td_gnd = td_gnd + percent_err .* randn(1, length(td_gnd)) .* td_gnd;

% Return Power
p_rSurf = p_t * g_t * A * rho_firstInterface ./ (16*pi*R_surf.^2);
p_rInterface = p_t * g_t * A * rho_secondInterface * L_1.^2 ./ (16*pi*R_interface.^2);
p_rInterface2 = p_t * g_t * A * rho_thirdInterface * L_1.^2 .* L_2.^2 ./ (16*pi*R_interface2.^2);
p_rGnd = p_t * g_t * A * rho_gnd * L_1.^2 .* L_2.^2 .* L_3.^2 ./ (16*pi*R_gnd.^2);

rawImage = zeros(length(t), length(x));
finalImage = zeros(length(t), length(x));

% Populate finalImage with filtered output
for i = 1:length(x)
    if snowH(i) == 0
        returnSurf = 0;
    else
        returnSurf = p_rSurf(i) * chirp(t - td_surf(i), B, f_c, T);
    end
    if iceH(i) == 0
        returnInterface = 0;
        returnGnd = p_rGnd(i) * rho_crevasse / rho_gnd * chirp(t - td_gnd(i), B, f_c, T);
    else
        returnInterface = p_rInterface(i) * chirp(t - td_interface(i), B, f_c, T);
        returnInterface2 = p_rInterface2(i) * chirp(t - td_interface2(i), B, f_c, T);
        returnGnd = p_rGnd(i) * chirp(t - td_gnd(i), B, f_c, T);
    end

returnNoise = p_n * randn(1, length(t));
returnTot = returnSurf + returnInterface + returnInterface2 + returnGnd + returnNoise;
rawImage(:, i) = returnTot;

S_return = fft(returnTot);
S_filtered = Hamm .* S_return .* conj(S_chirp);
s_filtered = ifft(S_filtered);
finalImage(:, i) = s_filtered;

end


figure(2);
subplot(111);
imagesc(abs(finalImage), 'XData', [0 100], 'YData', t * c / 2, [0 1e-4]);
% MIGHT NEED TO CHANGE LIMITS IN THE ABOVE EXPRESSION
ylim([0 droneH_init+10]);
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
ylim([0 droneH_init+10]);
xlabel("Horizontal distance (m)");
ylabel("Distance from radar (m)");
title("Radar return (unfiltered)");
colormap(flipud(gray));
a = colorbar;
a.Label.String = 'Received Power (W)';

%% Test Avalanche Risk
nLayer = 4; %infer from radargram
yMax = max(t*c/2);

[slopes, topH] = risk_assessment(finalImage, nLayer, yMax);

%% Calculate real average values
trueGround = polyfit(x, groundH, 1);
trueIce = polyfit(x, layer3Elevation, 1);
trueHoar = polyfit(x, iceElevation, 1);
trueSnow = polyfit(x, surfaceElevation, 1);

trueslopes = [atan(trueSnow(1)) * 180/pi atan(trueHoar(1)) * 180/pi atan(trueIce(1)) * 180/pi atan(trueGround(1)) * 180/pi];
trueMean = mean(surfaceElevation - iceElevation);

%% Get residuals

fracResidSlopes = (slopes - trueslopes)./trueslopes;
fracResidThick = (topH - trueMean) ./ trueMean;
