function a = dielectricConstants(n)

%% set constants (feel free to change)
T = 273; % K
f = 1e9; % 1GHz

p_dry = 0.1;
p_hoar = 0.3;
p_wet = 0.6;
Wv = 0.05;

%% get dielectric constants

er_dry = 1 + 1.91 * p_dry;
ei_dry = 1.59e6 * er_dry * (0.52*p_dry+0.62*p_dry^2)/(1+1.7*p_dry+0.7*p_dry^2)*(1/f + sqrt(f)/8.13e13)*exp(0.0367*(T-273));

er_hoar = 1 + 1.91 * p_hoar;
ei_hoar = 1.59e6 * er_hoar * (0.52*p_hoar+0.62*p_hoar^2)/(1+1.7*p_hoar+0.7*p_hoar^2)*(1/f + sqrt(f)/8.13e13)*exp(0.0367*(T-273));

er_wet = 1 + 2 * p_dry + 23.5 * Wv;
ei_water = 82.8 * (f/8.84e9) / (1+(f/8.84e9)^2);
ei_wet = (0.1 * Wv + 8.0*Wv^2)*ei_water;

er_ice = 3.1884 + 0.00091*(T - 273);
theta = 300/T - 1;
alpha = (0.0050+0.0062*theta) * exp(-22.1*theta);
beta = 0.0207/T *exp(335/T)/(exp(335/T)-1)^2 + 1.16e-11*(f/1e9)^2 + exp(0.0372*(T - 273.16)-9.963);

ei_ice = alpha/(f/1e9) + beta * (f/1e9);

er_rock = 5.45;

%% Make reference dictionaries

realDielectric = dictionary('drySnow', er_dry, ...
    'hoar', er_hoar, ...
    'wetSnow', er_wet, ...
    'ice', er_ice, ...
    'ground', er_rock, ...
    'air', 1);

imagDielectric = dictionary('drySnow', ei_dry, ...
    'hoar', ei_hoar, ...
    'wetSnow', ei_wet, ...
    'ice', ei_ice, ...
    'air', 0);

if n == 1
    a = realDielectric;
else
    a = imagDielectric;
end
end