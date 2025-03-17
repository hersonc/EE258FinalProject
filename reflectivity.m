function R = reflectivity(layer1, layer2, realDielectric)
% enter layer1 and layer2 as strings corresponding to layer types
% realDielectric = dictionary with real parts of dielectric constants for different materials
eps1 = realDielectric(layer1);
eps2 = realDielectric(layer2);
n1 = sqrt(eps1);
n2 = sqrt(eps2);
R = (n1-n2)^2/(n1+n2)^2;
end