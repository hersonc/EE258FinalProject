function L = attenuation(layer, depth, lambda, realDielectric, imagDielectric)
% enter layer as string corresponding to layer types
% depth is depth of layer
% lambda is wavelength of signal travelling through (corresponds to centre frequency)
% realDielectric = dictionary with real parts of dielectric constants for different materials
% imagDielectric = dictionary with imaginary parts of dielectric constants for different materials
epsReal = realDielectric(layer);
epsImag = imagDielectric(layer);
k = 2*pi/lambda;
L = exp(k * depth * epsImag/epsReal);
end