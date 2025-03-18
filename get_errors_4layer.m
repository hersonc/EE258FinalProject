percents = logspace(-1, 1, 50);
meanSlopeResids = zeros(50, 4);
meanThickResids = zeros(50, 1);
N = 100;
for n=1:N
SlopeResids = zeros(50, 4);
ThickResids = zeros(50, 1);
for p=1:50
    percent_err = percents(p)/100;
    FinalProjectSimulation3Layer;
    SlopeResids(p,:) = fracResidSlopes;
    ThickResids(p) = fracResidThick;
end
meanSlopeResids = meanSlopeResids + abs(SlopeResids);
meanThickResids = meanThickResids + abs(ThickResids);
if mod(n, 10) == 0
    disp(n)
end
end

meanSlopeResids = meanSlopeResids./N;
meanThickResids = meanThickResids./N;


meanSlopeResids0 = [abs(fracResidSlopesnoErr); meanSlopeResids];
meanThickResids0 = [abs(fracResidThicknoErr); meanThickResids];
fig1 = figure;
hold on;
box on;
plot([0 percents], meanSlopeResids0(:,1), "DisplayName", "Surface Slope")
plot([0 percents], meanSlopeResids0(:,2), "DisplayName", "Hoar Slope")
plot([0 percents], meanSlopeResids0(:,3), "DisplayName", "Ice Slope")
plot([0 percents], meanSlopeResids0(:,4), "DisplayName", "Ground Slope")
plot([0 percents], meanThickResids0, "DisplayName", "Snow Depth")
legend('Location','northwest');
xlabel('Measurement Error (%)')
ylabel('Absolute Fractional Misfit')
title('Fractional Misfit for Averaged Quantities with Measurement Error');
% xscale('log')
yscale('log')