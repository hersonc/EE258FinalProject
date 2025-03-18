function varargout = risk_assessment(finalImage, nLayer, yMax)
% input nLayer as assumed from radargram image

mag = abs(finalImage);
shape = size(mag);

layerMag = zeros(shape(2),nLayer);
layerHeight = zeros(shape(2),nLayer);
layerSlopes = zeros(shape(2)-1,nLayer);
for i=1:shape(2)
    col = mag(:,i);
    [pks, loc] = findpeaks(col,"SortStr","descend", "MinPeakDistance", 10, "NPeaks",nLayer);
    peaks = [pks loc];
    peaks = sortrows(peaks, 2);
    layerMag(i,:) = peaks(:,1);
    layerHeight(i,:) = peaks(:,2)./shape(1) .* yMax;
    if i > 1
        slopes = (layerHeight(i,:) - layerHeight(i-1,:))./0.5;
        layerSlopes(i,:) = slopes;
    end
end

topThickness = layerHeight(:,2) - layerHeight(:,1);
aveTop = mean(topThickness);

aveSlopes = zeros(1, nLayer);

for j=1:nLayer
    linfit = polyfit(linspace(0, 100, shape(2)), layerHeight(:,j), 1);
    aveSlopes(j) = atan(-linfit(1)) * 180/pi;
end

varargout{1} = aveSlopes;
varargout{2} = aveTop;

