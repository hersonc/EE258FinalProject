function y = chirp(t, B, f_c, T)
%chirp waveform
    alpha = B / T;
    y = zeros(1, length(t));
    for i = 1:length(t)
        if t(i) >= 0 && t(i) <= T
            y(i) = exp(-j*2*pi*(f_c * (t(i) - T/2) + 0.5 * alpha * (t(i)-T/2)^2));
        else
            y(i) = 0;
        end
    end
end

