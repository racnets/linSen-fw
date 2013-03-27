function y = searchRadius(v, h, e)
	y = (e .* 128 ./ 24000000) .* v ./((h ./ 25 .- 1) .* 65 .* 10^(-6));
end
