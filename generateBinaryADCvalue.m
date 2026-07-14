function [DACvalue] = generateBinaryADCvalue(DACbit, DACunit)
% For Binary mode DAC
DACvalue = zeros(DACbit+1, 1);
for i = 1:1:DACbit
    DACvalue(i+1) = sum(DACunit(1:2^(i-1)));
    DACunit(1:2^(i-1)) =[];
end
DACvalue(1) = DACunit;
end

