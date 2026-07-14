function [DACunit] = generateMismatchADCunit(DACbit, unitMis, showResult)
% Generate DAC (for ADC) with mismatch.
% DACbit: Bit of the DAC. No of DACunit = 2^DACbit-1.
% unitMis: 1=100%, 0.01=1% mismatch
% showResult: 'YES' for showing results

% Generate random resistors
DACunitR = 1+unitMis*randn(1, 2^DACbit); % 2^DACbit만큼의 value가 필요 -> 그 만큼의 저항 필요
% 평균이 0이고, 표준편차가 1인 가우시안 분포를 따르는 2^DACbit 개의 난수를 생성합니다.
%unitMis = 0.01이라면 저항 값은 약 ±1%의 변동을 가질 수 있습니다.

if strcmp(showResult, 'YES') == 1
    disp(['Mean is ', num2str(mean(DACunitR), 6), '.'])
    disp(['stdev is ', num2str(std(DACunitR), 3), '.'])
end 
% Change to conductance and averaging
DACunitG = 1./DACunitR;   %R -> Cap
DACunit = DACunitG/sum(DACunitG);


if length(DACunit) == 1
    DACunit = DACunit*2;
end
if strcmp(showResult, 'YES') == 1
%     disp(['DACunit = ', num2str(DACunit), '.'])
    disp(['Mean of DACunit is ', num2str(mean(DACunit)), '.'])
    disp(['stdev of DACunit is ', num2str(std(DACunit)), '.'])
end 

end

