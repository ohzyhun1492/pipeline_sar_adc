function [Q, VDAC] = BinarySAR_Residue(DACbitSAR, DACvalueSAR, IN, CMPno, CMPoff)
% twkang, 10/3/2024, copied from SJ's BinarySAR
% For NSSAR, it requires one more CDAC bit with VCM mode.
% But the residue is calculated from the outer Simulink loop. Still, it
% requires more CDAC bit
TempIN = IN;
Q = 0;
VDAC = 0;
idx_offset = 2;
% MSB -> LSB-1
for i = 1:1:DACbitSAR
%     REF = sqrt(2)*CMPno*randn(1) + CMPoff;
    REF = CMPno*randn(1) + CMPoff;
%     sqrt(2)는 입력이 -1~1이므로 범위 2배, 표준편차 sqrt(2)배
    if TempIN >= REF
        TempIN = TempIN - DACvalueSAR(DACbitSAR+idx_offset-i);
        Q = Q + 2^(DACbitSAR+1-i);
        VDAC = VDAC + DACvalueSAR(DACbitSAR+idx_offset-i);
    else
        TempIN = TempIN + DACvalueSAR(DACbitSAR+idx_offset-i);
        VDAC = VDAC - DACvalueSAR(DACbitSAR+idx_offset-i);
    end
end
% Final conversion
% REF = sqrt(2)*CMPno*randn(1) + CMPoff;
REF = CMPno*randn(1) + CMPoff;
if TempIN >= REF
    Q = Q + 2^0;
end

% Since it is NSSAR, Q should have one less bit
% Drop LSB -> floor
Q = floor(Q / 2);
end