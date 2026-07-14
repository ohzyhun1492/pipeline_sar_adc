function [Q, VDAC] = BinarySAR(DACbitSAR, DACvalueSAR, IN, CMPno, CMPoff)
TempIN = IN;
Q = 0;
VDAC = 0;
% MSB -> LSB-1
for i = 1:1:DACbitSAR
%     REF = sqrt(2)*CMPno*randn(1) + CMPoff;
    REF = CMPno*randn(1) + CMPoff;
%     sqrt(2)는 입력이 -1~1이므로 범위 2배, 표준편차 sqrt(2)배
    if TempIN >= REF
        TempIN = TempIN - DACvalueSAR(DACbitSAR+2-i);
        Q = Q + 2^(DACbitSAR+1-i);
        VDAC = VDAC + DACvalueSAR(DACbitSAR+2-i);
    else
        TempIN = TempIN + DACvalueSAR(DACbitSAR+2-i);
        VDAC = VDAC - DACvalueSAR(DACbitSAR+2-i);
    end
end
% Final conversion
% REF = sqrt(2)*CMPno*randn(1) + CMPoff;
REF = CMPno*randn(1) + CMPoff;
if TempIN >= REF
    Q = Q + 2^0;
end
end