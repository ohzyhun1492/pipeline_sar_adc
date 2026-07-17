%% Pipeline SAR ADC (Multi-CDAC: Big/Small Split) Automation & Analysis Script (With Mismatch, Noise & AMP)
clc; 
%%close all;
fs = 100e6;          % Simulink 블록들이 참조하는 실제 샘플링 주파수 (100MHz)
N = 1024;            % FFT 분석에 사용할 순수 샘플 개수 (Number of Samples)
M = 5;               % 입력 신호의 정확한 타겟 주기 횟수 (Cycles)
Vin = 0.95;           
bit_1st = 6;         
bit_2nd = 9;         
total_bits = bit_1st + bit_2nd; 
fin = (M / N) * fs;    
sim_time = (M + 3) / fin; % spectrum analysis  

%% 1. Non-ideality 파라미터 설정 (Cap Mismatch, Comparator, Amplifier)
show_debug = 'NO';   

sigma_u_1st = 0;
sigma_u_big = 0;
sigma_u_2nd = 0;
CMPno_1st = 0;
CMPno_2nd = 0;
CMPoff_1st = 0;
CMPoff_2nd = 0;
AMPno = 0;
AMPoff = 0;

Temp = 300;

Cu_1st = 8e-15;
Cu_big = 32e-15;
Cu_2nd = 1e-15;

% kT/C 노이즈 on/off (0 = off, 1 = on)
kTC_en_1st = 1;
kTC_en_big = 1;
kTC_en_2nd = 1;

% --- kT/C 샘플링 노이즈 파라미터 계산 ---
% Binary weighted CDAC + dummy cap 가정 → 총 unit = 2^(bit-1)
kB = 1.380649e-23;

C_1st = Cu_1st * 2^bit_1st;
C_big = Cu_big * 2^(bit_1st - 1);
C_2nd = Cu_2nd * 2^(bit_2nd - 1);

sigma_kTC_1st = sqrt(kB * Temp / C_1st);
sigma_kTC_big = sqrt(kB * Temp / C_big);
sigma_kTC_2nd = sqrt(kB * Temp / C_2nd);

var_kTC_1st = kTC_en_1st * sigma_kTC_1st^2;
var_kTC_big = kTC_en_big * sigma_kTC_big^2;
var_kTC_2nd = kTC_en_2nd * sigma_kTC_2nd^2;

mean_kTC = 0;

seed_kTC_1st = randi(2^31);
seed_kTC_big = randi(2^31);
seed_kTC_2nd = randi(2^31);

% sigma_u_1st = 0.0001;  % 1st-stage Small CDAC (판정용) Cap Mismatch
% sigma_u_big = 0.0001;  % 1st-stage Big CDAC (residue 생성용) Cap Mismatch  ★신규
% sigma_u_2nd = 0.0004;  % 2nd-stage Cap Mismatch
% 
% CMPno_1st = 0.0001;    % 1st-stage CMP Noise (Vnstd)
% CMPno_2nd = 0.001;     % 2nd-stage CMP Noise (Vnstd)
% CMPoff_1st = 0.001;    % 1st-stage CMP offset Mismatch (Vos)
% CMPoff_2nd = 0.001;    % 2nd-stage CMP offset Mismatch (Vos)
% 
% % --- [NEW] 1st-stage Residue Amplifier(AMP) 파라미터 추가 ---
% AMPno = 0.0001;    % AMP Dynamic Noise
% AMPoff = 0.0001;   % AMP Static Offset 
% ※ Big CDAC은 판정을 하지 않으므로 CMP noise/offset 없음 (미스매치만 존재)

%% 2. CDAC Mismatch 주입 및 가중치 생성
try
    % --- 1st Stage Small CDAC (판정용) Mismatch 주입 ---
    [DACunit_1st] = generateMismatchADCunit(bit_1st, sigma_u_1st, show_debug);
    DAC_1st_full = generateBinaryADCvalue(bit_1st, DACunit_1st);
    DAC_1st_scaled = DAC_1st_full * (2^bit_1st); % 정수 스케일 복원
    DAC_1st = DAC_1st_scaled(1 : bit_1st) / (2^(bit_1st-1)); 

    % --- 1st Stage Big CDAC (residue 생성용) Mismatch 주입 ★신규 ---
    % Small과 독립된 새 랜덤 실현 (별도 sigma_u_big 사용), 스케일/길이는 DAC_1st과 동일 규칙
    [DACunit_big] = generateMismatchADCunit(bit_1st, sigma_u_big, show_debug);
    DAC_big_full = generateBinaryADCvalue(bit_1st, DACunit_big);
    DAC_big_scaled = DAC_big_full * (2^bit_1st);
    DAC_1st_v2 = DAC_big_scaled(1 : (bit_1st + 1)) / (2^bit_1st);
    
if bit_2nd > 0
    % --- 2nd Stage CDAC Mismatch 주입 ---
    [DACunit_2nd] = generateMismatchADCunit(bit_2nd, sigma_u_2nd, show_debug);
    DAC_2nd_full = generateBinaryADCvalue(bit_2nd, DACunit_2nd);
    DAC_2nd_scaled = DAC_2nd_full * (2^bit_2nd); 
    DAC_2nd = DAC_2nd_scaled(1 : bit_2nd) / (2^(bit_2nd-1));
else
    DAC_2nd = 0;
end
catch ME
    warning('DAC 가중치 생성 및 미스매치 주입 에러: %s', ME.message);
end

%% 3. 충분한 주기의 시뮬레이션을 먼저 진행하여 원본 데이터 확보
model_name = 'pipeline_sar_adc_v2';
if ~bdIsLoaded(model_name), open_system(model_name); end
if evalin('base', 'exist(''Digital_OUT'',''var'')')
    evalin('base', 'clear Digital_OUT');
end
sim_opts = struct('SimulationMode', 'normal', 'StopTime', num2str(sim_time));
fprintf('▶ [시뮬레이션 구동] fs=%.2fMHz 변수 기반으로 총 %e초 실행...\n', fs*1e-6, sim_time);
out = sim(model_name, sim_opts); 
fprintf('▶ 시뮬레이션 완료. 데이터 수집 및 정밀 슬라이싱을 시작합니다.\n');

%% 4. 확보된 전체 데이터에서 실제 샘플링 데이터 정밀 추출 및 M주기 슬라이싱
if exist('out','var')
    try temp_data = out.Dout; catch, if exist('Dout','var'), temp_data = Dout; else, error('Workspace에서 출력 데이터를 찾을 수 없습니다.'); end, end
elseif exist('Dout','var')
    temp_data = Dout;
else
    error('Workspace에서 출력 데이터를 찾을 수 없습니다.');
end
if isa(temp_data, 'timeseries')
    signal_raw = double(temp_data.Data(:)); time_raw = double(temp_data.Time(:));          
else
    error('Simulink To Workspace 블록의 Save format을 Timeseries로 변경해주세요.');
end
dt = mean(diff(time_raw)); 
fs_actual = 1 / dt; 
total_captured = length(signal_raw);
discard_samples = ceil(fs_actual / fin); 
start_idx = discard_samples + 1;
end_idx = start_idx + N - 1;
if total_captured < end_idx
    error('시뮬레이션 데이터 수(%d)가 타겟 인덱스(%d)보다 부족합니다.', total_captured, end_idx);
end
signal_sliced = signal_raw(start_idx:end_idx); 
time_sliced = time_raw(start_idx:end_idx);     
signal_win = signal_sliced - mean(signal_sliced); 

%% 5. 추출된 M주기 데이터를 바탕으로 FFT 및 SNDR 계산
X_fft = fft(signal_win) / N;
NumUniquePts = ceil((N+1)/2);
X_mag = abs(X_fft(1:NumUniquePts));
X_mag(2:end) = X_mag(2:end) * 2; 
f = (0:NumUniquePts-1).' * (fs_actual / N);        
power_spec = 20*log10(X_mag + eps);  
signal_bins = 1;  
search_mag = X_mag(2:end);            
[~, sorted_idx] = sort(search_mag, 'descend');
top_idx_rel = sorted_idx(1:signal_bins);   
top_idx = top_idx_rel + 1;                 
P_signal = sum(X_mag(top_idx).^2);
P_total = sum(X_mag(2:end).^2);    
P_noise_distortion = P_total - P_signal;
SNDR = 10 * log10(P_signal / P_noise_distortion);
ENOB = (SNDR - 1.76) / 6.02;

%% 6. 결과 텍스트 및 아키텍처 비이상성 정보 출력
lsb_15bit = Vin / (2^total_bits);

fprintf('\n====================== [ Simulation Results ] ======================\n');
fprintf('Top %d signal bins indices: %s\n', signal_bins, mat2str(top_idx));
fprintf('Selected signal frequency bins (MHz): %s\n', mat2str((f(top_idx)*1e-6)));
fprintf('SNDR (multi-bin): %.2f dB\n', SNDR);
fprintf('ENOB: %.2f bits\n', ENOB);
fprintf('--------------------------------------------------------------------\n');
fprintf('📊 [주입된 하드웨어 노이즈 및 오프셋 요약 (LSB 환산)]\n');
fprintf('  ▶ 1st-Stage Small Comparator (판정용):\n');
fprintf('    - Noise (rms) : %.2f uV (≈ %.2f LSB)\n', CMPno_1st*1e6, CMPno_1st/lsb_15bit);
fprintf('    - Offset      : %.2f mV (≈ %.2f LSB)\n', CMPoff_1st*1e3, CMPoff_1st/lsb_15bit);
fprintf('  ▶ 1st-Stage Big CDAC (residue 생성, 판정 없음):\n');
fprintf('    - Cap Mismatch (sigma) : %.4f %%\n', sigma_u_big*100);
fprintf('    - Comparator Noise/Offset : 없음 (비교기 미사용)\n');
fprintf('  ▶ Residue Amplifier (AMP):\n');
fprintf('    - Noise (rms) : %.2f uV (≈ %.2f LSB)\n', AMPno*1e6, AMPno/lsb_15bit);
fprintf('    - Offset      : %.2f mV (≈ %.2f LSB)\n', AMPoff*1e3, AMPoff/lsb_15bit);
fprintf('  ▶ 2nd-Stage Comparator:\n');
fprintf('    - Noise (rms) : %.2f mV (≈ %.2f LSB)\n', CMPno_2nd*1e3, CMPno_2nd/lsb_15bit);
fprintf('    - Offset      : %.2f mV (≈ %.2f LSB)\n', CMPoff_2nd*1e3, CMPoff_2nd/lsb_15bit);
fprintf('--------------------------------------------------------------------\n');
fprintf('▶ [CDAC Weights 전체 리스트 확인 (MSB -> LSB 정렬)]\n');
if exist('DAC_1st', 'var')
    DAC_1st_msb = flipud(DAC_1st(:)); 
    fprintf('\n  * 1st Stage Small CDAC (%d-bit) 모든 가중치 (총 %d개):\n', bit_1st, length(DAC_1st_msb));
    for i = 1:length(DAC_1st_msb)
        fprintf('    Weight[%d] (MSB-%d) : %.6f\n', i, i-1, DAC_1st_msb(i));
    end
end
if exist('DAC_1st_v2', 'var')
    DAC_1st_v2_msb = flipud(DAC_1st_v2(:)); 
    fprintf('\n  * 1st Stage Big CDAC (%d-bit) 모든 가중치 (총 %d개):\n', bit_1st, length(DAC_1st_v2_msb));
    for i = 1:length(DAC_1st_v2_msb)
        fprintf('    Weight[%d] (MSB-%d) : %.6f\n', i, i-1, DAC_1st_v2_msb(i));
    end
end
if exist('DAC_2nd', 'var') && bit_2nd > 0
    DAC_2nd_msb = flipud(DAC_2nd(:)); 
    fprintf('\n  * 2nd Stage (%d-bit) 모든 가중치 (총 %d개):\n', bit_2nd, length(DAC_2nd_msb));
    for i = 1:length(DAC_2nd_msb)
        fprintf('    Weight[%d] (MSB-%d) : %.6f\n', i, i-1, DAC_2nd_msb(i));
    end
end
fprintf('====================================================================\n');

%% 7. 플롯 시각화 (하나의 창에 탭으로 통합)

main_fig = figure('Color', 'w', 'Position', [100, 100, 1200, 700], ...
                  'Name', 'Pipeline SAR ADC Analysis');
tabgroup = uitabgroup(main_fig);

% ==================== Tab 1: FFT 스펙트럼 ====================
tab1 = uitab(tabgroup, 'Title', 'FFT Spectrum');
ax1 = axes('Parent', tab1);

plot(ax1, f*1e-6, power_spec, 'LineWidth', 1.2, 'Color', [0 0.45 0.74]); 
grid(ax1, 'on'); hold(ax1, 'on');
plot(ax1, f(top_idx)*1e-6, power_spec(top_idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel(ax1, 'Frequency (MHz)');
ylabel(ax1, 'Power (dB)');
title(ax1, {sprintf('Pipeline SAR ADC (Multi-CDAC) Spectrum (ENOB: %.2f bits, SNDR: %.2f dB)', ENOB, SNDR), ...
       sprintf('Mismatch Applied: Small \\sigma=%.2f%%, Big \\sigma=%.2f%%, 2nd \\sigma=%.2f%%', ...
               sigma_u_1st*100, sigma_u_big*100, sigma_u_2nd*100)}, ...
       'FontSize', 11);
ylim(ax1, [-120 5]);

% ==================== Tab 2: Vres / Vres_amp 비교 ====================
tab2 = uitab(tabgroup, 'Title', 'Residue Voltage');

ax2a = subplot(1, 2, 1, 'Parent', tab2);
try
    if isa(out.Vres, 'timeseries')
        plot(ax2a, out.Vres.Time * 1e6, out.Vres.Data, 'LineWidth', 1.2, 'Color', [0 0.45 0.74]);
    else
        plot(ax2a, out.Vres, 'LineWidth', 1.2, 'Color', [0 0.45 0.74]);
    end
    grid(ax2a, 'on'); 
    xlabel(ax2a, 'Time (\mus)'); 
    ylabel(ax2a, 'Voltage (V)');
    title(ax2a, 'V_{res} (Before Residue Amp)');
catch
    text(ax2a, 0.5, 0.5, 'out.Vres 없음', 'HorizontalAlignment', 'center'); 
    axis(ax2a, 'off');
end

ax2b = subplot(1, 2, 2, 'Parent', tab2);
try
    if isa(out.Vres_amp, 'timeseries')
        plot(ax2b, out.Vres_amp.Time * 1e6, out.Vres_amp.Data, 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
    else
        plot(ax2b, out.Vres_amp, 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]);
    end
    grid(ax2b, 'on'); 
    xlabel(ax2b, 'Time (\mus)'); 
    ylabel(ax2b, 'Voltage (V)');
    title(ax2b, 'V_{res\_amp} (After Residue Amp)');
catch
    text(ax2b, 0.5, 0.5, 'out.Vres_amp 없음', 'HorizontalAlignment', 'center'); 
    axis(ax2b, 'off');
end

% ==================== Tab 3: kT/C 노이즈 (N1, N2, N3) ====================
tab3 = uitab(tabgroup, 'Title', 'kT/C Noise');

% N1: 1st Small CDAC noise
ax3a = subplot(3, 1, 1, 'Parent', tab3);
try
    if isa(out.N1, 'timeseries')
        plot(ax3a, out.N1.Time * 1e6, out.N1.Data * 1e6, 'LineWidth', 1.0, 'Color', [0 0.45 0.74]);
    else
        plot(ax3a, out.N1 * 1e6, 'LineWidth', 1.0, 'Color', [0 0.45 0.74]);
    end
    grid(ax3a, 'on'); 
    ylabel(ax3a, 'Noise (\muV)');
    title(ax3a, sprintf('N1: 1st Small CDAC kT/C Noise (\\sigma = %.2f \\muV)  [%s]', ...
          sigma_kTC_1st*1e6, ternary(kTC_en_1st,'ON','OFF')));
catch
    text(ax3a, 0.5, 0.5, 'out.N1 없음', 'HorizontalAlignment', 'center'); 
    axis(ax3a, 'off');
end

% N2: 1st Big CDAC noise
ax3b = subplot(3, 1, 2, 'Parent', tab3);
try
    if isa(out.N2, 'timeseries')
        plot(ax3b, out.N2.Time * 1e6, out.N2.Data * 1e6, 'LineWidth', 1.0, 'Color', [0.85 0.33 0.10]);
    else
        plot(ax3b, out.N2 * 1e6, 'LineWidth', 1.0, 'Color', [0.85 0.33 0.10]);
    end
    grid(ax3b, 'on'); 
    ylabel(ax3b, 'Noise (\muV)');
    title(ax3b, sprintf('N2: 1st Big CDAC kT/C Noise (\\sigma = %.2f \\muV)  [%s]', ...
          sigma_kTC_big*1e6, ternary(kTC_en_big,'ON','OFF')));
catch
    text(ax3b, 0.5, 0.5, 'out.N2 없음', 'HorizontalAlignment', 'center'); 
    axis(ax3b, 'off');
end

% N3: 2nd Stage CDAC noise
ax3c = subplot(3, 1, 3, 'Parent', tab3);
try
    if isa(out.N3, 'timeseries')
        plot(ax3c, out.N3.Time * 1e6, out.N3.Data * 1e6, 'LineWidth', 1.0, 'Color', [0.47 0.67 0.19]);
    else
        plot(ax3c, out.N3 * 1e6, 'LineWidth', 1.0, 'Color', [0.47 0.67 0.19]);
    end
    grid(ax3c, 'on'); 
    ylabel(ax3c, 'Noise (\muV)'); 
    xlabel(ax3c, 'Time (\mus)');
    title(ax3c, sprintf('N3: 2nd Stage CDAC kT/C Noise (\\sigma = %.2f \\muV)  [%s]', ...
          sigma_kTC_2nd*1e6, ternary(kTC_en_2nd,'ON','OFF')));
catch
    text(ax3c, 0.5, 0.5, 'out.N3 없음', 'HorizontalAlignment', 'center'); 
    axis(ax3c, 'off');
end

%% Helper Function
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end