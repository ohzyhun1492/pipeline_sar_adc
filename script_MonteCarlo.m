%% Pipeline SAR ADC Automation & Multiple Analysis Script
clc; clear; close all;

%% 1. 글로벌 및 시뮬레이션 반복 설정
num_simulations = 100;     % ◀ 반복할 시뮬레이션 횟수 지정 (원하는 만큼 변경 가능)
enob_history = zeros(num_simulations, 1);  % ENOB 결과 저장용 배열
sndr_history = zeros(num_simulations, 1); % SNDR 결과 저장용 배열

fs = 100e6;          
N = 1024;            
M = 5;               
Vin = 0.95;           
bit_1st = 6;         
bit_2nd = 9;         
total_bits = bit_1st + bit_2nd; 
fin = (M / N) * fs;    
sim_time = (M + 3) / fin; 

% Mismatch and Noise Parameter
sigma_u_1st = 0.0001;  %1st-stage Cap mismatch
sigma_u_2nd = 0.0004;  %2nd-stage Cap mismatch 
CMPno_1st = 0.0001;    %1st-stage CMP noise
CMPno_2nd = 0.001;     %2nd-stage CMP noise
CMPoff_1st = 0.001;    %1st-stage CMP offset
CMPoff_2nd = 0.001;    %2nd-stage CMP offset
AMPno = 0.001;         % AMP Dynamic Noise
AMPoff = 0.001;       % AMP Static Offset 

show_debug = 'NO';   

model_name = 'pipeline_sar_adc';
if ~bdIsLoaded(model_name), open_system(model_name); end

%% 2. 루프 실행 (몬테카를로 시뮬레이션)
fprintf('🚀 총 %d회의 Pipeline SAR ADC 시뮬레이션을 시작합니다.\n', num_simulations);
tic;

for sim_run = 1:num_simulations
    fprintf('\n🔄 [%d / %d] 시뮬레이션 진행 중...\n', sim_run, num_simulations);
    
    % --- 1st Stage CDAC Mismatch 주입 ---
    try
        [DACunit_1st] = generateMismatchADCunit(bit_1st, sigma_u_1st, show_debug);
        DAC_1st_full = generateBinaryADCvalue(bit_1st, DACunit_1st);
        DAC_1st_scaled = DAC_1st_full * (2^bit_1st); 
        DAC_1st = DAC_1st_scaled(1 : (bit_1st + 1)) / (2^bit_1st); 
        
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
        warning('DAC 가중치 생성 에러: %s', ME.message);
    end
    
    % 기본 워크스페이스 변수 클리어 (잔상 방지)
    if evalin('base', 'exist(''Digital_OUT'',''var'')'), evalin('base', 'clear Digital_OUT'); end
    
    % Simulink 구동 (수치 출력을 콘솔에 뿌리지 않도록 요약 출력)
    sim_opts = struct('SimulationMode', 'normal', 'StopTime', num2str(sim_time));
    out = sim(model_name, sim_opts); 
    
    % --- 데이터 추출 및 슬라이싱 ---
    try 
        temp_data = out.simout; 
    catch
        if exist('simout','var'), temp_data = simout; else, error('데이터를 찾을 수 없습니다.'); end
    end
    
    signal_raw = double(temp_data.Data(:)); 
    time_raw = double(temp_data.Time(:));          
    
    dt = mean(diff(time_raw)); 
    fs_actual = 1 / dt; 
    discard_samples = ceil(fs_actual / fin); 
    start_idx = discard_samples + 1;
    end_idx = start_idx + N - 1;
    
    signal_sliced = signal_raw(start_idx:end_idx); 
    signal_win = signal_sliced - mean(signal_sliced); 
    
    % --- FFT 및 SNDR/ENOB 계산 ---
    X_fft = fft(signal_win) / N;
    NumUniquePts = ceil((N+1)/2);
    X_mag = abs(X_fft(1:NumUniquePts));
    X_mag(2:end) = X_mag(2:end) * 2; 
    
    signal_bins = 1;  
    search_mag = X_mag(2:end);            
    [~, sorted_idx] = sort(search_mag, 'descend');
    top_idx = sorted_idx(1:signal_bins) + 1;                 
    
    P_signal = sum(X_mag(top_idx).^2);
    P_total = sum(X_mag(2:end).^2);    
    P_noise_distortion = P_total - P_signal;
    
    % 결과 기록
    sndr_current = 10 * log10(P_signal / P_noise_distortion);
    enob_current = (sndr_current - 1.76) / 6.02;
    
    sndr_history(sim_run) = sndr_current;
    enob_history(sim_run) = enob_current;
    
    fprintf('   └─ 결과: SNDR = %.2f dB, ENOB = %.2f bits\n', sndr_current, enob_current);
end

total_time = toc;
fprintf('\n====================================================================\n');
fprintf('✅ 모든 시뮬레이션 완료! (총 소요 시간: %.2f초)\n', total_time);
fprintf('====================================================================\n');

V_fs = 0.95;               % 변수 Vin 값과 동일하게 맞춤 (기존 0.95V)
total_bits = 15;           % bit_1st(6) + bit_2nd(9)
lsb_15bit = 1 / (2^total_bits); 

% 2) 계산이 끝난 후 fprintf 호출
fprintf('\n====================================================================\n');
fprintf('  📊 [Simulation Parameter & Architecture Check] \n');
fprintf('====================================================================\n');
fprintf('  ▶ 1st-Stage System (6-bit SAR Target):\n');
fprintf('    - CDAC Unit Cap Mismatch (sigma) : %.4f %%\n', sigma_u_1st * 100);
fprintf('    - Comparator Thermal Noise (rms) : %.2f uV  (≈ %.2f LSB_15bit)\n', CMPno_1st * 1e6, CMPno_1st / lsb_15bit);
fprintf('    - Comparator Offset Mismatch     : %.2f mV  (≈ %.2f LSB_15bit)\n', CMPoff_1st * 1e3, CMPoff_1st / lsb_15bit);
fprintf('--------------------------------------------------------------------\n');
fprintf('  ▶ 2nd-Stage System (9-bit SAR Target):\n');
fprintf('    - CDAC Unit Cap Mismatch (sigma) : %.4f %%\n', sigma_u_2nd * 100);
fprintf('    - Comparator Thermal Noise (rms) : %.2f mV  (≈ %.2f LSB_15bit)\n', CMPno_2nd * 1e3, CMPno_2nd / lsb_15bit);
fprintf('    - Comparator Offset Mismatch     : %.2f mV  (≈ %.2f LSB_15bit)\n', CMPoff_2nd * 1e3, CMPoff_2nd / lsb_15bit);
fprintf('====================================================================\n');

fprintf('📊 평균 ENOB: %.2f bits (표준편차: %.3f)\n', mean(enob_history), std(enob_history));
fprintf('📊 평균 SNDR: %.2f dB\n', mean(sndr_history));
fprintf('====================================================================\n');

%% 3. 통계 결과 플롯 시각화
figure('Color', 'w', 'Position', [100, 100, 1100, 450], 'Name', 'ENOB Monte Carlo Analysis');

% 3-1. 수렴 및 변화 추이 (Run Chart)
subplot(1, 2, 1);
plot(1:num_simulations, enob_history, '-o', 'LineWidth', 1.5, 'Color', [0 0.45 0.74], ...
     'MarkerFaceColor', [0 0.45 0.74], 'MarkerSize', 5);
hold on;
plot([1, num_simulations], [mean(enob_history), mean(enob_history)], 'r--', 'LineWidth', 2);
grid on;
xlabel('Simulation Run #');
ylabel('ENOB (bits)');
title('ENOB Variation Trend');
legend('Measured ENOB', sprintf('Mean: %.2f bits', mean(enob_history)), 'Location', 'best');

% 3-2. 분포 확인 (Histogram)
subplot(1, 2, 2);
histogram(enob_history, 'FaceColor', [0.4660 0.6740 0.1880], 'EdgeColor', 'w');
grid on;
xlabel('ENOB (bits)');
ylabel('Counts');
title(sprintf('ENOB Distribution'));
sgtitle('Pipeline SAR ADC Yield & Performance Statistics', 'FontSize', 14, 'FontWeight', 'bold');