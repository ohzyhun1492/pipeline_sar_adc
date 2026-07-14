%% ========================================================================
%% 메인 스크립트: Vin Parametric Sweep & 순수 simout 파형 plot
%% ========================================================================

%%%script_spectrum의 9번 코드와 1번 코드의 Vin을 주석처리 하고 돌릴 것

clc; clear; close all;

% 1. Vin 가변 배열 정의 (-1부터 1까지 0.1 간격, 총 21개)
Vin_array = -1:0.1:1; 
total_runs = length(Vin_array);

% 2. 시각화를 위한 Figure 설정 (하나의 그래프에 누적)
figure('Name', 'Simulink Output Waveform Sweep', 'Color', 'w');
hold on; grid on;
colors = jet(total_runs); 

disp('====================================================');
disp('   Simulink Parametric Sweep 시뮬레이션 시작   ');
disp('====================================================');

% 3. 루프 구동
for idx = 1:total_runs
    
    % [방어] 서브 스크립트의 clear 공격 대비 제어 변수 실시간 복구
    if ~exist('Vin_array', 'var'), Vin_array = -1:0.1:1; total_runs = 21; end
    if ~exist('colors', 'var'),    colors = jet(total_runs); end
    
    % 현재 루프의 Vin 설정 및 Base Workspace 주입
    Vin = Vin_array(idx); 
    assignin('base', 'Vin', Vin); 
    
    % -----------------------------------------------------------------
    % 변수 세팅부터 시뮬레이션까지 다 해주는 서브 스크립트 실행
    script_spectrum_v2; 
    % -----------------------------------------------------------------
    
    % ★ 핵심 요구사항: 각 Vin 조건별 시뮬레이션이 끝날 때마다 명령창에 즉시 표시
    fprintf('[완료] [%2d/%2d] 조건 실행 끝 -> Vin = %+.1f V\n', idx, total_runs, Vin);
    
    % [방어] 실행 직후 지워진 그래픽 변수 긴급 수금
    if ~exist('colors', 'var')
        total_runs = 21; 
        colors = jet(total_runs); Vin_array = -1:0.1:1;
    end
    
    % 4. 순수 simout 데이터 적출 (구조체든 변수든 다 찾아냄)
    current_simout = [];
    if exist('simout', 'var')
        current_simout = simout;
    elseif exist('out', 'var') && isprop(out, 'simout')
        current_simout = out.get('simout');
    elseif exist('ans', 'var') && isprop(ans, 'simout')
        current_simout = ans.get('simout');
    end
    
    % 5. 주요 지점(-1, 0, 1)만 범례 이름 설정
    if abs(Vin - (-1)) < 1e-4 || abs(Vin - 0) < 1e-4 || abs(Vin - 1) < 1e-4
        plot_disp = sprintf('Vin = %.1f V', Vin);
    else
        plot_disp = ''; 
    end
    
    % 6. 하나의 그래프에 누적 plot (Timeseries 및 일반 배열 대응)
    if isa(current_simout, 'timeseries')
        plot(current_simout.Time * 1e6, current_simout.Data, 'Color', colors(idx, :), 'LineWidth', 1.2, 'DisplayName', plot_disp);
        x_label_str = 'Time (\mus)';
    else
        plot(current_simout, 'Color', colors(idx, :), 'LineWidth', 1.2, 'DisplayName', plot_disp);
        x_label_str = 'Sample Index';
    end
end

% 7. 그래프 레이아웃 마감
hold off;
xlabel(x_label_str);
ylabel('Output Signal Amplitude (V)');
title('Simulink simout Output Signal Waveform (Parametric Sweep)');
legend('show', 'Location', 'best');

disp('====================================================');
disp('   모든 시뮬레이션 및 파형 누적 plot 완료.   ');
disp('====================================================');