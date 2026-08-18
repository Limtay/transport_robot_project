function opt_slider_crank()
    clear; clc;

    %% 1. 설계 변수 범위 설정 (Bounds)
    % x = [r(크랭크), l(커넥팅로드), s(스포크), e(오프셋)]
    % 오프셋(e)은 초반 변위 극대화를 위해 음수 영역으로 제한합니다.
    lb = [5,   20,  0,  -30]; % 하한값
    ub = [25,  60,  30,   0]; % 상한값

    %% 2. 유전 알고리즘(GA) 옵션 설정
    options = optimoptions('ga', ...
        'PopulationSize', 100, ...
        'MaxGenerations', 150, ...
        'Display', 'iter', ...
        'FunctionTolerance', 1e-4, ...
        'ConstraintTolerance', 1e-3);

    %% 3. 최적화 실행
    disp('최적화를 시작합니다...');
    [x_opt, fval] = ga(@objective_function, 4, [], [], [], [], lb, ub, @constraints, options);

    %% 4. 결과 출력
    r_opt = x_opt(1); l_opt = x_opt(2); s_opt = x_opt(3); e_opt = x_opt(4);

    fprintf('\n================ 최적화 결과 ================\n');
    fprintf('크랭크 반경 (r): %.2f mm\n', r_opt);
    fprintf('커넥팅 로드 (l): %.2f mm\n', l_opt);
    fprintf('스포크 길이 (s): %.2f mm\n', s_opt);
    fprintf('오프셋 편심 (e): %.2f mm\n', e_opt);

    % 각도별 도달 반경 검증
    D0   = calc_distance(r_opt, l_opt, s_opt, e_opt, 0);
    D60  = calc_distance(r_opt, l_opt, s_opt, e_opt, 60);
    D120 = calc_distance(r_opt, l_opt, s_opt, e_opt, 120);

    fprintf('\n[구동 구간별 상태]\n');
    fprintf('0도 (최대 확장): %.2f mm\n', D0);
    fprintf('60도 (수납 진입): %.2f mm\n', D60);
    fprintf('120도 (완전 수납): %.2f mm\n', D120);

    fprintf('\n[구간별 변위량(수축량)]\n');
    fprintf('0~60도 수축량: %.2f mm\n', D0 - D60);
    fprintf('60~120도 수축량: %.2f mm\n', D60 - D120);

    % 전체 구간 최대 도달 반경 검증 (하우징 간섭 확인용)
    [Dmax_retract, theta_at_max] = max_distance_in_range(r_opt, l_opt, s_opt, e_opt, 60, 120);
    fprintf('\n[60~120도 구간 최대 반경] %.2f mm (theta=%.1f도)\n', Dmax_retract, theta_at_max);
    fprintf('=============================================\n');
end

%% ====== 목적 함수 (Objective Function) ======
function f = objective_function(x)
    r = x(1); l = x(2); s = x(3); e = x(4);

    % 각도별 원점으로부터의 거리
    D0   = calc_distance(r, l, s, e, 0);
    D60  = calc_distance(r, l, s, e, 60);
    D120 = calc_distance(r, l, s, e, 120);

    % 에러 발생(허수) 시 페널티 부여
    if isnan(D0) || isnan(D60) || isnan(D120)
        f = 1e6; return;
    end

    % 구간별 변위 계산
    Disp_0_60 = D0 - D60;
    Disp_60_120 = D60 - D120;

    % [목표] 초반 변위(Disp_0_60)가 후반 변위(Disp_60_120)보다 최대한 커지도록 설계
    % 최솟값을 찾는 알고리즘이므로, (초반 변위 - 후반 변위)에 음수를 취해 최대화함
    f = -(Disp_0_60 - Disp_60_120);
end

%% ====== 비선형 구속 조건 (Nonlinear Constraints) ======
function [c, ceq] = constraints(x)
    r = x(1); l = x(2); s = x(3); e = x(4);
    ceq = [];

    % [보완 1] 제약 벡터 길이를 항상 일정하게 유지 (총 7개)
    % ga는 평가마다 제약 벡터 길이가 동일하길 기대하므로,
    % NaN(조립 불가)일 때도 모든 항을 위반(양수)으로 채워 도태시킵니다.
    NUM_C = 7;

    % 각도별 계산
    D0   = calc_distance(r, l, s, e, 0);
    D60  = calc_distance(r, l, s, e, 60);
    D120 = calc_distance(r, l, s, e, 120);

    if isnan(D0) || isnan(D60) || isnan(D120)
        c = ones(NUM_C, 1); return; % 계산 불가 시 전 항목 위반 처리하여 도태
    end

    c = zeros(NUM_C, 1);

    % 1. 기구학적 구속 (Locking 방지)
    % 커넥팅 로드가 가장 심하게 꺾일 때도 허수가 나오지 않도록 안전 마진 확보
    max_eccentricity = abs(e) + r;
    c(1) = max_eccentricity - l + 2; % l >= |e| + r + 2mm

    % 2. 60도~120도 구간 하우징 수납 조건 (반지름 50mm 이내)
    % [보완 2] 끝점(60,120도)뿐 아니라 구간 내부까지 촘촘히 검사하여
    % 샘플 사이에서 하우징을 뚫고 나가는 경우를 방지합니다.
    Dmax_retract = max_distance_in_range(r, l, s, e, 60, 120);
    c(2) = Dmax_retract - 50; % 60~120도 전 구간에서 50mm 이하

    % 3. 0도일 때 최소 확장 조건
    % 확장 시 적어도 50mm 보다는 밖으로 튀어나와야 의미가 있음
    c(3) = 51 - D0; % D0 >= 51

    % 4. [보완 3] 단조 수납 보장 (펼침 -> 수납이 역전되지 않도록)
    % D0 >= D60 >= D120 강제. 구간 내부 단조성까지 보장하기 위해
    % 0~120도 전체에서 도달 반경이 증가하는 지점이 없는지 검사합니다.
    max_increase = max_increase_in_range(r, l, s, e, 0, 120);
    c(4) = max_increase;        % 인접 샘플 간 반경 증가량 <= 0
    c(5) = D60 - D0;            % D0 >= D60 (끝점 기준 보조 제약)
    c(6) = D120 - D60;          % D60 >= D120 (끝점 기준 보조 제약)

    % 5. 의미 있는 초반 변위 확보 (선택적 안전장치)
    % 0~60도 수축량이 0보다 커야 "초반에 펼쳐졌다 줄어드는" 동작이 성립
    c(7) = -(D0 - D60);         % D0 - D60 >= 0
end

%% ====== 기구학 거리 계산 헬퍼 함수 ======
function D = calc_distance(r, l, s, e, theta_deg)
    theta = deg2rad(theta_deg);

    % 슬라이더 핀까지의 수평 거리 계산식 내 제곱근 내부
    radicand = l^2 - (e - r*sin(theta))^2;

    if radicand < 0
        D = NaN; % 조립 불가능한 기구학적 특이점
    else
        % 슬라이더 핀의 x좌표
        x_pin = r*cos(theta) + sqrt(radicand);

        % 스포크 끝단의 좌표는 (x_pin + s, e)
        % 원점(0,0)으로부터 스포크 끝단까지의 직선 거리 반환
        D = sqrt((x_pin + s)^2 + e^2);
    end
end

%% ====== [보완용] 구간 내 최대 도달 반경 (촘촘한 샘플링) ======
function [Dmax, theta_at_max] = max_distance_in_range(r, l, s, e, th_start, th_end)
    % 끝점만 보면 구간 사이에서 하우징을 뚫는 형상을 놓칠 수 있으므로
    % 1도 간격으로 촘촘히 평가하여 구간 내 최대 반경을 반환합니다.
    thetas = th_start:1:th_end;
    Dmax = -Inf;
    theta_at_max = th_start;

    for th = thetas
        D = calc_distance(r, l, s, e, th);
        if isnan(D)
            Dmax = NaN; theta_at_max = th; return; % 조립 불가 구간 존재
        end
        if D > Dmax
            Dmax = D;
            theta_at_max = th;
        end
    end
end

%% ====== [보완용] 구간 내 최대 반경 증가량 (단조 수납 검사) ======
function max_inc = max_increase_in_range(r, l, s, e, th_start, th_end)
    % 인접한 각도 사이에서 도달 반경이 "증가"하는 최대량을 반환합니다.
    % 펼침 -> 수납 동작이라면 항상 감소해야 하므로 이 값이 0 이하여야 정상입니다.
    thetas = th_start:1:th_end;
    max_inc = -Inf;
    D_prev = calc_distance(r, l, s, e, thetas(1));

    if isnan(D_prev)
        max_inc = 1; return; % 조립 불가 -> 위반 처리
    end

    for k = 2:numel(thetas)
        D_cur = calc_distance(r, l, s, e, thetas(k));
        if isnan(D_cur)
            max_inc = 1; return; % 조립 불가 -> 위반 처리
        end
        inc = D_cur - D_prev; % 양수면 반경이 증가(역전)했다는 의미
        if inc > max_inc
            max_inc = inc;
        end
        D_prev = D_cur;
    end
end
