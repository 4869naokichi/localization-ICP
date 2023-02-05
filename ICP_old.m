clear
clc

N = 90; % 地図の点の数
M = 50; % LRFの点の数

% 地図の点群
% th = linspace(0, 2 * pi, N);
% p1 = 75 * [cos(th); sin(th)] + [0; 1000];
% p2 = 50 * [cos(th); sin(th)] + [400; 2500];
% w1 = [linspace(-1500, -1050); 360 * ones([1, 100])];
% w2 = [-1050 * ones([1, 100]); linspace(360, 700)];
% map_points = [p1 w1 w2];
map_points = readmatrix('corridor.csv');

% LRFの初期化
lidar = serialport('COM5', 115200, 'Timeout', 0.1);
setURG(lidar);

R = eye(2);
t = [0; 0];

hf = figure;
while true
    % LRFの点群を取得
    angles = deg2rad(-45:360/1440:225);
    ranges = LidarScan(lidar);
    scan = lidarScan(ranges, angles);
    laser_points = scan.Cartesian';
    laser_points = R * laser_points + t;

    Eprev = 0;
    count = 0;
    while count < 50
        % 最近傍点を探索
        KDTree1 = KDTreeSearcher(map_points');
        KDTree2 = KDTreeSearcher(laser_points');
        [index, distance] = knnsearch(KDTree1, laser_points');

        % 有効な点のみを抽出する
        laser_points_valid = laser_points(:, distance < 200);
        index = index(distance < 200);

        % 誤差関数E(x)の値を計算
        E = 0;
        for i = 1:size(laser_points_valid, 2)
            E = E + norm(map_points(:, index(i)) - laser_points_valid(:, i))^2;
        end
        clc
        E = E / size(laser_points_valid, 2);
        disp(E)

        % プロット
        %{
        clf
        hold on
        daspect([1 1 1])
        plot(map_points(1, index), map_points(2, index), '.')
        plot(laser_points_valid(1, :), laser_points_valid(2, :), '.')
        for i = 1:size(laser_points_valid, 2)
            plot([laser_points_valid(1, i), map_points(1, index(i))], [laser_points_valid(2, i), map_points(2, index(i))], 'g')
        end
        plot(t(1), t(2), 'bo')
        xlim([-1500 1000])
        ylim([0 1500])
        xlabel('x [mm]')
        ylabel('y [mm]')
        legend('map', 'laser')
        pause(0)
        %}

        if E < 4500
            break
        end

        % 最適化
        map_points_ = map_points(:, index);
        mu_map = [mean(map_points_(1, :)); mean(map_points_(2, :))];
        map_points_ = map_points_ - mu_map;
        mu_laser = [mean(laser_points_valid(1, :)); mean(laser_points_valid(2, :))];
        laser_points_ = laser_points_valid - mu_laser;

        W = map_points_ * laser_points_';
        [U, S, V] = svd(W);
        R1 = U * V';
        t1 = mu_map - R1 * mu_laser;

        % 局所最適解からの脱却
        dE = abs(E - Eprev);
        Eprev = E;
        if dE < 1 % 局所最適解に陥っている
            theta = 0.1 * (rand() - 0.5);
            R1 = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            t1 = 1 * (rand([2, 1]) - [0.5; 0.5]);
        end

        % 点群を更新
        laser_points = R1 * laser_points + t1;
        R = R1 * R;
        t = t + t1;

        count = count + 1;
    end
    x = t(1);
    y = t(2);
    eul = rotm2eul([R [0; 0]; 0 0 1]);
    x = [x y eul(1)];
    disp(x)
    disp([num2str(count), '回反復しました。'])

    % 終了キーが押されたかの判定
    if strcmp(get(hf, 'currentcharacter'), 'q')
        close(hf)
        break
    end
    figure(hf)
    drawnow
end

writeline(lidar, 'QT'); % 計測停止
