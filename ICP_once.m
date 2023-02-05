clear
clc

N = 90; % 地図の点の数
M = 30; % LRFの点の数

% 地図の点群
th = linspace(0, 2 * pi, N);
p1 = 75 * [cos(th); sin(th)] + [0; 1000];
p2 = 50 * [cos(th); sin(th)] + [400; 2500];
w1 = [linspace(-1500, -1050); 360 * ones([1, 100])];
w2 = [-1050 * ones([1, 100]); linspace(360, 700)];
map_points = [p1 p2 w1 w2];

% R = eye(2); % LRFの初期角度
% t = [200; 200]; % LRFの初期位置 [mm]

% レーザーの点群
% th = linspace(-pi, 0, M);
% p1 = 75 * [cos(th); sin(th)] + [0; 1000];
% p2 = 50 * [cos(th); sin(th)] + [500; 1000];
% laser_points = [p1 p2];
% laser_points = inv(R) * laser_points - t + 15 * randn([2, 60]);

% LRFの初期化
lidar = serialport('COM5', 115200, 'Timeout', 0.1);
setURG(lidar);

R = eye(2);
t = [0; 0];
angles = deg2rad(-45:360/1440:225);
ranges = LidarScan(lidar);
scan = lidarScan(ranges, angles);
scan = removeInvalidData(scan, 'RangeLimits', [100 30000]);
laser_points = scan.Cartesian';
laser_points = R * laser_points + t;

Eprev = 0;
count = 0;
while count < 10
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
    E = E / size(laser_points_valid, 2)

    % プロット
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
    ylim([0 3000])
    xlabel('x [mm]')
    ylabel('y [mm]')
    legend('map', 'laser', 'c')
    pause(0.5)

    if E < 5000
        break
    end

    % 最適化
    map_points_ = map_points(:, index);
    mu_map = [mean(map_points_(1, :)); mean(map_points_(2, :))];
    map_points_ = map_points_ - mu_map;
    mu_laser = [mean(laser_points_valid(1, :)); mean(laser_points_valid(2, :))];
    laser_points_ = laser_points_valid - mu_laser;

    W = map_points_ * laser_points_valid';
    [U, S, V] = svd(W);
    R1 = U * V';
    t1 = mu_map - R1 * mu_laser;

    % 局所最適解からの脱却
    % dE = abs(E - Eprev);
    % Eprev = E;
    % if dE < 10 % 局所最適解に陥っている
    %     theta = rand() - 0.5;
    %     R1 = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    %     t1 = 10 * (rand([2, 1]) - [0.5; 0.5]);
    % end

    % 点群を更新
    laser_points = R1 * laser_points + t1;
    R = R1 * R
    t = t + t1

    count = count + 1;
end

x = t(1);
y = t(2);
eul = rotm2eul([R [0; 0]; 0 0 1]);
x = [x y eul(1)]
disp([num2str(count), '回反復しました。'])
writeline(lidar, 'QT'); % 計測停止
%waitforbuttonpress
close
