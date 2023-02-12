clear
clc

ValidDistance = 400; % [mm]

% 地図の読み込み
map_points = readmatrix('csv/field.csv');
map_points = [0 -1; 1 0] * map_points + [4000; 1000];

% LRFの初期化
lidar = serialport('COM5', 115200, Timeout=0.1);
setURG(lidar);

hf = figure;
while true
    % LRFの点群を取得
    angles = deg2rad(-45:360/1440:225);
    ranges = LidarScan(lidar);
    scan = lidarScan(ranges, angles);
    laser_points = scan.Cartesian';

    % 最近傍点を探索
    KDTree1 = KDTreeSearcher(map_points');
    KDTree2 = KDTreeSearcher(laser_points');
    [index, distance] = knnsearch(KDTree1, laser_points');

    % 有効な点のみを抽出する
    laser_points_valid = laser_points(:, distance < ValidDistance);
    index = index(distance < ValidDistance);

    % プロット
    clf
    hold on
    daspect([1 1 1])
    plot(map_points(1, :), map_points(2, :), '.')
    plot(laser_points_valid(1, :), laser_points_valid(2, :), '.')
    for i = 1:size(laser_points_valid, 2)
        plot([laser_points_valid(1, i), map_points(1, index(i))], [laser_points_valid(2, i), map_points(2, index(i))], 'g')
    end
    xlabel('x [mm]')
    ylabel('y [mm]')

    % 終了キーが押されたかの判定
    if strcmp(get(hf, 'currentcharacter'), 'q')
        close(hf)
        break
    end
end

writeline(lidar, 'QT'); % 計測停止
