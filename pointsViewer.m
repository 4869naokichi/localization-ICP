clear
clc

% 地図の読み込み
map_points = readmatrix('corridor.csv');

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

    % プロット
    clf
    hold on
    daspect([1 1 1])
    plot(map_points(1, :), map_points(2, :), '.')
    plot(laser_points(1, :), laser_points(2, :), '.')
    xlabel('x [mm]')
    ylabel('y [mm]')

    % 終了キーが押されたかの判定
    if strcmp(get(hf, 'currentcharacter'), 'q')
        close(hf)
        break
    end
end

writeline(lidar, 'QT'); % 計測停止
