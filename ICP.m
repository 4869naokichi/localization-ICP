clear
clc

% 地図の読み込み
map = readmatrix('room.csv');

% LRFの初期位置
pose = [0 0 0];

% LRFの初期化
lidar = serialport('COM5', 115200, Timeout=0.1);
setURG(lidar);

hf = figure;
while true
    % LRFの点群を取得
    angles = deg2rad(-45:360/1440:225);
    ranges = LidarScan(lidar);
    laser = lidarScan(ranges, angles);
    laser = laser.Cartesian';

    % LRFの姿勢を推定
    pose = matchScansICP(laser, map, pose);

    % 結果を表示
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    clf
    hold on
    daspect([1 1 1])
    plot(x, y, 'bo')
    quiver(x, y, 500 * cos(theta), 500 * sin(theta))
    xlim([-3000 3000])
    ylim([-3000 3000])
    xlabel('x [mm]')
    ylabel('y [mm]')

    % 終了キーが押されたかの判定
    if strcmp(get(hf, 'currentcharacter'), 'q')
        close(hf)
        break
    end
end

writeline(lidar, 'QT'); % 計測停止