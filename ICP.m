clear
clc

% 地図の読み込み
map = readmatrix('csv/field.csv');
map = [0 -1; 1 0] * map + [4000; 1000];

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

    % 終了キーが押されたかの判定
    if strcmp(get(hf, 'currentcharacter'), 'q')
        close(hf)
        break
    end
end

writeline(lidar, 'QT'); % 計測停止