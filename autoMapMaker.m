clear
clc

lidar = serialport('COM5', 115200, Timeout=0.1);
setURG(lidar);

sum = zeros([2, 1081]);
for i = 1:200
    angles = deg2rad(-45:360/1440:225);
    ranges = LidarScan(lidar);
    scan = lidarScan(ranges, angles);
    laser_points = scan.Cartesian';
    sum = sum + laser_points;
end
sum = sum / 200;

writeline(lidar, 'QT');

writematrix(sum, 'room.csv')
daspect([1 1 1])
plot(sum(1, :), sum(2, :), '.')
xlabel('x [mm]')
ylabel('y [mm]')
