function pose = matchScansICP(laser, map, InitialPose)

MaxIterations = 400;
ValidDistance = 200; % [mm]
ScoreTolerance = 1e-6;

x = InitialPose(1);
y = InitialPose(2);
theta = InitialPose(3);
t = [x; y];
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
laser = R * laser + t;

Eprev = Inf;
count = 0;
while count < MaxIterations
    % 最近傍点を探索
    KDTree1 = KDTreeSearcher(map');
    % KDTree2 = KDTreeSearcher(laser');
    [index, distance] = knnsearch(KDTree1, laser');

    % 有効な点のみを抽出する
    laser_valid = laser(:, distance < ValidDistance);
    index = index(distance < ValidDistance);

    % 誤差関数E(x)の値を計算
    E = 0;
    for i = 1:size(laser_valid, 2)
        E = E + norm(map(:, index(i)) - laser_valid(:, i))^2;
    end
    E = E / size(laser_valid, 2);
    
    % 収束判定
    deltaE = abs(Eprev - E);
    if deltaE < ScoreTolerance
        break
    end
    Eprev = E;

    % 最適化
    map_ = map(:, index);
    mu_map = [mean(map_(1, :)); mean(map_(2, :))];
    map_ = map_ - mu_map;
    mu_laser = [mean(laser_valid(1, :)); mean(laser_valid(2, :))];
    laser_ = laser_valid - mu_laser;

    W = map_ * laser_';
    [U, ~, V] = svd(W);
    R1 = U * V';
    t1 = mu_map - R1 * mu_laser;

    % 姿勢と点群を更新
    R = R1 * R;
    t = t + t1;
    laser = R1 * laser + t1;

    count = count + 1;
end

eul = rotm2eul([R [0; 0]; 0 0 1]);
pose = [t(1) t(2) eul(1)];

disp([num2str(count), '回反復しました。'])

end