function [rangescan] = LidarScan(lidar)

    writeline(lidar, 'GD0000108000');
    data = read(lidar, 3369, 'char');

    i = find(data == data(13)); % LFのインデックス
    rangedata = data(i(3)+1:end-1); % 3つ目のLFの次からがデータブロック

    % SUMとLFを取り除く
    for j = 0:49
        onlyrangedata((64*j)+1:(64*j)+64) = rangedata(1+(66*j):64+(66*j));
    end
    length = numel(rangedata);
    n = length - 66*50 - 2; % 余りnバイト
    onlyrangedata((64*50)+1:(64*50)+n) = rangedata(1+(66*50):n+(66*50));

    j = 0;
    encodeddist = zeros(1081,3);
    for i = 1:floor(numel(onlyrangedata)/3)
        encodeddist(i,:) = [onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
        j = j+1;
    end

    rangescan = zeros(1081,1);
    for k = 1:size(encodeddist,1)
        rangescan(k) = decodeSCIP(encodeddist(k,:));
    end

end