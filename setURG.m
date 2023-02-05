function [] = setURG(lidar)
    configureTerminator(lidar, 'LF', 'LF')
    writeline(lidar, 'SCIP2.0');
    readline(lidar);
    readline(lidar);
    readline(lidar);
    writeline(lidar, 'VV');
    readline(lidar);
    readline(lidar);
    readline(lidar);
    readline(lidar);
    readline(lidar);
    readline(lidar);
    readline(lidar);
    readline(lidar);
    writeline(lidar, 'BM');
    readline(lidar);
    readline(lidar);
    readline(lidar);
end