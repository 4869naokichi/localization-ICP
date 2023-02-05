close
clear
clc

A = imread('corridor.png');
A = rgb2gray(A);
L = ~logical(A);
map = [];
for i = 1:size(L, 1)
    for j = 1:size(L, 2)
        if L(i, j)
            map = [map [i; j]];
        end
    end
end

plot(map(1, :), map(2, :), '.')
daspect([1 1 1])
xlabel('x [mm]')
ylabel('y [mm]')

writematrix(map, 'corridor.csv')