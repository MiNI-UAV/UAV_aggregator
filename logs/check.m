clc;clear;
ahrs = readmatrix("ahrs.csv");
env = readmatrix("env.csv");

figure(1)
plot(ahrs(:,1),ahrs(:,2))
hold on
plot(env(:,1),env(:,5))
title("Roll")

figure(2)
plot(ahrs(:,1),ahrs(:,3))
hold on
plot(env(:,1),env(:,6))
title("Pitch")

figure(3)
plot(ahrs(:,1),ahrs(:,4))
hold on
plot(env(:,1),env(:,7))
title("Yaw")
