clc;clear; close all
num = 7;
ahrs = readmatrix(num2str(num) + "/ahrs.csv");
env = readmatrix(num2str(num) + "/env.csv");
ekf = readmatrix(num2str(num) + "/EKF.csv");

% 
% figure(10)
% plot(env(:,1),env(:,20))
% hold on
% plot(env(:,1),env(:,14))
% 
% figure(11)
% plot(env(:,1),env(:,21))
% hold on
% plot(env(:,1),env(:,15))
% 
% figure(12)
% plot(env(:,1),env(:,22))
% hold on
% plot(env(:,1),env(:,16))

% figure(7)
% plot(ahrs(:,1),ahrs(:,2))
% hold on
% plot(env(:,1),env(:,5))
% legend("ahrs", "env")
% title("Roll")
% 
% figure(8)
% plot(ahrs(:,1),ahrs(:,3))
% hold on
% plot(env(:,1),env(:,6))
% legend("ahrs", "env")
% title("Pitch")
% 
% figure(9)
% plot(ahrs(:,1),ahrs(:,4))
% hold on
% plot(env(:,1),env(:,7))
% legend("ahrs", "env")
% title("Yaw")

% figure(97)
% plot(ahrs(:,1),ahrs(:,5))
% hold on
% plot(ahrs(:,1),ahrs(:,8))
% plot(env(:,1),env(:,5))
% legend("gyro", "acc", "env")
% title("Roll")
% 
% figure(98)
% plot(ahrs(:,1),ahrs(:,6))
% hold on
% plot(ahrs(:,1),ahrs(:,9))
% plot(env(:,1),env(:,6))
% legend("gyro", "acc", "env")
% title("Pitch")
% 
% figure(99)
% plot(ahrs(:,1),ahrs(:,7))
% hold on
% plot(ahrs(:,1),ahrs(:,10))
% plot(env(:,1),env(:,7))
% legend("gyro", "acc", "env")
% title("Yaw")
% 
% figure(1)
% plot(ekf(:,1),ekf(:,2))
% hold on
% plot(env(:,1),env(:,2))
% legend("ekf", "env")
% title("X")
% 
% figure(2)
% plot(ekf(:,1),ekf(:,3))
% hold on
% plot(env(:,1),env(:,3))
% legend("ekf", "env")
% title("Y")

figure(3)
plot(ekf(:,1),ekf(:,4))
hold on
plot(env(:,1),env(:,4))
legend("ekf", "env")
title("Z")

% 
% figure(4)
% plot(ekf(:,1),ekf(:,5))
% hold on
% plot(env(:,1),env(:,8))
% legend("ekf", "env")
% title("VX")
% 
% figure(5)
% plot(ekf(:,1),ekf(:,6))
% hold on
% plot(env(:,1),env(:,9))
% legend("ekf", "env")
% title("VY")
% 
figure(6)
plot(ekf(:,1),ekf(:,7))
hold on
plot(env(:,1),env(:,10))
legend("ekf", "env")
title("VZ")

