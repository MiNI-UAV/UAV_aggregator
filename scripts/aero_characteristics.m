clc;clear; close all
num = 1697715036;
drone_name = "Maurice_2";
aoa = readmatrix("../logs/" + num2str(num) + "/" + drone_name + "/aoa.csv");
aos = readmatrix("../logs/" + num2str(num) + "/" + drone_name + "/aos.csv");


figure(1)
plot(rad2deg(aoa(:,1)),-aoa(:,4))
title("Lift cofficient")
xlabel("AOA [deg]");
ylabel("CL");

