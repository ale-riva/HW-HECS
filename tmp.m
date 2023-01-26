clear all
close all
clc
den1 = [1 0.5];
den2 = conv(den1,den1)
cl = tf(1,conv(den2,[1 0.14 0.01]))
pzmap(cl)
bode(cl)