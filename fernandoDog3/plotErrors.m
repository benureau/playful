clear all; 
close all; 
load restrictedErrors.txt; 
load unrestrictedErrors.txt; 

r = restrictedErrors; 
u = unrestrictedErrors; 
plot(u)
hold on
plot(r,'r')


