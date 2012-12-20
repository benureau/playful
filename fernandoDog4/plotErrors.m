clear all; 
close all; 
load restrictedErrors.txt; 
load unrestrictedErrors.txt; 

r = restrictedErrors; 
u = unrestrictedErrors; 
plot(r,'r')
hold on
plot(u,'g')

figure(2)
load fitness.txt
plot(fitness)

figure(3) 
load upredictWF.txt
imagesc(upredictWF)

figure(4) 
load predictWF.txt
imagesc(predictWF)

figure(5) 
load actorWF.txt
%imagesc(actorWF)
surf(actorWF)