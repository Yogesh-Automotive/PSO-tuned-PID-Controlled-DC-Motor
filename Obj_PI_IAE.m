function [Fval] =Obj_PI_IAE(x)
%____________________________________________________________________
global Kp;
global Ki;
%____________________________________________________________________
Kp=abs(x(1));
Ki=abs(x(2));
%Integral of Squared Error
%____________________________________________________________________
T=20;
[t,y] = sim('DC_Motor_SC',T); % Step response of closed-loop
load Error;
%______Integral of Absolute Error (IAE)_____________
L=length(Error(2,:));
for i=1:L
ERR(:,i) = abs(Error(2,i));
end
IAE=trapz(ERR);
Fval(1)=IAE;

fprintf(' and Fitness Value =%7.5f',Fval);
fprintf(' [Cromosomes Value x(1)=%7.5f',x(1));
fprintf(' x(2)=%7.5f]',x(2));
%____________________________________________________________________