%% Tunning of PID controller using Particle Swarm Optimization 
clc;
global Kp;
global Ki;
global Kd;
%Lower Limit L 
L=[0 0 0];
%Upper Limit U 
U=[5 5 5];
Kp_min=0;Kp_max=10;
Ki_min=0;Ki_max=5;
Kd_min=0;Kd_max=10;
n = 20;           % Size of the swarm " no of birds "
bird_setp  =20;   % Maximum number of "birds steps"
dim = 3;          % Dimension of the problem

c2 =1.2;          % PSO parameter C1 
c1 = 0.12;        % PSO parameter C2 
w =0.9;           % pso momentum or inertia  
fitness=0*ones(n,bird_setp);
                                       %-----------------------------%
                                       %    initialize the parameter %
                                       %-----------------------------%                                       
R1 = rand(dim, n);
R2 = rand(dim, n);
current_fitness =0*ones(n,1);
                                 %------------------------------------------------%
                                 % Initializing swarm and velocities and position %
                                 %------------------------------------------------%
% KP=(Kp_max*rand(1,n))-Kp_min;
% KI=(Ki_max*rand(1,n))-Ki_min;
% KD=(Kd_max*rand(1,n))-Kd_min;
% current_position = [KP;KI;KD];

 Len=length(L);
 for i=1:Len
 X(i,:)=(U(i)*rand(1,n))-L(i);
 end 
 current_position = [X];

velocity = .3*randn(dim, n);
local_best_position  = current_position;
                                 %-------------------------------------------%
                                 %     Evaluate initial population           %
                                 %-------------------------------------------%
for i = 1:n
    current_fitness(i) = Obj_PID_IAE(current_position(:,i));
end

local_best_fitness = current_fitness;
[global_best_fitness,g] = min(local_best_fitness);

for i=1:n
    globl_best_position(:,i) = local_best_position(:,g) ;
end
                                               %-------------------%
                                               %  VELOCITY UPDATE  %
                                               %-------------------%
velocity = w *velocity + c1*(R1.*(local_best_position-current_position)) + c2*(R2.*(globl_best_position-current_position));
                                               %------------------%
                                               %   SWARMUPDATE    %
                                               %------------------%                                                         
current_position = current_position + velocity ;
                                               %------------------------%
                                               %  evaluate anew swarm   %
                                               %------------------------%                                            

%% Main Loop
iter = 0 ;        % Iterations’counter
while  ( iter < bird_setp )
iter = iter + 1;

for i = 1:n,
current_fitness(i) = Obj_PID_IAE(current_position(:,i)) ;    
end

for i = 1 : n
        if current_fitness(i) < local_best_fitness(i)
           local_best_fitness(i)  = current_fitness(i);  
           local_best_position(:,i) = current_position(:,i)   ;
        end   
end
  
 [current_global_best_fitness,g] = min(local_best_fitness);  
    
if current_global_best_fitness < global_best_fitness
   global_best_fitness = current_global_best_fitness;
   
    for i=1:n
        globl_best_position(:,i) = local_best_position(:,g);
    end   
end
    Local_best_F(iter)=current_global_best_fitness;
    Global_best_F(iter)=global_best_fitness;
 velocity = w *velocity + c1*(R1.*(local_best_position-current_position)) + c2*(R2.*(globl_best_position-current_position));
 current_position = current_position + velocity;
 fprintf('\nThe value of interation iter %3.0f\n', iter );

end % end of while loop its mean the end of all step that the birds move it 
            xx=fitness(:,n);
            [Y,I] = min(xx);
            current_position(:,I);            
fprintf('\nThe value of current position %3.2f\n',current_position(:,I));
plot([1:bird_setp],Local_best_F(:),'-o',[1:bird_setp],Global_best_F(:),'-x');
%