function [Q,P,X] = LG_traj_min_ayQ(X0,X1)
maxForce = 6;  %Maximum actuator forces
duration = X1.time - X0.time;
L = 0.8;
problem.func.dynamics = @(t,x,u)( LG_cartPoleDynamics(x,u) );
problem.func.pathObj = @(t,x,u)( u.^2 );  %Force-squared cost function
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;
% problem.bounds.finalTime.low = 3;
% problem.bounds.finalTime.upp = 15;

problem.bounds.initialState.low = [X0.yQ,X0.dyQ,X0.ye,X0.dye]';% yQ,dyQ,x3,ye;
problem.bounds.initialState.upp = [X0.yQ,X0.dyQ,X0.ye,X0.dye]';
problem.bounds.finalState.low   = [X1.yQ,X1.dyQ,X1.ye,X1.dye]';
problem.bounds.finalState.upp   = [X1.yQ,X1.dyQ,X1.ye,X1.dye]';

problem.bounds.state.low = [-1.5;-inf; -L*sin(deg2rad(80));-inf];
problem.bounds.state.upp = [ 3.0; 2.2;  L*sin(deg2rad(90)); inf];

problem.bounds.control.low = -maxForce;
problem.bounds.control.upp =  maxForce;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time  = [0,duration];% tg, size(tg) = [1,Ng]
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e5);

% problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = duration/0.02+1;
% problem.options.method = 'hermiteSimpson';
% problem.options.method = 'rungeKutta';
problem.options.method = 'chebyshev';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%
soln = optimTraj(problem);
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t = linspace(soln.grid.time(1), soln.grid.time(end), problem.options.trapezoid.nGrid);
z = soln.interp.state(t);
u = soln.interp.control(t);
%
Q.tic = t';
X = z';
Q.ay = u'; %column vector
Q.y = X(:,1);
Q.x = 0*Q.y;
Q.z = 0*Q.y;
Q.dy = X(:,2);

P.ye = X(:,3);
P.dye = X(:,4);
P.xe = 0*P.ye;
P.ze = sqrt(L^2-P.ye.^2);
P.x = Q.x + P.xe;
P.y = Q.y + P.ye;
P.z = Q.z - P.ze;

end