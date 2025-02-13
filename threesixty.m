clear;
clc;
close all;
import AeroFunctions.*;
main()

function [] = main()
    %constants
    g = 9.81;
    p = 1.225 % air density
    b = 1.524 % wingspan
    Cd0 = 0.04; %drag coefficient
    Cl_max = 1.2 % max lift coefficient
    T_max = 70 %max thrust
    W_dry = 28; %aircraft mass
    vi = 15  %initial velocity
    
    % aerofunctions
    aerofuns = AeroFunctions;

    %Optimization
    [sol, score, exitflag] = runopt(W_dry, 15)
    
    %Display
    display(sol)
    fprintf('\n\n')

    %Dv - change in velocity
    dVar = linspace(0.5, 1.5, 99);
    dScorev = objectivefun(W_dry, sol.velocity360 .* dVar) ./ score;
    plot(dVar, dScorev, 'DisplayName', 'Velocity')
    %hold on sets the axes hold state to on, which retains plots in the current axes so that new plots 
    % added to the axes do not delete existing plots. 
    hold on 


    %plot format
    xlabel('% Change in Variable')
    ylabel(' Change in Score')
    title('Sensitivity Analysis')
end

function [sol, score, exitflag] = runopt(W0, v0)
    g = 9.81;
    p = 1.225 % air density
    b = 0.5243  % wingspan
    Cd0 = 0.04; %drag coefficient
    Cl_max = 1.2 % max lift coefficient
    T_max = 70 %max thrust
    W_dry = 28; %aircraft mass

    %Optimization variables
    velocity360 = optimvar("velocity360", "LowerBound", 15, "UpperBound", 50.2298)
   
   
    s = optimvar("s", "LowerBound", 0, "UpperBound", 20)
    acc1 = optimvar("acc1", "LowerBound", -20, "UpperBound", 20)
    acc2 = optimvar("acc2", "LowerBound", -20, "UpperBound", 20)
    radius = 5.05
    %initial starting point
    initialPoint.velocity360 = 30;
    initialPoint.s = 5;
    initialPoint.acc1 = 3;
    initialPoint.acc2 = 3;

    %optimization problem
    problem = optimproblem("ObjectiveSense", "Minimize")

    %objective to minimize time to complete the loop
    %fcn2optimexprs convert the function to an optimization expression
    problem.Objective = fcn2optimexpr(@objectivefinal, velocity360, radius, s, acc1, acc2);
    
    %Constraints
    aerofuns = AeroFunctions;
    problem.Constraints.constraint1 = aerofuns.lift(p, velocity360, b, Cl_max) >= W0 %sufficient lift 
    % (greater than weight of the aircraft)
    problem.Constraints.contraint2 = aerofuns.drag(p, velocity360, b, Cd0) <= T_max %drag has to be less than thrust
    problem.Constraints.constraint3 = velocity360^2 / radius <= 7 * g % Max 7g load factor

    % Solve
     opts = optimoptions('fmincon', ...
    'Display', 'iter-detailed', ...
    'ConstraintTolerance', 1e-6, ...
    'StepTolerance', 1e-6, ...
    'OptimalityTolerance', 1e-6, ...
    'Algorithm', 'interior-point', ...
    'EnableFeasibilityMode', true);
    [sol, score, exitflag] = solve(problem, initialPoint, 'Options', opts);
end

function display(sol)
    final_time = objectivefinal(sol.velocity360, 5.05, sol.s, sol.acc1, sol.acc2);
    disp(['Optimal Velocity    : ', num2str(sol.velocity360), ' m/s'])
    disp(['Radius    : ', num2str(5.05), ' m'])
    disp(['Time to Complete 360: ', num2str(final_time), 'sec'])
    disp(['Optimal Distance Before 360:  ', num2str(sol.s), ' m'])
    disp(['Optimal Distance After 360:  ', num2str(1000 - sol.s), ' m'])
    disp(['Optimal Acceleration Before 360:  ', num2str(sol.acc1), ' m/s^2'])
    disp(['Optimal Acceleration After 360:  ', num2str(sol.acc2), ' m/s^2'])
end

function time360 = objectivefun(velocity, radius)
    time360 = (2 * pi * radius) ./ velocity;
  
end

function finaltime = objectivefinal(velocity360, radius, s, a1, a2)
    
    time1 = (30 - velocity360) ./ a1
    time2 = ((velocity360 ./ -1) + sqrt((velocity360^2) + 2*a2* (1000-s))) ./ a2
    time360 = objectivefun(velocity360, radius);
    finaltime = time1 + time360 + time2 - 0.1 * a2;
end

