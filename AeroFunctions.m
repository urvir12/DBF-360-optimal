classdef AeroFunctions
    methods (Static)
        function L = lift(p, v, S, Cl)
            %Lift Force
            L = 0.5 * p * v^2 * Cl * S;
        end
        function D = drag(p, v, S, Cd)
            D = 0.5 * p * v^2 * Cd * S;
        end
    end
end