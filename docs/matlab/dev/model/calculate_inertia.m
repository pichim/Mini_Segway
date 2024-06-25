clc, clear variables
%%

% 0.97 - 2*0.095 = 0.7800

m = [0.035; ... % nucleo
    0.147; ...  % battery pack right
    0.147; ...  % battery pack right
    0.1; ...    % 
    0.1];
m = m / sum(m) * 0.78; % scale it to actual mass
sum(m)

r = [0.0, 0.0, 0.07; ...
     0.0, -0.095/2.0, 0.064; ... 
     0.0,  0.095/2.0, 0.064; ... 
     0.0, -0.05, 0.0; ...
     0.0, 0.05, 0.0];

J = zeros(3);
l_cog = zeros(1,3);
for i = 1:length(m)

    J = J + [r(i,2)^2 + r(i,3)^2, -r(i,1)*r(i,2)     , -r(i,1)*r(i,3)     ; ...
            -r(i,1)*r(i,2)      , r(i,1)^2 + r(i,3)^2, -r(i,2)*r(i,3)     ; ...
            -r(i,1)*r(i,3)      , -r(i,2)*r(i,3)     , r(i,1)^2 + r(i,2)^2];

    l_cog = l_cog + m(i) * r(i,:);

end

J
l_cog = l_cog / sum(m) * 1.2 % scale it a bit upwards

S_cog = [ 0       , -l_cog(3),  l_cog(2); ...
          l_cog(3),         0, -l_cog(1); ...
         -l_cog(2),  l_cog(1),  0       ];

J = J - sum(m) * S_cog.' * S_cog
