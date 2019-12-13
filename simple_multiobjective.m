 
function y = simple_multiobjective(x)
p = [-2.73768873269127,2.12332858117414,-0.105693287574911];
q = [2.62161727407408,0.205568391612961,0.00569023401222044];

   y(1) = p(1)*(xNormMass.^2) + p(2)*(xNormMass) + p(3);
   y(2) = q(1)*(xNormMass.^2) + q(2)*(xNormMass) + q(3);
   end