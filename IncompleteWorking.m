%% Further Incomplete Optimisation: Normalising Data Ahead of Finding Multiobjective Pareto Set 
load('BatterySet0812.mat');

%% Incomplete Section 1: Normalising the Data - ahead of comparing two variables against the objective function.

%Datasets normalised so that variables can be better compared 

BatterySet1112Cut = BatterySet1112(:,3:23);
BatterySet1112CutArray = table2array(BatterySet1112Cut);

NormArray = normc(BatterySet1112CutArray);

NormPackCost = (NormArray(:,16));
NormPackMass = (NormArray(:,14));
NormMaxPackPower = (NormArray(:,18));
NormRunTime250W = (NormArray(:,21));
NormPackAh = (NormArray(:,17));


%Using Polyfit to test quality of polynomial curve fitting of normalised
%data
figure('Name', 'Figure 1: Normalised: Pack Capacity(Ah) vs Pack Mass(Kg) with Polyfit to analyse fit') %Naming the figure
p = polyfit(NormPackMass, NormPackAh,1);
f = polyval(p,NormPackMass); 
plot(NormPackMass, NormPackAh,'o',NormPackMass,f,'-');
legend('data','linear fit');
hold on
%plot(xPower,yPackAh);
xlabel('Normalised Pack Mass (Kg)'); %Labelling X-Axis
ylabel('Normalised Pack Energy Capacity (Ah)'); %Labelling Y-Axis
title('Figure 1: Normalised: Pack Capacity(Ah) vs Pack Mass(Kg) with Polyfit to analyse fit'); %adds a title to the graph
hold off


figure('Name', 'Figure 2: Linear Fit of Data with 95% Prediction Interval') %Naming the figure
[p,S] = polyfit(NormPackMass, NormPackAh,1)
[y_fit,delta] = polyval(p,NormPackMass,S);

plot(NormPackMass, NormPackAh,'bo')
hold on
plot(NormPackMass,y_fit,'r-')
plot(NormPackMass,y_fit+2*delta,'m--',NormPackMass,y_fit-2*delta,'m--')
title('Figure 2: Linear Fit of Data with 95% Prediction Interval')
legend('Data','Linear Fit','95% Prediction Interval')
hold off


%% Normalised Scatter Plot of Pack Mass vs 2 variables (PackAh, and MaxPackPower)

% Using the basic fitting tool from the figure - the best fit equations
% were calculated

%Figure 2: Pack Capactiy vs Pack Mass
figure('Name', 'Figure 3: Normalised Pack Capacity (Ah) vs Pack Mass (Kg)') %Naming the figure

xpos = [NormPackMass];
ypos = [NormPackAh];
xlim([0 0.5]);
ylim([0 0.5]);
labels = TransposedSplitString;
h = labelpoints (xpos, ypos, labels, 'N', 0.2, 1); 
hold on
sz = 25;

scatter(NormPackMass, NormPackAh, sz,'filled');
xlabel('Normalised Pack Mass (Kg)') %Labelling X-Axis
ylabel('Normalised Pack Energy Capacity (Ah)') %Labelling Y-Axis
title('Figure 3: Normalised Pack Capacity (Ah) vs Pack Mass (Kg)') %adds a title to the graph
hold off

%Figure 2: Pack Max Power vs Pack Mass
figure('Name', 'Figure 4: Normalised Pack Max. Power (W) vs Pack Mass (Kg)') %Naming the figure

xpos = [NormPackMass];
ypos = [NormMaxPackPower];
xlim([0 0.5]);
ylim([0 0.5]);
labels = TransposedSplitString;
h = labelpoints (xpos, ypos, labels, 'N', 0.2, 1); 
hold on

sz = 25;

scatter(NormPackMass, NormMaxPackPower, sz,'filled');
xlabel('Normalised Pack Mass (Kg)') %Labelling X-Axis
ylabel('Normalised Pack Max. Power (W)') %Labelling Y-Axis
title('Figure 4: Normalised Pack Max. Power (W) vs Pack Mass (Kg)') %adds a title to the graph
hold off

%% Taking the Co-efficents on the line plots from Figures 3 & 4; and then plotting the lines against each other. 
%Intersection of the two curves could be a potential pareto set / the
%optimum points lie with in them

%Plot Coefficients
%Plot NormPackAh vs NormPackMass

xNormMass = linspace(0,1);  %adjust as needed
p = [-2.73768873269127,2.12332858117414,-0.105693287574911];
NormPackAhVsNormMassCurve = p(1)*(xNormMass.^2) + p(2)*(xNormMass) + p(3);

%NEEED THESEEE 08:33
%Plot PackAh vs Mass curve
q = [2.62161727407408,0.205568391612961,0.00569023401222044];
NormMaxPowervsNormMassCurve = q(1)*(xNormMass.^2) + q(2)*(xNormMass) + q(3);


figure('Name', 'Figure 5: Comparing Normalised Variables against Normalised Mass') %Naming the figure
title('Figure x: Comparing Normalised Variables against Normalised Mass') %adds a title to the graph
plot(xNormMass,NormMaxPowervsNormMassCurve);
hold on 
plot(xNormMass,NormPackAhVsNormMassCurve);

xlabel('Normalised Pack Mass (Kg)') %Labelling X-Axis
ylabel('Normalised Pack Max. Power (W) & Pack Power (Ah)') %Labelling Y-Axis
title('Figure 5: [Normalised] Pack Max. Power (W) & Pack Power (Ah) vs Pack Mass (Kg)') %adds a title to the graph

%y(1) = NormPackAhVsNormMassCurve;
%y(2) = NormMaxPowervsNormMassCurve;

FitnessFunction = @simple_multiobjective;
numberOfVariables = 1;
[x,fval] = gamultiobj(FitnessFunction,numberOfVariables);
