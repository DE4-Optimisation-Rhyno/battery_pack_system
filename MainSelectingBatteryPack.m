%% Overview: ReadMe

%The aim of this code is to identify the optimum battery pack from a
%an initial selection of 26. 

%The initial Battery set of 26 is in the BatterySet1112 table, it was
%imported as a .csv file that was assembled in Excel. The rows of the table
%are types of Li-ion battery cells. The columns list shows: the performance
%information (Ah, V, cost) from the dataset; equations used to
%calcluate pack performance (max Pack power (W). 

%The script runs in 6 sections; 
%The end results is prints the selected battery cell, and it's mass. 2
%battery packs (each pack of 12 cells) are required to meet the required
%motor power. The selected battery cell was LG 3.6V 3.5Ah 

%Section 1: Finding The Attainable Set of Battery Cells (given Equality constraints)
%Section 2: Highlighting Attainable Set of Battery Cells (3D Scatter Plots)
%Section 3: Selection of Battery based on minimising mass 
%Section 4: Cost as Secondary Objective Function
%Section 5: Self Adjudged Pareto Set (Mass vs Cost) - for the Attainable Set
%Section 6: Printing the Values of the Selected Battery 

%Press f5 to run the whole script.

%% Section 1: Finding The Attainable Set of Battery Cells (given Equality constraints)

load('BatterySet0812.mat');

%From the initially loaded in .csv (BattertSet1112), the key variables
%variables to analyse the dataset were extracted from the table. 

%PackAh - capacity of the pack of cells
%MaxPackPower - (Max Operating Current of cell)*Cell Voltage)*(no. cells)

PackAh = table2array(BatterySet1112(:,19)); 
PackCost = table2array(BatterySet1112(:,18));
PackMass = table2array(BatterySet1112(:,16));
MaxPackPower = table2array(BatterySet1112(:,20));


%Setting up Y-Lines to plot Equality Constraints on the scatter graph of
%batteries
xPower = [0 0.6 1.2];
yPower = [250 250 250]; 
yPackAh = [40 40 40];

%Figure 1: Max Pack Power vs Pack Mass
figure('Name', 'Figure 1: Max. Pack Power vs Pack Mass') %Naming the figure

xpos = [PackMass];
ypos = [MaxPackPower];
xlim([0 1.2]);
ylim([50 2000]);
labels = TransposedSplitString;
h = labelpoints (xpos, ypos, labels, 'N', 0.2, 1); 
hold on
sz = 25;
scatter(PackMass, MaxPackPower, sz,'filled');
hold on
plot(xPower,yPower) 
xlabel('Pack Mass (Kg)') %Labelling X-Axis
ylabel('Max Pack Power (W)') %Labelling Y-Axis
title('Figure 1: Max. Pack Power (W) vs Pack Mass (Kg)') %adds a title to the graph
hold off


%Figure 2: Pack Capactiy vs Pack Mass
figure('Name', 'Figure 2: Pack Capacity (Ah) vs Pack Mass (Kg)') %Naming the figure

xpos = [PackMass];
ypos = [PackAh];
xlim([0 1.2]);
ylim([0 60]);
labels = TransposedSplitString;
h = labelpoints (xpos, ypos, labels, 'N', 0.2, 1); 
hold on
sz = 25;
scatter(PackMass, PackAh, sz,'filled');
hold on
plot(xPower,yPackAh);
xlabel('Pack Mass (Kg)') %Labelling X-Axis
ylabel('Pack Energy Capacity (Ah)') %Labelling Y-Axis
title('Figure 2: Pack Capacity (Ah) vs Pack Mass (Kg)') %adds a title to the graph
hold off





%% Section 2: Highlighting Attainable Set of Battery Cells (3D Scatter Plots)

%The attainable set of battery packs satisfy both equality constraints of
%Maximum Pack Power and Pack capacity.
%The AttainableTable contains all the packs that satisfy both constraints.

BattAttainableAh = find(abs(PackAh)>40);
x(BattAttainableAh) = [];

BattAttainablePower = find(abs(MaxPackPower)>250);
x(BattAttainablePower) = [];


[val]=intersect(BattAttainableAh,BattAttainablePower);

AttainableTable = BatterySet1112(val',:);

%Extracting the relevant information from the Attainable Table

AttainablePackAh = table2array(AttainableTable(:,19));
AttainablePackCost = table2array(AttainableTable(:,18));
AttainablePackMass = table2array(AttainableTable(:,16));
AttainableMaxPackPower = table2array(AttainableTable(:,20));
AttainableRunTime250W = table2array(AttainableTable(:,23));

%Run time of the pack when 250W are being drawn from the pack.

AttainablePackCost = table2array(AttainableTable(:,18));

%Extracting Name data, so they can label points in the 3D scatter plot

AttainableBatteryNames = table2array(AttainableTable(:,1));
n2 = AttainableBatteryNames;
allOneString2 = sprintf('%s,' , n2);
allOneString2 = allOneString2(1:end-2); %strip final comma
splitString2 = split(allOneString2,',');
TransposedSplitString2 = splitString2';

%Plotting 3D scatter plots to represent the attainable set. The objective
%function (to minimise mass) is on the z-axis in both cases.

figure('Name', 'Figure 3: Pack Mass (Kg) vs Max Pack Power (W) vs Pack Capacity (Ah)') %Naming the figure
xpos = [AttainableMaxPackPower];
ypos = [AttainablePackAh];
zpos = [AttainablePackMass];
xlim([200 2000]);
ylim([35 55]);
zlim([0.5 1.2]);
labels2 = TransposedSplitString2;
%h2 = labelpoints (xpos, ypos, zpos, labels2, 'N', 0.2, 1);
S = 25;
scatter3(AttainableMaxPackPower, AttainablePackAh, AttainablePackMass, S,'filled')
xlabel('Max Pack Power (W)') %Labelling X-Axis
ylabel('Pack Capacity (Ah)') %Labelling Y-Axis
zlabel('Pack Mass (Kg)') %Labelling Y-Axis
text(AttainableMaxPackPower, AttainablePackAh, AttainablePackMass, TransposedSplitString2);
title('Figure 3: Pack Mass (Kg) vs Max Pack Power (W) vs Pack Capacity (Ah)') %adds a title to the graph

figure('Name', 'Figure 4: Pack Mass (Kg) vs Max Pack Power (W) vs Pack Run Time @ 250W (Hours)') %Naming the figure
xpos = [AttainableMaxPackPower];
ypos = [AttainableRunTime250W];
zpos = [AttainablePackMass];
xlim([200 2000]);
ylim([0.5 0.8]);
zlim([0.5 1.2]);
labels2 = TransposedSplitString2;
%h2 = labelpoints (xpos, ypos, zpos, labels2, 'N', 0.2, 1);
S = 25;
scatter3(AttainableMaxPackPower, AttainableRunTime250W, AttainablePackMass, S,'filled')
xlabel('Max Pack Power (W)') %Labelling X-Axis
ylabel('Pack Run Time @ 250W (Hours)') %Labelling Y-Axis
zlabel('Pack Mass (Kg)') %Labelling Y-Axis
text(AttainableMaxPackPower, AttainableRunTime250W, AttainablePackMass, TransposedSplitString2);
title('Figure 4: Pack Mass (Kg) vs Max Pack Power (W) vs Pack Run Time @ 250W (Hours)') %adds a title to the graph

%% Section 3: Selection of Battery based on minimising mass 

%First Objective function: to minimise mass with respect to equality
%constraints. The lightest battery from this set best satisfied the first objective
%function - this was LG 3.6V 3.5Ah . How it compared on a multi-objective basis is compared in
%section 5

%Figure X: Cost of Cells in the attainable Dataset
figure('Name', 'Figure 5: Pack Mass (Kg) of Attainable cells') %Naming the figure
%adds a title to the graph
XBattNames = 1:1:7;
YMass = AttainablePackMass';

hStem = stem(XBattNames,YMass);

%// Create labels.
Labels2 = {"3.6V 3000mAh FT";"3.6V 3.5Ah FT";"3.6V 3.4Ah CT";"3.7V 4200mAh FT";"40T 3.6V 4000mAh FT";"Li-Mn 3.7V 3500mAh FT";"Li-Mn HD 3.7V 4200mAh FT"}

%// Get position of each stem 'bar'. Sorry I don't know how to name them.
X_data2 = get(hStem, 'XData');
Y_data2 = get(hStem, 'YData');

%// Assign labels.
for labelID = 1 : numel(X_data2)
   text(X_data2(labelID), Y_data2(labelID) + 0.1, Labels2{labelID}, 'HorizontalAlignment', 'left','rotation',90);
end
 
xlabel('Battery Names') %Labelling X-Axis
ylabel('Battery Pack Mass (Kg)') %Labelling Y-Axis
xlim([0 8]);
ylim([0.5 2]);
title('Figure 5: Pack Mass (Kg) of Attainable cells')

%% Section 4: Cost as Secondary Objective Function

%Second Objective function: to minimise cost. The cheapest battery pack satisfied the second objective
%function. How it compared on a multi-objective basis is compared in
%section 5.


%Figure X: Cost of Cells in the attainable Dataset
figure('Name', 'Figure 6: Pack Cost (GBP) of Attainable cells') %Naming the figure
 %adds a title to the graph
XBattNames = 1:1:7;
YCost = AttainablePackCost';

hStem = stem(XBattNames,YCost);

%// Create labels.
Labels = {"3.6V 3000mAh FT";"3.6V 3.5Ah FT";"3.6V 3.4Ah CT";"3.7V 4200mAh FT";"40T 3.6V 4000mAh FT";"Li-Mn 3.7V 3500mAh FT";"Li-Mn HD 3.7V 4200mAh FT"};

%// Get position of each stem 'bar'. Sorry I don't know how to name them.
X_data = get(hStem, 'XData');
Y_data = get(hStem, 'YData');

%// Assign labels.
for labelID = 1 : numel(X_data)
   text(X_data(labelID), Y_data(labelID) + 3, Labels{labelID}, 'HorizontalAlignment', 'left','rotation',90);
end
 
xlabel('Battery Names') %Labelling X-Axis
ylabel('Battery Pack Cost (GBP)') %Labelling Y-Axis
xlim([0 8]);
ylim([130 300]);
title('Figure 6: Pack Cost (GBP) of Attainable cells')

%% Section 5: Self Adjudged Pareto Set (Mass vs Cost) - for the Attainable Set

%A multi-objective function that compared mass against cost, the optimum
%point is in the bottom left position of the graph. This seleced battery
%was LG 3.6V 3.5Ah - this battery was taken forward to the system-wide
%optimisation

%Figure 1: Max Pack Power vs Pack Mass
figure('Name', 'Figure 7: Attainable Set Mass (Kg) vs Attainable Set Cost (GBP)') %Naming the figure

xpos = [AttainablePackCost];
ypos = [AttainablePackMass];
xlim([130 260]);
ylim([0.5 1.5]);
labels = TransposedSplitString2;
h = labelpoints (xpos, ypos, labels, 'N', 0.2, 1); 
hold on
sz = 25;
scatter(AttainablePackCost, AttainablePackMass, sz,'filled');

xlabel('Pack Cost (GBP)') %Labelling X-Axis
ylabel('Pack Mass (Kg)') %Labelling Y-Axis
title('Figure 7: Attainable Set Mass (Kg) vs Attainable Set Cost (GBP)') %adds a title to the graph
hold off

%% Section 6: Printing the Values of the Selected Battery 

%Run this section to print the selected Battery type

SelectedBatteryPack = AttainableTable(2,:);
SelectedBatteryMass = AttainableTable(2,16);

%The Mass of the Optimal Battery Pack

OptimalMass = table2array(SelectedBatteryMass)



%% Further Incomplete Optimisation: Residuals 







