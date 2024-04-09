%script to build field/combine environment and control agent
%
%
%
%
%initialize
FieldModel = CombineRL;
%define field
Acres = 40; %acre
MeanYield = 200; %bu/acre
StDevYield = 10; %bu/acre
FieldConsistency = 0.1; %account for elevation and moisture changes - try 0.05 to 1.0?
FieldModel = FieldModel.CreateField(Acres,MeanYield,StDevYield,FieldConsistency);
%define combine
RowsPerPass = 16;
EnginePower = 300; %kW
BatteryCapacity = 30; %kWh
FieldModel = FieldModel.CreateCombine(RowsPerPass,EnginePower,BatteryCapacity);
%build harvest path through field
FieldModel = FieldModel.DefineHarvestPath;
%define control
GrainValue = 0.2; % ?$?, not really
FuelValue = 5; % ?$?
ControlSettings = [20,0.5,0,0.500000000000000,50,300]; %time step for control update, proportional gain, integral gain, initial battery SOC, initial motor power, initial engine power
StartingIndex = 1; %grid in field path to start harvest
FieldModel = FieldModel.DefineControl(GrainValue,FuelValue,ControlSettings,StartingIndex);
%solve with baseline (classical) controls
FieldModel = FieldModel.OperateCombine;
%plot simulation results
FieldModel.PlotSimulationResults
%write desired results to csv's

%
%
clearvars -except FieldModel