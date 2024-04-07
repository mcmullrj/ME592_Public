%script to build field/combine environment and control agent
%
%
%
%
%initialize
Environment = CombineRL;
%define field
Acres = 40; %acre
MeanYield = 200; %bu/acre
StDevYield = 10; %bu/acre
FieldConsistency = 0.1; %account for elevation and moisture changes - try 0.05 to 1.0?
Environment = Environment.CreateField(Acres,MeanYield,StDevYield,FieldConsistency);
%define combine
RowsPerPass = 16;
EnginePower = 300; %kW
BatteryCapacity = 30; %kWh
Environment = Environment.CreateCombine(RowsPerPass,EnginePower,BatteryCapacity);
%build harvest path through field
Environment = Environment.DefineHarvestPath;
%define control
GrainValue = 0.2; % ?$?, not really
FuelValue = 5; % ?$?
ControlSettings = [20,0,0,0.500000000000000,50,300]; %time step for control update, proportional gain, integral gain, initial battery SOC, initial motor power, initial engine power
StartingIndex = 1; %grid in field path to start harvest
Environment = Environment.DefineControl(GrainValue,FuelValue,ControlSettings,StartingIndex);
%solve with baseline (classical) controls
[StateVectorFinal,RewardFinal,DiagnosticsFinal,RewardMean] = Environment.OperateCombine;
%write desired results to csv's

%
%
clearvars -except Environment StateVectorFinal RewardFinal DiagnosticsFinal RewardMean