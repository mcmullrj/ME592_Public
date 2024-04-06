%script to build field/combine environment and control agent
%
%
ControlSettings = [20,0,0,0.500000000000000,50,300]; %time step for control update, proportional gain, integral gain, initial battery SOC, initial motor power, initial engine power
%
%
Environment = CombineRL;
Environment = Environment.CreateField(40,200,10,0.1);
Environment = Environment.CreateCombine(16,300,30);
Environment = Environment.DefineHarvestPath;
Environment = Environment.DefineControl(0.2,5,ControlSettings,1);
%
%
clearvars -except Environment