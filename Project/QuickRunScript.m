%script to build field/combine environment and control combine through it
%
%
Environment = CombineRL;
Environment = Environment.CreateField(40,200,10);
Environment = Environment.CreateCombine(16,300,30);
Environment = Environment.DefineHarvestPath;
[StateVector,Reward,Diagnostics,RewardMean] = Environment.OperateCombine;
%
%