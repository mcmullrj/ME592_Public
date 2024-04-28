function [InitialObservation,LoggedSignals] = InitializeCombineEnvironment()
%
%
%
%
InitialObservation = [10,0.95,0.5];
LoggedSignals.Diagnostics = [0,0,0,0]; %engine power, motor power, norm crop rate, fuel rate
LoggedSignals.StartTimeStep = [0.5,1,0]; %battery SOC, field position index, cumulative harvest time]
%
%
%
%
end