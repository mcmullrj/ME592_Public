function [StateVector,Reward,IsDone,LoggedSignals] = ControlCombineEnvironment(Actions,LoggedSignals)
%
%
%
%
%% define invariants
global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
    FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency...
    GrainPrice FuelPrice MaxHarvestDuration
%
%
%
%% inputs
PowerEngineRequest = Actions(1)*EnginePowerRef;
PowerMotorRequest = Actions(2)*BatteryMaxDischargeRate;
CombineSettingSetpoint = Actions(3);
BatterySOCStartTimeStep = LoggedSignals.StartTimeStep(1);
FieldIndexStartTimeStep = LoggedSignals.StartTimeStep(2);
TimeCumulativeField = LoggedSignals.StartTimeStep(3);
%limit inputs
if PowerEngineRequest < 0
    PowerEngineRequest = 0;
elseif PowerEngineRequest > EnginePowerRef
    PowerEngineRequest = EnginePowerRef;
end
if PowerMotorRequest > BatteryMaxDischargeRate*MotorEfficiency
    PowerMotorRequest = BatteryMaxDischargeRate*MotorEfficiency;
elseif PowerMotorRequest < -BatteryMaxChargeRate
    PowerMotorRequest = -BatteryMaxChargeRate;
end
if CombineSettingSetpoint < 0.1
    CombineSettingSetpoint = 0.1;
elseif CombineSettingSetpoint > 1.7
    CombineSettingSetpoint = 1.7;
end
if BatterySOCStartTimeStep <= 0.1
    if PowerMotorRequest > 0
        PowerMotorRequest = 0;
    end
elseif BatterySOCStartTimeStep >= 0.9
    if PowerMotorRequest < 0
        PowerMotorRequest = 0;
    end
end
%
%
%% adjust power inputs based on system state
BatteryEnergy = BatterySOCStartTimeStep.*BatteryCapacity; %kWh
PowerMotor = PowerMotorRequest; %kW
PowerEngine = PowerEngineRequest; %kW
%normalized power
PowerNorm = (PowerEngine+PowerMotor)/TotalPowerRef;
%overspecified power: meet requested engine power, reduce motor power to
%get total = max
if PowerNorm > 1
    PowerNorm = 1;
    PowerMotor = PowerNorm*TotalPowerRef-PowerEngine;
end
%
%
%% constants
MaxPowerNormForPropulsion = 0.25;
%
%
%% determine harvest rate for each grid
FieldMapGridDistance = FieldPath(2,1)-FieldPath(1,1); %m/grid
GridsPerTimeStepInterp = (SpeedCombineInterp./FieldMapGridDistance).*DurationTimeStep;
FieldIndexEndTimeStepInterp = FieldIndexStartTimeStep+round(GridsPerTimeStepInterp(end));
TimePerGrid = FieldMapGridDistance./SpeedCombineInterp; %sec
TimePerGrid(1) = 2*TimePerGrid(2);
%crop rate for each grid for each combine speed guess
CropRateNormInterp = (FieldPath(FieldIndexStartTimeStep:FieldIndexEndTimeStepInterp,2)*(1./TimePerGrid)).*(3600./FlowCropRef);
PowerNormGrainHarvestInterp = interp2(CombineSettingNorm,FlowCropNorm,PowerGrainProcessNorm,CombineSettingSetpoint.*ones(size(CropRateNormInterp)),CropRateNormInterp);
%battery energy change per grid
dBatteryEnergy = -PowerMotor.*(TimePerGrid./3600); %kWh
%solve for speed from power
PowerNormForPropulsion = PowerNorm-PowerNormGrainHarvestInterp;
SpeedFromPower = (PowerNormForPropulsion./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
%calculate error, pick speed for min error
Error2SpeedCombine = (ones(length(SpeedFromPower(:,1)),1)*SpeedCombineInterp-SpeedFromPower).^2; %squared error
%find min error for each row in potential solution space
SpeedCombineGrid = zeros(size(SpeedFromPower(:,1)));
CropRateNormGrid = zeros(size(SpeedFromPower(:,1)));
PowerEngineGrid = ones(size(SpeedFromPower(:,1))).*PowerEngine; %kW
PowerMotorGrid = ones(size(SpeedFromPower(:,1))).*PowerMotor; %kW
BatteryEnergyGrid = ones(size(SpeedFromPower(:,1))).*BatteryEnergy; %kWh
BatterySOCGrid = zeros(size(SpeedFromPower(:,1)));
TimeGrid = zeros(size(SpeedFromPower(:,1)));
for k1 = 1:length(Error2SpeedCombine(:,1))
    MinError2SpeedCombine = min(Error2SpeedCombine(k1,:));
    for k = 1:length(Error2SpeedCombine(k1,:))
        if Error2SpeedCombine(k1,k) == MinError2SpeedCombine
            IndexSpeedSolution = k;
        end
    end
    %check if motor power needs to change
    if k1 > 1
        BatteryEnergyGrid(k1) = BatteryEnergyGrid(k1-1)+dBatteryEnergy(IndexSpeedSolution);
    end
    BatterySOCGrid(k1) = BatteryEnergyGrid(k1)/BatteryCapacity;
    %update to correct error
    if BatterySOCGrid(k1) > 0.9
        BatterySOCGrid(k1) = 0.9;
        PowerMotor = 0; %start discharging
        dBatteryEnergy = -PowerMotor.*(TimePerGrid./3600);
        PowerNorm = (PowerEngine+PowerMotor)/TotalPowerRef;
        if PowerNorm > 1
            PowerNorm = 1;
            PowerMotor = PowerNorm*TotalPowerRef-PowerEngine;
        end
        PowerNormForPropulsion(k1:end,:) = PowerNorm-PowerNormGrainHarvestInterp(k1:end,:);
        SpeedFromPower(k1:end,:) = (PowerNormForPropulsion(k1:end,:)./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
        Error2SpeedCombine(k1:end,:) = (ones(length(SpeedFromPower(k1:end,1)),1)*SpeedCombineInterp-SpeedFromPower(k1:end,:)).^2; %squared error
        PowerMotorGrid(k1) = PowerMotor;
    elseif BatterySOCGrid(k1) < 0.1
        BatterySOCGrid(k1) = 0.1;
        PowerMotor = 0; %start charging
        dBatteryEnergy = -PowerMotor.*(TimePerGrid./3600);
        PowerNorm = (PowerEngine+PowerMotor)/TotalPowerRef;
        if PowerNorm > 1
            PowerNorm = 1;
            PowerMotor = PowerNorm*TotalPowerRef-PowerEngine;
        end
        PowerNormForPropulsion(k1:end,:) = PowerNorm-PowerNormGrainHarvestInterp(k1:end,:);
        SpeedFromPower(k1:end,:) = (PowerNormForPropulsion(k1:end,:)./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
        Error2SpeedCombine(k1:end,:) = (ones(length(SpeedFromPower(k1:end,1)),1)*SpeedCombineInterp-SpeedFromPower(k1:end,:)).^2; %squared error
        PowerMotorGrid(k1) = PowerMotor;
    end
    BatteryEnergyGrid(k1) = BatterySOCGrid(k1).*BatteryCapacity;
    %
    %original code
%     if BatterySOCGrid(k1) < 0.1 && PowerMotor > 0
%         %stop discharging battery and recalculate speed error
%         PowerMotor = 0;
%         PowerNorm = PowerEngine/TotalPowerRef;
%         PowerNormForPropulsion(k1:end,:) = PowerNorm-PowerNormGrainHarvestInterp(k1:end,:);
%         SpeedFromPower(k1:end,:) = (PowerNormForPropulsion(k1:end,:)./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
%         Error2SpeedCombine(k1:end,:) = (ones(length(SpeedFromPower(k1:end,1)),1)*SpeedCombineInterp-SpeedFromPower(k1:end,:)).^2; %squared error
%     elseif BatterySOCGrid(k1) > 0.9 && PowerMotor < 0
%         %stop charging battery and recalculate speed error
%         PowerMotor = 0;
%         PowerNorm = PowerEngine/TotalPowerRef;
%         PowerNormForPropulsion(k1:end,:) = PowerNorm-PowerNormGrainHarvestInterp(k1:end,:);
%         SpeedFromPower(k1:end,:) = (PowerNormForPropulsion(k1:end,:)./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
%         Error2SpeedCombine(k1:end,:) = (ones(length(SpeedFromPower(k1:end,1)),1)*SpeedCombineInterp-SpeedFromPower(k1:end,:)).^2; %squared error        
%     end
    %
    %check if combine is at its governed speed limit
    if IndexSpeedSolution == length(Error2SpeedCombine(k1,:))
        %combine is at its governed speed limit
        PowerNormGrid = MaxPowerNormForPropulsion+PowerNormGrainHarvestInterp(k1,end);
        PowerEngineGrid(k1) = PowerNormGrid*TotalPowerRef-PowerMotorGrid(k1);
    end
    %select answers for crop volume and combine speed
    TimeGrid(k1) = TimePerGrid(IndexSpeedSolution);
    SpeedCombineGrid(k1) = SpeedCombineInterp(IndexSpeedSolution)/1000*3600; %km/hr
    CropRateNormGrid(k1) = CropRateNormInterp(k1,IndexSpeedSolution);
    %PowerMotorGrid(k1) = PowerMotor;
end
%
%
%% find grids traveled during time step
TimeCumulative = 0;
Grid = 1;
while TimeCumulative < DurationTimeStep
%     TimeGrid = FieldMapGridDistance/(SpeedCombineGrid(Grid)*1000/3600); %sec
    TimeCumulative = TimeCumulative+TimeGrid(Grid);
    Grid = Grid+1;
end
if Grid > length(Error2SpeedCombine(:,1))
    Grid = length(Error2SpeedCombine(:,1));
end
FieldIndexEndTimeStep = FieldIndexStartTimeStep+Grid-1;
SpeedCombineGrid = SpeedCombineGrid(1:Grid);
CropRateNormGrid = CropRateNormGrid(1:Grid);
TimeGrid = TimeGrid(1:Grid);
TimeCumulative = sum(TimeGrid);
PowerEngineGrid = PowerEngineGrid(1:Grid);
PowerMotorGrid = PowerMotorGrid(1:Grid);
%
%
%% calculate harvest efficiencies
EfficiencyGrainHarvestGrid = interp2(CombineSettingNorm,FlowCropNorm,GrainEfficiency,CombineSettingSetpoint.*ones(size(CropRateNormGrid)),CropRateNormGrid);
GrainPerGrid = FieldPath(FieldIndexStartTimeStep:FieldIndexEndTimeStep,3).*EfficiencyGrainHarvestGrid;
% GrainHarvestRate = sum(GrainPerGrid)/(TimeCumulative/3600);
SpecificFuelConsumptionGrid = (interp1(NormFuelEfficiencyVsPower(:,1),NormFuelEfficiencyVsPower(:,2),PowerEngineGrid/EnginePowerRef,'linear','extrap'))*FuelEfficiencyRef; %g/kW/hr
FuelRateGrid = SpecificFuelConsumptionGrid.*PowerEngineGrid/(0.82*3.785*1000); %gal/hr
FuelConsumptionGrid = FuelRateGrid.*(TimeGrid./3600); %gal
FuelRate = sum(FuelConsumptionGrid)/(TimeCumulative/3600);
% GrainHarvestValue = GrainHarvestRate*GrainPrice; %$/hr
% FuelCost = FuelRate*FuelPrice; %$/hr
GrainHarvested = sum(GrainPerGrid); %bu in time step
FuelConsumed = sum(FuelConsumptionGrid); %gal in time step
GrainHarvestValue = GrainHarvested*GrainPrice; %$
if TimeCumulativeField > MaxHarvestDuration
    GrainHarvestValue = GrainHarvested*(GrainPrice*0.75); %$
end
FuelCost = FuelConsumed*FuelPrice; %$
%
%
%% mean values for time step
SpeedCombine = sum(SpeedCombineGrid.*(TimeGrid./TimeCumulative));
EfficiencyGrainHarvest = sum(EfficiencyGrainHarvestGrid.*(TimeGrid./TimeCumulative));
CropRateNorm = sum(CropRateNormGrid.*(TimeGrid./TimeCumulative));
PowerEngineMean = sum(PowerEngineGrid.*(TimeGrid./TimeCumulative));
PowerMotorMean = sum(PowerMotorGrid.*(TimeGrid./TimeCumulative));
BatterySOC = BatterySOCGrid(end);
TimeCumulativeField = TimeCumulativeField+TimeCumulative;
%
%
%% outputs
StateVector = [SpeedCombine/10,EfficiencyGrainHarvest,BatterySOC];
Reward = GrainHarvestValue-FuelCost;
LoggedSignals.Diagnostics = [PowerEngineMean,PowerMotorMean,CropRateNorm,FuelRate];
FieldIndexStartNextTimeStep = FieldIndexEndTimeStep+1;
LoggedSignals.StartTimeStep = [BatterySOC,FieldIndexStartNextTimeStep,TimeCumulativeField];
if (length(FieldPath(:,1))-max(GridsPerTimeStepInterp)) < FieldIndexStartNextTimeStep %not enough field left to evaluate next time step
    IsDone = 1;
else
    IsDone = 0;
end
%
%
%
%
end