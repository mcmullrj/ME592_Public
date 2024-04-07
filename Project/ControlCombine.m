function [StateVector,Reward,Diagnostics,FieldIndexEndTimeStep] = ControlCombine(FieldIndexStartTimeStep,BatterySOCStartTimeStep,PowerEngineRequest,PowerMotorRequest,CombineSettingSetpoint)
%
%
%
%
%% define invariants
global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
    FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency GrainPrice FuelPrice
%
%
%% adjust power inputs based on system state
BatteryEnergy = BatterySOCStartTimeStep.*BatteryCapacity; %kWh
PowerMotor = PowerMotorRequest; %kW
PowerEngine = PowerEngineRequest; %kW
%physical limits for power requests
if PowerMotor > BatteryMaxDischargeRate*MotorEfficiency
    PowerMotor = BatteryMaxDischargeRate*MotorEfficiency;
elseif PowerMotor < -BatteryMaxChargeRate
    PowerMotor = -BatteryMaxChargeRate;
end
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
%solve for speed from power
BatterySOCInterp = ones(size(PowerNormGrainHarvestInterp)).*BatterySOCStartTimeStep;
BatteryEnergy = ones(size(SpeedCombineInterp)).*BatteryEnergy; %create row vector to start tracking battery charge
PowerNormForPropulsion = PowerNorm-PowerNormGrainHarvestInterp;
PowerEngineInterp = ones(size(PowerNormGrainHarvestInterp)).*PowerEngine;
PowerMotorInterp = ones(size(PowerNormGrainHarvestInterp)).*PowerMotor;
PowerEngine = PowerEngineInterp(1,:);
PowerMotor = PowerMotorInterp(1,:);
for k1 = 1:length(PowerNormForPropulsion(:,1))
    %update norm power if battery depleted (discharging) or fully charged
    %(charging)
    BatteryEnergy = BatteryEnergy-PowerMotor.*(TimePerGrid./3600); %kWh
    BatterySOCInterp(k1,:) = BatteryEnergy./BatteryCapacity;
    for k2 = 1:length(PowerNormForPropulsion(1,:))
        if BatterySOCInterp(k1,k2) < 0.1 && PowerMotor(k2) >= 0 %discharging
            PowerMotor(k2) = 0;
            PowerMotorInterp(k1,k2) = 0;
            PowerNormForPropulsion(k1,k2) = PowerEngine(k2)/TotalPowerRef-PowerNormGrainHarvestInterp(k1,k2);
        elseif BatterySOCInterp(k1,k2) > 0.9 && PowerMotor(k2) <= 0 %charging
            PowerMotor(k2) = 0;
            PowerEngine(k2) = PowerEngine(k2)-PowerMotor(k2); %all engine power goes to combine
            PowerMotorInterp(k1,k2) = 0;
            PowerEngineInterp(k1,k2) = PowerEngine(k2);
            PowerNormForPropulsion(k1,k2) = PowerEngine(k2)/TotalPowerRef-PowerNormGrainHarvestInterp(k1,k2);
        end
    end
    %check if any power left for propulsion
    for k2 = 1:length(PowerNormForPropulsion(1,:))
        if PowerNormForPropulsion(k1,k2) < 0
            PowerNormForPropulsion(k1,k2) = 0; %0 power for propulsion is min possible
        end
    end
end
SpeedFromPower = (PowerNormForPropulsion./MaxPowerNormForPropulsion).^0.5.*(SpeedCombineRef*1000/3600);
%calculate error, pick speed for min error
Error2SpeedCombine = (ones(length(SpeedFromPower(:,1)),1)*SpeedCombineInterp-SpeedFromPower).^2; %squared error
%find min error for each row in potential solution space
SpeedCombineGrid = zeros(size(SpeedFromPower(:,1)));
CropRateNormGrid = zeros(size(SpeedFromPower(:,1)));
PowerEngineGrid = ones(size(SpeedFromPower(:,1))); %kW
PowerMotorGrid = ones(size(SpeedFromPower(:,1))); %kW
BatterySOCGrid = zeros(size(SpeedFromPower(:,1)));
TimeGrid = zeros(size(SpeedFromPower(:,1)));
for k1 = 1:length(Error2SpeedCombine(:,1))
    MinError2SpeedCombine = min(Error2SpeedCombine(k1,:));
    for k = 1:length(Error2SpeedCombine(k1,:))
        if Error2SpeedCombine(k1,k) == MinError2SpeedCombine
            IndexSpeedSolution = k;
        end
    end
    if IndexSpeedSolution == length(Error2SpeedCombine(k1,:))
        %combine is at its governed speed limit
        PowerNormGrid = MaxPowerNormForPropulsion+PowerNormGrainHarvestInterp(k1,end);
        PowerEngineGrid(k1) = PowerNormGrid*TotalPowerRef-PowerMotorInterp(k1,end);
    else
        PowerEngineGrid(k1) = PowerEngineInterp(IndexSpeedSolution); %kW
    end
    %select answers for crop volume and combine speed
    TimeGrid(k1) = TimePerGrid(IndexSpeedSolution);
    SpeedCombineGrid(k1) = SpeedCombineInterp(IndexSpeedSolution)/1000*3600; %km/hr
    CropRateNormGrid(k1) = CropRateNormInterp(k1,IndexSpeedSolution);
    PowerMotorGrid(k1) = PowerMotorInterp(k1,IndexSpeedSolution);
    BatterySOCGrid(k1) = BatterySOCInterp(k1,IndexSpeedSolution);
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
GrainHarvestRate = sum(GrainPerGrid)/(TimeCumulative/3600);
SpecificFuelConsumptionGrid = (interp1(NormFuelEfficiencyVsPower(:,1),NormFuelEfficiencyVsPower(:,2),PowerEngineGrid/EnginePowerRef,'linear','extrap'))*FuelEfficiencyRef; %g/kW/hr
FuelRateGrid = SpecificFuelConsumptionGrid.*PowerEngineGrid/(0.82*3.785*1000); %gal/hr
FuelConsumptionGrid = FuelRateGrid.*(TimeGrid./3600); %gal
FuelRate = sum(FuelConsumptionGrid)/(TimeCumulative/3600);
GrainHarvestValue = GrainHarvestRate*GrainPrice; %$/hr
FuelCost = FuelRate*FuelPrice; %$/hr
%
%
%% mean values for time step
SpeedCombine = sum(SpeedCombineGrid.*(TimeGrid./TimeCumulative));
EfficiencyGrainHarvest = sum(EfficiencyGrainHarvestGrid.*(TimeGrid./TimeCumulative));
CropRateNorm = sum(CropRateNormGrid.*(TimeGrid./TimeCumulative));
PowerEngineMean = sum(PowerEngineGrid.*(TimeGrid./TimeCumulative));
PowerMotorMean = sum(PowerMotorGrid.*(TimeGrid./TimeCumulative));
BatterySOC = BatterySOCGrid(end);
%
%
%% outputs
%randn
StateVector = [FieldIndexEndTimeStep,SpeedCombine,EfficiencyGrainHarvest,BatterySOC];
Reward = GrainHarvestValue-FuelCost;
Diagnostics = [PowerEngineMean,PowerMotorMean,CropRateNorm,FuelRate];
%
%
%
%
end