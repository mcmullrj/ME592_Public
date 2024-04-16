classdef CombineRL
    %model for simplified combine/field for RL project
    
    properties
        Environment = [];
        Agent = [];
        SimulationResults = [];
        RLModel = [];
    end
    
    methods
        function obj = CombineRL
            %Construct an instance of this class
            %load normalized combine performance settings
            obj.Environment.Combine.Performance = load('CombinePerformanceNorm.mat');
        end
        
        function obj = CreateField(obj,Acres,MeanYield,StDevYield,FieldConsistency)
            %create field environment
            %FieldConsistency approx. 0.05 to 0.1 - standard deviation
            %multiplier for total crop volume
            %
            %update units
            Acres = Acres*4047; %m^2 from acres
            RowSpacing = 30*0.0254; %m = 30 inches
            %MeanYield = bushels/acre
            %StDevYield = bushels/acre
            %Mean Moisture = percent
            %
            %constants
            MassFractionGrain = 0.4; %guess volume fraction and mass fraction are similar
            %
            %assume field is square and ignore end rows, length = width
            LengthWidthField = Acres^0.5; %m
            %create a matrix of field squares for each crop row
            NumberRowsCrop = round(LengthWidthField/RowSpacing);
            RowCoordinate = (0:1:NumberRowsCrop-1).*RowSpacing; %m, row vector
            FieldMap.GridRows = RowCoordinate; %m
            FieldMap.GridColumns = FieldMap.GridRows'; %m
            %
            %create yield map for field
            %assign yield map to 10-acre subplots
            LengthWidthYieldSubplot = (5*4047)^0.5; %m
            if LengthWidthField > LengthWidthYieldSubplot
                YieldCoordinate = (0:LengthWidthYieldSubplot:LengthWidthField); %m, yield row vector
                YieldCoordinate(end+1) = YieldCoordinate(end)+LengthWidthYieldSubplot; %m, extend yield mapping
            else
                YieldCoordinate = [0,LengthWidthField];
            end
            YieldMapCoarse.Rows = ones(length(YieldCoordinate),1)*YieldCoordinate;
            YieldMapCoarse.Columns = YieldMapCoarse.Rows';
            %assign grain volume and crop volume to each subplot - assume
            %standard distributions
            YieldDeviation = StDevYield.*(randn(size(YieldMapCoarse.Rows)));
            YieldMapCoarse.GrainVolume = MeanYield+YieldDeviation;
            CropVolumeDeviation = (FieldConsistency.*MassFractionGrain).*(randn(size(YieldMapCoarse.Rows)));
            YieldMapCoarse.CropVolume = YieldMapCoarse.GrainVolume./(MassFractionGrain+CropVolumeDeviation);
            FieldMap.GrainVolume = griddata(YieldMapCoarse.Rows,YieldMapCoarse.Columns,YieldMapCoarse.GrainVolume,ones(length(RowCoordinate),1)*RowCoordinate,(ones(length(RowCoordinate),1)*RowCoordinate)','natural');
            FieldMap.CropVolume = griddata(YieldMapCoarse.Rows,YieldMapCoarse.Columns,YieldMapCoarse.CropVolume,ones(length(RowCoordinate),1)*RowCoordinate,(ones(length(RowCoordinate),1)*RowCoordinate)','natural');
            %assemble output
            obj.Environment.FieldMap = FieldMap;
        end
        
        function PlotField(obj)
            FieldRows = ones(size(obj.Environment.FieldMap.GridColumns))*obj.Environment.FieldMap.GridRows;
            FieldColumns = obj.Environment.FieldMap.GridColumns*ones(size(obj.Environment.FieldMap.GridRows));
            Grain = obj.Environment.FieldMap.GrainVolume;
            Crop = obj.Environment.FieldMap.CropVolume;
            %grain volume map
            figure(1)
            [C,h] = contourf(FieldRows,FieldColumns,Grain);
            clabel(C,h)
            xlabel('Field Length (m)')
            ylabel('Field Width (m)')
            title('Grain Volume Density (bushels/acre)')
            %crop volume map
            figure(2)
            [C,h] = contourf(FieldRows,FieldColumns,Crop);
            clabel(C,h)
            xlabel('Field Length (m)')
            ylabel('Field Width (m)')
            title('Total Crop Volume Density (bushels/acre)')            
        end
        
        function obj = CreateCombine(obj,RowsHeader,PowerEngine,CapacityBattery)
            %power
            RowSpacing = obj.Environment.FieldMap.GridRows(2)-obj.Environment.FieldMap.GridRows(1); %m
            obj.Environment.Combine.DesignParameterColumns = {'HeaderRows','EngineMaxPower_kW','BatteryCapacity_kWh','BatteryMaxChargeRate_kW','BatteryMaxDischargeRate_kW','MotorEfficiency'};
            obj.Environment.Combine.Design = [RowsHeader,PowerEngine,CapacityBattery,0.7*CapacityBattery,2*CapacityBattery,0.9];
            obj.Environment.Combine.Performance.FlowCropRef = RowSpacing*RowsHeader*11*1000/4047*500; %bu/hr crop, 11 km/hr, 500 bu/acre total crop density
            obj.Environment.Combine.Performance.EnginePowerRef = PowerEngine;
            obj.Environment.Combine.Performance.TotalPowerRef = 0.0241*obj.Environment.Combine.Performance.FlowCropRef; %kW
        end
        
        function obj = DefineHarvestPath(obj)
            %create streamline for combine path through field
            %acres per field grid
            DistanceFieldGrid = obj.Environment.FieldMap.GridRows(2)-obj.Environment.FieldMap.GridRows(1); %m
            AreaFieldGrid = (DistanceFieldGrid^2)/4047; %acres
            %create one long field pass for harvest
            RowsHeader = obj.Environment.Combine.Design(1);
            Turnaround = zeros(RowsHeader*5,2);
            FieldPassBreakpoints = (1:RowsHeader:length(obj.Environment.FieldMap.GridColumns));
            FieldLong = [];
            OddPass = 1;
            for k = 1:length(FieldPassBreakpoints)-1
                RowsFieldPass = obj.Environment.FieldMap.CropVolume(FieldPassBreakpoints(k):FieldPassBreakpoints(k+1)-1,:);
                CropVolumeFieldPass = (ones(1,RowsHeader)*RowsFieldPass).*AreaFieldGrid; %bu
                RowsFieldPass = obj.Environment.FieldMap.GrainVolume(FieldPassBreakpoints(k):FieldPassBreakpoints(k+1)-1,:);
                GrainVolumeFieldPass = (ones(1,RowsHeader)*RowsFieldPass).*AreaFieldGrid; %bu
                %add grain moisture and elevation maps
                VolumeFieldPass = [CropVolumeFieldPass;GrainVolumeFieldPass]';
                if OddPass == 1 %forward
                    FieldLong = [FieldLong;VolumeFieldPass];
                    OddPass = 0;
                else %return
                    FieldLong = [FieldLong;flipud(VolumeFieldPass)];
                    OddPass = 1;
                end
                FieldLong = [FieldLong;Turnaround];
            end
            FieldLong = [(1:1:length(FieldLong(:,1)))'.*DistanceFieldGrid,FieldLong];
            obj.Environment.Harvest.NumberofPasses = length(FieldPassBreakpoints);
            obj.Environment.Harvest.FieldPathColumns = {'PathDistance_m','CropVolumeInDistanceStep_bu','GrainVolumeInDistanceStep_bu'};
            obj.Environment.Harvest.FieldPath = FieldLong;
        end
        
        function obj = DefineControl(obj,GrainValue,FuelValue,ControlSettings,StartingFieldIndex)
            %set control parameters
            %define magnitude for rewards
            obj.Agent.Reward.GrainValue = GrainValue;
            obj.Agent.Reward.FuelValue = FuelValue;
            %
            %define control inputs
            obj.Agent.Control.TimeStepStateUpdate = ControlSettings(1);
            obj.Agent.Control.Gains = ControlSettings(2:3); %proportional, integral
            obj.Agent.Control.InitialState = ControlSettings(4:6); %battery SOC, motor power, engine power
            obj.Agent.Control.FieldIndexStart = StartingFieldIndex;
        end
        
        function obj = OperateCombine(obj)
            %solve path through virtual field
            %
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency GrainPrice FuelPrice
            %
            %
            %control settings
            DurationTimeStep = obj.Agent.Control.TimeStepStateUpdate;
            BatterySOC = obj.Agent.Control.InitialState(1);
            PowerEngineRequest = obj.Agent.Control.InitialState(3);
            PowerMotorRequest = obj.Agent.Control.InitialState(2);
            FieldIndexStart = obj.Agent.Control.FieldIndexStart; %start at first grid
            CombineControlSetpoint = 1.0;
            FieldIndexStartTimeStep = 1;
            %
            %
            %reward definition
            GrainPrice = obj.Agent.Reward.GrainValue;
            FuelPrice = obj.Agent.Reward.FuelValue;
            %
            %
            %combine definition
            CombineSettingNorm = obj.Environment.Combine.Performance.CombineSettingNorm;
            FlowCropNorm = obj.Environment.Combine.Performance.FlowCropNorm;
            PowerGrainProcessNorm = obj.Environment.Combine.Performance.PowerGrainProcessNorm;
            GrainEfficiency = obj.Environment.Combine.Performance.GrainEfficiency;
            NormFuelEfficiencyVsPower = obj.Environment.Combine.Performance.NormFuelEfficiencyVsPower;
            BatteryCapacity = obj.Environment.Combine.Design(3); %kWh
            BatteryMaxChargeRate = obj.Environment.Combine.Design(4); %kWh
            BatteryMaxDischargeRate = obj.Environment.Combine.Design(5); %kWh
            MotorEfficiency = obj.Environment.Combine.Design(3); %kWh
            FlowCropRef = obj.Environment.Combine.Performance.FlowCropRef;
            SpeedCombineRef = obj.Environment.Combine.Performance.SpeedCombineRef;
            TotalPowerRef = obj.Environment.Combine.Performance.TotalPowerRef;
            EnginePowerRef = obj.Environment.Combine.Performance.EnginePowerRef;
            FuelEfficiencyRef = obj.Environment.Combine.Performance.FuelEfficiencyRef;
            %
            %
            %field definition
            if FieldIndexStart > 1 && FieldIndexStart < length(FieldPath(:,1))
                FieldPath = [obj.Environment.Harvest.FieldPath(FieldIndexStart:end,:);obj.Environment.Harvest.FieldPath(1:FieldIndexStart-1,:)];
            elseif FieldIndexStart <= -1 && FieldIndexStart > -length(FieldPath(:,1)) %flip directions
                FieldPath = obj.Environment.Harvest.FieldPath(:,1);
                FieldPath(:,2:3) = flipud(obj.Environment.Harvest.FieldPath(:,2:3));
                if FieldIndexStart ~= -1
                    FieldPath = [obj.Environment.Harvest.FieldPath(abs(FieldIndexStart):end,:);obj.Environment.Harvest.FieldPath(1:abs(FieldIndexStart)-1,:)];
                end
            else
                FieldPath = obj.Environment.Harvest.FieldPath;
            end
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %
            %simulate combine through field
            ControlInputVectorFinal = [];
            StateVectorFinal = [];
            RewardFinal = [];
            DiagnosticsFinal = [];
            Actions = [PowerEngineRequest,PowerMotorRequest,CombineControlSetpoint];
            LoggedSignals.StartTimeStep = [BatterySOC,FieldIndexStartTimeStep];
            IsDone = 0;
            while IsDone == 0
                [StateVector,Reward,IsDone,LoggedSignals] = ControlCombineEnvironment(Actions,LoggedSignals);
                StateVectorFinal = [StateVectorFinal;StateVector];
                RewardFinal = [RewardFinal;Reward];
                DiagnosticsFinal = [DiagnosticsFinal;LoggedSignals.Diagnostics];
                ControlInputVectorFinal = [ControlInputVectorFinal;Actions];
                %feedback controller
                %combine control
                ControlCombineSetpointMax = 0.7*(StateVector(1)/10)+0.35;
                Gain = obj.Agent.Control.Gains(1);
                Actions(3) = Actions(3)+Gain*(ControlCombineSetpointMax-Actions(3));
                %battery charge/discharge
                if Actions(2) >= 0 %using battery energy
                    if StateVector(3) < 0.2% || Diagnostics(1) < 0.9*PowerEngineRequest
                        Actions(2) = -40; %charge battery
                    end
                else %charging battery
                    if StateVector(3) > 0.7
                        Actions(2) = 50; %discharge battery
                    end
                end
            end
            obj.SimulationResults.RewardCumulative = sum(RewardFinal);
            Time = (1:DurationTimeStep:DurationTimeStep*length(RewardFinal(:,1)))'-1; %sec
            obj.SimulationResults.ControlInputVector = [Time,ControlInputVectorFinal];
            obj.SimulationResults.StateVector = [Time,StateVectorFinal];
            obj.SimulationResults.Reward = [Time,RewardFinal];
            obj.SimulationResults.Diagnostics = [Time,DiagnosticsFinal];
            %
            %
        end
        
        function PlotSimulationResults(obj)
            %plot simulation
            ControlVector = obj.SimulationResults.ControlInputVector;
            StateVector = obj.SimulationResults.StateVector;
            Reward = obj.SimulationResults.Reward;
            Diagnostics = obj.SimulationResults.Diagnostics;
            %plot results
            figure(1)
            plot(ControlVector(:,1),ControlVector(:,4))
            xlabel('Time (sec)')
            ylabel('Combine Control Setting')
            %
            figure(2)
            plot(ControlVector(:,1),ControlVector(:,2:3))
            xlabel('Time (sec)')
            ylabel('Power Request (kW)')
            legend('Engine','Motor')
            %
            figure(3)
            plot(StateVector(:,1),StateVector(:,2))
            xlabel('Time (sec)')
            ylabel('Combine Speed (km/hr)')
            %
            figure(4)
            plot(StateVector(:,1),StateVector(:,3).*100)
            xlabel('Time (sec)')
            ylabel('Grain Harvest Efficiency (%)')
            %
            figure(5)
            plot(StateVector(:,1),StateVector(:,4))
            xlabel('Time (sec)')
            ylabel('Battery State of Charge')
            %
            figure(6)
            plot(Reward(:,1),Reward(:,2))
            xlabel('Time (sec)')
            ylabel('Reward ($ per time step)')
            %
            figure(7)
            plot(Diagnostics(:,1),Diagnostics(:,2))
            xlabel('Time (sec)')
            ylabel('Engine Power (kW)')
            %
            figure(8)
            plot(Diagnostics(:,1),Diagnostics(:,3))
            xlabel('Time (sec)')
            ylabel('Battery Power (kW)')
            %
            %
        end

        function obj = BuildRLModel(obj)
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency GrainPrice FuelPrice
            %
            %control settings
            DurationTimeStep = obj.Agent.Control.TimeStepStateUpdate;
            %
            %reward definition
            GrainPrice = obj.Agent.Reward.GrainValue;
            FuelPrice = obj.Agent.Reward.FuelValue;
            %
            %combine definition
            CombineSettingNorm = obj.Environment.Combine.Performance.CombineSettingNorm;
            FlowCropNorm = obj.Environment.Combine.Performance.FlowCropNorm;
            PowerGrainProcessNorm = obj.Environment.Combine.Performance.PowerGrainProcessNorm;
            GrainEfficiency = obj.Environment.Combine.Performance.GrainEfficiency;
            NormFuelEfficiencyVsPower = obj.Environment.Combine.Performance.NormFuelEfficiencyVsPower;
            BatteryCapacity = obj.Environment.Combine.Design(3); %kWh
            BatteryMaxChargeRate = obj.Environment.Combine.Design(4); %kWh
            BatteryMaxDischargeRate = obj.Environment.Combine.Design(5); %kWh
            MotorEfficiency = obj.Environment.Combine.Design(3); %kWh
            FlowCropRef = obj.Environment.Combine.Performance.FlowCropRef;
            SpeedCombineRef = obj.Environment.Combine.Performance.SpeedCombineRef;
            TotalPowerRef = obj.Environment.Combine.Performance.TotalPowerRef;
            EnginePowerRef = obj.Environment.Combine.Performance.EnginePowerRef;
            FuelEfficiencyRef = obj.Environment.Combine.Performance.FuelEfficiencyRef;
            %
            %field definition
            FieldPath = obj.Environment.Harvest.FieldPath;
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %build rl objects for Matlab
            Actions = rlNumericSpec([1,3],'LowerLimit',[220,-obj.Environment.Combine.Design(4),0.85],'UpperLimit',[obj.Environment.Combine.Performance.EnginePowerRef,obj.Environment.Combine.Design(5),1.15]);
            States = rlNumericSpec([1,3]);
            obj.RLModel.RLEnvironment = rlFunctionEnv(States,Actions,'ControlCombineEnvironment','InitializeCombineEnvironment');
            AgentInitializeProps = rlAgentInitializationOptions('NumHiddenUnit',128);
%             AgentProps = rlPPOAgentOptions('ExperienceHorizon',128);
            obj.RLModel.RLAgent = rlPPOAgent(States,Actions,AgentInitializeProps);
        end

        function TrainingInfo = TrainRLModel(obj)

            trainOpts = rlTrainingOptions;
            trainOpts.MaxEpisodes = 500;
            trainOpts.StopTrainingCriteria = 'EpisodeCount';

            TrainingInfo = train(obj.RLModel.RLAgent,obj.RLModel.RLEnvironment,trainOpts);

        end
        
    end
end

