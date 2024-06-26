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
            obj.Environment.Harvest.MaxDuration = FieldLong(end,1)/(10*1000/3600); %sec, max time to harvest field before penalizing reward
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
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency...
                GrainPrice FuelPrice MaxHarvestDuration
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
            BatteryMaxChargeRate = obj.Environment.Combine.Design(4); %kW
            BatteryMaxDischargeRate = obj.Environment.Combine.Design(5); %kW
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
            MaxHarvestDuration = obj.Environment.Harvest.MaxDuration;
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %
            %simulate combine through field
            ControlInputVectorFinal = [];
            StateVectorFinal = [];
            RewardFinal = [];
            DiagnosticsFinal = [];
            Actions = [PowerEngineRequest/EnginePowerRef,PowerMotorRequest/BatteryMaxDischargeRate,CombineControlSetpoint];
%             Actions = [1,1,0.93];
            [InitialObservation,LoggedSignals] = InitializeCombineEnvironment();
%             LoggedSignals.StartTimeStep = [BatterySOC,FieldIndexStartTimeStep];
            IsDone = 0;
            while IsDone == 0
                [StateVector,Reward,IsDone,LoggedSignals] = ControlCombineEnvironment(Actions,LoggedSignals);
                StateVectorFinal = [StateVectorFinal;StateVector];
                RewardFinal = [RewardFinal;Reward];
                DiagnosticsFinal = [DiagnosticsFinal;LoggedSignals.Diagnostics];
                ControlInputVectorFinal = [ControlInputVectorFinal;Actions];
                %feedback controller
                %combine control
                ControlCombineSetpointMax = 0.7*(StateVector(1))+0.35;
                Gain = obj.Agent.Control.Gains(1);
                Actions(3) = Actions(3)+Gain*(ControlCombineSetpointMax-Actions(3));
%                 Actions(3) = 0.93;
                %battery charge/discharge
                if Actions(2) >= 0 %using battery energy
                    if StateVector(3) <= 0.2% || Diagnostics(1) < 0.9*PowerEngineRequest
                        Actions(2) = -21/BatteryMaxDischargeRate; %charge battery
                    end
                else %charging battery
                    if StateVector(3) >= 0.7
                        Actions(2) = 50/BatteryMaxDischargeRate; %discharge battery
                    end
                end
            end
            StateVectorFinal(:,1) = StateVectorFinal(:,1).*10; %get combine speed back to km/hr
            obj.SimulationResults.RewardCumulative = sum(RewardFinal);
            Time = (1:DurationTimeStep:DurationTimeStep*length(RewardFinal(:,1)))'-1; %sec
            obj.SimulationResults.ControlInputVector = [Time,ControlInputVectorFinal];
            obj.SimulationResults.StateVector = [Time,StateVectorFinal];
            obj.SimulationResults.Reward = [Time,RewardFinal];
            obj.SimulationResults.Diagnostics = [Time,DiagnosticsFinal];
            %
            %
        end
        
        function PlotSimulationResults(obj,PlotFlag)
            %plot simulation
            %PlotFlag = 0 to plot only the manual baseline
            %PlotFlag = 1 to plot only RL results
            %PlotFlag = 2 to plot both
            if PlotFlag > 0
                ControlVectorRL = obj.RLModel.SimulationResultsStatesActionsReward{1:end-1,[1,5:7]};
                StateVectorRL = obj.RLModel.SimulationResultsStatesActionsReward{:,1:4};
                RewardRL = obj.RLModel.SimulationResultsStatesActionsReward{2:end,[1,8]};
            end
            ControlVector = obj.SimulationResults.ControlInputVector;
            StateVector = obj.SimulationResults.StateVector;
            Reward = obj.SimulationResults.Reward;
            Diagnostics = obj.SimulationResults.Diagnostics;
            %plot results
            figure(1)
            if PlotFlag == 0 || PlotFlag == 2
                plot(ControlVector(:,1),ControlVector(:,4))
                legend('Baseline')
                if PlotFlag == 2
                    hold on
                    plot(ControlVectorRL(:,1),ControlVectorRL(:,4))
                    legend('Baseline','RL Policy')
                end
            else
                plot(ControlVectorRL(:,1),ControlVectorRL(:,4))
                legend('RL Policy')
            end
            xlabel('Time (sec)')
            ylabel('Combine Control Setting')
            %
            figure(2)
            if PlotFlag == 0 || PlotFlag == 2
                plot(ControlVector(:,1),ControlVector(:,2).*obj.Environment.Combine.Performance.EnginePowerRef,ControlVector(:,1),ControlVector(:,3).*obj.Environment.Combine.Design(5))
                legend('Engine','Motor')
                if PlotFlag == 2
                    hold on
                    plot(ControlVectorRL(:,1),ControlVectorRL(:,2).*obj.Environment.Combine.Performance.EnginePowerRef,ControlVectorRL(:,1),ControlVectorRL(:,3).*obj.Environment.Combine.Design(5))
                    legend('Baseline Engine','Baseline Motor','RL Policy Engine','RLPolicyMotor')
                end
            else
                plot(ControlVectorRL(:,1),ControlVectorRL(:,2).*obj.Environment.Combine.Performance.EnginePowerRef,ControlVectorRL(:,1),ControlVectorRL(:,3).*obj.Environment.Combine.Design(5))
                legend('RL Policy Engine','RLPolicyMotor')
            end
            xlabel('Time (sec)')
            ylabel('Power Request (kW)')
            %
            figure(3)
            if PlotFlag == 0 || PlotFlag == 2
                plot(StateVector(:,1),StateVector(:,2))
                legend('Baseline')
                if PlotFlag == 2
                    hold on
                    plot(StateVectorRL(:,1),StateVectorRL(:,2))
                    legend('Baseline','RL Policy')
                end
            else
                plot(StateVectorRL(:,1),StateVectorRL(:,2))
                legend('RL Policy')
            end
            xlabel('Time (sec)')
            ylabel('Combine Speed (km/hr)')
            %
            figure(4)
            if PlotFlag == 0 || PlotFlag == 2
                plot(StateVector(:,1),StateVector(:,3))
                legend('Baseline')
                if PlotFlag == 2
                    hold on
                    plot(StateVectorRL(:,1),StateVectorRL(:,3))
                    legend('Baseline','RL Policy')
                end
            else
                plot(StateVectorRL(:,1),StateVectorRL(:,3))
                legend('RL Policy')
            end
            xlabel('Time (sec)')
            ylabel('Grain Harvest Efficiency (%)')
            %
            figure(5)
            if PlotFlag == 0 || PlotFlag == 2
                plot(StateVector(:,1),StateVector(:,4))
                legend('Baseline')
                if PlotFlag == 2
                    hold on
                    plot(StateVectorRL(:,1),StateVectorRL(:,4))
                    legend('Baseline','RL Policy')
                end
            else
                plot(StateVectorRL(:,1),StateVectorRL(:,4))
                legend('RL Policy')
            end
            xlabel('Time (sec)')
            ylabel('Battery State of Charge')
            %
            figure(6)
            if PlotFlag == 0 || PlotFlag == 2
                plot(Reward(:,1),Reward(:,2))
                legend('Baseline')
                if PlotFlag == 2
                    hold on
                    plot(RewardRL(:,1),RewardRL(:,2))
                    legend('Baseline','RL Policy')
                end
            else
                plot(RewardRL(:,1),RewardRL(:,2))
                legend('RL Policy')
            end
            xlabel('Time (sec)')
            ylabel('Reward ($ per time step)')
            %
            if PlotFlag == 0
                figure(7)
                plot(Diagnostics(:,1),Diagnostics(:,2))
                xlabel('Time (sec)')
                ylabel('Engine Power (kW)')
            end
            %
            if PlotFlag == 0
                figure(8)
                plot(Diagnostics(:,1),Diagnostics(:,3))
                xlabel('Time (sec)')
                ylabel('Battery Power (kW)')
            end
            %
            %
        end

        function obj = BuildRLModel(obj)
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency...
                GrainPrice FuelPrice MaxHarvestDuration
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
            MaxHarvestDuration = obj.Environment.Harvest.MaxDuration;
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %build rl objects for Matlab
            Actions = rlNumericSpec([1,3],'LowerLimit',[0.75,-obj.Environment.Combine.Design(4)/obj.Environment.Combine.Design(5),0.85],'UpperLimit',[1,1,1.15]);
            States = rlNumericSpec([1,3]);
            obj.RLModel.RLEnvironment = rlFunctionEnv(States,Actions,'ControlCombineEnvironment','InitializeCombineEnvironment');
            AgentInitializeProps = rlAgentInitializationOptions('NumHiddenUnit',128);
%             AgentProps = rlPPOAgentOptions('ExperienceHorizon',128);
%             obj.RLModel.RLAgent = rlPPOAgent(States,Actions,AgentInitializeProps);
            obj.RLModel.RLAgent = rlTD3Agent(States,Actions,AgentInitializeProps);
%             obj.RLModel.RLAgent = rlDDPGAgent(States,Actions,AgentInitializeProps);
%             obj.RLModel.RLAgent = rlDQNAgent(States,Actions,AgentInitializeProps);
        end

        function TrainingInfo = TrainRLModel(obj)
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency...
                GrainPrice FuelPrice MaxHarvestDuration
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
            MaxHarvestDuration = obj.Environment.Harvest.MaxDuration;
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %
%             trainOpts = rlTrainingOptions(...
%                 'MaxEpisodes',400,...
%                 'MaxStepsPerEpisode',1000,...
%                 'ScoreAveragingWindowLength',10,...
%                 'StopTrainingCriteria',"AverageReward",...
%                 'StopTrainingValue',270,...
%                 'SaveAgentCriteria',"EpisodeReward",...
%                 'SaveAgentValue',265);
            trainOpts = rlTrainingOptions(...
                'MaxEpisodes',400,...
                'MaxStepsPerEpisode',1000,...
                'ScoreAveragingWindowLength',5,...
                'SaveAgentCriteria',"EpisodeReward",...
                'SaveAgentValue',254);
%             TrainingLogger = rlDataLogger();
%             TrainingLogger.LoggingOptions.LoggingDirectory = "TrainingLog";
%             TrainingLogger.AgentLearnFinishedFcn = @AgentLearnLoss;
            TrainingInfo = train(obj.RLModel.RLAgent,obj.RLModel.RLEnvironment,trainOpts);%,Logger=TrainingLogger);
            %
        end

        function obj = OperateCombineRL(obj)
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency...
                GrainPrice FuelPrice MaxHarvestDuration
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
            MaxHarvestDuration = obj.Environment.Harvest.MaxDuration;
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            %
            %simulate field with RL policy
            SimulationResultsRL = sim(obj.RLModel.RLAgent,obj.RLModel.RLEnvironment);
            %
            %convert results to something useful
            SimulationResultsRLObservation = timetable2table(timeseries2timetable(SimulationResultsRL.Observation.obs1));
            SimulationResultsRLAction = timetable2table(timeseries2timetable(SimulationResultsRL.Action.act1));
            SimulationResultsRLReward = timetable2table(timeseries2timetable(SimulationResultsRL.Reward));
            TimeSignal = (1:1:length(SimulationResultsRLObservation{:,1}))'.*20-20;
            SimulationResultsRLStates = zeros(length(SimulationResultsRLObservation{:,1}),4);
            SimulationResultsRLStates(:,1) = TimeSignal;
            SimulationResultsRLActions = zeros(length(TimeSignal),4);
            SimulationResultsRLActions(:,1) = TimeSignal;
            SimulationResultsRLRewards = zeros(length(TimeSignal),2);
            SimulationResultsRLRewards(:,1) = TimeSignal;            
            for k = 1:length(TimeSignal)
                ObservationsTimeStep = SimulationResultsRLObservation{k,2};
                SimulationResultsRLStates(k,2) = ObservationsTimeStep(:,:,1);
                SimulationResultsRLStates(k,3) = ObservationsTimeStep(:,:,2);
                SimulationResultsRLStates(k,4) = ObservationsTimeStep(:,:,3);
                if k < length(TimeSignal)
                    ActionsTimeStep = SimulationResultsRLAction{k,2};
                    SimulationResultsRLActions(k,2) = ActionsTimeStep(:,:,1);
                    SimulationResultsRLActions(k,3) = ActionsTimeStep(:,:,2);
                    SimulationResultsRLActions(k,4) = ActionsTimeStep(:,:,3);
                end
                if k > 1
                    RewardsTimeStep = SimulationResultsRLReward{k-1,2};
                    SimulationResultsRLRewards(k,2) = RewardsTimeStep(:,:,1);
                end
            end
            SimulationResultsRLStates(:,2) = SimulationResultsRLStates(:,2).*10; %convert normalized state to combine speed in km/hr
            SimulationResultsRLTable = array2table([SimulationResultsRLStates,SimulationResultsRLActions(:,2:end),SimulationResultsRLRewards(:,2)]);
            SimulationResultsRLTable.Properties.VariableNames = {'Time_sec','CombineSpeed_kmph','GrainEfficiency','BatterySOC',...
                'RequestedEnginePower_kW','RequestedMotorPower_kW','CombineControlSetting','Reward_$'};
            obj.RLModel.SimulationResultsStatesActionsReward = SimulationResultsRLTable;
            %
        end
        
    end
end

