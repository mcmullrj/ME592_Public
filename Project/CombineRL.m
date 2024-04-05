classdef CombineRL
    %model for simplified combine/field for RL project
    
    properties
        Environment = [];
    end
    
    methods
        function obj = CombineRL
            %Construct an instance of this class
            %load normalized combine performance settings
            obj.Environment.Combine.Performance = load('CombinePerformanceNorm.mat');
        end
        
        function obj = CreateField(obj,Acres,MeanYield,StDevYield)
            %create field environment
            %
            %update units
            Acres = Acres*4047; %m^2 from acres
            RowSpacing = 30*0.0254; %m = 30 inches
            %MeanYield = bushels/acre
            %StDevYield = bushels/acre
            %Mean Moisture = percent
            %
            %constants
            %56 lbs/bu, convert to mass or keep as volume?
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
            LengthWidthYieldSubplot = (10*4047)^0.5; %m
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
            CropVolumeDeviation = (0.05.*MassFractionGrain).*(randn(size(YieldMapCoarse.Rows)));
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
            Turnaround = zeros(RowsHeader*4,2);
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
        
        function [StateVectorFinal,RewardFinal,DiagnosticsFinal,RewardMean] = OperateCombine(obj)
            %solve path through virtual field
            %
            %define invariants
            global DurationTimeStep FieldPath SpeedCombineInterp CombineSettingNorm FlowCropNorm PowerGrainProcessNorm GrainEfficiency NormFuelEfficiencyVsPower...
                FlowCropRef SpeedCombineRef TotalPowerRef EnginePowerRef FuelEfficiencyRef BatteryCapacity BatteryMaxChargeRate BatteryMaxDischargeRate MotorEfficiency GrainPrice FuelPrice
            %
            %
            %reward definition
            DurationTimeStep = 20; %sec
            GrainPrice = 0.2;
            FuelPrice = 5;
            %
            %
            %combine definition
            CombineSettingNorm = obj.Environment.Combine.Performance.CombineSettingNorm;
            FlowCropNorm = obj.Environment.Combine.Performance.FlowCropNorm;
            PowerGrainProcessNorm = obj.Environment.Combine.Performance.PowerGrainProcessNorm;
            GrainEfficiency = obj.Environment.Combine.Performance.GrainEfficiency;
            NormFuelEfficiencyVsPower = obj.Environment.Combine.Performance.NormFuelEfficiencyVsPower;
            BatteryCapacity = obj.Environment.Combine.Design(3); %kWh
            BatteryMaxChargeRate = obj.Environment.Combine.Design(3); %kWh
            BatteryMaxDischargeRate = obj.Environment.Combine.Design(3); %kWh
            MotorEfficiency = obj.Environment.Combine.Design(3); %kWh
            FlowCropRef = obj.Environment.Combine.Performance.FlowCropRef;
            SpeedCombineRef = obj.Environment.Combine.Performance.SpeedCombineRef;
            TotalPowerRef = obj.Environment.Combine.Performance.TotalPowerRef;
            EnginePowerRef = obj.Environment.Combine.Performance.EnginePowerRef;
            FuelEfficiencyRef = obj.Environment.Combine.Performance.FuelEfficiencyRef;
            %
            %
            %field definition
            FieldPath = obj.Environment.Harvest.FieldPath;
            FieldMapGridDistance = FieldPath(2,1)-FieldPath(1,1); %m/grid
            SpeedCombineInterp = (0:0.02:1).*SpeedCombineRef.*1000./3600; %m/sec
            GridsPerTimeStepInterp = (SpeedCombineInterp./FieldMapGridDistance).*DurationTimeStep;
            %
            %
            %initialize system control
            FieldIndexStartTimeStep = 1; %start at first grid 
%             PowerEngineRequest = EnginePowerRef;
            PowerEngineRequest = 300;
%             PowerMotorRequest = TotalPowerRef-EnginePowerRef;
            PowerMotorRequest = 20;
            CombineSettingSetpoint = 1.0;
            BatterySOC = 0.5;
            %
            %
            %simulate combine through field
            StateVectorFinal = [];
            RewardFinal = [];
            DiagnosticsFinal = [];
            MaxFieldIndexStart = (length(FieldPath(:,1))-max(GridsPerTimeStepInterp));
            while FieldIndexStartTimeStep < MaxFieldIndexStart
                [StateVector,Reward,Diagnostics] = ControlCombine(FieldIndexStartTimeStep,BatterySOC,PowerEngineRequest,PowerMotorRequest,CombineSettingSetpoint);
                FieldIndexStartTimeStep = StateVector(1);
                BatterySOC = StateVector(4);
                StateVectorFinal = [StateVectorFinal;StateVector];
                RewardFinal = [RewardFinal;Reward];
                DiagnosticsFinal = [DiagnosticsFinal;Diagnostics];
                %feedback controller
                
            end
            RewardMean = mean(RewardFinal);
            Time = (1:DurationTimeStep:DurationTimeStep*length(RewardFinal(:,1)))'-1; %sec
            StateVectorFinal = [Time,StateVectorFinal];
            RewardFinal = [Time,RewardFinal];
            DiagnosticsFinal = [Time,DiagnosticsFinal];
            %
            %
            %plot results
            figure(1)
            plot(StateVectorFinal(:,1),StateVectorFinal(:,3))
            xlabel('Time (sec)')
            ylabel('Combine Speed (km/hr)')
            %
            figure(2)
            plot(StateVectorFinal(:,1),StateVectorFinal(:,4).*100)
            xlabel('Time (sec)')
            ylabel('Grain Harvest Efficiency (%)')
            %
            figure(3)
            plot(StateVectorFinal(:,1),StateVectorFinal(:,5))
            xlabel('Time (sec)')
            ylabel('Battery State of Charge')
            %
            figure(4)
            plot(RewardFinal(:,1),RewardFinal(:,2))
            xlabel('Time (sec)')
            ylabel('Reward ($/hr)')
            %
            figure(5)
            plot(DiagnosticsFinal(:,1),DiagnosticsFinal(:,2))
            xlabel('Time (sec)')
            ylabel('Engine Power (kW)')
            %
            figure(6)
            plot(DiagnosticsFinal(:,1),DiagnosticsFinal(:,3))
            xlabel('Time (sec)')
            ylabel('Battery Power (kW)')
            %
            %
        end
    end
end
