function [LossActor,LossCritic] = PlotLearningLoss(FolderName)
%
%
%
%
LossLogs = dir(FolderName);
LogFileIndex = [];
for k = 1:length(LossLogs)
    if LossLogs(k).bytes > 0
        LogFileIndex = [LogFileIndex,k];
    end
end
%
%
LossActor = zeros(length(LogFileIndex));
LossCritic = zeros(length(LogFileIndex),2);
for k = 1:length(LogFileIndex)
    LogFile = load([FolderName,'/',LossLogs(LogFileIndex(k)).name]);
    %PPO
%     LossActor(k) = LogFile.agentLearnData.ActorLoss{end};
%     LossCritic(k) = LogFile.agentLearnData.CriticLoss{end};
    %TD-DDPG
    LossLogActor = LogFile.agentLearnData.ActorLoss;
    LossLogCritic = LogFile.agentLearnData.CriticLoss;
    LossLogActorPrep = [];
    LossLogCriticPrep = [];
    for k1 = 1:length(LossLogActor)
        if ~isempty(LossLogActor(k1))
            LossLogActorPrep = [LossLogActorPrep;LossLogActor{k1}];
        end
        if ~isempty(LossLogCritic(k1))
            LossLogCriticPrep = [LossLogCriticPrep;LossLogCritic{k1}];
        end
    end
    LossActor(k) = mean(LossLogActorPrep);
    LossCritic(k,:) = mean(LossLogCriticPrep);
end
%
%
figure(1)
plot(LossActor)
xlabel('Episode')
ylabel('Actor Loss')
%
figure(2)
plot(LossCritic)
xlabel('Episode')
ylabel('Critic Loss')
%
%
%
%
end