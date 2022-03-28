% Code for BILP_algorithm 
%
% Requires:
% Gurobi ILP solver
close all
clear all
clc
load sceneInfo.mat
AddPath; % Add the necessary paths 
Mice_Image_Detection_Path % Add the path for the image sequence and the detections 

%% Parameters 

% Parameters for Heuristics
param.Prun_Thre=0; % Parameter for pruning detections with the confidence score less than this value  
param.tret=15; % Removing tracks with life time (# frames) less than this threshold
param.Term_Frame=500; % The parameter for termination condition

% Parameters for motion model
q1=0.5; % The standard deviation of the process noise for the dynamic model 
qm = 7; % The standard deviation of the measurement noise
param.Vmax=7; % maximum velocity that a target can has (used for initialization only) 

param.PD=0.89; % Detection Probabilty or the average of true positive rate for detections 
param.Beta=0.1/(u_image*v_image); % Beta is False detection (clutter) likelihhod=Pfa/(u_image*v_image)
param.Gate=(50)^0.5; % Gate size for gating
param.S_limit=150; % parameter to stop the gate size growing too much
param.N_H=1; % Number of m-best solutions 
model.multiscale=1; % Number of processing frames

% Parameters for visualization
param.Plott='Yes'; % Make it 'Yes' for any visualization 
param.Box_plot='Yes'; % Make it 'Yes' to show the bounding Box for each target
param.Font=18; % Font size fot the text

%% Tracking Model 
Tracking_Models
load(Detection_address) % load detection
detections_all=detections(1:end);
seg_mark=[0,200,400,600,800,1000];
XeT_all={};
Ff_all={};
occlusion_all={};
for seg=1:numel(seg_mark)-1
    detections_seg=detections_all(seg_mark(seg)+1:seg_mark(seg+1));
    testData_seg=testData(seg_mark(seg)+1:seg_mark(seg+1),:);
    %% Initialization
    detections_seg(1).bx=gtInfo.X(seg_mark(seg)+1,1+6*(seg-1):6*seg);
    detections_seg(1).by=gtInfo.Y(seg_mark(seg)+1,1+6*(seg-1):6*seg);
    detections_seg(1).ht=gtInfo.H(seg_mark(seg)+1,1+6*(seg-1):6*seg);
    detections_seg(1).wd=gtInfo.W(seg_mark(seg)+1,1+6*(seg-1):6*seg);
    detections_seg(1).xi= detections_seg(1).bx+ detections_seg(1).wd/2;
    detections_seg(1).yi=detections_seg(1).by+detections_seg(1).ht/2;
    detections_seg(1).sc=[1,1,1,1,1,1];
    detections_seg(1).labels={'head','tail','head','tail','head','tail'};
    detections_seg(2).bx=gtInfo.X(seg_mark(seg)+2,1+6*(seg-1):6*seg);
    detections_seg(2).by=gtInfo.Y(seg_mark(seg)+2,1+6*(seg-1):6*seg);
    detections_seg(2).ht=gtInfo.H(seg_mark(seg)+2,1+6*(seg-1):6*seg);
    detections_seg(2).wd=gtInfo.W(seg_mark(seg)+2,1+6*(seg-1):6*seg);
    detections_seg(2).xi= detections_seg(2).bx+ detections_seg(2).wd/2;
    detections_seg(2).yi=detections_seg(2).by+detections_seg(2).ht/2;
    detections_seg(2).sc=[1,1,1,1,1,1];
    detections_seg(2).labels={'head','tail','head','tail','head','tail'};
    model.X0=Initialization_mice(detections_seg,param,model); % The initial mean
    model.P0=blkdiag([qm 0;0 1],[qm 0;0 1]); % The initial covariance
    %% BILP algorithm
    [XeT,PeT,Ff,Term_Con]=BILP(detections_seg,model,param,testData_seg);
    
    %% Post-processing (post processing and Removing tracks with small life spans)
    X_size=cellfun(@(x) size(x,2), XeT, 'UniformOutput', false);
    Ff=cellfun(@(x,y,z) x(1):x(1)+y-1-z, Ff,X_size,Term_Con, 'ErrorHandler', @errorfun, ...
        'UniformOutput', false);
    Ff_size=cellfun(@(x) size(x,2), Ff, 'UniformOutput', false);
    XeT=cellfun(@(x,y) x(:,1:y),XeT,Ff_size, 'ErrorHandler', @errorfun, ...
        'UniformOutput', false);
    XeT2=XeT;
    Ff2=Ff;
    Ff(cellfun('size', XeT,2)<param.tret)=[];
    XeT(cellfun('size', XeT,2)<param.tret)=[];
    for n=1:size(XeT,2)
        XeT_all=[XeT_all,XeT{n}];
        Ff_all=[Ff_all,Ff{n}+seg_mark(seg)];
    end
end










