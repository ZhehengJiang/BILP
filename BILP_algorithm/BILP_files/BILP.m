function [XeT,PeT,Ff,Term_Con_2,occlusion]=BILP(detections,model,param,testData)

start_tra=1;
narginchk(3, 5)

hwait = waitbar(0,'Estimated time for Tracking');
elapsedTime=10^6;

Frame = size(detections,2); % Total Number of frames


% Detection parameter
Prun_Thre=param.Prun_Thre;% Parameter for pruning detections with the confidence score less than this value

% Track termination parameter
Term_Frame=param.Term_Frame;% The parameter for termination condition

% Visualization Parameter
Plott=param.Plott; % % Make it 'Yes' for any visualization
Box_plot=param.Box_plot; % Make it 'Yes' to show the bounding Box for each target
Font=param.Font;

% Motion Models
F=model.F;% The transition matrix for the dynamic model
Q=model.Q;% The process covariance matrix for the dynamic model
H=model.H;% Measurement matrix
R=model.R;% Measurement covariance matrix

% The initial state's distribution models
X0=model.X0; % The initial mean
P0=model.P0; % The initial covariance


%BILP Parameters
PD=param.PD; % Detection Probabilty
Beta=param.Beta;% False detection (clutter) likelihhod
Gate=param.Gate;% Gate size for gating
S_limit=param.S_limit; % parameter to stop the gate size growing too much
msp=model.multiscale; % Time-Frame windows
mbest=param.N_H; % Threshold for considering m-best number of Hypotheses

% IMM model (Constant or Sate-Dependent) 
mu0i=model.mui0; % The initial values for the IMM probability weights
TPM_Option=model.TPM_Option; % TPM_Option='Constant' or 'State-Dependent';
TPM=model.TPM; % Markov Transition probability matrix in the case of constant;
H_TPM=model.H_TPM;


Kt=size(F,3);%The Number of Dynamic Models
if strcmp(TPM_Option,'State-Dependent')
    if Kt~=2
        error('The state-dependent transion probabilty matrix (TPM) is only designed for two models; if you have more or less than two models, use constant option')
    end
    warndlg('please note that p(Kt = 1 |Kt-1,.) = 1 - S^k * exp(.) and p(Kt = 2 |Kt-1,.) = S^k*exp(.) necessarily')
end

DMV=size(H,1);DSV=size(H,2);

N_Target=size(X0,1);
%*********************** Initial IMM Parameters ************************

%Initial State Vector & IMM Initial Parameters
Xe=cell(Kt,N_Target);
XeT=cell(1,N_Target);
occlusion=cell(1,N_Target);
Pe=cell(Kt,N_Target);
PeT=cell(1,N_Target);
BxT=cell(1,N_Target);
target_label=cell(1,N_Target);

mui=cell(1,N_Target);muij=cell(1,N_Target);
PT=cell(1,N_Target);
C=cell(1,N_Target);
Lambda=cell(1,N_Target);

Ff=cell(1,N_Target);
Term_Con=cell(1,N_Target);
Term_Con_2=cell(1,N_Target);
for ij=1:N_Target
    occlusion{1,ij}=0;
    if strcmp(TPM_Option,'Constant')
        PT{1,ij}=TPM;
    elseif strcmp(TPM_Option,'State-Dependent') %It is not generalized
        P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*P0*H_TPM'))^0.5...
            *exp(-0.5*(X0(ij,:)*H_TPM')*((TPM{2,1}+H_TPM*P0*H_TPM')^(-1))*(X0(ij,:)*H_TPM')');
        P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*P0*H_TPM'))^0.5...
            *exp(-0.5*(X0(ij,:)*H_TPM')*((TPM{1,1}+H_TPM*P0*H_TPM')^(-1))*(X0(ij,:)*H_TPM')');
        PT{1,ij}=[1-P12 P12;1-P22 P22];
    end
    mui{1,ij}=mu0i;
    Ff{1,ij}=[1 1];
    Term_Con{1,ij}=0;
    Term_Con_2{1,ij}=0;
    for r=1:Kt
        Xe{r,ij}=X0(ij,:)';
        if r==1
            XeT{1,ij}=mui{1,ij}(r)*Xe{r,ij};
        else
            XeT{1,ij}=XeT{1,ij}+mui{1,ij}(r)*Xe{r,ij};
        end
    end
    for r=1:Kt
        Pe{r,ij}=P0;
        if r==1
            PeT{1,ij}=mui{1,ij}(r)*(Pe{r,ij}+(XeT{1,ij}-Xe{r,ij})*(XeT{1,ij}-Xe{r,ij})');
        else
            PeT{1,ij}=PeT{1,ij}+mui{1,ij}(r)*(Pe{r,ij}+(XeT{1,ij}-Xe{r,ij})*(XeT{1,ij}-Xe{r,ij})');
        end
    end
    
end


Terminated_objects_index=[];
waitbar((1)/(Frame),hwait,['Frame # ',num2str(1),'   Estimated time: ',num2str((Frame-1)*elapsedTime),' s'])

%************ loading Image and detections for first frame ****************
f=1; % first frame
XYZ=cell(1,Frame);
XYZ{1,f}=[detections(f).xi(detections(f).sc>Prun_Thre);detections(f).yi(detections(f).sc>Prun_Thre)]';

if strcmp(Plott,'Yes')
    I = imread(testData.imageFilename{f});
    if strcmp(Box_plot,'Yes')
        X_D2=detections(f).bx(detections(f).sc>Prun_Thre);
        Y_D2=detections(f).by(detections(f).sc>Prun_Thre);
        W_D2=detections(f).wd(detections(f).sc>Prun_Thre);
        H_D2=detections(f).ht(detections(f).sc>Prun_Thre);
        target_label=detections(f).labels(detections(f).sc>Prun_Thre);
        WH=cell(1,Frame);
        WH{1,f}=[X_D2;Y_D2;W_D2;H_D2];
        for no=1:N_Target
            BxT{1,no}=[X_D2(no);Y_D2(no);W_D2(no);H_D2(no)];
        end
    end
end

%****************** Visualization for first frame *************************
if strcmp(Plott,'Yes')
    rng(10^4)
    %colorord=.25+.75*rand(10^4,3);
    colorord=0.4+0.6*rand(10,3);
    figure,imshow(I,'border','tight','initialmagnification','fit')
    if strcmp(Box_plot,'Yes')
        for no=1:N_Target
            Xx=X_D2(no);
            Yy=Y_D2(no);
            Ww=W_D2(no);
            Hh=H_D2(no);
            hold on
            rectangle('Position',[Xx,Yy,Ww,Hh],'EdgeColor',colorord(no,:))
            text(Xx+Ww/2,Yy+Hh/3,target_label(no),'Color',colorord(no,:),'FontSize',Font)
            drawnow
            hold off
        end
    end
end

detections_filter=detections;

for f=2:Frame
    ticID = tic;
    % Load detections
%     index_tail=find(strcmp(detections(f).labels,'tail'));
%     index_head=find(strcmp(detections(f).labels,'head'));
%     tail_bboxes=[detections(f).bx(index_tail)' detections(f).by(index_tail)' detections(f).wd(index_tail)' detections(f).ht(index_tail)'];
%     tail_scores=[detections(f).sc(index_tail)'];
%     head_bboxes=[detections(f).bx(index_head)' detections(f).by(index_head)' detections(f).wd(index_head)' detections(f).ht(index_head)'];
%     head_scores=[detections(f).sc(index_head)'];
%     index_filter=[];
%     for ii=1:size(tail_scores,1)
%         for iii=1:size(head_scores,1)
%             iou=bboxOverlapRatio(tail_bboxes(ii,:),head_bboxes(iii,:));
%             if iou>0.8
%                 if tail_scores(ii,:)>head_scores(iii,:)
%                     index_filter=[index_filter index_head(iii)];
%                 else
%                     index_filter=[index_filter index_tail(ii)];                                       
%                 end
%             end
%         end
%     end
%     detections_filter(f).xi(index_filter)=[];
%     detections_filter(f).yi(index_filter)=[];
%     detections_filter(f).sc(index_filter)=[];
%     detections_filter(f).bx(index_filter)=[];
%     detections_filter(f).by(index_filter)=[];
%     detections_filter(f).wd(index_filter)=[];
%     detections_filter(f).ht(index_filter)=[];
    XYZ{1,f}=[detections_filter(f).xi(detections_filter(f).sc>Prun_Thre);detections_filter(f).yi(detections_filter(f).sc>Prun_Thre)]';
    ob_labels=detections_filter(f).labels(detections_filter(f).sc>Prun_Thre);
    %  Plot detections and load image if necessary
    if strcmp(Plott,'Yes')
        I = imread(testData.imageFilename{f});
        imshow(I)
        if strcmp(Box_plot,'Yes')
            X_D2=detections_filter(f).bx(detections_filter(f).sc>Prun_Thre);
            Y_D2=detections_filter(f).by(detections_filter(f).sc>Prun_Thre);
            W_D2=detections_filter(f).wd(detections_filter(f).sc>Prun_Thre);
            H_D2=detections_filter(f).ht(detections_filter(f).sc>Prun_Thre);
            WH{1,f}=[X_D2;Y_D2;W_D2;H_D2];
        end
        
    end
    
    %Motion models
    N_Target=size(Xe,2);
    Mes_Tar=false(size(XYZ{1,f},1),N_Target,Kt);
    MXe=zeros(DSV,Kt,N_Target);PXe=zeros(DSV,DSV,Kt,N_Target);
    S=zeros(DMV,DMV,Kt,N_Target);K=zeros(DSV,DMV,Kt,N_Target);
    Target_Obs_indx=cell(N_Target,Kt);
    Rt_In_Pr=cell(N_Target,Kt);
    Target_Obs_indx_Total=cell(N_Target,1);
    Target_probabilty=cell(N_Target,Kt);
    Target_probabilty_Total=cell(N_Target,1);
    
    Curr_Obs=[];
    %************* Prediction Step **************
    
    for no=1:N_Target
        if ~ismember(no,Terminated_objects_index)
            k=f-Ff{1,no}(1,1)+1;
            C{1,no}(:,k-1)=mui{1,no}(k-1,:)*PT{1,no};
            X0j=zeros(DSV,Kt);
            PI0j=zeros(DSV,DSV,Kt);
            for r2=1:Kt
                % IMM prediction
                for r1=1:Kt
                    muij{1,no}(r1,r2,k-1)= PT{1,no}(r1,r2)*mui{1,no}(k-1,r1)/ C{1,no}(r2,k-1);
                    X0j(:,r2)=X0j(:,r2)+muij{1,no}(r1,r2,k-1)*Xe{r1,no}(:,k-1);
                end
                
                for r1=1:Kt
                    PI0j(:,:,r2)=PI0j(:,:,r2)+muij{1,no}(r1,r2,k-1)...
                        *(Pe{r1,no}(:,:,k-1)+(Xe{r1,no}(:,k-1)-X0j(:,r2))...
                        *(Xe{r1,no}(:,k-1)-X0j(:,r2))');
                end
                
                % Motion Preditction Step& Hypothesis Tree Reconstruction Step
                [Target_Obs_indx{no,r2},Target_probabilty{no,r2},MXe(:,r2,no),...
                    PXe(:,:,r2,no),S(:,:,r2,no),K(:,:,r2,no),Rt_In_Pr{no,r2}]=...
                    Tree_Constructor(X0j(:,r2),PI0j(:,:,r2),F(:,:,r2),...
                    Q(:,:,r2),H,R,XYZ(1,f:min(f+msp-1,Frame)),S_limit,Gate,PD,Beta,DMV);

                Mes_Tar(Target_Obs_indx{no,r2}{1}{1},no,r2)=true;
                Target_Obs_indx_Total{no,1}=[Target_Obs_indx_Total{no,1};...
                    Target_Obs_indx{no,r2}{1}{1}(~ismember(Target_Obs_indx{no,r2}{1}{1}, ...
                    Target_Obs_indx_Total{no,1}))];
                Lambda{1,no}(r2,k)=sum(Target_probabilty{no,r2}{1}{1});
                
            end
        end
    end
    %*************** Joint Probabilistic Data Association *****************
    exist_ind=(~ismember(1:N_Target,Terminated_objects_index));
    Final_probabilty=cell(1,Kt);
    Mes_Tar2=Mes_Tar(:,exist_ind,:);[Umt, Vmt, Zmt]=size(Mes_Tar2);
    Mes_Tar=false(Vmt+Umt,Vmt+Umt,Zmt);
    for r=1:Kt
        Final_probabilty{1,r}=cell(1,N_Target);
        Mes_Tar(:,:,r)=[false(Vmt,Vmt+Umt);Mes_Tar2(:,:,r) false(Umt,Umt)];
        Final_probabilty{1,r}(1,exist_ind) =Approx_Multiscan_BILP_Probabilities(Mes_Tar(:,:,r),Rt_In_Pr(exist_ind,r),mbest);
        
    end
    for no=1:N_Target
        ob_l=ob_labels(Target_Obs_indx_Total{no,1});
        for i=1:size(ob_l,2)
            if ~strcmp(target_label{no},ob_l{i})
                Final_probabilty{1,1}{1,no}(i+1)=Final_probabilty{1,1}{1,no}(i+1)/1000;
            end
            for ii=1:size( Final_probabilty{1,1},2)
                Final_probabilty{1,1}(ii)=cellfun(@(x) x/sum(x), Final_probabilty{1,1}(ii), 'UniformOutput', false);
            end
        end
    end

    %**************************** Update step *****************************
    for no=1:N_Target
        if ~ismember(no,Terminated_objects_index)
            k=f-Ff{1,no}(1,1)+1;
            Pij=cell(Kt,1);
            XeT{1,no}(:,k)=zeros(DSV,1);
            PeT{1,no}(:,:,k)=zeros(DSV,DSV);
            CTotal= Lambda{1,no}(:,k)'*C{1,no}(:,k-1);
            PIJT=zeros(1,size(Target_Obs_indx_Total{no,1},1)+1);
            for r=1:Kt
                NN=length(Target_Obs_indx{no,r}{1}{1})+1;
                Pij{r,1}=(Final_probabilty{1,r}{1,no})';
                [~, indM]=max(Pij{r,1});
                if isempty(Target_Obs_indx{no,r}{1}{1})
                    Xe{r,no}(:,k)=MXe(:,r,no);
                    dP=0;
                else
                    Yij=XYZ{1,f}(Target_Obs_indx{no,r}{1}{1},1:DMV)-repmat((H*MXe(:,r,no))',[size(Target_Obs_indx{no,r}{1}{1}),1]);
                    Ye=(Pij{r,1}(2:NN)*Yij)';
                    Xe{r,no}(:,k)=MXe(:,r,no)+K(:,:,r,no)*Ye;
                    dP=K(:,:,r,no)*(repmat(Pij{r,1}(2:NN),[DMV 1]).*Yij'*Yij-(Ye*Ye'))*K(:,:,r,no)';
                    if rem(no,2)==0
                        dis_back=pdist([Xe{r,no}([1,3],k)';Xe{r,no-1}([1,3],k-1)']);
                        dis_before=pdist([Xe{r,no}([1,3],k-1)';Xe{r,no-1}([1,3],k-1)']);
                    else
                        dis_back=pdist([Xe{r,no}([1,3],k)';Xe{r,no+1}([1,3],k-1)']);
                        dis_before=pdist([Xe{r,no}([1,3],k-1)';Xe{r,no+1}([1,3],k-1)']);
                    end
                    
                    if rem(no,2)==0
                        if Term_Con{1,no-1}==0
                            if dis_back/dis_before<0.7||dis_back/dis_before>1.7
                                Xe{r,no}(:,k)=MXe(:,r,no);
                                Xe{r,no}([2,4],k)=0;
                                dP=0;
                            end
                        end
                        if  Term_Con{1,no}>2
                            Xe{r,no}([2,4],k)=0;
                        end
                    else
                        if Term_Con{1,no+1}==0
                            if dis_back/dis_before<0.7||dis_back/dis_before>1.9
                                Xe{r,no}(:,k)=MXe(:,r,no);
                                Xe{r,no}([2,4],k)=0;
                                dP=0;
                            end
                        end
                        if  Term_Con{1,no}>2
                            Xe{r,no}([2,4],k)=0;
                        end
                    end
                end
                                
                Pst=PXe(:,:,r,no)-K(:,:,r,no)*S(:,:,r,no)*K(:,:,r,no)';
                Po=Pij{r,1}(1)*(PXe(:,:,r,no))+(1-Pij{r,1}(1))*Pst;
                Pe{r,no}(:,:,k)=Po+dP;
                
                mui{1,no}(k,r)=Lambda{1,no}(r,k)*C{1,no}(r,k-1)/CTotal;
                PIJT([true,(ismember(Target_Obs_indx_Total{no,1},Target_Obs_indx{no,r}{1}{1}))'])=...
                    PIJT([true,(ismember(Target_Obs_indx_Total{no,1},Target_Obs_indx{no,r}{1}{1}))'])+...
                    mui{1,no}(k,r)*Pij{r,1};
                XeT{1,no}(:,k)=XeT{1,no}(:,k)+mui{1,no}(k,r)*Xe{r,no}(:,k);
            end
            Target_probabilty_Total{no,1}=PIJT;
            for r=1:Kt
                PeT{1,no}(:,:,k)= PeT{1,no}(:,:,k)+mui{1,no}(k,r)*(Pe{r,no}(:,:,k)+...
                    (XeT{1,no}(:,k)-Xe{r,no}(:,k))*(XeT{1,no}(:,k)-Xe{r,no}(:,k))');
            end
            
            if strcmp(TPM_Option,'State-Dependent') %It is not generalized
                P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*Pe{1,no}(:,:,k)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{1,no}(:,k))'*((TPM{2,1}+H_TPM*Pe{1,no}(:,:,k)*H_TPM')^(-1))*H_TPM*Xe{1,no}(:,k));
                P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*Pe{2,no}(:,:,k)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{2,no}(:,k))'*((TPM{1,1}+H_TPM*Pe{2,no}(:,:,k)*H_TPM')^(-1))*H_TPM*Xe{2,no}(:,k));
                PT{1,no}=[1-P12 P12;1-P22 P22];
            end
            
            %********************* Initiation & Termination  ***********************
            [~, indM]=max(PIJT);
            if indM==1
                Term_Con{1,no}=Term_Con{1,no}+1;
%                 Term_Con{1,no}=0;
                if (strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes'))

                    Xx=XeT{1,no}(1,k)-BxT{1,no}(3,k-1)/2;
                    Yy=XeT{1,no}(3,k)-BxT{1,no}(4,k-1)/2;
                    Ww=BxT{1,no}(3,k-1);
                    Hh=BxT{1,no}(4,k-1);
                    BxT{1,no}(:,k)=[Xx;Yy;Ww;Hh];
                    if f>=start_tra
                        hold on
                        plot(XeT{1,no}(1,start_tra:end),XeT{1,no}(3,start_tra:end),'Color',colorord(no,:),'LineWidth',2);
                    end
                    if (strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes')&&Term_Con{1,no}<=100)
                        ob_l=ob_labels(Target_Obs_indx_Total{no,1});
                        
                        occlusion{1,no}=[occlusion{1,no} 0];
                        hold on
                         rectangle('Position',[Xx,Yy,Ww,Hh],'EdgeColor',colorord(no,:),'LineWidth',3)
                         text(Xx+Ww/2,Yy+Hh*3/4,num2str(no),'Color','b','FontSize',Font,'rotation',90,'FontWeight','bold','BackgroundColor',[0.85 0.85 0.85],'Margin',0.001)
                        hold off
                    else
                        occlusion{1,no}=[occlusion{1,no}(1:end-2) 1 1];
                        occlusion{1,no}=[occlusion{1,no} 1];
                    end
                end
            else
                occlusion{1,no}=[occlusion{1,no} 0];
                Term_Con{1,no}=0;
                if (strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes'))
                    meas_inx=Target_Obs_indx_Total{no,1};
                    Ww=PIJT(2:end)*W_D2(meas_inx)'/sum(PIJT(2:end));
                    Hh=PIJT(2:end)*H_D2(meas_inx)'/sum(PIJT(2:end));
                    Xx=XeT{1,no}(1,k)-Ww/2;
                    Yy=XeT{1,no}(3,k)-Hh/2;
                    BxT{1,no}(:,k)=[Xx;Yy;Ww;Hh];
                    if (strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes'))
                        ob_l=ob_labels(Target_Obs_indx_Total{no,1});
                        hold on
                         rectangle('Position',[Xx,Yy,Ww,Hh],'EdgeColor',colorord(no,:),'LineWidth',3)
                         text(Xx+Ww/2,Yy+Hh*3/4,num2str(no),'Color','b','FontSize',Font,'rotation',90,'FontWeight','bold','BackgroundColor',[0.85 0.85 0.85],'Margin',0.001)
                        if f>=start_tra
                        plot(XeT{1,no}(1,start_tra:end),XeT{1,no}(3,start_tra:end),'Color',colorord(no,:),'LineWidth',2);
                        end
                        hold off
                    end
                end
            end
            if ~isempty(Target_Obs_indx_Total{no,1})
                Curr_Obs =[Curr_Obs;Target_Obs_indx_Total{no,1}];   %#ok<AGROW>
            end
            
            if  (Term_Con{1,no}<=Term_Frame&&size(XeT{1,no},2)>1)||...
                    (Term_Con{1,no}>=0&&size(XeT{1,no},2)<=1)%Termination Condition
                Ff{1,no}(1,2)=f;
            else
                Terminated_objects_index=[Terminated_objects_index no]; %#ok<AGROW>
            end
            
        end
    end
    
    labels=detections_filter(f).labels(detections_filter(f).sc>Prun_Thre);
    if f>=33
        stop=1;
    end
    %%% we do not use this code here
    All_Obs=1:size(XYZ{1,f},1);
    New_Targets=All_Obs(~ismember(All_Obs,Curr_Obs));%Initiation Condition
    New_Targets=[];
    if ~isempty(New_Targets)
        for ij=1:length(New_Targets)
            target_label=detections_filter(f).labels(detections_filter(f).sc>Prun_Thre);
            mui{1,N_Target+ij}=mu0i;
            XeT{1,N_Target+ij}=zeros(DSV,1);
            PeT{1,N_Target+ij}=zeros(DSV,DSV);
            for r=1:Kt
                Xe{r,N_Target+ij}=H'*XYZ{1,f}(New_Targets(ij),:)';
                Pe{r,N_Target+ij}=P0;                
                
                XeT{1,N_Target+ij}=XeT{1,N_Target+ij}+mui{1,N_Target+ij}(1,r)*Xe{r,N_Target+ij};
            end
            for r=1:Kt
                PeT{1,N_Target+ij}= PeT{1,N_Target+ij}+mui{1,N_Target+ij}(1,r)*...
                    (Pe{r,N_Target+ij}+(XeT{1,N_Target+ij}-Xe{r,N_Target+ij})...
                    *(XeT{1,N_Target+ij}-Xe{r,N_Target+ij})');
                
            end
            
            if strcmp(TPM_Option,'Constant')
                PT{1,N_Target+ij}=TPM;
            elseif strcmp(TPM_Option,'State-Dependent') %It is not genralized
                P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*Pe{1,N_Target+ij}(:,:,1)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{1,N_Target+ij}(:,1))'*((TPM{2,1}+H_TPM*Pe{1,N_Target+ij}(:,:,1)*H_TPM')^(-1))*H_TPM*Xe{1,N_Target+ij}(:,1));
                P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*Pe{2,N_Target+ij}(:,:,1)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{2,N_Target+ij}(:,1))'*((TPM{1,1}+H_TPM*Pe{2,N_Target+ij}(:,:,1)*H_TPM')^(-1))*H_TPM*Xe{2,N_Target+ij}(:,1));
                PT{1,N_Target+ij}=[1-P12 P12;1-P22 P22];
            end
            
            Ff{1,N_Target+ij}=[f f];
            Term_Con{1,N_Target+ij}=0;
            
            %***************** Visualization for new targets *************************
            
            if (strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes'))
                BxT{1,N_Target+ij}=[X_D2(New_Targets(ij));Y_D2(New_Targets(ij));W_D2(New_Targets(ij));H_D2(New_Targets(ij))];
                
            end
            
            %****************** Visualization for first frame *************************
            if strcmp(Plott,'Yes')&&strcmp(Box_plot,'Yes')
                Xx=X_D2(New_Targets(ij));
                Yy=Y_D2(New_Targets(ij));
                Ww=W_D2(New_Targets(ij));
                Hh=H_D2(New_Targets(ij));
                hold on
                rectangle('Position',[Xx,Yy,Ww,Hh],'EdgeColor',colorord(N_Target+ij,:))
                text(Xx+Ww/2,Yy+Hh/3,num2str(N_Target+ij),'Color',colorord(N_Target+ij,:),'FontSize',Font)
                hold off
            end
            
            
        end
    end
    if strcmp(Plott,'Yes')
        drawnow
    end
    elapsedTime = toc(ticID);
    waitbar((f)/(Frame),hwait,['Frame # ',num2str(f),'   Estimated time: ',num2str(round((Frame-f)*elapsedTime)),' s (',...
        num2str(round(elapsedTime*100)/100),' Sec/Frame)'])
end
Term_Con=cell(1,N_Target);
close(hwait)

