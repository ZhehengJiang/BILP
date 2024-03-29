function Final_probabilty=Approx_Multiscan_BILP_Probabilities(M,Obj_info,mbest)

U=size(Obj_info,1);
Final_probabilty=cell(1,U);
M2=sparse(M);
[~,C]=graphconncomp(M2,'Directed','false');
C2=C(1:U);
NR=cell2mat(cellfun(@(x) size(x.Prob,1),Obj_info,'UniformOutput', false));

for i=unique(C2)
    ix=(C2==i);
    NR_C=NR(ix);
    if size(NR_C,1)==1
        TNH=NR_C;
    else
        TNH=prod(NR_C);
    end
    if TNH<10000||isinf(mbest)
        Final_probabilty(ix)=BILP_Probabilty_Calculator(Obj_info(ix));
    else
        
        Final_probabilty(ix)=MBest_BILP_Probabilty_Calculator(Obj_info(ix),mbest);
    end
    
end


