function result = registerOppositeSidesSE2(moving, fixed)
%REGISTEROPPOSITESIDESSE2 MATLAB-form equivalent of the Python cross-side solve.
% Inputs are Nx3 points ALREADY gravity-levelled and roughly RTK-positioned.
% Frame here: X=across row, Y=along row, Z=up; metres, yaw in radians.
% Moving is Side B, fixed is Side A. A low-stem subset is used for fitting;
% the returned 4x4 column-vector transform applies to the complete scan.
% Requires Statistics and Machine Learning Toolbox (knnsearch).
% This is an explanatory numerical port, not runtime-tested in MATLAB here.
arguments
    moving (:,3) double
    fixed (:,3) double
end
M = moving(moving(:,3)>.13 & moving(:,3)<.38 & ...
    moving(:,2)>-1 & moving(:,2)<8 & ...
    moving(:,1)>-.8125 & moving(:,1)<.7875, 1:2);
F = fixed(fixed(:,3)>.13 & fixed(:,3)<.38 & ...
    fixed(:,2)>-1 & fixed(:,2)<8 & ...
    fixed(:,1)>-.8125 & fixed(:,1)<.7875, 1:2);
assert(size(M,1)>=50 && size(F,1)>=50,'Not enough overlapping low-stem points.');
% Use pcdownsample(...,'gridAverage',.025) before calling for a closer
% sampling match to the Python implementation; retain originals for export.
bestScore = Inf; bestT = eye(3);
for dAlong = -.8:.2:1.2
    for dAcross = [.5 .25 0] % sign-flipped Python across-row initial seeds
        T = [eye(2), [dAcross;dAlong]; 0 0 1];
        for threshold = [.25 .13 .07]
            for iteration = 1:35
                Q = M*T(1:2,1:2).' + T(1:2,3).';
                [idx,d] = knnsearch(F,Q);
                keep = d<threshold;
                if nnz(keep)<50, break; end
                A=Q(keep,:); B=F(idx(keep),:);
                w=1./(1+(d(keep)/.045).^2); w=w/sum(w);
                ca=sum(A.*w,1); cb=sum(B.*w,1);
                H=(A-ca).' * ((B-cb).*w);
                [U,~,V]=svd(H); R=V*U.';
                if det(R)<0, V(:,end)=-V(:,end); R=V*U.'; end
                shift=cb.'-R*ca.';
                E=[R shift;0 0 1]; T=E*T;
                if norm(E-eye(3),'fro')<1e-5, break; end
            end
        end
        Q=M*T(1:2,1:2).'+T(1:2,3).';
        [~,dm]=knnsearch(F,Q); [~,df]=knnsearch(Q,F);
        score=.5*(mean(min(dm,.12).^2)+mean(min(df,.12).^2));
        if score<bestScore, bestScore=score; bestT=T; end
    end
end
T4=eye(4); T4(1:2,1:2)=bestT(1:2,1:2); T4(1:2,4)=bestT(1:2,3);
result=struct('T',T4,'rotation',T4(1:3,1:3), ...
    'translation',T4(1:3,4).','objective',bestScore, ...
    'dAcrossMeters',T4(1,4),'dAlongMeters',T4(2,4), ...
    'yawRadians',atan2(T4(2,1),T4(1,1)));
% Example: aligned = pctransform(pointCloud(moving), ...
%     rigidtform3d(result.rotation,result.translation));
end
