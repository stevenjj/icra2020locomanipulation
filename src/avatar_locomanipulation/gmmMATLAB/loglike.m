function [llh] = loglike(alphs,mus,Sigs)
global d K N data
llh=0;
    for i = 1:N
        lh=0.0;
        for k = 1:K
            lh = lh+alphs(k)*GaussianCalc(data(:,i),mus(:,:,k),Sigs(:,:,k));
        end
        llh = llh+log(lh);
    end
end