function [alphs,mus,Sigs] = Mstep(gam)
global d K N data
n = ones(K,1);

%% Alpha Calc
for k=1:K
    n(k) = sum(gam(:,k));
end
% n = n(:,1);
alphs = n./N;

%% Mu Calc
for k=1:K
    mu_sum = 0.0;
    for i = 1:N
        mu_sum = mu_sum+gam(i,k)*data(:,i);
    end
    mus(:,:,k) = mu_sum./n(k);
end

%% Sigma Calc
for k=1:K
    Sig_sum = 0.0;
    for i = 1:N
    Sig_sum = Sig_sum + gam(i,k) * (data(:,i)-mus(:,:,k))*(data(:,i)-mus(:,:,k))';
    end
    Sigs(:,:,k) = Sig_sum./n(k);
end
end