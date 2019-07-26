function [alphs,mus,Sigs, gam, llh] = GMMEM
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global d K N data
error = 1000.0;
tol = 1e-9;
mus = rand(d,1,K);
alphs = 1/K*ones(K,1);
for k = 1:K
Sigs(:,:,k) = eye(d,d);
end

llh = loglike(alphs,mus,Sigs);
while abs(error)>tol
    llh_prev = llh;
    gam = Estep(alphs,mus,Sigs);
    [alphs,mus,Sigs] = Mstep(gam);
    llh = loglike(alphs,mus,Sigs);
    error = llh-llh_prev;
end
end
