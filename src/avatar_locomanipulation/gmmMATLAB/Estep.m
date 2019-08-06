function [gam] = Estep(alphs,mus,Sigs)
global d K N data
for i = 1:N
    den = 0;
    for k = 1:K 
        den = den+alphs(k)*GaussianCalc(data(:,i),mus(:,:,k),Sigs(:,:,k));
    end
    for k = 1:K
        gam(i,k) = alphs(k)*GaussianCalc(data(:,i),mus(:,:,k),Sigs(:,:,k))/den;
    end
end
end
