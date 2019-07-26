function  [P] = GaussianCalc(x,mu,Sig)
global d K N data
    if isempty(d)
    d = length(x);
    end
    P = exp(-0.5 * (x-mu)' * (pinv(Sig, 1e-9)*(x-mu)))*((2*pi())^(d/2)*det(Sig)^(.5))^(-1);
end 