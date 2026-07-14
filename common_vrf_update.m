function [state,beta,lambda,temp] = common_vrf_update(state, prediction_error, p)
%COMMON_VRF_UPDATE One VRF factor from the stacked 12-state prediction error.

if isempty(state)
    if p.rls.tau_n > p.rls.tau_d
        error('common_vrf_update:Windows','tau_n must not exceed tau_d.');
    end
    state.z = zeros(12,p.rls.tau_d);
    state.last_ratio = 0;
    state.last_temp = 0;
    beta = 1;
    lambda = 1;
    temp = 0;
    return
end

if strcmpi(p.rls.type,'VRF')
    state.z = circshift(state.z,1,2);
    state.z(:,1) = prediction_error(:);

    numerator = sqrt(sum(sum(state.z(:,1:p.rls.tau_n).^2))/p.rls.tau_n);
    denominator = sqrt(sum(sum(state.z.^2))/p.rls.tau_d);

    if denominator == 0
        ratio = 0;
    else
        ratio = numerator/denominator;
    end

    if ratio < 1
        temp = 0;
    else
        temp = ratio - 1;
    end

    beta = 1 + p.rls.eta*temp;
    beta = min(beta,p.rls.beta_max);
elseif strcmpi(p.rls.type,'CRF')
    beta = 1/p.rls.lambda;
    ratio = 0;
    temp = 0;
else
    error('common_vrf_update:Type','RLS type must be VRF or CRF.');
end

lambda = 1/beta;
state.last_ratio = ratio;
state.last_temp = temp;
end
