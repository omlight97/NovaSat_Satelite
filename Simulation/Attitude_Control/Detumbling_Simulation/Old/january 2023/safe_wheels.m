function [safe_momentum] = safe_wheels(controller_momentum)
    
    safe_momentum = 0.04; % peak allowed momentum of a reaction wheel

    if controller_momentum > safe_momentum
        safe_momentum = controller_momentum; % we can't use more momentum then what we're allowed
    else
        return
    end
end