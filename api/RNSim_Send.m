function RNSim_Send(sockfd, cmd, dat, is_feedback)
    %% @author: LIU-Yinyi
    if nargin == 2 && cmd == "update"
        dat = zeros(1, 5);
    elseif nargin < 3
        show_help;
        return
    end
    
    if nargin < 4
        is_feedback = false;
    end

    if cmd == "update"
        sockfd([1, 0, 0, 0, 0, 0]);
        if is_feedback
            disp('[RNSim] Sent UPDATE Command');
        end
    elseif cmd == "control"
        pack = [2, dat];
        if size(pack, 2) == 6
            sockfd(pack)
        end
        if is_feedback
            printf('[RNSim] Sent CONTROL Command: <id = %.0f, x = %.2f, y = %.2f, z = %.2f, yaw = %.2f>\n',...
                pack(1), pack(2), pack(3), pack(4), pack(5));
        end
    else
        show_help;
    end
end

function show_help()
    disp("----- Help Desk -----");
    disp("RNSim_Send(sockfd, cmd, dat)")
    disp("@param sockfd: required from return value of RNSim_Init");
    disp("@param cmd: (1)update (2)control");
    disp("@param dat: [id, x, y, z, yaw]");
    disp("@example: RNSim_Send(sockfd, 'control', [0, 1, 0, 1, 0]);")
    disp("   which means: id = 0, set x = 1, y = 0, z = 1, yaw = 0 (Radian)");
end
