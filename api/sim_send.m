function sim_send(cmd, id, dat, is_feedback)
    %% @author: LIU-Yinyi
    if nargin == 1 && cmd == "help"
        show_help();
    end
    
    if evalin("base", "exist('sockfd_sender', 'var')") == 0
        disp('[sim] WARN: you might not have initialized.');
        return
    end
    
    sockfd_sender = evalin('base', 'sockfd_sender');
    
    if nargin == 1 && cmd == "update"
        id = -1;
        dat = zeros(1, 4);
    elseif nargin == 2 && cmd ~= "control"
        dat = zeros(1, 4);
    elseif nargin < 3
        show_help;
        return
    end
    
    if nargin < 4
        is_feedback = false;
    end

    if cmd == "update"
        pack = [1, id, dat];
        sockfd_sender(double(pack));
        if is_feedback
            disp('[sim] sent UPDATE command');
        end
    elseif cmd == "control"
        pack = [2, id, dat];
        if size(pack, 2) == 6
            sockfd_sender(double(pack))
        end
        if is_feedback
            printf('[sim] sent CONTROL command: <id = %.0f, x = %.2f, y = %.2f, z = %.2f, yaw = %.2f>\n',...
                pack(2), pack(3), pack(4), pack(5), pack(6));
        end
    elseif cmd == "get_state"
        pack = [10, id, dat];
        sockfd_sender(double(pack));
    elseif cmd == "get_rgb"
        pack = [11, id, dat];
        sockfd_sender(double(pack));
    elseif cmd == "get_depth"
        pack = [12, id, dat];
        sockfd_sender(double(pack));
    else
        show_help;
    end
end

function show_help()
    disp("----- Help Desk -----");
    disp("sim_send(cmd, id, dat)")
    disp("@note: required previous call from sim_init");
    disp("@param cmd: (1)update (2)control");
    disp("@param id: target uav id (-1 means all)");
    disp("@param dat: [x, y, z, yaw]");
    disp("@example: sim_send('control', 0, [1, 0, 1, 0]);")
    disp("   which means: id = 0, set x = 1, y = 0, z = 1, yaw = 0 (Radian)");
end
