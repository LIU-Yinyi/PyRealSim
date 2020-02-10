function dat = sim_recv(cmd, id)
    %% @author: LIU-Yinyi
    if nargin == 1 && cmd == "help"
        show_help();
    end
    
    if evalin("base", "exist('sockfd_recver', 'var')") == 0
        disp('[sim] WARN: you might not have initialized.');
        return
    end
    if evalin("base", "exist('sockfd_imager', 'var')") == 0
        disp('[sim] WARN: you might not have initialized.');
        return
    end
    
    sockfd_recver = evalin('base', 'sockfd_recver');
    sockfd_imager = evalin('base', 'sockfd_imager');
    
    if cmd == "state"
        sim_send("get_state", id)
        dat = sockfd_recver();
        dat = typecast(dat, 'double');
    elseif cmd == "rgb"
        sim_send("get_rgb", id)
        dat = read(sockfd_imager);
        if size(dat, 2) == 307200
            dat = reshape(dat, 4, 320, 240);
            dat = permute(dat, [3, 2, 1]);
            dat = dat(:,:,3:-1:1);
        end
    elseif cmd == "depth"
        sim_send("get_depth", id)
        dat = read(sockfd_imager);
        dat = typecast(dat, 'single');
        if size(dat, 2) == 76800
            dat = reshape(dat, 320, 240);
            dat = dat.';
        end
    end
    
end

function show_help()
    disp("----- Help Desk -----");
    disp("dat = sim_recv(cmd, id)")
    disp("@note: required previous call from sim_init");
    disp("@param cmd: (1)state (2)rgb (3)depth");
    disp("@param id: target uav id (-1 means all)");
    disp("@example: dat = sim_recv('rgb', 0);")
    disp("   which means: receive image data of RGB image from UAV-0");
end