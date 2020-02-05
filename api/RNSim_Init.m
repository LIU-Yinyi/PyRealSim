function sockfd = RNSim_Init()
    %% @author: LIU-Yinyi
    sockfd = dsp.UDPSender('RemoteIPAddress', '127.0.0.1', 'RemoteIPPort', 6666);
    show_help();
end

function show_help()
    disp("----- Help Desk -----");
    disp("@example: sockfd = RNSim_Init()");
    disp("@note: sockfd for RNSim_Send(sockfd, cmd, dat)");
    disp("@note: use clear sockfd to release udp");
end