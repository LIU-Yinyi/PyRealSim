%% @author: LIU-Yinyi
sockfd_sender = dsp.UDPSender('RemoteIPAddress', '127.0.0.1', 'RemoteIPPort', 6666);
sockfd_recver = dsp.UDPReceiver('RemoteIPAddress', '0.0.0.0', 'LocalIPPort', 6667);
sockfd_imager = tcpclient('127.0.0.1', 6668);
show_help();
disp("[sim] created UDP object and init done.")


function show_help()
    disp("----- Help Desk -----");
    disp("@example: sim_init()");
    disp("@note: sockfd_sender for sim_send(cmd, dat)");
    disp("@note: sockfd_recver for dat = sim_recv(cmd)");
    disp("@note: sim_quit to release object and quit");
end