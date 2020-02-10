if exist('sockfd_sender', 'var')
    release(sockfd_sender)
    clear sockfd_sender
end

if exist('sockfd_recver', 'var')
    release(sockfd_recver)
    clear sockfd_recver
end

if exist('sockfd_imager', 'var')
    clear sockfd_imager
end

disp('[sim] released UDP object done and quit.');
