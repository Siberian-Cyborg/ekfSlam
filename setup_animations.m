function h= setup_animations()
    %h.xt= patch(0,0,'b'); % vehicle true patch(0,0,'b','erasemode','xor')
    h.xv= patch(0,0,'b'); % vehicle estimate patch(0,0,'r','erasemode','xor')
    h.cov0=plot(0,0,'b'); 
    %h.pth= plot(0,0,'k.','markersize',2); % vehicle path estimate plot(0,0,'k.','markersize',2,'erasemode','background')
    %h.obs= plot(0,0,'r'); % observations plot(0,0,'r','erasemode','xor')
    h.xf1= plot(0,0,'r+'); % estimated features plot(0,0,'r+','erasemode','xor')
    h.xf2= plot(0,0,'g+'); % estimated features plot(0,0,'r+','erasemode','xor')
    h.xf3= plot(0,0,'k+'); % estimated features plot(0,0,'r+','erasemode','xor')
    h.cov1= plot(0,0,'r'); % covariance ellipses plot(0,0,'r','erasemode','xor')
    h.cov2= plot(0,0,'g'); % covariance ellipses plot(0,0,'r','erasemode','xor')
    h.cov3= plot(0,0,'k'); % covariance ellipses plot(0,0,'r','erasemode','xor')
end

