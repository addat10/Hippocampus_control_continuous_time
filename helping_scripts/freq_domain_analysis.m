function []=freq_domain_analysis(G,K,filters)
[So,Si,~]=get_loop_tfs(G,K);
Ws=filters.Ws;
Wk=filters.Wk;
Vr=filters.Vr;
Vd=filters.Vd;
%% Frequency domain analysis
figure()
subplot(221);   sigma(So,   'k-',inv(Ws),'r--',inv(Vr),'r--'); title('S')
subplot(222);   sigma(G*Si, 'k-',inv(Ws),'r--',inv(Vd),'r--'); title('SG')
subplot(223);   sigma(K*So,  'k-',inv(Wk),'r--',inv(Vr),'r--');title('KS')
subplot(224);   sigma(K*G*Si,'k-',inv(Wk),'r--',inv(Vd),'r--');title('KSG')
end