num_sample = 1500;
num_pi = 7;
test_num_sample = num_sample*2;
test_num_pi = num_pi*2;
gravity = [0 0 -9.8];

% mdl_puma560;
% twolink_link1 = p560.links(1);
% twolink_link2 = p560.links(2);
%---------------mdh------------------
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = 0; d(2) = 0; a(2) = 4; alp(2) = 0;
twolink_link1 =  Link([th(1), d(1), a(1), alp(1)], 'modified');
twolink_link2 = Link([th(2), d(2), a(2), alp(2)], 'modified');
%-----------------------------------
%--------------dyn_params-----------
twolink_link1.m = 20; twolink_link2.m = 15;%
twolink_link1.r = [2 0 0];
twolink_link2.r = [1.5 0 0];
twolink_link1.I = [0 0 0; 0 0 0; 0 0 0.5];
twolink_link2.I = [0 0 0; 0 0 0; 0 0 0.2];

twolink_link1.G = 0;
twolink_link2.G = 0;
twolink_link1.Jm = 1;
twolink_link2.Jm = 1;
%----------------------------------
rob_twolink = SerialLink([twolink_link1,twolink_link2],...
    'name','rob_two_link');
twolink_p = [sin(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)',...
    cos(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)'];
twolink_v = [cos(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)',...
    -sin(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)'];
twolink_a = [-sin(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)',...
    -cos(0:(test_num_pi*pi/(test_num_sample-1)):test_num_pi*pi)'];
tor_twolink = zeros(length(twolink_p),rob_twolink.n);
for i = 1:length(twolink_p)
    tor_twolink(i,:) = rob_twolink.rne(twolink_p(i,:),twolink_v(i,:),...
        twolink_a(i,:),gravity);
end
iden_tor_q_dq_ddq = [tor_twolink(1:num_sample,:),twolink_p(1:num_sample,:),...
    twolink_v(1:num_sample,:),twolink_a(1:num_sample,:)];
test_tor_q_dq_ddq = [tor_twolink(num_sample+1:end,:),...
    twolink_p(num_sample+1:end,:),...
    twolink_v(num_sample+1:end,:),twolink_a(num_sample+1:end,:)];
csvwrite('iden_tor_q_dq_ddq.csv',iden_tor_q_dq_ddq); 
csvwrite('test_tor_q_dq_ddq.csv',test_tor_q_dq_ddq); 
