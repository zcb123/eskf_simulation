clear
close all;
load("data/N41_2024-11-19_11-19-29.mat");

% atto_t = double(data.ATTO.t)/1e6;
%%
imu_t = double(data.IMU1.t)/1e6;
imuL_t = double(data.IML1.t)/1e6;
figure
plot(imu_t,data.IMU1.GX,imuL_t,data.IML1.GXL);      %对应机体坐标系y轴
figure
plot(imu_t,data.IMU1.AZ,imuL_t,data.IML1.AZL);
%%
res_t = double(data.CTRL.t)/1e6;
res_1 = data.CTRL.res0;
res_2 = data.CTRL.res1;
res_3 = data.CTRL.res2;
res_4 = data.CTRL.res3;
figure
plot(res_t,res_1,res_t,res_2,res_t,res_3,res_t,res_4);
legend('res1','res2','res3','res4');
%%
eskf_t = double(data.ESKF.t)/1e6;
figure
subplot(311)
plot(eskf_t,data.ESKF.roll);
legend('roll');
subplot(312)
plot(eskf_t,data.ESKF.pitch);
legend('pitch');
subplot(313)
plot(eskf_t,data.ESKF.yaw);
legend('yaw');
%%
eskf_t = double(data.ESKF.t)/1e6;

eskf_dt = 0.008*ones(length(eskf_t),1);
eskf_dt(1:end-1,1) = diff(eskf_t);

pD_delta = 0.02*ones(length(eskf_t),1);
pD_delta(1:end-1,1) = diff(data.ESKF.pD);
vD_cal = pD_delta./eskf_dt;
% vD_cal = rmoutliers(vD_cal);
pN_delta = 0.02*ones(length(eskf_t),1);
pN_delta(1:end-1,1) = diff(data.ESKF.pN);
vN_cal = pN_delta./eskf_dt;

pE_delta = 0.02*ones(length(eskf_t),1);
pE_delta(1:end-1,1) = diff(data.ESKF.pE);
vE_cal = pE_delta./eskf_dt;

figure('Name','vel eskf_cal')
subplot(311)
plot(eskf_t,data.ESKF.vN,eskf_t,vN_cal);
grid on
legend('vNeskf','vNcal')
subplot(312)
plot(eskf_t,data.ESKF.vE,eskf_t,vE_cal);
grid on
legend('vEeskf','vEcal')
subplot(313)
plot(eskf_t,data.ESKF.vD,eskf_t,vD_cal);
grid on
legend('vDeskf','vDcal')

%%
rtk_t = double(data.RTK.t)/1e6;
figure
plot(eskf_t,vD_cal,rtk_t,data.RTK.vD);
legend('vDcal','vDrtk');
%%
figure
plot(eskf_dt)
rtk_t = double(data.RTK.t)/1e6;
rtk_dt = 0.008*ones(length(rtk_t),1);
rtk_dt(2:end,1) = diff(rtk_t);
vD_rtk_delta = 0.002*ones(length(rtk_t),1);
vD_rtk_delta(2:end,1) = diff(data.RTK.vD);
aD_rtk_cal = vD_rtk_delta./rtk_dt;
vD_delta = 0.002*ones(length(eskf_t),1);
vD_delta(2:end,1) = diff(data.ESKF.vD);
aD_cal = vD_delta/0.002;

vN_rtk_delta = 0.02*ones(length(rtk_t),1);
vN_rtk_delta(2:end,1) = diff(data.RTK.vN);
aN_rtk_cal = vN_rtk_delta./rtk_dt;
vN_delta = 0.002*ones(length(eskf_t),1);
vN_delta(2:end,1) = diff(data.ESKF.vN);
aN_cal = vN_delta/0.002;

vE_rtk_delta = 0.02*ones(length(rtk_t),1);
vE_rtk_delta(2:end,1) = diff(data.RTK.vE);
aE_rtk_cal = vE_rtk_delta./rtk_dt;
vE_delta = 0.002*ones(length(eskf_t),1);
vE_delta(2:end,1) = diff(data.ESKF.vE);
aE_cal = vE_delta/0.002;

figure('Name','Acc Rtk_Eskf_Cal')
subplot(311)
plot(rtk_t,aN_rtk_cal,eskf_t,data.ESKF.aN,eskf_t,aN_cal);
grid on
legend('aNrtkCal','aNeskf','aNcal');
subplot(312)
plot(rtk_t,aE_rtk_cal,eskf_t,data.ESKF.aE,eskf_t,aE_cal);
grid on
legend('aErtkCal','aEeskf','aEcal');
subplot(313)
plot(rtk_t,aD_rtk_cal,eskf_t,data.ESKF.aD,eskf_t,aD_cal);
grid on
legend('aDrtkCal','aDeskf','aDcal');

%%
figure
subplot(311)
plot(eskf_t,data.ESKF.aN,eskf_t,data.ESKF.vN);
legend('aN','vN')
grid on
%%
figure
plot(eskf_t,data.ESKF.aE,'*-',eskf_t,data.ESKF.vE,'*-');
legend('aE','vE')
grid on


%%
figure
plot(eskf_t,data.ESKF.aD,'*-',eskf_t,data.ESKF.vD)
legend('aD','vD')
grid on
%%
% rtk_t = double(data.RTK.t)/1e6;
% rtk_dt = 0.002*ones(length(rtk_t),1);
% rtk_dt(2:end,1) = diff(rtk_t);
% 
% pD_delta_rtk = 0.02*ones(length(rtk_t),1);
% pD_delta_rtk(2:end,1) = diff(data.RTK.pD);
% vD_cal_rtk = pD_delta_rtk./rtk_dt;
% 
% pN_delta_rtk = 0.02*ones(length(rtk_t),1);
% pN_delta_rtk(2:end,1) = diff(data.RTK.pN);
% vN_cal_rtk = pN_delta_rtk./rtk_dt;
% 
% pE_delta_rtk = 0.02*ones(length(rtk_t),1);
% pE_delta_rtk(2:end,1) = diff(data.RTK.pE);
% vE_cal_rtk = pE_delta_rtk./rtk_dt;
% 
% figure('Name','vel rtk_cal')
% subplot(311)
% plot(rtk_t,data.RTK.vN,rtk_t,vN_cal_rtk);
% grid on
% legend('vNrtk','vNcal')
% subplot(312)
% plot(rtk_t,data.RTK.vE,rtk_t,vE_cal_rtk);
% grid on
% legend('vErtk','vEcal')
% subplot(313)
% plot(rtk_t,data.RTK.vD,rtk_t,vD_cal_rtk);
% grid on
% legend('vDrtk','vDcal')

%%
% rtk_t = double(data.RTK.t)/1e6;
% rtk_dt = 0.008*ones(length(rtk_t),1);
% rtk_dt(1:end-1,1) = diff(rtk_t);
% figure
% plot(rtk_t,rtk_dt);
% legend('rtkDt')

%%
figure('Name','pos-vel eskf')
subplot(311)
plot(eskf_t,data.ESKF.pN,eskf_t,data.ESKF.vN);
grid on
legend('pN','vN');
subplot(312)
plot(eskf_t,data.ESKF.pE,eskf_t,data.ESKF.vE);
grid on
legend('pE','vE');
subplot(313)
plot(eskf_t,data.ESKF.pD,eskf_t,data.ESKF.vD);
grid on
legend('pD','vD');

%%
figure('Name','bias')
plot(data.ESTU.abX)
hold on 
plot(data.ESTU.abY)
hold on
plot(data.ESTU.abZ)
legend('abX','abY','abZ');
%%
figure
plot(data.ESTU.mR);
hold on
plot(data.ESTU.pR);
hold on
plot(data.ESTU.vR);
legend('mR','pR','vR');
%%
figure('Name','vel eskf_rtk')
subplot(311)
plot(eskf_t,data.ESKF.vN,rtk_t,data.RTK.vN);
grid on
legend('vNkf','vNrtk');
subplot(312)
plot(eskf_t,data.ESKF.vE,rtk_t,data.RTK.vE);
grid on
legend('vEkf','vErtk');
subplot(313)
plot(eskf_t,data.ESKF.vD,rtk_t,data.RTK.vD);
grid on
legend('vDkf','vDrtk');
%%
figure('Name','pos eskf_rtk')
subplot(311)
plot(eskf_t,data.ESKF.pN,rtk_t,data.RTK.pN);
grid on
legend('pNkf','pNrtk');
subplot(312)
plot(eskf_t,data.ESKF.pE,rtk_t,data.RTK.pE);
grid on
legend('pEkf','pErtk');
subplot(313)
plot(eskf_t,data.ESKF.pD,rtk_t,data.RTK.pD);
grid on
legend('pDkf','pDrtk');
%%
rtk_t = double(data.RTK.t)/1e6;
figure('Name','RTK status');
subplot(211)
plot(rtk_t,data.RTK.svs);
legend('startNum');
subplot(212)
plot(rtk_t,data.RTK.fix,rtk_t,data.RTK.pdop,rtk_t,data.RTK.hdop);
legend('fixtype','pdop','hdop');