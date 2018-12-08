gyroVar = 1e-5 * ones(3,1);
gyroBiasVar = 1e-9 * ones(3,1);

attVarInit =1e-5 * ones(3,1);
gyroBiasVarInit = 1e-7 * ones(3,1);


gyroBias = 1e-9 * ones(3,1);

accVar = 1e-3 * ones(3,1);
MagVar = 1e-4 * ones(3,1);

magDec = 8.3;
noise = NoiseParameter(gyroVar,gyroBias,gyroBiasVar,gyroBiasVarInit,...
                accVar,MagVar,attVarInit,magDec);

filter = IMU_ErrorStateKalmanFilter('NAV_1.mat',noise);
filter.plot();

