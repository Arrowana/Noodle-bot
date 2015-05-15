N_sample=100
import math as m

gyro_x_cal=sum()/N_sample
gyro_y_cal=sum()/N_sample
gyro_z_cal=sum()/N_sample

acc_x_cal=sum()/N_sample
acc_y_cal=sum()/N_sample
acc_z_cal=sum()/N_sample

s_acc=
s_gyro=

acc_pitch=m.atan2((acc_x-acc_x_cal)/s_acc,(acc_z-acc_z_cal)/s_acc)
gyro_pitch+=dt*(gyro_x-gyro_x_cal)/s_gyro

acc_roll=m.atan2((acc_y-acc_y_cal)/s_acc,(acc_z-acc_z_cal)/s_acc)
gyro_roll+=(gyro_y-gyro_y_cal)/s_gyro

alpha= tau/(tau+dt)

# Complementary filter
# angle=alpha*(angle+speed*dt)-(1-alpha)*acc
