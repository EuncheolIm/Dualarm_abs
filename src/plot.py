import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def lmake_xy( dir_file):
        xy = np.genfromtxt(dir_file, delimiter='', dtype = np.float64)
        return xy

data = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/test1.txt")
# data2 = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/comp_clik.txt")
# data = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/comp_clik.txt")
print("d : ", data.shape)


mode = 5
if (mode == 1):
        pos = data[:,:6] 
        ref = data[:,6:12] 
        pos[3:] = pos[3:]*180/np.pi
        ref[3:] = ref[3:]*180/np.pi

        plt.figure(figsize=(20,10))
        for i in range(6):
                plt.subplot(3,2,i+1)

                plt.plot(ref[:,i],  label = "ref")
                plt.plot(pos[:,i], label = "MPC")

                plt.xlabel(" time (ms) ")
                # plt.ylabel(" angle (deg)")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()
elif(mode == 2):
        pos = ["x","y","z", "roll","pitch","yaw"]

        current_pos = data[:,:6] 
        reference_pos = data[:,6:12] 
        current_pos[:,3:] = current_pos[:,3:]* 180/np.pi
        reference_pos[:,3:] = reference_pos[:,3:]* 180/np.pi
        
        # clikpos = data2[:,:6] 
        # clikpos[:,3:] = clikpos[:,3:]* 180/np.pi
        # reference_pos = data[:,6:12] 

        # error = reference_pos - current_pos

        plt.figure(figsize=(20,10))
        for i in range(6):
                plt.subplot(2,3,i+1)
                if (i<3):
                        plt.title("position {}".format([pos[i]]))
                        plt.ylabel(" distance (m)")
                else:
                        plt.title("orientation {}".format([pos[i]]))
                        plt.ylabel(" angle (deg)")
                plt.plot(reference_pos[:,i], label = "reference")
                plt.plot(current_pos[:,i],  label = "MPC")
                # plt.plot(clikpos[:,i],  label = "CLIK")
                
                # plt.plot(current_pos1[:,i],  label = "CLIK")
                # plt.plot(error[:,i], ':',  label = "error")
                plt.xlabel(" time (ms) ")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()
elif(mode == 3):

        ref = data[:,:6] 
        pos = data[:,6:12] 

        ref_x = ref[:,0]
        ref_y = ref[:,1]
        ref_z = ref[:,2]

        x = pos[:,0]
        y = pos[:,1]
        z = pos[:,2]
        
        ref = data[:,:6] 
        pos = data[:,6:12] 


        fig = plt.figure(figsize=(16,12))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x,y,z, label = " ref ")
        ax.plot(ref_x,ref_y,ref_z ,label = " pos ")
        # ax.plot(x1,y1,z1, label = "constraint y<0.05 ")
        # ax.plot(pred_x,pred_y,pred_z , 'bo', label = " desired ")
        ax.set_xlabel("x-axis (m)")
        ax.set_ylabel("y-axis (m)")
        ax.set_zlabel("z-axis (m)")

        plt.legend()
        plt.show()
        
elif(mode == 4):
        Fx = data[:,19:] 
        # Tx = data[:,21:] 
        plt.figure(figsize=(20,10))
        for i in range(6):
                plt.subplot(2,3,i+1)
                pos = ["Fx","Fy","Fz", "Tau x","Tau y","Tau z"]
                if (i<3):
                        plt.title("transition {}".format([pos[i]]))
                        plt.ylabel(" force ")
                else:
                        plt.title("rotation {}".format([pos[i]]))
                        plt.ylabel(" tau ")
                plt.plot(Fx[:,i], label = " force ")

                plt.xlabel(" time (ms) ")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()
elif(mode == 5):
        current_q = data[:,:7] *180/np.pi
        reference_q = data[:,7:14]*180/np.pi
        plt.figure(figsize=(20,10))
        for i in range(7):
                plt.subplot(4,2,i+1)

                plt.plot(reference_q[:,i], label = "reference")
                plt.plot(current_q[:,i],  label = "MPC")

                plt.xlabel(" time (ms) ")
                plt.ylabel(" angle (deg)")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()
elif(mode == 6):
        torque = data[:,:7] 
        plt.figure(figsize=(20,10))
        for i in range(7):
                plt.subplot(4,2,i+1)

                plt.plot(torque[:,i], label = "torque")

                plt.xlabel(" time (ms) ")
                plt.ylabel(" torque (N.m)")
                plt.subplots_adjust(
                wspace=0.25, # the width of the padding between subplots 
                hspace=0.3) #
                
                
                
                plt.legend()
        plt.show()


else:
        print(np.tanh(1))
        
        
        
        
        # unit(0) = yaml_node["MPC_ddot_x"]["x_pos"].as<double>();
	# 	unit(1) = yaml_node["MPC_ddot_x"]["x_ori"].as<double>();
	# 	unit(2) = yaml_node["MPC_ddot_x"]["xdot_pos"].as<double>();
	# 	unit(3) = yaml_node["MPC_ddot_x"]["xdot_ori"].as<double>();
        # unit(4) = yaml_node["MPC_ddot_x"]["R"].as<double>();
	# 	unit(5) = yaml_node["MPC_ddot_x"]["dT"].as<double>();
        # unit(6) = yaml_node["MPC_ddot_x"]["Np"].as<double>();
	# 	unit(7) = yaml_node["MPC_ddot_x"]["loop"].as<double>();

	# 	// for(int i=0; i<3; i++){
	# // 	Q_temp(i,i) = param(0);					// x_pos
	# // 	Q_temp(i+3,i+3) = param(1);				// x_ori
	# // 	Q_temp(i+6,i+6) = param(2);				// xdot_pos
	# // 	Q_temp(i+9,i+9) = param(3);				// xdot_ori
	# // }
 
  # x_pos : 100
  # x_ori : 1
  # xdot_pos : 0.1
  # xdot_ori : 0.1
  # R : 0.001
  # dT : 0.01
  # Np : 10
  # loop : 10

