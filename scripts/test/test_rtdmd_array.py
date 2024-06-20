import numpy as np

# 以下のエラーに対するテスト
# Traceback (most recent call last):
#   File "/home/ta2ya/catkin_ws/src/px4_sim_pkg/scripts/test/test_rtdmd_array.py", line 11, in <module>
#     newData[0:4] = x_data
# ValueError: could not broadcast input array from shape (4,1) into shape (4,)


# 修正コード1: dimension of the array ###################
imu_data_rtdmd = []
newData = np.zeros(4)

data = [1, 2, 3, 4]
imu_data_rtdmd.append(data)

x_data = np.array(imu_data_rtdmd).T

# flattenを付けることで、x_data.shapeを(4, 1)から(4,)に変更
newData[0:4] = x_data.flatten()

# u_data = newData[0:2, :]   # IndexError: too many indices for array: array is 1-dimensional, but 2 were indexed
u_data = newData[0:2]



# 修正コード2: dimension of the x_data ###################
imu_data_rtdmd = []
newData = np.zeros(4)

data = [1, 2, 3, 4]
imu_data_rtdmd.append(data)
imu_data_rtdmd.append(data)
imu_data_rtdmd.append(data)

x_data = np.array(imu_data_rtdmd).T

# newData[0:4] = x_data.flatten()   # ValueError: could not broadcast input array from shape (12,) into shape (4,)
newData[0:4] = x_data[0:4, 0].flatten() 

u_data = newData[0:2]



# 修正コード3 ###################
# xPredictV = np.concatenate((np.zeros((stateDim, aug)), xPredictV[:, 0:]), axis=1)
# IndexError: too many indices for array: array is 1-dimensional, but 2 were indexed
xPredictV = []
stateDim = 6
aug = 3

x_k1 = np.array([1, 2, 3, 4, 5, 6])

xPredictV.append(x_k1)
xPredictV.append(x_k1)
xPredictV.append(x_k1)

# after loop
xPredictV = np.array(xPredictV).T
xPredictV = np.concatenate((np.zeros((stateDim, aug)), xPredictV[:, 0:]), axis=1)


print(xPredictV)

# # 元コード ###################
# imu_data_rtdmd = []
# newData = np.zeros(4)

# data = [1, 2, 3, 4]
# imu_data_rtdmd.append(data)

# x_data = np.array(imu_data_rtdmd).T

# newData[0:4] = x_data

