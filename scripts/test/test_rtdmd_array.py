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

x_data = np.array(imu_data_rtdmd).T

# flattenを付けることで、x_data.shapeを(4, 1)から(4,)に変更
newData[0:4] = x_data.flatten()

# u_data = newData[0:2, :]   # IndexError: too many indices for array: array is 1-dimensional, but 2 were indexed
u_data = newData[0:2]





# # 元コード ###################
# imu_data_rtdmd = []
# newData = np.zeros(4)

# data = [1, 2, 3, 4]
# imu_data_rtdmd.append(data)

# x_data = np.array(imu_data_rtdmd).T

# newData[0:4] = x_data

