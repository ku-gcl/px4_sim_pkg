from datetime import datetime
import os
import numpy as np


# A行列を現在の日時を含むファイル名で保存
home_directory = os.path.expanduser('~')
current_date_str = datetime.now().strftime("%Y-%m-%d")
data_directory = os.path.join(home_directory, 'rosbag', current_date_str)
# dataディレクトリが存在しない場合は作成
if not os.path.exists(data_directory):
    os.makedirs(data_directory)
current_time_str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
filename = os.path.join(data_directory, f"A_matrix_{current_time_str}.csv")
a = np.array([[1, 1, 1], [2, 3, 4]])
np.savetxt(filename, a, delimiter=',')
print(f"A matrix saved to {filename}")
