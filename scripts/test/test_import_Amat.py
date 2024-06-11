import glob
import os
import numpy as np

# ホームディレクトリを展開
home_directory = os.path.expanduser('~')

# 指定されたパターンにマッチするファイルのリストを取得
pattern = os.path.join(home_directory, 'rosbag', '2024-03-26', 'A_matrix*.csv')
files = glob.glob(pattern)

# 最新のファイルを見つける
latest_file = max(files, key=os.path.getmtime)

# 最新のファイルの内容を読み込む
A_loaded = np.loadtxt(latest_file, delimiter=',')
print(A_loaded)
