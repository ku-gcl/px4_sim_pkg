from scipy.io import loadmat
import px4_sim_pkg.RTDMD as RTDMD
import px4_sim_pkg.DMD as DMD
import numpy as np

# .matファイルを読み込む
# filename = "../dmd/data/2022-12-13-flightdata.mat"
filename = "data/2022-12-13-flightdata.mat"
mat = loadmat(filename)
y = mat['y'].T
u = mat['u'].T
Ts = 16/400
l, m = y.shape
tvec = np.arange(0, Ts*l, Ts)
split_index = 1687


# 訓練セット（最初の半分）
train_data_y = y[:, 0:split_index]
train_data_u = u[:, 0:split_index]

# 検証セット（残りの半分）
test_data_y = y[:, split_index:]
test_data_u = u[:, split_index:]


# パラメータ設定
stateDim, _ = y.shape
inputDim, _ = u.shape
delta = 10**(-3)
lam = 0.97

aug = 2*stateDim+1
# aug = 2
# aug = 1

# RTDMD初期化
rtdmd = RTDMD.RTDMD(delta, lam, aug, stateDim, inputDim)




# %%
dmd = DMD.DMD()
train_data_delay_y = rtdmd.create_delay_coordinates(train_data_y)
train_data_delay_u = rtdmd.create_delay_coordinates(train_data_u)
Y = dmd.concatenate(train_data_delay_y, train_data_delay_u)
dmd.splitdata(XX=Y, stateDim=stateDim, aug=aug)
A = dmd.DMD()

_, nTime = train_data_y.shape

xPredict = rtdmd.predict_state(A, dmd.X, nTime)
Error = rtdmd.calculate_fit_error(train_data_y, xPredict)

# %%
# 検証データ
test_data_delay_y = rtdmd.create_delay_coordinates(test_data_y)
test_data_delay_u = rtdmd.create_delay_coordinates(test_data_u)
YV = dmd.concatenate(test_data_delay_y, test_data_delay_u)
_, NV = test_data_delay_y.shape

# xPredictVDMD = predict_state(A, YV, NV, aug, stateDim)
# ErrorVDMD = calculate_fit_error(test_data_y[:, aug:], xPredictVDMD[:, aug:])

# %%
# 予測値格納用の配列初期化
xPredictV = np.zeros((stateDim, NV))

# RTDMD 初期化
rtdmd.rtdmdResult["AB"] = A
prevData = YV[:, 0]

# ログ記録用
dmdlog = [None] * NV

# メインループ
for i in range(NV-1):
    newData = YV[:, i+1]
    rtdmd.update_rtdmd(prevData, newData, i+1)
    xPredictV[:, i+1] = rtdmd.rtdmdResult['y_hat']
    
    dmdlog[i] = rtdmd.rtdmdResult
    prevData = newData


# %%
# 予測前の1~aug列までは0埋めしておく
xPredictV = np.concatenate((np.zeros((stateDim, aug)), xPredictV[:, 1:]), axis=1)

# 誤差計算
ErrorV = rtdmd.calculate_fit_error(test_data_y[:, aug+1:], xPredictV[:, aug+1:])

print(ErrorV)
