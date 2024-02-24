import math



class Trajectory():
    def __init__(self):
        pass

    def circle(self, time_sec, radius=1.0, altitude=1.5):
        """
        指定された時間に基づいて、円軌道上のポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        radius (float): 円軌道の半径
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = radius * math.cos(time_sec)
        y = radius * math.sin(time_sec)
        z = altitude
        return x, y, z

    def eight(self, time_sec, size=1.0, altitude=1.5):
        """
        指定された時間に基づいて、8の字軌道上のポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        size (float): 8の字の大きさ（この値でx軸とy軸の振幅が決まります）
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = size * math.sin(time_sec)
        y = size * math.sin(time_sec) * math.cos(time_sec)
        z = altitude
        return x, y, z
