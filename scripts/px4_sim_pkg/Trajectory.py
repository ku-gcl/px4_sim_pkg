import math



class Trajectory():
    def __init__(self):
        pass
    
    def hover(self, time_sec, x=0.0, y=0.0, altitude=1.5):
        """
        指定された高度でホバリングするためのポイントを計算します。
        
        Parameters:
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = x
        y = y
        z = altitude
        return x, y, z


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

    def eightXY(self, time_sec, amplitude=1.0, altitude=1.5):
        """
        指定された時間に基づいて、8の字軌道上のポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        amplitude (float): 8の字の大きさ（この値でx軸とy軸の振幅が決まります）
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = amplitude * math.sin(time_sec)
        y = amplitude * math.sin(time_sec) * math.cos(time_sec)
        z = altitude
        return x, y, z

    def eightXZ(self, time_sec, amplitude=1.0, altitude=1.5):
        """
        指定された時間に基づいて、8の字軌道上のポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        amplitude (float): 8の字の大きさ（この値でx軸とy軸の振幅が決まります）
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = amplitude * math.sin(time_sec)
        y = 0
        z = amplitude * math.sin(time_sec) * math.cos(time_sec) + altitude
        return x, y, z
    
    def updown(self, time_sec, amplitude=0.5, base_altitude=1.5, w=0.7):
        """
        上下運動を実現するためのポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        amplitude (float): 運動の振幅
        base_altitude (float): 基本となる飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = 0
        y = 0
        z = base_altitude + amplitude * math.sin(w*time_sec)
        return x, y, z
    
    def rightleft(self, time_sec, amplitude=1.0, altitude=1.5, w=0.7):
        """
        左右運動を実現するためのポイントを計算します。
        
        Parameters:
        time_sec (float): 開始からの経過時間（秒）
        amplitude (float): 運動の振幅
        altitude (float): 飛行高度
        
        Returns:
        tuple: (x, y, z) 座標
        """
        x = amplitude * math.sin(w*time_sec)
        y = 0
        z = altitude
        return x, y, z
