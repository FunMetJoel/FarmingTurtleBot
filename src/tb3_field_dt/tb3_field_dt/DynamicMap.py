import numpy as np

class DynamicMap():
    """A map that can dynamically grow
    
    Parameters:  
    resolution (float): meter per pixel
    """
    def __init__(self, resolution: float, origin:tuple[float, float] = (0.0, 0.0), startSize:tuple[float, float] = (10, 10)):
        self.resolution:float = resolution
        self.originX = origin[0]
        self.originY = origin[1]
        self.sizeX = startSize[0]
        self.sizeY = startSize[1]
        self.array:np._ArrayFloat_co = np.full(startSize, np.nan, dtype=np.float64)
    
    def setPixelAtLocation(self, worldX:float, worldY:float, value:float):
        pX, pY = self.worldToPixelCords(worldX, worldY)
        if not self.__inBounds(pX, pY):
            self.__expandToInclude(pX, pY)
            pX, pY = self.worldToPixelCords(worldX, worldY)
        self.__setPixel(pX, pY, value)
    
    def __setPixel(self, x:int, y:int, value:float):
        self.array[y, x] = value

    def __expandToInclude(self, x:int, y:int):
        oldPixelOriginX = 0
        oldPixelOriginY = 0
        print("pos", x, y)
        print("size", self.sizeX, self.sizeY)
        if (self.sizeX <= x):
            self.sizeX = x + 1
        elif (x < 0):
            self.originX = float(x) * self.resolution + self.originX
            self.sizeX = self.sizeX - x
            oldPixelOriginX = -x

        if (self.sizeY <= y):
            self.sizeY = y + 1
        elif (y < 0):
            self.originY = float(y) * self.resolution + self.originY
            self.sizeY = self.sizeY - y
            oldPixelOriginY = -y

        print("OldOrigin", oldPixelOriginX, oldPixelOriginY)
        print("size", self.sizeX, self.sizeY)

        oldMap = self.array.copy()
        print(oldMap)
        self.array = np.full((self.sizeY, self.sizeX), np.nan, dtype=np.float64)
        print(self.array)
        print(oldPixelOriginX, oldPixelOriginX + oldMap.shape[1], oldPixelOriginY, oldPixelOriginY + oldMap.shape[0])
        self.array[oldPixelOriginY:oldPixelOriginY + oldMap.shape[0], oldPixelOriginX:oldPixelOriginX + oldMap.shape[1]] = oldMap
        print(self.array)

    def __inBounds(self, x:int, y:int) -> bool:
        return (self.sizeX > x) and (0 <= x) and (self.sizeY > y) and (0 <= y)
    
    def pixelToWorldCords(self, x:int, y:int) -> tuple[float, float]: 
        worldX:float = (float(x) * self.resolution) + self.originX
        worldY:float = (float(y) * self.resolution) + self.originY
        return worldX, worldY
    
    def worldToPixelCords(self, x:float, y:float) -> tuple[int, int]:
        # (0,0) should be at the center of the pixel, so we round to the nearest pixel
        pixelX:int = round((x - self.originX) / self.resolution)
        pixelY:int = round((y - self.originY) / self.resolution)
        return pixelX, pixelY
    
    def toOccupancyGridData(self, min:float, max:float):
        occupancyGridData = np.empty((self.sizeX, self.sizeY), dtype=np.int8) # Array with values between -1 and 100
       
        scale:float = max - min
        occupancyGridData = ((self.array - min) * (100.0 / scale)).clip(0, 100).round()
    
        occupancyGridData[np.isnan(self.array)] = -1

        return occupancyGridData.astype(int)
    
    def subtractEverywhere(self, value:float):
        self.array = self.array - value
        self.array[self.array < 0.001] = 0.001
            



